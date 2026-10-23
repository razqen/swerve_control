#!/usr/bin/env python3
"""
rover_tui.py - eDEX-style control deck for the rover.

Runs in the background (each inside its own pty):
  * python3 controller_sh.py
  * ros2 run teleop_twist_keyboard teleop_twist_keyboard
  * ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0   (ESP-0)
  * ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1   (ESP-1)

Keys you press in this window are forwarded to teleop_twist_keyboard, and the
nine-key pad lights up. Each micro-ROS agent is supervised: a yellow/red line
(or the agent dying) triggers `esptool --port <dev> run`; if that comes back
clean the agent is started again.

Run it from a shell where ROS is sourced (so `ros2` and micro_ros_agent work):
    source /opt/ros/<distro>/setup.bash && source ~/<ws>/install/setup.bash
    python3 rover_tui.py

Keys:  u i o / j k l / m , .   drive (space = stop)
       Caps Lock toggles crab/holonomic mode (shifted teleop keys); Space = stop
       q/z  all speeds +/-10%   w/x  linear +/-   e/c  angular +/-
       1 / 2  manual reset ESP-0 / ESP-1      F5 / F6 / F7 / F8  restart controller / teleop / pivot echo / joy_drive
       Ctrl+Q  quit (sends a stop first)
"""
from __future__ import annotations

import asyncio
import fcntl
import os
import pty
import re
import signal
import struct
import termios
import time
from collections import deque

from rich.console import Group
from rich.text import Text
from textual import events
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.widgets import Button, Footer, RichLog, Static

# ════════════════════════════ config ════════════════════════════
CONTROLLER_CMD = ["python3", "controller_sh.py"]
CONTROLLER_CWD = None  # None = directory you launch this from
TELEOP_CMD = ["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"]
PIVOT_ECHO_CMD = ["ros2", "topic", "echo", "/pivot_encoders"]
JOY_DRIVE_CMD = ["python3", "joy_drive.py"]

ESPS = [("ESP-0", "/dev/ttyUSB0"), ("ESP-1", "/dev/ttyUSB1")]


def agent_cmd(port: str) -> list[str]:
    return ["ros2", "run", "micro_ros_agent", "micro_ros_agent", "serial", "--dev", port]


def esptool_cmd(port: str) -> list[str]:
    return ["esptool", "--port", port, "run"]


MAX_ATTEMPTS = 5          # consecutive reset cycles before giving up (press 1/2 to retry)
BOOT_DELAY_S = 2.0        # let the ESP boot before restarting the agent
RETRY_DELAY_S = 3.0       # wait between failed esptool attempts
ESPTOOL_TIMEOUT_S = 30
HEALTHY_RESET_S = 30      # this long ONLINE clears the attempt counter
WARN_TRIGGERS_RESET = True  # yellow lines trigger a reset too (you said yellow OR red)
STALL_TIMEOUT_S = None    # e.g. 20 -> reset if agent never gets a session from the ESP

# ════════════════════ output classification ═════════════════════
ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]")
SGR_RE = re.compile(r"\x1b\[([0-9;]*)m")
NON_SGR_CSI_RE = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-ln-~]")
ERR_RE = re.compile(
    r"\berror\b|\bfail(ed|ure)?\b|fatal|not found|no such file|permission denied|"
    r"cannot|could not|exception|traceback|\bcritical\b", re.I)
WARN_RE = re.compile(r"\bwarn(ing)?\b|time(d)? ?out|session closed|deleted|\blost\b|disconnect", re.I)
GOOD_RE = re.compile(
    r"session established|(participant|topic|publisher|subscriber|datawriter|datareader|"
    r"requester|replier) created", re.I)
ESPTOOL_BAD_RE = re.compile(
    r"error|fatal|failed|could not open|timed out|no serial data|exception|traceback", re.I)
SPEED_RE = re.compile(r"speed\s+([\d.]+)\s+turn\s+([\d.]+)")


def strip_ansi(s: str) -> str:
    return ANSI_RE.sub("", s)


def classify(raw: str) -> str:
    """-> 'err' | 'warn' | 'ok' | 'info' using ANSI colour first, keywords second."""
    codes: set[int] = set()
    for m in SGR_RE.finditer(raw):
        for c in m.group(1).split(";"):
            if c.isdigit():
                codes.add(int(c))
    plain = strip_ansi(raw)
    if codes & {31, 91} or ERR_RE.search(plain):
        return "err"
    if codes & {33, 93} or WARN_RE.search(plain):
        return "warn"
    if codes & {32, 92} or GOOD_RE.search(plain):
        return "ok"
    return "info"


CLS_STYLE = {"ok": "green", "warn": "yellow", "err": "bold red", "info": "grey62", "note": "cyan"}


def make_line(raw: str, cls: str) -> Text:
    cleaned = NON_SGR_CSI_RE.sub("", raw)
    t = Text.from_ansi(cleaned)
    if "\x1b[" not in cleaned:
        t.stylize(CLS_STYLE.get(cls, ""))
    t.no_wrap = True
    t.overflow = "ellipsis"
    return t


# ═════════════════════════ pty process ══════════════════════════
class PtyProc:
    """Child process on a pty (line-buffered output, real TTY for teleop's termios)."""

    def __init__(self, cmd, on_line, on_exit=None, cwd=None):
        self.cmd = list(cmd)
        self.on_line = on_line
        self.on_exit = on_exit
        self.cwd = cwd
        self.proc: asyncio.subprocess.Process | None = None
        self.master: int | None = None
        self.buf = ""
        self.stopping = False
        self._loop: asyncio.AbstractEventLoop | None = None
        self._task: asyncio.Task | None = None

    @property
    def alive(self) -> bool:
        return self.proc is not None and self.proc.returncode is None

    async def start(self) -> None:
        master, slave = pty.openpty()
        try:
            fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack("HHHH", 40, 200, 0, 0))
            env = {**os.environ, "TERM": "xterm-256color", "PYTHONUNBUFFERED": "1",
                   "RCUTILS_COLORIZED_OUTPUT": "1"}
            self.proc = await asyncio.create_subprocess_exec(
                *self.cmd, stdin=slave, stdout=slave, stderr=slave,
                cwd=self.cwd, env=env, start_new_session=True)
        except BaseException:
            os.close(master)
            raise
        finally:
            os.close(slave)
        flags = fcntl.fcntl(master, fcntl.F_GETFL)
        fcntl.fcntl(master, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        self.master = master
        self._loop = asyncio.get_running_loop()
        self._loop.add_reader(master, self._readable)
        self._task = asyncio.create_task(self._wait())

    def _pump(self) -> int:
        """1 = got data, 0 = nothing right now, -1 = EOF."""
        if self.master is None:
            return -1
        try:
            data = os.read(self.master, 65536)
        except BlockingIOError:
            return 0
        except OSError:
            return -1
        if not data:
            return -1
        self.buf += data.decode("utf-8", "replace")
        parts = re.split(r"\r\n|\r|\n", self.buf)
        self.buf = parts.pop()
        for p in parts:
            if p.strip():
                self.on_line(p)
        return 1

    def _unregister(self) -> None:
        if self._loop is not None and self.master is not None:
            try:
                self._loop.remove_reader(self.master)
            except Exception:
                pass

    def _readable(self) -> None:
        if self._pump() < 0:
            self._unregister()

    async def _wait(self) -> None:
        assert self.proc is not None
        rc = await self.proc.wait()
        while self._pump() > 0:
            pass
        if self.buf.strip():
            self.on_line(self.buf)
            self.buf = ""
        self._unregister()
        if self.master is not None:
            try:
                os.close(self.master)
            except OSError:
                pass
            self.master = None
        if self.on_exit:
            self.on_exit(self, rc)

    def send(self, s: str) -> None:
        if self.alive and self.master is not None:
            try:
                os.write(self.master, s.encode())
            except OSError:
                pass

    async def stop(self, grace: float = 2.0) -> None:
        self.stopping = True
        if self.proc is None:
            return
        if self.proc.returncode is None:
            try:
                os.killpg(self.proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                await asyncio.wait_for(self.proc.wait(), grace)
            except asyncio.TimeoutError:
                try:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                await self.proc.wait()
        if self._task is not None:
            await asyncio.wait({self._task}, timeout=1.0)


async def run_cmd(cmd: list[str], timeout: float) -> tuple[int, list[str]]:
    try:
        proc = await asyncio.create_subprocess_exec(
            *cmd, stdin=asyncio.subprocess.DEVNULL, stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT, start_new_session=True)
    except Exception as e:  # e.g. esptool not installed
        return 127, [str(e)]
    try:
        out, _ = await asyncio.wait_for(proc.communicate(), timeout)
    except asyncio.TimeoutError:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        await proc.wait()
        return 124, ["esptool timed out"]
    text = strip_ansi(out.decode("utf-8", "replace"))
    lines = [ln.strip() for ln in re.split(r"[\r\n]+", text) if ln.strip()]
    return proc.returncode or 0, lines


# ═══════════════════════ teleop key pad ═════════════════════════
KEYS = [
    [("u", "↖", "FWD·LEFT"), ("i", "↑", "FORWARD"), ("o", "↗", "FWD·RIGHT")],
    [("j", "↶", "ROT LEFT"), ("k", "■", "STOP"), ("l", "↷", "ROT RIGHT")],
    [("m", "↙", "BACK·LEFT"), (",", "↓", "BACK"), (".", "↘", "BACK·RIGHT")],
]
SHIFT_MAP = {"<": ",", ">": "."}
# teleop_twist_keyboard bindings we forward (+ holonomic shift variants, z up/down, speed keys)
TELEOP_KEYS = set("uiojlm,.UIOJLM<>tbqzwxec")

# palette per brightness level: (border, key, arrow, label, background)
PAL = {
    "cyan": {
        0: ("#1f6f7d", "#2fd0e6", "#3fb8c8", "#2a6f7a", None),
        1: ("#2aa5b8", "#4fe3f5", "#4fe3f5", "#2aa5b8", "#06252c"),
        2: ("#4fe3f5", "#9ff6ff", "#9ff6ff", "#4fe3f5", "#0b4b57"),
        3: ("#b8fbff", "#001014", "#001014", "#001014", "#19e6ff"),
    },
    "red": {
        0: ("#7d2f1f", "#e6573a", "#c8543f", "#7a3a2a", None),
        1: ("#b8452a", "#f57a4f", "#f57a4f", "#b8452a", "#2c0f06"),
        2: ("#f5694f", "#ffb09f", "#ffb09f", "#f5694f", "#57190b"),
        3: ("#ffd0c4", "#140400", "#140400", "#140400", "#ff5a36"),
    },
}
CW = 13  # cell inner width


def _st(fg: str, bg: str | None) -> str:
    return f"{fg} on {bg}" if bg else fg


class KeyPad(Static):
    def __init__(self, **kw):
        super().__init__(**kw)
        self.last: dict[str, float] = {}
        self.lin: float | None = None
        self.ang: float | None = None
        self.crab_mode = False

    def on_mount(self) -> None:
        self.border_title = "TELEOP // TWIST"
        self.set_interval(1 / 20, self.redraw)

    def press(self, ch: str) -> None:
        self.last[SHIFT_MAP.get(ch, ch.lower())] = time.monotonic()

    def set_speed(self, lin: float, ang: float) -> None:
        self.lin, self.ang = lin, ang

    def set_crab_mode(self, enabled: bool) -> None:
        self.crab_mode = enabled

    def _level(self, key: str, now: float) -> int:
        age = now - self.last.get(key, -1e9)
        return 3 if age < 0.25 else 2 if age < 0.40 else 1 if age < 0.60 else 0

    def redraw(self) -> None:
        now = time.monotonic()
        out = Text(no_wrap=True)
        for row in KEYS:
            cells = []
            for key, arrow, label in row:
                lv = self._level(key, now)
                border, kfg, afg, lfg, bg = PAL["red" if key == "k" else "cyan"][lv]
                cells.append((lv, border, kfg, afg, lfg, bg, key, arrow, label))
            for ri in range(5):
                for ci, (lv, border, kfg, afg, lfg, bg, key, arrow, label) in enumerate(cells):
                    if ci:
                        out.append(" ")
                    bs = _st(border, bg)
                    if ri == 0:
                        out.append("╭" + "─" * CW + "╮", style=bs)
                    elif ri == 4:
                        out.append("╰" + "─" * CW + "╯", style=bs)
                    else:
                        txt, fg, bold = {1: (key.upper(), kfg, True),
                                         2: (arrow, afg, True),
                                         3: (label, lfg, False)}[ri]
                        out.append("│", style=bs)
                        out.append(txt.center(CW), style=_st(fg, bg) + (" bold" if bold else ""))
                        out.append("│", style=bs)
                out.append("\n")
        out.append("\n")
        if self.lin is None:
            out.append("  LIN  --.--   ANG  --.--", style="#2a6f7a")
        else:
            out.append(f"  LIN {self.lin:5.2f} m/s   ANG {self.ang:5.2f} rad/s", style="bold #4fe3f5")
        out.append("\n")
        out.append("  q/z all±  w/x lin±  e ang+  c crab toggle  space = stop", style="#2a6f7a")
        out.append("\n  CRAB MODE  ", style="bold #001014 on #4fe3f5" if self.crab_mode else "#2a6f7a")
        out.append("ON" if self.crab_mode else "OFF", style="bold #001014 on #19c37d" if self.crab_mode else "bold #e5c07b")
        self.update(out)


# ═════════════════════════ process log ══════════════════════════
class ProcLog(RichLog):
    can_focus = False

    def __init__(self, title, cmd, cwd=None, on_text=None, **kw):
        super().__init__(wrap=False, highlight=False, markup=False, max_lines=500,
                         auto_scroll=True, **kw)
        self.title = title
        self.cmd = cmd
        self.cwd = cwd
        self.on_text = on_text
        self.proc: PtyProc | None = None
        self.closing = False

    def set_status(self, s: str) -> None:
        self.border_title = f"{self.title} · {s}"

    async def on_mount(self) -> None:
        self.set_status("STARTING")
        await self.start()

    async def start(self) -> None:
        proc = PtyProc(self.cmd, self._line, self._exit, cwd=self.cwd)
        self.proc = proc
        try:
            await proc.start()
        except Exception as e:
            self.write(Text(f"! cannot launch {' '.join(self.cmd)}: {e}", style="bold red"))
            self.set_status("FAILED")
            return
        self.set_status("● RUNNING")

    def _line(self, raw: str) -> None:
        self.write(make_line(raw, "info"))
        if self.on_text:
            self.on_text(strip_ansi(raw))

    def _exit(self, proc: PtyProc, rc: int) -> None:
        if proc is not self.proc or proc.stopping:
            return
        self.write(Text(f"» process exited (rc={rc})", style="bold red"))
        self.set_status(f"EXITED rc={rc}")

    def send(self, s: str) -> None:
        if self.proc:
            self.proc.send(s)

    async def restart(self) -> None:
        if self.proc:
            await self.proc.stop()
        self.write(Text("» restarting…", style="cyan"))
        await self.start()

    async def shutdown(self) -> None:
        self.closing = True
        if self.proc:
            await self.proc.stop()


# ═══════════════════ micro-ROS agent supervisor ═════════════════
STATE_COLOR = {
    "STOPPED": "grey50", "STARTING": "cyan", "ONLINE": "bright_green", "WARN": "yellow",
    "ERROR": "bold red", "RESETTING": "magenta", "FAILED": "bold red",
}


def fmt_dur(s: float) -> str:
    s = int(max(s, 0))
    return f"{s // 3600:02d}:{s % 3600 // 60:02d}:{s % 60:02d}"


class EspPanel(Static):
    def __init__(self, esp_name: str, port: str, hotkey: int):
        super().__init__()
        self.esp_name, self.port, self.hotkey = esp_name, port, hotkey
        self.status = "STOPPED"
        self.proc: PtyProc | None = None
        self.tail: deque[Text] = deque(maxlen=40)
        self.attempts = 0
        self.resets = 0
        self.recovering = False
        self.closing = False
        self.started_at = 0.0
        self.ok_since = 0.0
        self.last_line_t = 0.0
        self.last_fault = ""
        self._bg: set[asyncio.Task] = set()

    # ── lifecycle ──
    async def on_mount(self) -> None:
        self.border_title = f"{self.esp_name} · micro-ROS agent"
        self.set_interval(0.2, self.redraw)
        await self.start_agent()

    def spawn(self, coro) -> None:
        t = asyncio.create_task(coro)
        self._bg.add(t)
        t.add_done_callback(self._bg.discard)

    def set_status(self, s: str) -> None:
        for st in STATE_COLOR:
            self.remove_class(f"s-{st.lower()}")
        self.status = s
        self.add_class(f"s-{s.lower()}")

    def push(self, raw: str, cls: str) -> None:
        self.tail.append(make_line(raw, cls))

    def note(self, msg: str, cls: str = "note") -> None:
        self.push("» " + msg, cls)

    async def start_agent(self) -> None:
        self.set_status("STARTING")
        self.started_at = time.monotonic()
        proc = PtyProc(agent_cmd(self.port), self.on_line, self.on_agent_exit)
        self.proc = proc
        try:
            await proc.start()
        except Exception as e:
            self.proc = None
            self.note(f"cannot launch agent: {e}", "err")
            self.set_status("FAILED")

    async def stop_agent(self) -> None:
        if self.proc:
            p, self.proc = self.proc, None
            await p.stop()

    async def shutdown(self) -> None:
        self.closing = True
        await self.stop_agent()

    # ── agent output → health ──
    def on_line(self, raw: str) -> None:
        if self.closing or self.recovering:
            return
        cls = classify(raw)
        plain = strip_ansi(raw)
        self.push(raw, cls)
        self.last_line_t = time.monotonic()
        if cls == "err":
            self.set_status("ERROR")
            self.trigger(plain.strip())
        elif cls == "warn":
            self.set_status("WARN")
            if WARN_TRIGGERS_RESET:
                self.trigger(plain.strip())
        elif GOOD_RE.search(plain) and self.status in ("STARTING", "WARN"):
            self.set_status("ONLINE")
            self.ok_since = time.monotonic()

    def on_agent_exit(self, proc: PtyProc, rc: int) -> None:
        if proc is not self.proc or proc.stopping or self.recovering or self.closing:
            return
        self.note(f"agent exited (rc={rc})", "err")
        self.set_status("ERROR")
        self.trigger(f"agent exited rc={rc}")

    # ── recovery: esptool reset → restart agent ──
    def trigger(self, reason: str) -> None:
        if self.recovering or self.closing:
            return
        self.recovering = True
        self.spawn(self._recover(reason))

    def manual_reset(self) -> None:
        self.attempts = 0
        self.trigger("manual reset")

    async def _recover(self, reason: str) -> None:
        try:
            self.last_fault = reason[:90]
            self.note(f"fault: {reason[:90]}", "err")
            await self.stop_agent()
            while not self.closing:
                if self.attempts >= MAX_ATTEMPTS:
                    self.set_status("FAILED")
                    self.note(f"gave up after {MAX_ATTEMPTS} resets - press {self.hotkey} to retry", "err")
                    return
                self.attempts += 1
                self.resets += 1
                self.set_status("RESETTING")
                self.note(f"esptool run ({self.attempts}/{MAX_ATTEMPTS})")
                rc, out = await run_cmd(esptool_cmd(self.port), ESPTOOL_TIMEOUT_S)
                for ln in out[-4:]:
                    self.push(ln, "info")
                if rc == 0 and not ESPTOOL_BAD_RE.search("\n".join(out)):
                    self.note("reset clean, restarting agent", "ok")
                    await asyncio.sleep(BOOT_DELAY_S)
                    if self.closing:
                        return
                    await self.start_agent()
                    return
                self.note(f"reset failed (rc={rc})", "err")
                await asyncio.sleep(RETRY_DELAY_S)
        finally:
            self.recovering = False

    # ── drawing ──
    def redraw(self) -> None:
        now = time.monotonic()
        if self.status == "ONLINE" and now - self.ok_since > HEALTHY_RESET_S:
            self.attempts = 0
        if (STALL_TIMEOUT_S and self.status == "STARTING" and self.proc
                and now - self.started_at > STALL_TIMEOUT_S):
            self.trigger("no session from ESP")

        color = STATE_COLOR[self.status]
        dot = "◉" if now - self.last_line_t < 0.35 else "●"
        head = Text(no_wrap=True)
        head.append(f"{dot} {self.status}", style=f"bold {color}")
        head.append("   ")
        up = fmt_dur(now - self.ok_since) if self.status == "ONLINE" else "--:--:--"
        head.append(f"{self.port}  up {up}  resets {self.resets}  try {self.attempts}/{MAX_ATTEMPTS}",
                    style="grey62")
        rule = Text("─" * 200, style="#1f3f46", no_wrap=True, overflow="crop")
        lines = list(self.tail)[-14:]
        self.update(Group(head, rule, *lines))


# ═══════════════════════════ the app ════════════════════════════
class RoverTUI(App):
    CSS = """
    Screen { background: #03080b; }
    #hdr { height: 1; background: #07161b; padding: 0 1; }
    #top { height: 1fr; }
    #esps { width: 1fr; }
    KeyPad { width: 52; height: 1fr; border: round #1f6f7d; border-title-color: #4fe3f5;
             content-align: center middle; background: #03080b; }
    EspPanel { height: 1fr; border: round #1f6f7d; border-title-color: #4fe3f5;
               padding: 0 1; background: #03080b; overflow: hidden; }
    EspPanel.s-online    { border: round #19c37d; border-title-color: #19c37d; }
    EspPanel.s-starting  { border: round #4fe3f5; }
    EspPanel.s-warn      { border: round #e5c07b; border-title-color: #e5c07b; }
    EspPanel.s-error     { border: round #ff4d4d; border-title-color: #ff4d4d; }
    EspPanel.s-resetting { border: round #c678dd; border-title-color: #c678dd; }
    EspPanel.s-failed    { border: heavy #ff4d4d; border-title-color: #ff4d4d; }
    #controls { width: 52; height: 100%; }
    #crab { height: 3; margin: 0 1; border: round #1f6f7d; background: #03080b; color: #2fd0e6; }
    #crab.crab-on { border: heavy #19c37d; background: #06251a; color: #8affc5; text-style: bold; }
    #bottom { height: 12; }
    ProcLog { width: 1fr; min-width: 0; border: round #1f6f7d; border-title-color: #4fe3f5; background: #03080b; }
    Footer { background: #07161b; }
    """

    BINDINGS = [
        Binding("1", "reset_esp(0)", "Reset ESP-0"),
        Binding("2", "reset_esp(1)", "Reset ESP-1"),
        Binding("f5", "restart_controller", "Restart controller"),
        Binding("f6", "restart_teleop", "Restart teleop"),
        Binding("f7", "restart_pivot_echo", "Restart pivot echo"),
        Binding("f8", "restart_joy_drive", "Restart joy_drive.py"),
        Binding("ctrl+q", "quit", "Quit"),
    ]

    def __init__(self):
        super().__init__()
        self._down = False

    def compose(self) -> ComposeResult:
        self.pad = KeyPad(id="pad")
        self.controller = ProcLog("controller_sh.py", CONTROLLER_CMD, cwd=CONTROLLER_CWD)
        self.teleop = ProcLog("teleop_twist_keyboard", TELEOP_CMD, on_text=self._teleop_text)
        self.pivot_echo = ProcLog("ros2 topic echo /pivot_encoders", PIVOT_ECHO_CMD)
        self.joy_drive = ProcLog("joy_drive.py", JOY_DRIVE_CMD)
        self.esps = [EspPanel(n, p, i + 1) for i, (n, p) in enumerate(ESPS)]
        yield Static(id="hdr")
        self.crab_button = Button("CRAB MODE: OFF  [c to toggle]", id="crab", disabled=True)
        with Horizontal(id="top"):
            with Vertical(id="controls"):
                yield self.pad
                yield self.crab_button
            with Vertical(id="esps"):
                for p in self.esps:
                    yield p
        with Horizontal(id="bottom"):
            yield self.controller
            yield self.teleop
            yield self.pivot_echo
            yield self.joy_drive
        yield Footer()

    def on_mount(self) -> None:
        self.set_interval(0.5, self._header)

    def set_crab_mode(self, enabled: bool) -> None:
        self.pad.set_crab_mode(enabled)
        self.crab_button.label = "CRAB MODE: ON  [c to toggle]" if enabled else "CRAB MODE: OFF  [c to toggle]"
        self.crab_button.set_class(enabled, "crab-on")

    def _header(self) -> None:
        t = Text(no_wrap=True)
        t.append(" ◢ ROVER//CTRL ◣ ", style="bold #001014 on #4fe3f5")
        t.append(f"  {time.strftime('%H:%M:%S')}  ", style="#4fe3f5")
        for p in self.esps:
            t.append(f" {p.esp_name} ", style="grey62")
            t.append(p.status, style=STATE_COLOR[p.status])
            t.append("  ")
        self.query_one("#hdr", Static).update(t)

    def _teleop_text(self, plain: str) -> None:
        m = SPEED_RE.search(plain)
        if m:
            self.pad.set_speed(float(m.group(1)), float(m.group(2)))

    # ── input ──
    def on_key(self, event: events.Key) -> None:
        ch = event.character
        if ch == " ":
            ch = "k"

        # Use c as a dedicated crab-mode toggle. Do not forward it to teleop,
        # where c normally decreases angular speed.
        if ch and ch.lower() == "c":
            self.set_crab_mode(not self.pad.crab_mode)
            event.stop()
            return

        if ch and ch in TELEOP_KEYS | {"k"}:
            # teleop_twist_keyboard uses shifted movement keys for strafing.
            # In crab mode, translate the UI's normal movement keys into those
            # shifted variants explicitly; do not depend on terminal modifiers.
            if ch.lower() in "uiojlm":
                ch = ch.upper() if self.pad.crab_mode else ch.lower()
            elif ch == "," and self.pad.crab_mode:
                ch = "<"
            elif ch == "." and self.pad.crab_mode:
                ch = ">"
            elif ch.isalpha() and ch.lower() not in "uiojlm":
                ch = ch.lower()
            self.teleop.send(ch)
            self.pad.press(ch)
            event.stop()

    def action_reset_esp(self, i: int) -> None:
        if i < len(self.esps):
            self.esps[i].manual_reset()

    async def action_restart_controller(self) -> None:
        await self.controller.restart()

    async def action_restart_teleop(self) -> None:
        await self.teleop.restart()

    async def action_restart_pivot_echo(self) -> None:
        await self.pivot_echo.restart()

    async def action_restart_joy_drive(self) -> None:
        await self.joy_drive.restart()

    # ── shutdown ──
    async def shutdown_all(self) -> None:
        if self._down:
            return
        self._down = True
        self.teleop.send("k")  # publish a stop before we go
        await asyncio.sleep(0.15)
        await asyncio.gather(*(p.shutdown() for p in self.esps), self.controller.shutdown(),
                             self.teleop.shutdown(), self.pivot_echo.shutdown(), self.joy_drive.shutdown(),
                             return_exceptions=True)

    async def action_quit(self) -> None:
        await self.shutdown_all()
        self.exit()

    async def on_unmount(self) -> None:
        await self.shutdown_all()


if __name__ == "__main__":
    RoverTUI().run()
