#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <Wire.h>

#define MOTOR_SPEED 255

#define PCA9548A_ADDR  0x70  // Default address of PCA9548A
#define AS5600_ADDR    0x36  // Fixed address of AS5600 encoder
#define TOTAL_CHANNELS 8     // Channels 0 to 7

#define SDA_PIN 4
#define SCL_PIN 15

int wheel_ch[4] = {7, 5, 0, 1};

const int TOLERANCE = 5;

const int limitSwitchPin[4] = { 36, 35, 34, 39 }; //needs rechecking
const int pwm[4] = { 23, 21, 18, 17 }; //5,17,16,22
const int dir[4] = { 22, 19, 5, 16 }; //15,21,4,23
const int ABS_ENC_PIN[4] = { 32, 25, 27, 12 }; // 26,14,25,33
int ZERO_DEG_OFFSET[4] =  {  2277, 2246, 1394, 1768 };
int target_enc[4] =  { ZERO_DEG_OFFSET[0], ZERO_DEG_OFFSET[1], ZERO_DEG_OFFSET[2], ZERO_DEG_OFFSET[3] };

long error[4] = { 0, 0, 0, 0 };
const int dirclockhigh[4] = {false,false,false,true};

long current_position[4] = { 0, 0, 0, 0 };  // Start at 0

rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg;
float msg_data[4]; // Buffer for incoming array

rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray pub_msg;
float pub_msg_data[4]; // Buffer for outgoing encoder array

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node; 

// Select a single channel (0 to 7) on the PCA9548A
void selectI2CChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Read 12-bit raw angle from the active channel's AS5600 sensor
uint16_t readAS5600Angle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C); // RAW ANGLE register high byte
  if (Wire.endTransmission(true) != 0) {
    return 0xFFFF; // Communication error or sensor disconnected
  }

  if (Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) == 2) {
    uint8_t highByte = Wire.read();
    uint8_t lowByte  = Wire.read();
    return ((highByte & 0x0F) << 8) | lowByte;
  }

  return 0xFFFF; // Timeout or read failure
}

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  // Ensure we received exactly 4 angles
  if (msg->data.size >= 4) {
    for (int i = 0; i < 4; i++) {
      target_enc[i] = ZERO_DEG_OFFSET[i] + (int)((msg->data.data[i] / 360.0f) * 4096.0f);
      
      // wrapping around target in case angles were greater than 360 or negative
      target_enc[i] = target_enc[i] % 4096;
      if (target_enc[i] < 0) target_enc[i] += 4096;
    }
    // Lock into ROS mode so loop() doesn't overwrite these targets
  }
}

void setMotor(int speed, bool direction, int M_PWM, int M_DIR) {
  //make pin compatible
  if (speed == 0) {
    analogWrite(M_PWM, 0);
    return;
  }
  digitalWrite(M_DIR, direction ? HIGH : LOW);
  analogWrite(M_PWM, speed);
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(pwm[i], OUTPUT);
    pinMode(dir[i], OUTPUT);
  }
  pinMode(ABS_ENC_PIN[0],INPUT);
  pinMode(ABS_ENC_PIN[1],INPUT);
  pinMode(ABS_ENC_PIN[2],INPUT);
  pinMode(ABS_ENC_PIN[3],INPUT);

  pinMode(limitSwitchPin[0], INPUT_PULLDOWN);
  pinMode(limitSwitchPin[1], INPUT_PULLDOWN);
  pinMode(limitSwitchPin[2], INPUT_PULLDOWN); 
  pinMode(limitSwitchPin[3], INPUT_PULLDOWN);

  Wire.begin(SDA_PIN, SCL_PIN);

  // Set a 50ms timeout so disconnected channels do not block execution
  Wire.setTimeOut(5);

  set_microros_transports(); // Initializes transport (default over Serial)

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "rover_pivot_node", "", &support);

  // Configure message memory
  msg.data.capacity = 4;
  msg.data.data = msg_data;
  msg.data.size = 0;

  // Create subscriber for a Float32MultiArray on topic "target_angles"
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "target_angles"
  );

  pub_msg.data.capacity = 4;
  pub_msg.data.data = pub_msg_data;
  pub_msg.data.size = 4;

  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "pivot_encoders"
  );
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  for (int i = 0; i < 4; i++) {
    selectI2CChannel(wheel_ch[i]);
    current_position[i] = readAS5600Angle();

    // error = target - current
    // CW rotation  -> encoder DECREASES
    // ACW rotation -> encoder INCREASES
    error[i] = target_enc[i] - current_position[i];
    if (error[i] > 2048) error[i] -= 4096;
    else if (error[i] < -2048) error[i] += 4096;

    bool motor_dir = (error[i] > 0) ? !dirclockhigh[i] : dirclockhigh[i];
    
    if (abs(error[i]) <= TOLERANCE) setMotor(0, false, pwm[i], dir[i]);
    else setMotor(MOTOR_SPEED, motor_dir, pwm[i], dir[i]);
  
  }
  // publishing current encoder values on "pivot_encoders"
  for (int i = 0; i < 4; i++) pub_msg_data[i] = (float)current_position[i];
  rcl_ret_t ret = rcl_publish(&publisher, &pub_msg, NULL);
  (void)ret;
}

