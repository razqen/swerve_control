#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

#define MOTOR_SPEED 255
const int TOLERANCE = 75;

const int limitSwitchPin[4] = { 36, 35, 34, 39 }; //needs rechecking
const int pwm[4] = { 23, 21, 18, 17 }; //5,17,16,22
const int dir[4] = { 22, 19, 5, 16 }; //15,21,4,23
const int ABS_ENC_PIN[4] = { 32, 25, 27, 12 }; // 26,14,25,33
int ZERO_DEG_OFFSET[4] =  { 2350, 1820, 1383, 1920 };
int target_enc[4] =  { ZERO_DEG_OFFSET[0], ZERO_DEG_OFFSET[1], ZERO_DEG_OFFSET[2], ZERO_DEG_OFFSET[3] };
bool homed[4] = { false, false, false, false };
bool homingDir[4];  
bool was_homing = false;

long error[4] = { 0, 0, 0, 0 };
const int dirclockhigh[4] = {false,false,false,true};

long current_position[4] = { 0, 0, 0, 0 };  // Start at 0
bool homing = false;

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

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  
  was_homing = homing;
  // Ensure we received exactly 4 angles
  if (msg->data.size >= 4) {

    homing = (msg->data.data[0] == 0.0f && msg->data.data[1] == 0.0f && msg->data.data[2] == 0.0f && msg->data.data[3] == 0.0f);

    if (homing && !was_homing) {
      for (int i = 0; i < 4; i++) {
      homed[i] = false;
      long startErr = (long)ZERO_DEG_OFFSET[i] - current_position[i];
      if (startErr > 2048) startErr -= 4096;
      else if (startErr < -2048) startErr += 4096;

      if (abs(startErr) <= TOLERANCE) homingDir[i] = dirclockhigh[i];
      else homingDir[i] = (startErr > 0) ? !dirclockhigh[i] : dirclockhigh[i];
      }
    }
  }

  for (int i = 0; i < 4; i++) {
    target_enc[i] = ZERO_DEG_OFFSET[i] + (int)((msg->data.data[i] / 360.0f) * 4096.0f);
      
    // wrapping around target in case angles were greater than 360 or negative
    target_enc[i] = target_enc[i] % 4096;
    if (target_enc[i] < 0) target_enc[i] += 4096;
  }
  // Lock into ROS mode so loop() doesn't overwrite these targets
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
    current_position[i] = analogRead(ABS_ENC_PIN[i]);
    bool swHigh = (digitalRead(limitSwitchPin[i]) == HIGH);

    error[i] = target_enc[i] - current_position[i];
    if (error[i] > 2048) error[i] -= 4096;
    else if (error[i] < -2048) error[i] += 4096;

    bool motor_dir = (error[i] > 0) ? !dirclockhigh[i] : dirclockhigh[i];

    if (homing) {
        if (swHigh) {
            setMotor(0, false, pwm[i], dir[i]);
            if (!homed[i]) {
              ZERO_DEG_OFFSET[i] = current_position[i];
              homed[i] = true;
            }
        } else {
            if (homed[i]) {
              // wheel just left the switch
              int homeErr = ZERO_DEG_OFFSET[i] - current_position[i];
              if (homeErr > 2048) homeErr -= 4096;
              else if (homeErr < -2048) homeErr += 4096;
              homingDir[i] = (homeErr > 0) ? !dirclockhigh[i] : dirclockhigh[i];
            }
            homed[i] = false;
            setMotor(MOTOR_SPEED, homingDir[i], pwm[i], dir[i]);
        }
    } else {
        homed[i] = homed[i] || swHigh;
        if (abs(error[i]) <= TOLERANCE) setMotor(0, false, pwm[i], dir[i]);
        else setMotor(MOTOR_SPEED, motor_dir, pwm[i], dir[i]);
    }
  }

  // publishing current encoder values on "pivot_encoders"
  for (int i = 0; i < 4; i++) pub_msg_data[i] = (float)current_position[i];
  rcl_ret_t ret = rcl_publish(&publisher, &pub_msg, NULL);
  (void)ret;
}


