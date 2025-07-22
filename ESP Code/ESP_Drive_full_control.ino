#include <Bluepad32.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/bool.h>

rcl_publisher_t drive_publisher;
std_msgs__msg__Int32 drive_msg;

rcl_publisher_t steer_publisher;
std_msgs__msg__Int32 steer_msg;

rcl_publisher_t mode_publisher;
std_msgs__msg__Int8 mode_msg;

rcl_publisher_t button_publisher;
std_msgs__msg__Int8 button_msg;

rcl_subscription_t drive_subscriber;
std_msgs__msg__Int32 drive_input_msg;

rcl_subscription_t steer_subscriber;
std_msgs__msg__Int32 steer_input_msg;

rcl_subscription_t button_subscriber;
std_msgs__msg__Int8 button_input_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

ControllerPtr myController = nullptr;

const int ESC_PIN = 25;     // ESC connected to this pin (Throttle via RT)
const int SERVO_PIN = 26;   // Servo connected to this pin (Steering via Left Joystick X)
int mode;
int pre_mode;
int escMin = 1500;
int escMax = 1500;
int ESC_min_Y = 1000; // Mode 3
int ESC_min_B = 1350; // Mode 2
int ESC_min_A = 1425; // Mode 1
int ESC_stp_N = 1500; // Mode 0
int ESC_max_A = 1575; // Mode 1
int ESC_max_B = 1650; // Mode 2
int ESC_max_Y = 2000; // Mode 3
int drive = 1500;
int SteerX = 1500;  // Default center position
int pwmSteerUs = 1500;  // Default center position

unsigned long lastDriveInputTime = 0;
const unsigned long driveInputTimeout = 500;  // ms
bool rosDriveActive = false;

unsigned long lastSteerInputTime = 0;
const unsigned long steerInputTimeout = 500;  // ms
bool rosSteerActive = false;


// for speed mode switch when driving
void handleModeSelection() {
    if (!myController || !myController->isConnected()) return;  // 🛡️ Early return if not ready
    if (myController->a()) {
        mode = 1;
    }
    else if (myController->b()) {
        mode = 2;
    }
    else if (myController->y()) {
        mode = 3;
    }
    else if (myController->l1() && myController->r1()) {
        mode = 0;
    }
    else if (myController->x()) {
        mode = 0;
    }

    switch (mode) {
        case 1:
            escMin = ESC_min_A;
            escMax = ESC_max_A;
            break;
        case 2:
            escMin = ESC_min_B;
            escMax = ESC_max_B;
            break;
        case 3:
            escMin = ESC_min_Y;
            escMax = ESC_max_Y;
            break;
        case 0:
        default:
            escMin = ESC_stp_N;
            escMax = ESC_stp_N;
            break;
    }

    if (mode != pre_mode) {
        Serial.printf("Switched to Mode %d | ESC Range: %d - %d\n", mode, escMin, escMax);
        pre_mode = mode;
    }
}

// Called when controller connects
void onConnectedController(ControllerPtr ctl) {
    Serial.println("Xbox Controller connected.");
    myController = ctl;
}

// Called when controller disconnects
void onDisconnectedController(ControllerPtr ctl) {
    Serial.println("Xbox Controller disconnected.");
    if (myController == ctl) {
        myController = nullptr;
    }
}

void drive_callback(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    drive = msg->data;
    lastDriveInputTime = millis();
}

void steer_callback(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    pwmSteerUs = msg->data;
    lastSteerInputTime = millis();
}

void button_callback(const void * msgin) {
    const std_msgs__msg__Int8 * msg = (const std_msgs__msg__Int8 *)msgin;
    uint8_t buttons = msg->data;
    // Optionally handle buttons here — e.g., switching modes remotely
    if (buttons & 1) mode = 1;        // A
    else if (buttons & 2) mode = 2;   // B
    else if (buttons & 4) mode = 3;   // Y
    else if (buttons & 16 && buttons & 32) mode = 0;  // LB + RB
}

void setup() { 
  Serial.begin(115200);
  delay(2000); // Allow time for serial and Bluetooth setup
  Serial.println("Starting setup...");\

  set_microros_transports();

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);
  rclc_publisher_init_default(
    &drive_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "Drive"
  );
  rclc_publisher_init_default(
    &steer_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "Steer"
  );
  rclc_publisher_init_default(
    &mode_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "Mode"
  );
  rclc_publisher_init_default(
    &button_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "Button"
  );
  rclc_subscription_init_default(
    &drive_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "drive_input"
  );
  rclc_subscription_init_default(
    &steer_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "steer_input"
  );
  rclc_subscription_init_default(
    &button_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "buttons_input"
  );

  drive_msg.data = 1500;  // default stop

  // Init executor (1 pub + 3 subs)
  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &drive_subscriber, &drive_input_msg, &drive_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &steer_subscriber, &steer_input_msg, &steer_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &button_subscriber, &button_input_msg, &button_callback, ON_NEW_DATA);

  // Bluepad32 setup
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n",
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  // ESC PWM - Throttle (50Hz)
  ledcAttachPin(ESC_PIN, 0);
  ledcSetup(0, 50, 16);

  // Servo PWM - Steering (50Hz)
  ledcAttachPin(SERVO_PIN, 1);
  ledcSetup(1, 50, 16);

  mode = 0;
  pre_mode = -1;
  drive = ESC_stp_N;
}

void loop() {
  bool updated = BP32.update();
  handleModeSelection();
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  rosDriveActive = (millis() - lastDriveInputTime) <= driveInputTimeout;
  rosSteerActive = (millis() - lastSteerInputTime) <= steerInputTimeout;

  uint8_t buttons = 0;
  if (myController->a())  buttons |= 1;    // 00000001
  if (myController->b())  buttons |= 2;    // 00000010
  if (myController->y())  buttons |= 4;    // 00000100
  if (myController->x())  buttons |= 8;    // 00001000
  if (myController->l1()) buttons |= 16;   // 00010000
  if (myController->r1()) buttons |= 32;   // 00100000


  if (updated && myController && myController->isConnected() && myController->hasData()) {
    if (!rosDriveActive) {
      // Read inputs
      int throttle = myController->throttle();         // RT trigger
      int brake = myController->brake();         // LT trigger     

      // Map throttle to 1000-2000us PWM
      if(brake < 1 && throttle > 5){
        if(myController->x()){
          drive = 1000;
          mode = 0;
        }
        else{
          drive = map(throttle, 0, 1023, ESC_stp_N, escMax);
        }
      }
      else if(brake > 5 && throttle < 1){
        if(myController->x()){
          drive = 2000;
          mode = 0;
        }
        else{
          drive = map(brake, 0, 1023, ESC_stp_N, escMin);
        }
      }
    
      else{
        drive = 1500;
      }
    }

    int dutyThrottle = (drive * 65535L) / 20000L;
    ledcWrite(0, dutyThrottle);

    // Map steering to servo range (1000-2000us)
    if (!rosSteerActive) {
      int steerX = myController->axisX();  // -511 to 512
      pwmSteerUs = map(steerX, -511, 512, 1000, 2000);
    }            // Left joystick X (-511 to 512)
    int dutySteer = (pwmSteerUs * 65535L) / 20000L;
    ledcWrite(1, dutySteer);

    drive_msg.data = drive;
    rcl_publish(&drive_publisher, &drive_msg, NULL);
    steer_msg.data = pwmSteerUs;
    rcl_publish(&steer_publisher, &steer_msg, NULL);
    button_msg.data = buttons;
    rcl_publish(&button_publisher, &button_msg, NULL);
    mode_msg.data = mode;
    rcl_publish(&mode_publisher, &mode_msg, NULL);

    // Print debug info
    Serial.printf("Mode: %4d (PWM: %d us), Steer X: %4d (PWM: %d us)\n",
                  mode, drive, SteerX, pwmSteerUs);
  }
}
