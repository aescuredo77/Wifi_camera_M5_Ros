#include "esp_camera.h"
#include <WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
//#include <std_msgs/Int16.h>
//#include <std_msgs/Float64.h>
#include <std_msgs/Int16MultiArray.h>
#include "camera_pins.h"
#define camera armpap_camera00
#define port 11510
#define pub_canal "/armpap00/camera00/pub"
#define sub_canal "/armpap00/camera00/cmd"


const char* ssid = "Ros_Robotic";
const char* password = "45276699";
IPAddress server(192, 168, 0, 151);   // Set the rosserial socket server 
const uint16_t serverPort = port;    // Set the rosserial socket server port 
char cmd_msg[8]; 
void startCameraServer();
camera_fb_t * fb = NULL;

void cmdCallback(const std_msgs::Int16MultiArray& msg) {
  for (int i = 0; i < 8; i++) {
    Serial.print(msg.data[i]);Serial.print(",");
    cmd_msg[i] = char(msg.data[i]);
    }
}
std_msgs::String str_msg;
std_msgs::Int16MultiArray cmd;
std_msgs::MultiArrayDimension myDim;
std_msgs::MultiArrayLayout mylayout;       
ros::Publisher pub(pub_canal, &str_msg);                                       ///////////////////////////////////////////////
ros::Subscriber<std_msgs::Int16MultiArray> sub_cmd(sub_canal, &cmdCallback);   /////////////////////////////////////////////// 
ros::NodeHandle camera;

void setup() {
  Serial.begin(57600);
  Serial.setDebugOutput(false);
  Serial.println();
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;//UXGA;
  config.jpeg_quality = 20;
  config.fb_count = 2;
     
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  s->set_vflip(s, 1);//flip it back
  s->set_brightness(s, 1);//up the blightness just a bit
  s->set_saturation(s, -2);//lower the saturation

  //drop down frame size for higher initial frame rate
  //s->set_framesize(s, FRAMESIZE_QVGA);
  Serial.printf("Connect to %s\r\n", ssid, password);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  camera.getHardware()->setConnection(server, serverPort);                      /////////////////////////////////////////////////
  camera.initNode();
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
                                                              /////////////////////////////////////////////////
  camera.advertise(pub);                                                        /////////////////////////////////////////////////
  camera.subscribe(sub_cmd);
}

void loop() {
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    //ESP.restart();
  }
  esp_camera_fb_return(fb);
  camera.spinOnce();
  delay(10);
}
