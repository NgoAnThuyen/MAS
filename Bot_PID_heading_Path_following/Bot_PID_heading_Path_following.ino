#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>
//#include <WiFiClient.h>
#include <ArduinoJson.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif
const int ledPin1 = 12;
const int ledPin2 = 13;
const int ledPin3 = 25;
const int ledPin4 = 26;
const float pi = 3.14159;
// cài đặt PWM
const int freq = 1000;  // tần số xung
const int ledChannel1 = 0; // kênh PWM
const int ledChannel2 = 1; // kênh PWM
const int ledChannel3 = 2; // kênh PWM
const int ledChannel4 = 3; // kênh PWM
const int resolution = 8; // độ phân giải 8bit

bool pwmReady, wifiReady;

const char* ssid = "OpenWrt";
const char* password = "20222022";
WiFiUDP Udp;
unsigned int localUdpPort = 4210;  //  port to listen on
char incomingPacket[255];          // buffer for incoming packets
unsigned long previousMillis = 0;
float left_cmd, right_cmd;
float x, y, pre_x, pre_y, theta, pre_theta, x_d, y_d, pre_x_d, pre_y_d;
float x_e, y_e;
float s, pre_s;
float pre_U, theta_d;
float filteredTheta;
float pre_error_theta, sum_error, sum_error_u;
int pathIndex = 0, subIndex = 15;
String data_path;
float error;
float de;

unsigned long previousTime;
float cnt_time;
StaticJsonDocument<1000> doc_path;

float fixAngle(float angle)
{
    while (angle > pi)
    {
        angle -= 2.0f * pi;
    }
    while (angle < -pi)
    {
        angle += 2.0f * pi;
    }
    return angle;
}

void setup() {
  Serial.begin(115200);
  pwmConfiguration();
int status = WL_IDLE_STATUS;
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.println("");
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to wifi");
  Udp.begin(localUdpPort);

  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  xTaskCreatePinnedToCore(TaskCommunication,"TaskCommunication", 4096, NULL, 2, NULL, 0);
  delay(500);
  xTaskCreatePinnedToCore(TaskControl,"TaskControl", 4096, NULL, 3, NULL, 1);
  delay(500);
}
void loop()
{

}


void TaskControl(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  for (;;)
  {
    unsigned long T1 = micros();
    int packetSize = Udp.parsePacket();
    if (packetSize) 
    {
    int len = Udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
    }
    StaticJsonDocument<200> doc_pos;
    DeserializationError abc = deserializeJson(doc_pos, incomingPacket);
    x = doc_pos["position"][0].as<float>();
    y = doc_pos["position"][1].as<float>();
    theta = doc_pos["position"][2].as<float>();
    }
    
    float x_dot =( x - pre_x ) / 0.1;
    float y_dot =( y - pre_y ) / 0.1;
    float v = sqrt(x_dot*x_dot + y_dot*y_dot);
    // Update path variable
    float R = 0.5;
    float U_dh = R;
    float gamma_d = atan2(cos(s),-sin(s));
 
    x_e = (x - x_d)*cos(gamma_d) + (y - y_d)*sin(gamma_d);
    y_e = -(x - x_d)*sin(gamma_d) + (y - y_d)*cos(gamma_d); 
    float k1 = 50;
    s = s +((k1*x_e*0.05 ));//+ v*cos(theta-gamma_d))/U_dh)*0.05;
    x_d = 0.5*cos(s);
    y_d = 0.5*sin(s);

    Serial.print("s ");
    Serial.print(s);
    Serial.print(" xe ");
    Serial.print(x_e);
    Serial.print(" ye ");
    Serial.println(y_e);
    pre_s = s;
    theta_d = gamma_d - atan(y_e/0.3);
    // theta_d = theta_d + 1.8*pi/180.0;
    // if (theta_d > 2*pi) theta_d = 0;
    // Serial.println(theta_d);
    // theta_d = 1.57;
    float tau_r = PID_heading(theta, theta_d, 0.05);
    int max_tau_r = 30;
    if (tau_r > max_tau_r) tau_r = max_tau_r;
    else if (tau_r < -max_tau_r) tau_r = -max_tau_r;
    float tau_u = 75;
    left_cmd = tau_u + tau_r;
    right_cmd = tau_u - tau_r;
    int max_ = 255;
    if (left_cmd > max_) left_cmd = max_;
    else if (left_cmd < -max_) left_cmd = -max_;
    if (right_cmd > max_) right_cmd = max_;
    else if (right_cmd < -max_) right_cmd = -max_;
    Left_Motor_PWM((int)left_cmd);
    Right_Motor_PWM((int)right_cmd);
    
    // Left_Motor_PWM(-100);
    // Right_Motor_PWM(100);
    unsigned long T2 = micros();
    unsigned long DT = T2 - T1;
    // Serial.println(DT);
    vTaskDelay(50);
  }
  // vTaskDelay(100);
}

void TaskCommunication(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  for (;;)
  {
    StaticJsonDocument<256> docSendToPC;
    JsonObject root = docSendToPC.to<JsonObject>();
    root["desiredTheta"] = theta_d;  
    root["ye"] = y_e;  
    // root["sumerror"] = sum_error;  
    JsonArray position = root.createNestedArray("position");
    position.add(x);
    position.add(y);
    position.add(theta);
  
    String payload;
    serializeJson(docSendToPC, payload);
    //Serial.println(payload);
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    uint8_t buffer[255];
    // payload.toCharArray(buffer, payload.length());
    Udp.printf(payload.c_str(),255);
    Udp.endPacket();
    docSendToPC.clear();
    vTaskDelay(100);
  }
}

float PID_heading(float theta, float theta_d, float dt)
{
  // float error_ = theta - theta_d;
  error = atan2(sin((theta - theta_d)), cos((theta - theta_d)));
  // float directionalErr = cos(error_);
  // if (directionalErr < 0.0f)
  // {
  //   error = fixAngle(error_ - pi);
  // }
  // else
  // {
  //   error = fixAngle(error_);
  // }
  float output = 0;
  if (abs(error) > 0.00)
  {
    sum_error += error;
    de =  atan2(sin((error - pre_error_theta)), cos((error - pre_error_theta)));
    output = 35.0 * error + 5.0 * sum_error * dt + 5.0 * de / dt;
    pre_error_theta = error;
  }
  return output;
}


void pwmConfiguration()
{
  ledcSetup(ledChannel1, freq, resolution);
  ledcSetup(ledChannel2, freq, resolution);
  ledcSetup(ledChannel3, freq, resolution);
  ledcSetup(ledChannel4, freq, resolution);
  // xuất pwm ra chân 16
  ledcAttachPin(ledPin1, ledChannel1);
  ledcAttachPin(ledPin2, ledChannel2);
  ledcAttachPin(ledPin3, ledChannel3);
  ledcAttachPin(ledPin4, ledChannel4);
}
void Right_Motor_PWM(float PWM_)
{
  if (PWM_ >= 0)
  {
    ledcWrite(ledChannel1, PWM_);
    ledcWrite(ledChannel2, 0);
  }
  else
  {
    ledcWrite(ledChannel1, 0);
    ledcWrite(ledChannel2, abs(PWM_));
  }
}


void Left_Motor_PWM(float PWM_)
{
  if (PWM_ >= 0)
  {
    ledcWrite(ledChannel3, PWM_);
    ledcWrite(ledChannel4, 0);
  }
  else
  {
    ledcWrite(ledChannel3, 0);
    ledcWrite(ledChannel4, abs(PWM_));
  }
}
