#include <Arduino.h>
#include <WiFi.h>

// Buffer for reading the data packet
unsigned char packet[2];

// Motor pins and Channels definition
int IN1 = 2;
int IN2 = 14;
int IN3 = 15;
int IN4 = 13;
const int channel1 = 1;
const int channel2 = 2;
const int channel3 = 3;
const int channel4 = 4;
const int frequency = 1000;
const int resolution = 8;


// Wifi connections
const char* ssid     = "SSS";
const char* password = "12345678";
const uint16_t port = 8585;
const char *host = "192.168.137.1";
WiFiClient client;


// order the motor to move
void control_motor(int speed, int steer, bool left)
{
  if ((speed > steer) && (left == 0)) {
    Serial.println("Move forward or right");
    ledcWrite(channel1, speed + steer);
    ledcWrite(channel4, speed - steer);
  }
  else if ((speed <= steer) && (left == 0)) {
    Serial.println("Move forward or right");
    ledcWrite(channel1, speed + steer);
    ledcWrite(channel4, 0);
  }
  else if ((speed > steer) && (left == 1)) {
    Serial.println("Move left");
    ledcWrite(channel1, speed - steer);
    ledcWrite(channel4, speed + steer);
  }
  else if ((speed <= steer) && (left == 1)) {
    Serial.println("Move left");
    ledcWrite(channel1, 0);
    ledcWrite(channel4, speed + steer);
  }
  else {
    Serial.println("Move forward of right with high speed");
    ledcWrite(channel1, 0);
    ledcWrite(channel4, 0);
  }
}

// starting the program
void setup(void)
{
  Serial.begin(115200);

  // setting up the analog channels
  ledcSetup(channel1, frequency, resolution);
  ledcAttachPin(IN1, channel1) ;
  ledcSetup(channel2, frequency, resolution);
  ledcAttachPin(IN2, channel2) ;
  ledcSetup(channel3, frequency, resolution);
  ledcAttachPin(IN3, channel3) ;
  ledcSetup(channel4, frequency, resolution);
  ledcAttachPin(IN4, channel4);
  
  // WiFi congifurations
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname("car2");
  WiFi.begin(ssid, password);

  // connecting to the network
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


// main program
void loop()
{
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  if (!client.connected()) {
    // connecting to the server
    while (!client.connect(host, port))
    {
      client.connect(host, port);
      Serial.println("Connection to server failed");
      delay(1000);
      return;
    }
    Serial.println("Connection to server succeed");
  }

  // if there is a data
  while (client.available() > 0)
  {
    // read and store it in tha buffer
    client.read(packet, 2);
    unsigned char speed = packet[0];
    signed char steer = packet[1];
    // extract the needed information from the buffer
    bool left = steer < 0;
    int speed_abs = abs(speed);
    int steer_abs = abs(steer);
    Serial.printf("(speed, steer, left) : (%d, %d, %d)\n", speed_abs, steer_abs, left);

    // deliver the order to the robot
    control_motor(speed_abs, steer_abs, left);
  }
}
