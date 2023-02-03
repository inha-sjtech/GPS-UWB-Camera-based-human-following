#define ROSSERIAL_ARDUINO_TCP

#include <WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>

const char* ssid = "Conference Room";
const char* password = "1398134490";

IPAddress server(192,168,0,201);

const uint16_t serverPort = 11411;

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char message[12] = "hellow word";

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.print("connect try : ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP: ");
  Serial.println(WiFi.localIP());

  nh.getHardware() -> setConnection(server, serverPort);
  nh.initNode();

  Serial.print("IP = ");
  Serial.println(nh.getHardware() -> getLocalIP());

  nh.advertise(chatter);

}

void loop() {
  if (nh.connected()){
    Serial.println("connected");
    str_msg.data = message;
    chatter.publish(&str_msg);
    Serial.print("ok");
  }else{
    Serial.println("not connected");
  }
  nh.spinOnce();
  delay(100);
}
