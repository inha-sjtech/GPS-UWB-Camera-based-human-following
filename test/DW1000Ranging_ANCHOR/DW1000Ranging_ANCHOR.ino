// #define ROSSERIAL_ARDUINO_TCP

// #include <WiFi.h>
// #include <ros.h>
// #include <std_msgs/String.h>

#include <SPI.h>
#include "DW1000Ranging.h"

// const char* ssid = "Conference Room";
// const char* password = "1398134490";

// IPAddress server(192,168,0,201);

// const uint16_t serverPort = 11411;

// ros::NodeHandle nh;

// std_msgs::String str_msg;
// ros::Publisher chatter("chatter", &str_msg);

// char message[10] = "";

// connection pins
const uint8_t PIN_SCK = 18;
const uint8_t PIN_MOSI = 23;
const uint8_t PIN_MISO = 19;
const uint8_t PIN_SS = 2;
const uint8_t PIN_RST = 15;
const uint8_t PIN_IRQ = 17;

void setup() {
  Serial.begin(115200);
  //   Serial.println();
  // Serial.print("connect try : ");
  // Serial.println(ssid);

  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED){
  //   delay(500);
  //   Serial.print(".");
  // }

  // Serial.println("");
  // Serial.println("WiFi connected");
  // Serial.println("IP: ");
  // Serial.println(WiFi.localIP());

  // nh.getHardware() -> setConnection(server, serverPort);
  // nh.initNode();

  // Serial.print("IP = ");
  // Serial.println(nh.getHardware() -> getLocalIP());

  // nh.advertise(chatter);

  delay(1000);
  //init the configuration
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachBlinkDevice(newBlink);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  //DW1000Ranging.useRangeFilter(true);
  
  //we start the module as an anchor
  DW1000Ranging.startAsAnchor("82:17:5B:D5:A9:9A:E2:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY);
}

void loop() {
  DW1000Ranging.loop();
  // if (nh.connected()){
  //   Serial.println("connected");
  //   str_msg.data = message;
  //   chatter.publish(&str_msg);
  //   Serial.print("ok");
  // }else{
  //   Serial.println("not connected");
  // }
  // nh.spinOnce();
  // delay(50);
}

void newRange() {
  Serial.print("from: "); Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print("\t Range: "); Serial.print(DW1000Ranging.getDistantDevice()->getRange()); Serial.print(" m");
  // float val = DW1000Ranging.getDistantDevice()->getRange();
  // dtostrf(val, 10,0, message);
  Serial.print("\t RX power: "); Serial.print(DW1000Ranging.getDistantDevice()->getRXPower()); Serial.println(" dBm");
}

void newBlink(DW1000Device* device) {
  Serial.print("blink; 1 device added ! -> ");
  Serial.print(" short:");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device* device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}

