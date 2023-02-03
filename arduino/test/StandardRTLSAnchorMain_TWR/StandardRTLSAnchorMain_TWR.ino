#define ROSSERIAL_ARDUINO_TCP

#include <WiFi.h>
#include <ros.h>
#include <std_msgs/Float32.h>

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>

const char* ssid = "Conference Room";
const char* password = "1398134490";
IPAddress server(192,168,0,201);
const uint16_t serverPort = 11411;
ros::NodeHandle nh;
std_msgs::Float32 float32_msg1;
std_msgs::Float32 float32_msg2;
ros::Publisher floatter1("floatter1", &float32_msg1);
ros::Publisher floatter2("floatter2", &float32_msg2);

typedef struct Position {
  double x;
  double y;
} Position;

// connection pins
const uint8_t PIN_SCK = 18;
const uint8_t PIN_MOSI = 23;
const uint8_t PIN_MISO = 19;
const uint8_t PIN_SS = 2;
const uint8_t PIN_RST = 15;
const uint8_t PIN_IRQ = 17;

// Extended Unique Identifier register. 64-bit device identifier. Register file: 0x01
char EUI[] = "AA:BB:CC:DD:EE:FF:00:01";

Position position_self = {0,0};
Position position_B = {3,0};
Position position_C = {3,2.5};

double range_self;
double range_B;
double range_C;

boolean received_B = false;

byte target_eui[8];
byte tag_shortAddress[] = {0x05, 0x00};

byte anchor_b[] = {0x02, 0x00};
uint16_t next_anchor = 2;
byte anchor_c[] = {0x03, 0x00};

String rangeString;
String rangeReportString;

device_configuration_t DEFAULT_CONFIG = {
  false,
  true,
  true,
  true,
  false,
  SFDMode::STANDARD_SFD,
  Channel::CHANNEL_5,
  DataRate::RATE_850KBPS,
  PulseFrequency::FREQ_16MHZ,
  PreambleLength::LEN_256,
  PreambleCode::CODE_3
};

frame_filtering_configuration_t ANCHOR_FRAME_FILTER_CONFIG = {
  false,
  false,
  true,
  false,
  false,
  false,
  false,
  true /* This allows blink frames */
};

void setup() {
  // DEBUG monitoring
  Serial.begin(115200);
  Serial.print("connect try : ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  nh.getHardware() -> setConnection(server, serverPort);
  nh.initNode();

  Serial.print("IP = ");
  Serial.println(nh.getHardware() -> getLocalIP());

  nh.advertise(floatter1);
  nh.advertise(floatter2);

  Serial.println(F("### DW1000Ng-arduino-ranging-anchorMain ###"));
  // initialize the driver
  DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
  
  // general configuration
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  DW1000Ng::enableFrameFiltering(ANCHOR_FRAME_FILTER_CONFIG);
  
  DW1000Ng::setEUI(EUI);

  DW1000Ng::setPreambleDetectionTimeout(64);
  DW1000Ng::setSfdDetectionTimeout(273);
  DW1000Ng::setReceiveFrameWaitTimeoutPeriod(5000);

  DW1000Ng::setNetworkId(RTLS_APP_ID);
  DW1000Ng::setDeviceAddress(1);

  DW1000Ng::setAntennaDelay(16436);
  
  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000Ng::getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000Ng::getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);    
}

void loop() {
  if(DW1000NgRTLS::receiveFrame()){
    size_t recv_len = DW1000Ng::getReceivedDataLength();
    byte recv_data[recv_len];
    DW1000Ng::getReceivedData(recv_data, recv_len);

    if(recv_data[0] == BLINK) {
      DW1000NgRTLS::transmitRangingInitiation(&recv_data[2], tag_shortAddress);
      DW1000NgRTLS::waitForTransmission();
      RangeAcceptResult result = DW1000NgRTLS::anchorRangeAccept(NextActivity::RANGING_CONFIRM, next_anchor);
      if(!result.success) return;
      range_self = result.range;
      rangeString = "Range1: "; rangeString += range_self; rangeString += " m";

    } else if(recv_data[9] == 0x60) {
      double range = static_cast<double>(DW1000NgUtils::bytesAsValue(&recv_data[10],2) / 1000.0);
      rangeReportString = "Range2: "; 
      rangeReportString += range;
      if(recv_data[7] == anchor_b[0] && recv_data[8] == anchor_b[1]) {
        Serial.println(rangeString);
        Serial.println(rangeReportString);
        // Serial.println(rangeString + rangeReportString);
        if (nh.connected()){
          // Serial.println("connected");
          float32_msg1.data = range_self;
          float32_msg2.data = range;
          floatter1.publish(&float32_msg1);
          floatter2.publish(&float32_msg2);
          // Serial.print("ok");
        }else{
          // Serial.println("not connected");
        }
        nh.spinOnce();
      }
    }
  }
}