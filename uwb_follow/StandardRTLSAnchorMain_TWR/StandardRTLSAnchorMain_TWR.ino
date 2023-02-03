#define ROSSERIAL_ARDUINO_TCP

#include <WiFi.h>
#include <ros.h>
#include <std_msgs/Float32.h>

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>

const char* ssid = "김여호수아의 iPhone";
const char* password = "11111111";
IPAddress server(172,20,10,2);
const uint16_t serverPort = 11411;
ros::NodeHandle nh;
std_msgs::Float32 uwb_main_range_msg;
std_msgs::Float32 uwb_main_rx_power_msg;
ros::Publisher uwb_main_range("uwb_main_range", &uwb_main_range_msg);
ros::Publisher uwb_main_rx_power("uwb_main_rx_power", &uwb_main_rx_power_msg);

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
Position position_B = {0.3,-0.5};
Position position_C = {-0.3,-0.5};

double range_self;
double range_B;
double range_C;

boolean received_B = false;

byte target_eui[8];
byte tag_shortAddress[] = {0x05, 0x00};

byte anchor_b[] = {0x02, 0x00};
uint16_t next_anchor = 2;
byte anchor_c[] = {0x03, 0x00};

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

    nh.advertise(uwb_main_range);
    nh.advertise(uwb_main_rx_power);

    Serial.println(F("### DW1000Ng-arduino-ranging-anchorMain ###"));
    // initialize the driver

    DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);

    Serial.println(F("DW1000Ng initialized ..."));
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

/* using https://math.stackexchange.com/questions/884807/find-x-location-using-3-known-x-y-location-using-trilateration */
// void calculatePosition(double &x, double &y) {
  
//     /* This gives for granted that the z plane is the same for anchor and tags */
//     double A = ( (-2*position_self.x) + (2*position_B.x) );
//     double B = ( (-2*position_self.y) + (2*position_B.y) );
//     double C = (range_self*range_self) - (range_B*range_B) - (position_self.x*position_self.x) + (position_B.x*position_B.x) - (position_self.y*position_self.y) + (position_B.y*position_B.y);
//     double D = ( (-2*position_B.x) + (2*position_C.x) );
//     double E = ( (-2*position_B.y) + (2*position_C.y) );
//     double F = (range_B*range_B) - (range_C*range_C) - (position_B.x*position_B.x) + (position_C.x*position_C.x) - (position_B.y*position_B.y) + (position_C.y*position_C.y);

//     x = (C*E-F*B) / (E*A-B*D);
//     y = (C*D-A*F) / (B*D-A*E);
    
//     String temp = "main : "; temp += range_self; temp += " B : "; temp += range_B; temp += " C : "; temp += range_C;
//     temp += " x : "; temp += x; temp += " y : "; temp += y;
//     Serial.println(temp);
// }

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
            if (result.range != 0){
              range_self = result.range;              
            }
            String rangeString = "Range: "; rangeString += range_self; rangeString += " m";
            rangeString += "\t RX power: "; rangeString += DW1000Ng::getReceivePower(); rangeString += " dBm";
            Serial.println(rangeString);
            if (nh.connected()){
                // Serial.println("connected");
                uwb_main_range_msg.data = range_self;
                uwb_main_rx_power_msg.data = DW1000Ng::getReceivePower();
                uwb_main_range.publish(&uwb_main_range_msg);
                uwb_main_rx_power.publish(&uwb_main_rx_power_msg);
                // Serial.print("ok");
            }else{
                // Serial.println("not connected");
            }
            nh.spinOnce();
        } 
        // else if(recv_data[9] == 0x60) {
        //     double range = static_cast<double>(DW1000NgUtils::bytesAsValue(&recv_data[10],2) / 1000.0);
        //     String rangeReportString = "Range from: "; rangeReportString += recv_data[7];
        //     rangeReportString += " = "; rangeReportString += range;
        //     // Serial.println(rangeReportString);
        //     if(received_B == false && recv_data[7] == anchor_b[0] && recv_data[8] == anchor_b[1]) {
        //         if (range != 0){
        //           range_B = range;
        //         }
        //         received_B = true;
        //     } else if(received_B == true && recv_data[7] == anchor_c[0] && recv_data[8] == anchor_c[1]){
        //         if(range != 0){
        //           range_C = range;
        //         }
        //         double x,y;
        //         calculatePosition(x,y);
        //         String positioning = "Found position - x: ";
        //         positioning += x; positioning +=" y: ";
        //         positioning += y;
        //         // Serial.println(positioning);
        //         received_B = false;
        //     } else {
        //         received_B = false;
        //     }
        // }
    }
}