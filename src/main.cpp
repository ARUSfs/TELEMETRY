//DEFINE CAN BUS
#include <mcp_can.h>
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

MCP_CAN CAN0(21);  // Set CS to pin 21


#define CAN_ECU_1            0x01
#define CAN_ECU_2            0x02
#define CAN_ECU_3            0x03
#define CAN_EMBRAGUE         0x101

//ID ECU 1
#define CAN_RPM                    0     
#define CAN_RBP                    2
#define CAN_FBP                    3
#define CAN_OIL_PRESSURE           4
#define CAN_LAMBDA                 6

//ID ECU 2
#define CAN_TP                     0
#define CAN_STEERING_WHEEL         1
#define CAN_WSFL                   2
#define CAN_WSFR                   3
#define CAN_WSRL                   4
#define CAN_WSRR                   5
#define CAN_SLIP                   6
#define CAN_BOTTON                 7

//ID ECU 3
#define CAN_BATTERY_VOLTAGE        0
#define CAN_ECT                    1
#define CAN_TEMP_RADIATOR          2
#define CAN_TEMP_COCKPIT           3
#define CAN_IAT                    4
#define CAN_TEMP_OIL               5
#define CAN_LOG_ECU                6
#define CAN_GEAR                   7

/////////////////////////////////////////////////////

//DEFINE JSON PARAMETERS
#define ARDUINOJSON_USE_DOUBLE 0
#define ARDUINOJSON_USE_LONG_LONG 0
#include <ArduinoJson.h>

StaticJsonDocument<384> JSON_OUT;
// String input;

StaticJsonDocument<32> JSON_IN;

////////////////////////////

int rpm = 0;
int rbp = 0;
int fbp = 0;
int oil_pressure=0;
int lambda = 0;

////////////////////////////

int tp=0;
int steering_wheel = 0;
int WSFL = 0;
int WSFR = 0;
int WSRL = 0;
int WSRR = 0;
int slip=0;
int botton = 0;

////////////////////////////

int battery_voltage = 0;
int ect = 0;
int temp_radiator=0;
int temp_cockpit=0;
int iat=0;
int temp_oil=0;
int log_ecu=0;
int gear=0;

char envio[384];

////////////////////////////

int rpmSalida=8000;
bool salidaFlag=0;
bool saliendoFlag=0;
////////////////////////////

// Select your modem:
#define TINY_GSM_MODEM_SIM7600 // 


// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// In this case we use ESP32 so we have to use Serial1
#define SerialAT Serial1

/* We use ESP32 in this case
// or Software Serial on Uno, Nano
#else
#include <SoftwareSerial.h>
SoftwareSerial SerialAT(2, 3);  // RX, TX
#endif
*/

// See all AT commands, if wanted
 //#define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

//Use this define to choose between GPRS or WIFI:
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[] = "airtelwap.es"; // APN use https://wiki.apnchanger.org
const char gprsUser[] = "wap@wap";
const char gprsPass[] = "wap125";


// MQTT details
const char* broker = "35.246.136.225";
const char* mqttUsername = "";  // MQTT username
const char* mqttPassword = "";  // MQTT password
const char *root_topic_subscribe = "input";
const char *root_topic_publish = "output";


#include <TinyGsmClient.h>
#include <PubSubClient.h>

//This mode is just to debugg with the modem
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif

TinyGsmClient client(modem);
PubSubClient  mqtt(client);

#define MODEM_TX             14
#define MODEM_RX             12

char msg[25];

long count=0;// MQTT details

uint32_t lastReconnectAttempt = 0;

const int ledPin = 13;
const int ledPin1 = 5;

void mqttCallback(char* topic, byte* payload, unsigned int len) 
{

  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();

  // Only proceed if incoming message's topic matches, is just an example
  if (String(topic) == root_topic_subscribe) 
  {

    DeserializationError error = deserializeJson(JSON_IN, payload,32);

    if (error) 
    {
     Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
    }

  rpmSalida = JSON_IN["RPM"]; 

  SerialMon.println(rpmSalida);
  }
}

boolean mqttConnect() 
{
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker
  //boolean status = mqtt.connect("GsmClientTest");

  // Or, if you want to authenticate MQTT:
  boolean status = mqtt.connect("GsmClientName", "mqtt_user", "mqtt_pass");

  if (status == false)
  {
    SerialMon.println(" fail");
    ESP.restart();
    return false;
  }

  SerialMon.println(" success");
  mqtt.subscribe(root_topic_subscribe);

  return mqtt.connected();

}

#include <Arduino.h>


void setup() 
{
  // Set console baud rate
  SerialMon.begin(115200);
  delay(1000);

pinMode (ledPin, OUTPUT);
pinMode (ledPin1, OUTPUT);

  //SETUP CAN BUS////////////////////////////
  if(CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) == CAN_OK) Serial.print("MCP2515 Init Okay!!\r\n");
  else Serial.print("MCP2515 Init Failed!!\r\n");
  pinMode(17, INPUT);                       // Setting pin 22 for INT input


  CAN0.init_Mask(0,0,0x07FF0000);                // Init first mask...
  CAN0.init_Filt(0,0,0x00000000);                // Init first filter...
  CAN0.init_Filt(1,0,0x04010000);                // Init second filter...
  
  CAN0.init_Mask(1,0,0x07FF0000);                // Init second mask... 
  CAN0.init_Filt(2,0,0x00010000);                // Init third filter...
  CAN0.init_Filt(3,0,0x00020000);                // Init fourth filter...
  CAN0.init_Filt(4,0,0x00030000);                // Init fifth filter...
  CAN0.init_Filt(5,0,0x01010000);                // Init sixth filter...
  
  Serial.println("MCP2515 Library Mask & Filter Example...");
  
  CAN0.setMode(MCP_NORMAL);
  //////////////////////////////////////////

  

  // !!!!!!!!!!!
  // Set your reset, enable, power pins here
  // !!!!!!!!!!!

  SerialMon.println("Wait...");

  // Set GSM module baud rate, IMPORTANT in the case of usign ESP32 we have to use this function with these specific parameters
  SerialAT.begin(3686400, SERIAL_8N1, MODEM_TX, MODEM_RX);

  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  delay(1000);

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  uint8_t LTE = 38; // Lo indica el la guia de usuario del SerialAT

  //AÑADIDO POR MI
  modem.setNetworkMode(LTE);

  delay(6000);

  SerialMon.print("Network Mode: ");

  SerialMon.println(modem.getNetworkMode());

//////////////////////////////////////////////////

  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3) { modem.simUnlock(GSM_PIN); }

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) 
  {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) { SerialMon.println("Network connected"); }

  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) 
  {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  else
  {

  SerialMon.println(" success");

  }

  if (modem.isGprsConnected()) { SerialMon.println("GPRS connected"); }

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);


}

void loop() 
{

  if (!mqtt.connected()) 
  {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) 
    {
      lastReconnectAttempt = t;
      if (mqttConnect()) { lastReconnectAttempt = 0; }
    }
    delay(100);
    return;
  }
   
  

 if(!digitalRead(17))                    // If pin 17 is low, read receive buffer

 {
      
    digitalWrite (ledPin, HIGH);	// turn on the LED

    CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)
    


    if((unsigned long)rxId==(unsigned long)CAN_ECU_1)
    
    {

      rpm=((rxBuf[CAN_RPM] << 8 | rxBuf[CAN_RPM+1]));
      rbp=((rxBuf[CAN_RBP]));
      fbp=(rxBuf[CAN_FBP]);
      oil_pressure=((rxBuf[CAN_OIL_PRESSURE] << 8 | rxBuf[CAN_OIL_PRESSURE+1]));
      lambda=((rxBuf[CAN_LAMBDA] << 8 | rxBuf[CAN_LAMBDA+1]));

      if(salidaFlag)
      {
        if(rpm>rpmSalida)
        {
          unsigned char txBuf[8]={1,1,1,1,1,1,1,1};

          CAN0.sendMsgBuf(0x401,0,8,txBuf);
      
          salidaFlag=0;


          digitalWrite (ledPin1, LOW);	// turn off the LED

        }

      }

    }

    if((unsigned long)rxId==(unsigned long)CAN_ECU_2)
    {

      tp=((rxBuf[CAN_TP]));
      steering_wheel=((rxBuf[CAN_STEERING_WHEEL]));
      WSFL=(rxBuf[CAN_WSFL]);
      WSFR=(rxBuf[CAN_WSFR]);
      WSRL=(rxBuf[CAN_WSRL]);
      WSRR=(rxBuf[CAN_WSRR]);
      slip=((rxBuf[CAN_SLIP]));
      botton=(rxBuf[CAN_BOTTON]);

    }

    if((unsigned long)rxId==(unsigned long)CAN_ECU_3)
    {
        battery_voltage=(rxBuf[CAN_BATTERY_VOLTAGE]);
        ect=rxBuf[CAN_ECT];
        temp_radiator=rxBuf[CAN_TEMP_RADIATOR];
        temp_cockpit=rxBuf[CAN_TEMP_COCKPIT];
        iat=rxBuf[CAN_IAT];
        temp_oil=rxBuf[CAN_TEMP_OIL];
        log_ecu=rxBuf[CAN_LOG_ECU];
        gear=rxBuf[CAN_GEAR];
    }
    
    if((unsigned long)rxId==(unsigned long)CAN_EMBRAGUE)
    {
      if(((rxBuf[5]&&(0x08))&&(rxBuf[5]&&(0x10))) && saliendoFlag==0)
      {    
        
        salidaFlag=1;

        saliendoFlag=1;

         digitalWrite (ledPin1, HIGH);	// turn off the LED

      }

      if((((rxBuf[5]&&(0x08))&&(rxBuf[5]&&(0x10)))==0) && saliendoFlag==1)
      {    

        saliendoFlag=0;

      }
    }




              JSON_OUT["RPM"] = rpm;
              JSON_OUT["Marcha"] = gear;
            
        
              JSON_OUT["WSRL"] = WSRL;
              JSON_OUT["WSRR"] = WSRR;
              JSON_OUT["Bomba"] = 0;
              JSON_OUT["Ventilador"] = 0;
              JSON_OUT["Arranque"] = 0;

              JSON_OUT["TP"] = tp;
              JSON_OUT["Temp_Motor"] = ect;
              JSON_OUT["Temp_Aceite"] = temp_oil;
              JSON_OUT["Temp_Agua"] = temp_radiator;
              JSON_OUT["Presion_Aceite"] = oil_pressure;
              JSON_OUT["Bateria"] = battery_voltage;
              JSON_OUT["Log"] = log_ecu;

              JSON_OUT["WSFL"] = WSFL;
              JSON_OUT["WSFR"] = WSFR;

              JSON_OUT["Giro_Volante"] = steering_wheel;


    serializeJson(JSON_OUT, envio);
 
    mqtt.publish(root_topic_publish,envio);

  mqtt.loop();
  

}

else
{
    digitalWrite (ledPin, LOW);	// turn off the LED
    mqtt.loop();
}



}
