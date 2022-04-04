//DEFINE CAN BUS
#include <mcp_can.h>
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

MCP_CAN CAN0(5);  // Set CS to pin 5


#define CAN_ECU_1            0x140
#define CAN_ECU_2            0x150
#define CAN_ECU_3            0x170
#define CAN_ECU_4            0x180

//ID ECU 1
#define CAN_RPM                    0     
#define CAN_TP                     1
#define CAN_FBP                    2
#define CAN_RBP                    3
#define CAN_STEERING_WHEEL         4
#define CAN_GEAR                   5
#define CAN_LAMBDA                 6


//ID ECU 2
#define CAN_ECT                    0
#define CAN_TEMP_RADIADOR          1
#define CAN_TEMP_COCKPIT           2
#define CAN_TEMP_IAT               3
#define CAN_TEMP_ACEITE            4
#define CAN_TEMP_ECT               5
#define CAN_PRESION_ACEITE         6


//ID ECU 3

/////////////////////////////////////////////////////

//DEFINE JSON PARAMETERS
#define ARDUINOJSON_USE_DOUBLE 0
#define ARDUINOJSON_USE_LONG_LONG 0
#include <ArduinoJson.h>

StaticJsonDocument<384> JSON_OUT;

int rpm = 0;
int tp = 0;
int fbp = 0;
int presion_aceite=0;
int lambda = 0;

int bateria=0;
int temp_motor = 0;
int temp_radiador = 0;
int temp_cockpit = 0;
int iat = 0;
int temp_aceite = 0;
int log_use=0;
int marcha = 0;
int steering_wheel = 0;
int temp_ect = 0;
int wsf=0;
int wsb=0;
char envio[384];

//////////////////////////////

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
const char* broker = "35.242.245.93";
const char* mqttUsername = "";  // MQTT username
const char* mqttPassword = "";  // MQTT password
const char *root_topic_subscribe = "input";
const char *root_topic_publish = "datos";


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

#define MODEM_TX             27
#define MODEM_RX             26

char msg[25];

long count=0;// MQTT details

uint32_t lastReconnectAttempt = 0;

void mqttCallback(char* topic, byte* payload, unsigned int len) 
{

  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();

  // Only proceed if incoming message's topic matches, is just an example
  if (String(topic) == topic) 
  {
    
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


void setup() 
{
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

  //SETUP CAN BUS////////////////////////////
  if(CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.print("MCP2515 Init Okay!!\r\n");
  else Serial.print("MCP2515 Init Failed!!\r\n");
  pinMode(22, INPUT);                       // Setting pin 22 for INT input


  CAN0.init_Mask(0,0,0x07FF0000);                // Init first mask...
  CAN0.init_Filt(0,0,0x02400000);                // Init first filter...
  CAN0.init_Filt(1,0,0x02500000);                // Init second filter...
  
  CAN0.init_Mask(1,0,0x07FF0000);                // Init second mask... 
  CAN0.init_Filt(2,0,0x01400000);                // Init third filter...
  CAN0.init_Filt(3,0,0x01500000);                // Init fourth filter...
  CAN0.init_Filt(4,0,0x01600000);                // Init fifth filter...
  CAN0.init_Filt(5,0,0x01700000);                // Init sixth filter...
  
  Serial.println("MCP2515 Library Mask & Filter Example...");
  
  CAN0.setMode(MCP_NORMAL);
  //////////////////////////////////////////

  

  // !!!!!!!!!!!
  // Set your reset, enable, power pins here
  // !!!!!!!!!!!

  SerialMon.println("Wait...");

  // Set GSM module baud rate, IMPORTANT in the case of usign ESP32 we have to use this function with these specific parameters
  SerialAT.begin(3686400, SERIAL_8N1, 27, 26);

  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  uint8_t LTE = 38;

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
   
  

 if(!digitalRead(22))                    // If pin 22 is low, read receive buffer
 {
    

    CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)
    


    if((unsigned long)rxId==(unsigned long)CAN_ECU_1)
    {

      rpm=(rxBuf[CAN_RPM]*100);
      tp=(rxBuf[CAN_TP])-50;
      fbp=(rxBuf[CAN_FBP]*100);
      fbp=(rxBuf[CAN_RBP]*100);
      steering_wheel=(rxBuf[CAN_STEERING_WHEEL]);
      marcha=(rxBuf[CAN_GEAR]);
      lambda=(rxBuf[CAN_LAMBDA+1] << 8 | rxBuf[CAN_LAMBDA]);

    }

    if((unsigned long)rxId==(unsigned long)CAN_ECU_2)
    {

      temp_motor=(rxBuf[CAN_ECT]);
      temp_radiador=(rxBuf[CAN_TEMP_RADIADOR]);
      temp_cockpit=(rxBuf[CAN_TEMP_COCKPIT]);
      iat=(rxBuf[CAN_TEMP_IAT]);
      temp_aceite=(rxBuf[CAN_TEMP_ACEITE]);
      temp_ect=(rxBuf[CAN_TEMP_ECT]);
      presion_aceite=(rxBuf[CAN_PRESION_ACEITE]<<8 | rxBuf[CAN_PRESION_ACEITE+1]);

    }

    if((unsigned long)rxId==(unsigned long)CAN_ECU_3)
    {
        wsf=rxBuf[0]*10;
        wsb=rxBuf[1]*10;
    }
    
    if((unsigned long)rxId==(unsigned long)CAN_ECU_4)
    {

        bateria=rxBuf[0]*10;
        int log_use=(rxBuf[7]*100)-50;

    }


              JSON_OUT["RPM"] = rpm;
              JSON_OUT["Marcha"] = marcha;
            
        
              JSON_OUT["WSRL"] = 0;
              JSON_OUT["WSRR"] = 0;
              JSON_OUT["Bomba"] = 0;
              JSON_OUT["Ventilador"] = 0;
              JSON_OUT["Arranque"] = 0;

              JSON_OUT["TP"] = tp;
              JSON_OUT["Temp_Motor"] = temp_motor-50;
              JSON_OUT["Temp_Aceite"] = temp_aceite-50;
              JSON_OUT["Temp_Agua"] = temp_radiador-50;
              JSON_OUT["Presion_Aceite"] = presion_aceite;
              JSON_OUT["Bateria"] = bateria;
              JSON_OUT["Log"] = log_use;

              JSON_OUT["WSFL"] = wsb;
              JSON_OUT["WSFR"] = wsf;

              JSON_OUT["Giro_Volante"] = steering_wheel;


    serializeJson(JSON_OUT, envio);
 
    mqtt.publish(root_topic_publish,envio);

/*

      Serial.print("ID: ");
      Serial.print(rxId, HEX);
      Serial.print(" Data: ");
      for(int i = 0; i<len; i++)           // Print each byte of the data
      {
        if(rxBuf[i] < 0x10)                // If data byte is less than 0x10, add a leading zero
        {
          Serial.print("0");
        }
        Serial.print(rxBuf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      
    */

  mqtt.loop();
  

}

}
