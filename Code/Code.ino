/*

   This is the code for 4G Multipurpose GPS tracker project by techiesms. 

   Features of this project:-
    1. Realtime Location tracking via MQTT protocol using Mobile App ( 8sec interval )
    2. Realtime Battery Status indicator ( 60sec Interval )
    3. SOS button to make Emergency call and send current location 
    4. AutoCall Receive 

The code is tested with Arduino IDE version 2.3.2 & ESP8266 Boards package version 3.1.2

*/

#define DEBUG true


#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>  //https://github.com/vshymanskyy/TinyGSM ( Tested on Version 0.12.0)
#include <PubSubClient.h>   //https://pubsubclient.knolleary.net (Tested on Version 2.8)
#include <SimpleTimer.h>    //https://github.com/kiryanenko/SimpleTimer (Tested on Version 1.0.0)
#include <Wire.h>
#include <SoftwareSerial.h>


// ############################################################### Pinouts

#define SOS_Button 0
#define rxPin 14
#define txPin 12


// ############################################################### GPRS credentials
const char apn[] = "www";
const char gprsUser[] = "";
const char gprsPass[] = "";

#define GSM_PIN ""  // Provide GSM PIN if any

// ############################################################### MQTT details

const char* broker = "io.adafruit.com";                      // MQTT Broker Name
const char* GsmClientName = "ABC123";                        // MQTT Client Name
const char* mqtt_user = " ";                        // MQTT Username
const char* mqtt_pass = " ";  // MQTT Password

const char* pubVoltageTopic = " ";     // Battery Voltage Topic
const char* pubPercentageTopic = " ";  // Battery Percentage Topic
const char* pubLatitudeTopic = " ";    // Latitude Topic
const char* pubLongitudeTopic = " ";   // Longitude Topic
const char* pubSpeedTopic = " ";       // Speed Topic


// ############################################################### SOS Number
String phoneNumber = " ";



// ############################################################### Set Timer Intervals

#define Location_data_interval 8  // Time Interval for sending location
#define Battery_Status 60         // Time Interval for sending battery status
#define SOS_Time 5                // Time Interval for how long the button need to be pressed to trigger SOS

#define SerialMon Serial                // For Serial monitor
SoftwareSerial mySerial(rxPin, txPin);  // For SIM Module
TinyGsm modem(mySerial);


TinyGsmClient client(modem);
PubSubClient mqtt(client);

SimpleTimer Timer;   // Timer for Sending Location & Speed Data
SimpleTimer Timer1;  // Timer for Sending Battery Status


uint32_t lastReconnectAttempt = 0;
const int analogPin = A0;                // Analog pin connected to the voltage divider output
const float voltageDividerRatio = 2.62;  // Ratio of the voltage divider (100k / (100k + 100k))
const float adcMaxValue = 1024.0;        // Maximum ADC value for NodeMCU
const float adcReference = 4.1;          // ADC reference voltage of the NodeMCU

boolean stringComplete = false;
String inputString = "";
String fromGSM = "";
bool CALL_END = 1;
char* response = " ";
String res = "";
int c = 0;
String Speed;
String lat;
String lon;
String temp = "";
String msg;


void setup() {
  delay(2000);


  Serial.begin(9600);    // For Serial Monitor Response
  mySerial.begin(9600);  // For SIMA7672s Module

  // Powering you the Modem
  pinMode(16, OUTPUT);
  digitalWrite(16, HIGH);
  delay(50);
  digitalWrite(16, LOW);


  pinMode(SOS_Button, INPUT_PULLUP);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  modem.restart();
  SerialMon.println("Resetting Modem...");

  // modem.init();  SerialMon.println("Initializing Modem...");


  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3) {
    modem.simUnlock(GSM_PIN);
  }

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) {
    SerialMon.println("Network connected");
  }

  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  }

  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);

  Timer.setInterval((Location_data_interval * 1000));  // Timer for Location & Speed Data
  Timer1.setInterval((Battery_Status * 1000));         // Timer for Battery Status


  // AT

  msg = sendData("AT", 1000, DEBUG);
  SerialMon.println("at");
  while (msg.indexOf("OK") == -1) {
    msg = sendData("AT", 1000, DEBUG);
    delay(1000);
  }

  // AT+CGNSSPWR

  msg = sendData("AT+CGNSSPWR=1,1", 2000, DEBUG);
  SerialMon.println("at+cgnsspwr=1,1");
  while (msg.indexOf("+CGNSSPWR: READY!") == -1) {
    msg = sendData("AT+CGNSSPWR=1,1", 1000, DEBUG);
    delay(4000);
  }

  // AT+CGNSSPORTSWITCH

  msg = sendData("AT+CGNSSPORTSWITCH=1,1", 2000, DEBUG);
  SerialMon.println("at+cgnssportswitch=1,1");
  while (msg.indexOf("OK") == -1) {
    msg = sendData("AT+CGNSSPORTSWITCH=1,1", 1000, DEBUG);
    delay(1000);
  }
}

void loop() {

  // Checking if SIM Module is connected to Network or Not

  if (!modem.isNetworkConnected()) {
    SerialMon.println("Network disconnected");
    if (!modem.waitForNetwork(180000L, true)) {
      SerialMon.println(" fail");
      delay(10000);
      return;
    }
    if (modem.isNetworkConnected()) {
      SerialMon.println("Network re-connected");
    }


    // Checking if SIM Module is connected to Internet or Not

    if (!modem.isGprsConnected()) {
      SerialMon.println("GPRS disconnected!");
      SerialMon.print(F("Connecting to "));
      SerialMon.print(apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
      }
      if (modem.isGprsConnected()) {
        SerialMon.println("GPRS reconnected");
      }
    }
  }


  // Checking if SIM Module is connected to MQTT Broker or Not

  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }

  // Handling MQTT Tasks in Backend
  mqtt.loop();


  // When Location Timer get Overflowed
  if (Timer.isReady()) {
    msg = sendData("AT+CGNSSINFO", 2000, DEBUG);
    delay(500);
    Timer.reset();
  }

  // When Battery Status Timer get Overflowed
  if (Timer1.isReady()) {
    battry_filtering();
    delay(500);
    Timer1.reset();
  }


  // When SOS Button is pressed.
  if (digitalRead(SOS_Button) == LOW && CALL_END == 1) {
    Serial.print("Calling In..");  // Waiting for 5 sec
    for (c = 0; c < SOS_Time; c++) {
      Serial.println((SOS_Time - c));
      delay(1000);
      if (digitalRead(SOS_Button) == HIGH)
        break;
    }

    if (c == 5) {
      Get_gmap_link(1);  // Send Location with Call
    }

    //only write a full message to the GSM module
    if (stringComplete) {
      Serial1.print(inputString);
      inputString = "";
      stringComplete = false;
    }
  }
  delay(300);
  String response = "";


  //Waiting for Incoming call to automaticallty recevie it
  if (mySerial.available()) {
    String call = mySerial.readString();

    // Check if the received message contains "RING"
    if (call.indexOf("RING") != -1) {
      SerialMon.println("---------ITS RINGING-------");
      msg = sendData("ATA", 2000, DEBUG);

      delay(1000);
      call = "";
    }
  }
}



// ############################################################### Generating Google Maps Link, Sending SMS and Making Call to SOS number
void Get_gmap_link(bool makeCall) {


  if (lat.length() == 0 || lon.length() == 0) {
    String noGpsMessage = "No GPS data available";
    // Send SMS with no GPS data message
    bool res = modem.sendSMS(phoneNumber, noGpsMessage);

    Serial.println(noGpsMessage);
  } else {
    Serial.println(lat);
    Serial.println(lon);

    String Gmaps_link = "I'm Here " + ("http://maps.google.com/maps?q=" + lat + "+" + lon);  //http://maps.google.com/maps?q=38.9419+-78.3020
    //------------------------------------- Sending SMS with Google Maps Link with our Location

    bool res = modem.sendSMS(phoneNumber, Gmaps_link);
  }

  delay(2000);
  Serial.println("Calling Now");
  msg = sendData("ATD" + phoneNumber + ";", 2000, DEBUG);
  delay(1000);
}

void battry_filtering() {
  // Read the raw ADC value
  int rawValue = analogRead(analogPin);

  // Convert raw ADC value to voltage
  float voltage = (rawValue / adcMaxValue) * adcReference * voltageDividerRatio;

  // Calculate battery percentage
  int batteryPercentage = map(voltage * 100, 3.2 * 100, 4.2 * 100, 0, 100);  // Multiply by 100 for percentage mapping
  if (batteryPercentage > 100) {
    batteryPercentage = 100;
  } else if (batteryPercentage < 0) {
    batteryPercentage = 0;
  }

  // Print the voltage and percentage to the serial monitor
  Serial.print("Raw ADC Value: ");
  Serial.println(rawValue);
  Serial.print("Battery Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");
  Serial.print("Battery Percentage: ");
  Serial.print(batteryPercentage);
  Serial.println(" %");

  char voltageStr[8];
  dtostrf(voltage, 1, 2, voltageStr);
  mqtt.publish(pubVoltageTopic, voltageStr);

  char percentageStr[4];
  itoa(batteryPercentage, percentageStr, 10);
  mqtt.publish(pubPercentageTopic, percentageStr);
}


// ############################################################### Filtering out specific data from the response coming from SIM Module
void Speed_filtering() {
  response = &temp[0];
  int i = 0;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;

  int j = i;
  while (response[j] != ',') j++;
  Speed = temp.substring(i, j);

  int kmph = Speed.toInt();
  Serial.print("KMPH - ");
  Serial.println(kmph);

  kmph = kmph * 1.852;
  Speed = (String)kmph;

  Serial.print("Speed - ");
  Serial.println(Speed);

  mqtt.publish(pubSpeedTopic, Speed.c_str());
}

void lat_filtering() {
  response = &temp[0];
  int i = 0;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;

  int j = i;
  while (response[j] != ',') j++;
  lat = temp.substring(i, j);

  Serial.print("lat - ");
  Serial.println(lat);

  mqtt.publish(pubLatitudeTopic, lat.c_str());
}

void log_filtering() {
  response = &temp[0];
  int i = 0;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;
  while (response[i] != ',') i++;
  i++;

  int j = i;
  while (response[j] != ',') j++;
  lon = temp.substring(i, j);

  Serial.print("log - ");
  Serial.println(lon);

  mqtt.publish(pubLongitudeTopic, lon.c_str());
}




// ############################################################### MQTT Callback function
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  for (int i = 0; i < len; i++) {
    SerialMon.print((char)payload[i]);
  }
  SerialMon.println();
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  boolean status = mqtt.connect(GsmClientName, mqtt_user, mqtt_pass);

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");
  return mqtt.connected();
}

String sendData(String command, const int timeout, boolean debug) {
  String response = "";
  mySerial.println(command);
  long int time = millis();
  while ((time + timeout) > millis()) {
    while (mySerial.available()) {
      char c = mySerial.read();
      response += c;
    }
  }
  if (debug) {
    Serial.print("response: ");
    Serial.println(response);
  }
  temp = response;
  if (command == "AT+CGNSSINFO") {
    lat_filtering();
    log_filtering();
    Speed_filtering();
  }
  return response;
}
