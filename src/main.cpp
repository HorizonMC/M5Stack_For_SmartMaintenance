#include <M5Stack.h>
#include "arduinoFFT.h"
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <AsyncMqttClient.h>
#include <WiFiUdp.h>
#include <WiFi.h>
#include <time.h>

#include "sensor.h"
#include "io.h"
#include "pm.h"
#include "SPIFFS.h"
#include "BrokerConfig.h"
#include "NetworkConfig.h"

bool debug = false;

AsyncMqttClient mqttClient;

// MQTT Config
// #define MQTT_HOST 		IPAddress(128,199,120,225)
// #define MQTT_PORT 		1883
// #define mqtt_user 		"TEST"
// #define mqtt_password   "12345"

// #define MQTT_HOST 		IPAddress(130,211,244,161)
// #define MQTT_PORT 		1883
// #define mqtt_user 		"admin"
// #define mqtt_password   "public"

#define MQTT_HOST 		IPAddress(203,185,64,8)
#define MQTT_PORT 		1883
#define mqtt_user 		"guest"
#define mqtt_password   "guest"

String session_key = "";

uint64_t  chipid=ESP.getEfuseMac();
const String deviceId =  String((uint16_t)(chipid>>32), HEX) + String((uint32_t)chipid, HEX);
bool isWiFiReset = false;
bool isSmartConfig = false;

const char * hostName = "esp-async";
const char* http_username = "";
const char* http_password = "";
const String statusTopic = "/device/"+deviceId+"/status";
const String statusGetTopic = "/device/"+deviceId+"/status/get";
const String deviceTopic = "/device/"+deviceId+"/properties";
const String deviceResultTopic = "/device/"+deviceId+"/properties/result";
const String dataTopic = "/device/"+deviceId+"/data";
const String willTopic = "/device/"+deviceId+"/will";
const String sessionTopic = "/device/"+deviceId+"/session";
const String reset = "/device/"+deviceId+"/reset";
int timer = 0;


	//#define WIFI_SSID "see_dum"
	//#define WIFI_PWD "0863219053"
  //#define WIFI_SSID "HUAWEI"
	//#define WIFI_PWD "0000000000"
  // #define WIFI_SSID "DESS_MAKER_LAB"
	// #define WIFI_PWD "1212312121"
  // #define WIFI_SSID "MAKER_LAB"
	// #define WIFI_PWD "1212312121"
  // #define WIFI_SSID "Horizon"
  // #define WIFI_PWD "1q2w3e4r"
  #define WIFI_SSID "NSTDA-D"
	#define WIFI_PWD "i0t#dEsS"

#define STACK_SIZE    1024

const char* ntpServer = "time.navy.mi.th";
const long  gmtOffset_sec = 3600 * 7;
const int   daylightOffset_sec = 0;

#define M5STACK_FIRE_NEO_NUM_LEDS  10
#define M5STACK_FIRE_NEO_DATA_PIN  15
#define M5STACKFIRE_MICROPHONE_PIN 34
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(M5STACK_FIRE_NEO_NUM_LEDS, M5STACK_FIRE_NEO_DATA_PIN, NEO_GRB + NEO_KHZ800);
//#include <WiFi.h>

//MicrophoneSpectrumTFT
double AdcMeanValue = 0;
#define NUMBEROFSAMPLES 1000
uint16_t micValue[NUMBEROFSAMPLES];
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
int Mic_Hz = 0;
int maxAmplitudeDB = 0;


#define M5STACKFIRE_MICROPHONE_PIN 34
#define M5STACKFIRE_SPEAKER_PIN 25 // speaker DAC, only 8 Bit

const int LCD_WIDTH = 320;
const int LCD_HEIGHT = 240;
const int SAMPLES = 320;
const int DOTS_DIV = 30;

#define HORIZONTAL_RESOLUTION 320
#define VERTICAL_RESOLUTION   80
#define POSITION_OFFSET_Y      20
#define SIGNAL_LENGTH 512

double oldSignal[SIGNAL_LENGTH];
double adcBuffer[SIGNAL_LENGTH];
double vImag[SIGNAL_LENGTH];

#define SAMPLINGFREQUENCY 40000
#define SAMPLING_TIME_US     ( 1000000UL/SAMPLINGFREQUENCY )
#define ANALOG_SIGNAL_INPUT        M5STACKFIRE_MICROPHONE_PIN

#define CH1COLOR YELLOW
#define CH2COLOR CYAN
#define GREY 0x7BEF

unsigned long powerOffButtonTimeA = 0;
unsigned long powerOffButtonTimeB = 0;
unsigned long powerOffButtonTimeC = 0;

unsigned long keepWakeUpTime = 0;
unsigned long lastUpdateChargingTime = 0;
bool isLastChargeFull = false;
bool isLastCharging = false;

void connectToMqtt();
void DrawGrid();

void Show_mpu(){
  M5.Lcd.setTextColor(RED);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(5, 90);
  M5.Lcd.print("max. frequency: "); M5.Lcd.print(SAMPLINGFREQUENCY / 2); M5.Lcd.println(" Hz");

  int _MPUx=64+75;
  int _MPUy=128+85;
  int _MPUz=192+85;

  // M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE , BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(5, 110); M5.Lcd.printf("Vibration");

  M5.Lcd.setCursor(65, 130); M5.Lcd.printf("x");
  M5.Lcd.setCursor(_MPUx, 130); M5.Lcd.printf("y");
  M5.Lcd.setCursor(_MPUy, 130); M5.Lcd.printf("z");

  M5.Lcd.setTextColor(YELLOW , BLACK);
  M5.Lcd.fillRect(5, 155, 270, 85, BLACK);
  M5.Lcd.setCursor(5, 155); M5.Lcd.printf("acc");
  M5.Lcd.setCursor(65, 155); M5.Lcd.print((int)(1000 * IMU.ax));
  M5.Lcd.setCursor(_MPUx, 155); M5.Lcd.print((int)(1000 * IMU.ay));
  M5.Lcd.setCursor(_MPUy, 155); M5.Lcd.print((int)(1000 * IMU.az));


  M5.Lcd.setCursor(5, 175); M5.Lcd.printf("gyro");
  M5.Lcd.setCursor(65, 175); M5.Lcd.print((IMU.gx));
  M5.Lcd.setCursor(_MPUx, 175); M5.Lcd.print((IMU.gy));
  M5.Lcd.setCursor(_MPUy, 175); M5.Lcd.print((IMU.gz));



  M5.Lcd.setCursor(5, 195); M5.Lcd.printf("mag");
  M5.Lcd.setCursor(65, 195); M5.Lcd.print((int)(IMU.mx));
  M5.Lcd.setCursor(_MPUx, 195); M5.Lcd.print((int)(IMU.my));
  M5.Lcd.setCursor(_MPUy, 195); M5.Lcd.print((int)(IMU.mz));

  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(_MPUz, 155); M5.Lcd.print("(mg)");
  M5.Lcd.setCursor(_MPUz, 175); M5.Lcd.print("(rad/s)");
  M5.Lcd.setCursor(_MPUz, 195); M5.Lcd.print("(mG)");
}

void vDhtTask(void * pvParameters){
  while(true){

    // Show_mpu();
    vTaskDelay(50);
  }
}
void vBtnTask (void* pvParameter) {

  while(true) {

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void printLocalTime(void *pvParameters)
{
  while(true) {
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
      return;
    }
    // Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    Serial.println(&timeinfo, "%a, %d/%m/%Y %H:%M:%S");
    // Serial.println(timeinfo.tm_hour);
    if((timeinfo.tm_hour==17&&timeinfo.tm_min==00&&timeinfo.tm_sec==00)||(timeinfo.tm_hour==23&&timeinfo.tm_min==59&&timeinfo.tm_sec==59)||(timeinfo.tm_hour==6&&timeinfo.tm_min==00&&timeinfo.tm_sec==00)){
      // ESP.restart();
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


void onMqttConnect(bool sessionPresent) {
	Serial.println("Connected to MQTT.");
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(130, 115);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.print("Connected to MQTT. ");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  Serial.print("subscribe : ");
	mqttClient.subscribe (deviceTopic.c_str(), 0);
	mqttClient.subscribe (dataTopic.c_str(), 0);
  mqttClient.subscribe (statusGetTopic.c_str(), 0);
  mqttClient.subscribe (reset.c_str(), 0);

  Serial.println("Session Key: " + session_key);
  DynamicJsonBuffer jsonBuffer;
	JsonObject& session = jsonBuffer.createObject();
	session["deviceid"] = deviceId;
  session["session"] = session_key;
  String sessionJson = String();
	session.printTo(sessionJson);
  mqttClient.publish(sessionTopic.c_str(), 0, false, sessionJson.c_str());

}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  connectToMqtt();
  if (reason == AsyncMqttClientDisconnectReason::TLS_BAD_FINGERPRINT) {
    Serial.println("Bad server fingerprint.");
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
	Serial.print(topic);
	Serial.print(":\r\n\t");
	Serial.println((char*)payload);
	DynamicJsonBuffer jsonBuffer;
	JsonObject& root = jsonBuffer.parseObject(payload);
	if (root.success()) {
		if (String(topic) == deviceTopic) {
			// send_device_properties();
		}
		if (String(topic) == dataTopic) {
      int gpio = root["gpio"].as<int>();
			// toggle(gpio);
		}

    if (String(topic) == statusGetTopic) {
      // toggle_status(CHANNEL_1);
      // toggle_status(CHANNEL_2);
      // toggle_status(CHANNEL_3);
      // toggle_status(CHANNEL_4);
    }
	}
  if (String(topic) == reset) {
    String id = root["id"];
    Serial.print(id+" : ");
    Serial.println(deviceId);
    if(id == deviceId)
    {
      Serial.println("Reset");
      // int i = (int)payload;
      int i = root["reset"].as<int>();
      if(i == 1)
      {
        Serial.println("Reset Form network");
        ESP.restart();
      }
    }
  }
}

void connectToMqtt() {
	mqttClient.connect();
}

void mqttclient_config() {
  BrokerSettings.load();
  mqttClient.onConnect(onMqttConnect);
	mqttClient.onDisconnect(onMqttDisconnect);
	mqttClient.onSubscribe(onMqttSubscribe);
	mqttClient.onUnsubscribe(onMqttUnsubscribe);
	mqttClient.onMessage(onMqttMessage);
	mqttClient.onPublish(onMqttPublish);
	mqttClient.setClientId(deviceId.c_str());
  mqttClient.setCredentials(BrokerSettings.user_name.c_str(),BrokerSettings.password.c_str());
	String willPayload = "{ \"id\": \""+deviceId+"\", \"msg\": \"The connection from this device is lost:(\" }";
	mqttClient.setKeepAlive(20).setWill(willTopic.c_str(), 1, true, willPayload.c_str());
	mqttClient.setServer(BrokerSettings.serverHost.c_str(), BrokerSettings.serverPort);
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);
    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }
    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void WiFiEvent(WiFiEvent_t event)
{
    // M5.Lcd.fillScreen(WHITE);
    // M5.Lcd.setCursor(190, 0);
    // M5.Lcd.setTextColor(BLACK);
    // M5.Lcd.setTextSize(1);
    // M5.Lcd.printf("[WiFi-event] event: %d\n", event);
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
        case SYSTEM_EVENT_STA_STOP:
        break;

        case SYSTEM_EVENT_STA_START:
        // mqttclient_config();
        break;
        case SYSTEM_EVENT_WIFI_READY:
        break;
        case SYSTEM_EVENT_SCAN_DONE:
        break;
        case SYSTEM_EVENT_STA_GOT_IP: {
            Serial.println("WiFi got ip address");
            Serial.println("IP address: ");
            Serial.printf("[WiFi-event] event: %d\n", event);
            Serial.println(WiFi.localIP());
            configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
             xTaskCreatePinnedToCore(printLocalTime, "printLocalTime", STACK_SIZE * 2,  NULL, tskIDLE_PRIORITY+3, NULL, 0);
             // vTaskDelay(2000 / portTICK_PERIOD_MS);
             // WiFi.begin(WIFI_SSID,WIFI_PWD);
            break;
        }
        case SYSTEM_EVENT_STA_DISCONNECTED:{
          struct tm timeinfo;
          if(!getLocalTime(&timeinfo)){
            Serial.println("Failed to obtain time");
            return;
          }
            Serial.println("WiFi lost connection");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            WiFi.begin(WIFI_SSID,WIFI_PWD);
            Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
        }
        case SYSTEM_EVENT_STA_LOST_IP:{
            Serial.println("WiFi lost IP");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            WiFi.begin(WIFI_SSID,WIFI_PWD);
        }
            break;
        break;
    }
}

void wifi_config() {
#ifndef WIFI_SSID
  NetworkSettings.load();
  if (NetworkSettings.active) {
	  WiFi.onEvent(WiFiEvent);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    Serial.println("WiFi load config");
    Serial.println(NetworkSettings.active);
    Serial.println(NetworkSettings.ssid);
    Serial.println(NetworkSettings.password);
    Serial.println(NetworkSettings.dhcp);
    Serial.println(NetworkSettings.ip);
    Serial.println(NetworkSettings.netmask);
    Serial.println(NetworkSettings.gateway);
    WiFi.mode(WIFI_STA);
    Serial.printf("Connecting to %s\n", NetworkSettings.ssid);
    if (String(WiFi.SSID()) != NetworkSettings.ssid) {
      WiFi.begin(NetworkSettings.ssid.c_str(), NetworkSettings.password.c_str());
    }
  } else {
    WiFi.mode(WIFI_STA);
    Serial.println("Start SmartConfig...");
    if (WiFi.beginSmartConfig()){
      while (!WiFi.smartConfigDone()) {
        delay(500);
        Serial.print(".");
      }
      Serial.println();
      Serial.println("SmartConfig done");
      int value = 500;
      xQueueSend(qu_task, &value, 10);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      server_config();
      connectToMqtt();
      Serial.println();
      NetworkSettings.active = true;
      NetworkSettings.ssid = WiFi.SSID();
      NetworkSettings.password = WiFi.psk();
      NetworkSettings.dhcp = true;
      NetworkSettings.ip = WiFi.localIP();
      NetworkSettings.netmask = WiFi.subnetMask();
      NetworkSettings.gateway = WiFi.gatewayIP();
      NetworkSettings.save();
      Serial.println("");
      Serial.println("WiFi SmartConfig connected");
      Serial.println(NetworkSettings.active);
      Serial.println(NetworkSettings.ssid);
      Serial.println(NetworkSettings.password);
      Serial.println(NetworkSettings.dhcp);
      Serial.println(NetworkSettings.ip);
      Serial.println(NetworkSettings.netmask);
      Serial.println(NetworkSettings.gateway);
    }
  }
#else
    WiFi.onEvent(WiFiEvent);
    Serial.println("WiFi load config");
    WiFi.mode(WIFI_STA);
    Serial.printf("Connecting to %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PWD);
#endif
}




void deepSleep() {
  M5.Lcd.fillScreen( BLACK );
  M5.Lcd.fillRect(10, 1, 150, 160, BLACK);
  M5.Lcd.setCursor(50, 120);
  M5.Lcd.setTextColor(WHITE);  //M5.Lcd.setTextSize(3);
  M5.Lcd.setTextSize(2);

  M5.Lcd.print("DeepSleep");
  delay(500);
  pureDeepSleep();
}

void DrawGrid()
{
	for (int x = 0; x <= SAMPLES; x += 2) // Horizontal Line
	{
		for (int y = 30; y <= VERTICAL_RESOLUTION+10; y += DOTS_DIV)
		{
			M5.Lcd.drawPixel(x, y, GREY);
			// CheckSW();
		}
		if (VERTICAL_RESOLUTION == 90)
		{
			M5.Lcd.drawPixel(x, VERTICAL_RESOLUTION - 1, GREY);
		}
	}
	for (int x = 0; x <= SAMPLES; x += DOTS_DIV) // Vertical Line
	{
		for (int y = 30; y <= VERTICAL_RESOLUTION+10; y += 2)
		{
			M5.Lcd.drawPixel(x, y, GREY);
			// CheckSW();
		}
	}

}



void showSignal()
{
  int n;

  int oldx;
  int oldy;
  int oldSig;
  int x, y;

  for (n = 0; n < SIGNAL_LENGTH/2; n++)
  {
    x = n;
    y = map(adcBuffer[n], 0, 512, VERTICAL_RESOLUTION, POSITION_OFFSET_Y);

    if (n > 0)
    {
      // delete old line element
      M5.Lcd.drawLine(oldx , oldSig, x, oldSignal[n], BLACK );

      // draw new line element
      if (n < SIGNAL_LENGTH - 1) // don't draw last element because it would generate artifacts
      {
        M5.Lcd.drawLine(oldx,    oldy, x,            y, GREEN );
      }
    }
    oldx = x;
    oldy = y;
    oldSig = oldSignal[n];
    oldSignal[n] = y;
  }
}

void setup()
{
  M5.begin();
  pixels.begin();
  Wire.begin();
  pixels.Color(0, 0, 0);
  pixels.clear();
  pixels.show();
  Serial.begin(115200);
  Serial.println ("Smart maintenace V1.001");
  Serial.print ("ESP32 cpu frequency: ");
  Serial.println(ESP.getCpuFreqMHz());
  Serial.print ("DeviceId :");
  Serial.println(deviceId);

  ledcDetachPin(SPEAKER_PIN);
  pinMode(SPEAKER_PIN, INPUT);

  if(!SPIFFS.begin(true)){
      Serial.println("SPIFFS Mount Failed");
      while(true);
  }
  listDir(SPIFFS, "/", 0);

  // wifi_config();

Serial.println();
Serial.print("Connecting to ");
Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("MacAddress: ");
  Serial.println(WiFi.macAddress());
  mqttclient_config();
  connectToMqtt();
  setupMPU9250();

  //WiFi.mode(WIFI_OFF);
  //btStop();

  dacWrite(M5STACKFIRE_SPEAKER_PIN, 0); // make sure that the speaker is quite
  M5.Lcd.begin();
  M5.Lcd.fillScreen( BLACK );
  M5.Lcd.fillRect(10, 1, 150, 160, BLACK);
  M5.Lcd.setCursor(5, 5);
  M5.Lcd.setTextColor(WHITE);  //M5.Lcd.setTextSize(3);
  M5.Lcd.setTextSize(2);

  M5.Lcd.print("Smart Maintenance");
  M5.Lcd.setTextColor(RED);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(5, 90);
  M5.Lcd.print("max. frequency: "); M5.Lcd.print(SAMPLINGFREQUENCY / 2); M5.Lcd.println(" Hz");
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(130, 115);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.print(WiFi.localIP());
  M5.Lcd.setTextSize(2);

  // xTaskCreatePinnedToCore(vBtnTask, "vBtnTask", STACK_SIZE * 2,  NULL, tskIDLE_PRIORITY+2, NULL, 0);
  // xTaskCreatePinnedToCore(vDhtTask, "vDhtTask", STACK_SIZE * 2,  NULL, tskIDLE_PRIORITY-1, NULL, 1);

}

void MicrophoneSpectrumTFT(){
  int n;
  uint32_t nextTime = 0;
    // record signal
  for (n = 1; n < SIGNAL_LENGTH; n++)
  {
    double v = analogRead( ANALOG_SIGNAL_INPUT );
    AdcMeanValue += (v - AdcMeanValue) * 0.001;
    adcBuffer[n] = v - AdcMeanValue;

    // wait for next sample
    while (micros() < nextTime);
    nextTime = micros() + SAMPLING_TIME_US;
  }

  FFT.Windowing(adcBuffer, SIGNAL_LENGTH, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(adcBuffer, vImag, SIGNAL_LENGTH, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(adcBuffer, vImag, SIGNAL_LENGTH); /* Compute magnitudes */
  //int x = FFT.MajorPeak(adcBuffer, SIGNAL_LENGTH, 1000000UL / SAMPLING_TIME_US);//SAMPLINGFREQUENCY
  Mic_Hz = FFT.MajorPeak(adcBuffer, SIGNAL_LENGTH, SAMPLINGFREQUENCY);

  maxAmplitudeDB = 0;
  for (n = 1; n < SIGNAL_LENGTH; n++)
  {
    int a = log10(adcBuffer[n]) * 20 - 54.186; // convert amplitude to dB scale, dB relative to log10(512samples)*20=54.186dB
    if (a > maxAmplitudeDB) maxAmplitudeDB = a;
    adcBuffer[n] = (a + 30) * 5; // scale for TFT display
    vImag[n] = 0; // clear imaginary part
  }
  M5.Lcd.fillRect(230, 0, 90, 40, BLUE);
  M5.Lcd.setTextColor(YELLOW,BLUE);
  M5.Lcd.setCursor(235, 1);
  M5.Lcd.setTextSize(2);
  M5.Lcd.print(Mic_Hz); M5.Lcd.print(" Hz");
  M5.Lcd.setCursor(235, 21);
  M5.Lcd.print(maxAmplitudeDB); M5.Lcd.print(" dB");

  showSignal();
}
void NeoPixelVUmeter (){
  uint32_t power = 0;
  uint32_t meanValue = 0;
  for (uint32_t n = 0; n < NUMBEROFSAMPLES; n++)
  {
    int value = analogRead(M5STACKFIRE_MICROPHONE_PIN);
    micValue[n] = value;
    meanValue += value;
    delayMicroseconds(20);
  }
  meanValue /= NUMBEROFSAMPLES;
  for (uint32_t n = 0; n < NUMBEROFSAMPLES; n++)
  {
    power += (micValue[n] - meanValue) * (micValue[n] - meanValue);
  }
  power /= NUMBEROFSAMPLES;


  if(debug) Serial.println(power);

  int threshold = 1000;
  for (uint8_t n = 0; n < M5STACK_FIRE_NEO_NUM_LEDS; n++)
  {
    pixels.setPixelColor(n, pixels.Color(0, 0, 1));
    if (power > threshold)  pixels.setPixelColor(n, pixels.Color(100, 0, 0));
    threshold *= 5;
  }
  pixels.show();

}
void Publish_Data(){
  if (mqttClient.connected())
    {
    if (timer >= 5000) {
      String json = "{";
          json += "\"type\": 2";
          json += ", \"id\": \""+deviceId+"\"";
          json += ", \"ax\": "+String((int)(1000 * IMU.ax));
          json += ", \"ay\": "+String((int)(1000 * IMU.ay));
          json += ", \"az\": "+String((int)(1000 * IMU.az));
          json += ", \"gx\": "+String((IMU.gx));
          json += ", \"gy\": "+String((IMU.gy));
          json += ", \"gz\": "+String((IMU.gz));
          json += ", \"mx\": "+String((int)(IMU.mx));
          json += ", \"my\": "+String((int)(IMU.my));
          json += ", \"mz\": "+String((int)(IMU.mz));
          json += ", \"Mic\": "+ String(Mic_Hz);
          json += ", \"Mic_dB\": "+ String(maxAmplitudeDB);
          json += ", \"Heap\": "+String(ESP.getFreeHeap());
          json += "}";
          Serial.println("Publish: "+json);
          mqttClient.publish(dataTopic.c_str(), 1, true, json.c_str());
          timer = 0;
    }else timer += 100;
  }else {
    Serial.println("connection lost, reconnect...");
    if (timer >= 5000) {
        connectToMqtt();
        timer = 0;
    }else timer += 100;
   }

}
void CheckSw(){
  if (digitalRead(BtnPinC) == 0) {
    unsigned long currTime = millis();
    keepWakeUpTime = currTime;
    if (powerOffButtonTimeC == 0) {
      powerOffButtonTimeC = currTime;
    }
    if (powerOffButtonTimeC != 0 && currTime - powerOffButtonTimeC > 1000) {
      Serial.println("long pressed, delay and going to sleep now");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      deepSleep();
    }
  }else  powerOffButtonTimeC = 0;

  if (digitalRead(BtnPinB) == 0) {
    unsigned long currTime = millis();
    keepWakeUpTime = currTime;
    if (powerOffButtonTimeB == 0) {
      powerOffButtonTimeB = currTime;
    }
    if (powerOffButtonTimeB != 0 && currTime - powerOffButtonTimeB > 1000) {
      Serial.println("long pressed, delay and going to calibrateMPU9250");
      setupMPU9250();
    }
  }else  powerOffButtonTimeB = 0;
}

void loop(void)
{
  NeoPixelVUmeter();
  MicrophoneSpectrumTFT();
  CheckSw();
  DrawGrid();
  readMPU9250();
  Show_mpu();
  Publish_Data();
  // dacWrite(M5STACKFIRE_SPEAKER_PIN, 0);
  // m5.Speaker.mute();
}
