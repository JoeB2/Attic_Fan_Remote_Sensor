/*
      Remote: ESPNOW, DS3231 - Send outside temp & pressure inch of Hg
      1) Remote Sensor:

            Use BMP280 sensor, store sensor data in a struct "cond" conditions
            Use espnow: broadcast struct "cond" at remoteSleepSeconds interval (user adjustable)
            attic esp returns the struct with the remoteSleepSeconds
            and if the current remoteSleepSeconds is different we update cond.sleepTime & RTC memory.

      2) uncomment //#define dbg enables Serial prints at several key locations in the code including the MAC

      3)    esp01 for the remote MCU: requires solder a connection from GPIO16 to RST.
            deepSleep wakes up the MCU by output of signal at GPIO16.
      
      NOTE:
            the attic sensor/controller esp's now_Msg struct must have the exact same definition for the cond struct
*/
// extern "C" {
//   #include "user_interface.h" // 
// }

//#define dbg

#include <Arduino.h>
#include <espnow.h>
#include <ESP8266WiFi.h>

// BME280 sensor
#include <Wire.h>  // should be called: i2c.h or soft_i2c.h  imho
#include <TinyBME280.h>
//#include <RTClib.h>

#define READINGS_DELAY 20000  // 1 minutes: this is only used if no ack received from attic ESP
#define SDA 0
#define SCL 2

ADC_MODE(ADC_VCC);

// ".this" mac address
byte attic_mac[] = {0xDC,0x4F,0x22,0x49,0x59,0x15};    // Attic MAC DC:4F:22:49:59:15
uint8_t peer[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // MAC: MultiCast"
unsigned long startMillis=millis();
uint8_t reset = 0; // count loop cycles where pressure == 0 || humidity == 0; at 3 restart the BME at 7 reset the mcu

typedef struct remote_Data{
  float temp;
  double press;
  byte humid;
  uint64_t remoteSleepSeconds=10;
  float vcc;
 
  String toStr(){
    char c[150];
    sprintf(c, "{\"temp\":%.1f,\"press\":%.2f,\"humid\",:%u,\"vcc:%.2f,\"remoteSleepSeconds\":%llu}", temp, press, humid, vcc, remoteSleepSeconds);
    return c;
  }
}remote_data_t;
remote_data_t cond;

/******  PROTOTYPES  *******/
  void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len);
  void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus);

  void setup() {
    delay(1000);
    struct	rst_info	*rtc_info	=	system_get_rst_info();
    if(rtc_info->reason != REASON_DEEP_SLEEP_AWAKE){// 
      uint64_t zero=0;
      system_rtc_mem_write(64, &zero, 8);
    }
    uint64_t *sleep;
    if(system_rtc_mem_read(64, sleep, 2)) cond.remoteSleepSeconds = *sleep?*sleep:11;

#ifdef dbg
  Serial.begin(115200);Serial.flush();
#endif

    system_deep_sleep_set_option(RF_DEFAULT); //  RF calibration during waking
    WiFi.setPhyMode(WIFI_PHY_MODE_11B );
    // ESP.eraseConfig();
    //WiFi.setOutputPower(20.5);  // range: 0dbm - 20.5dbm

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init())Serial.println("Error initializing ESP-NOW");
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    esp_now_add_peer(attic_mac, ESP_NOW_ROLE_COMBO, 7, NULL, 0);

    Wire.begin(SDA, SCL);
    BME280setup();

    startMillis=millis()-READINGS_DELAY;      // don't delay on startup
  }
  void loop() {
      if(millis()-startMillis > READINGS_DELAY){

      int i=0;
      do{
        BME280setup();delay(25);
        cond.temp  = .018*BME280temperature()+32;
        cond.press = BME280pressure()*0.00029529983071445;
        cond.humid = BME280pressure()*0.00029529983071445;
        cond.vcc   = ESP.getVcc()/1000; // at 3.0volts we shutdown; battery needs to be charged
        system_soft_wdt_feed();
        delay(1000);
#ifdef dbg
  Serial.printf("%i BME t: %i, BME h: %i, BME p: %i\n", __LINE__,.018*BME280temperature()+32, BME280pressure()*0.00029529983071445, BME280pressure()*0.00029529983071445, cond.toStr().c_str());
#endif
      }while((int)cond.temp & 0b11011111 == 0 && (cond.press == 0 || cond.humid == 0)  && i++ < 5);
      if(cond.temp ==32 && (cond.press ==0 || cond.humid == 0))ESP.restart();

      esp_now_send(attic_mac, (uint8_t *)&cond, sizeof(cond));

#ifdef dbg
  Serial.print(" *** MAC: ");Serial.println(WiFi.macAddress());
#endif
        startMillis=millis();
      }
  }
  void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {// updates local scaleDiv.  updates the correct s#_Msg
Serial.printf("%i inbound LEN: %i", __LINE__, len);

    uint64_t sleep;
    memcpy(&sleep, incomingData, sizeof(sleep));
    if(sleep != cond.remoteSleepSeconds){
      system_rtc_mem_write(64, &sleep, 2);
      cond.remoteSleepSeconds = sleep;
    }
#ifdef dbg
  Serial.printf("\n%i sleep: %llu  cond: %s\n", __LINE__, sleep, cond.toStr().c_str());Serial.flush();
#endif

    if(ESP.getVcc()/1000<3.00)ESP.deepSleepInstant(ESP.deepSleepMax(), WAKE_RF_DEFAULT);
    if(cond.remoteSleepSeconds)ESP.deepSleep(cond.remoteSleepSeconds * 1000000, WAKE_RF_DEFAULT); // toDo: fix GPIO16 pin
  }
  void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
#ifdef dbg
    if (sendStatus == 0){Serial.printf("\n%i NOW: Delivery success\n", __LINE__);Serial.flush();}
    else Serial.printf("\n%i NOW: Delivery fail\n", __LINE__);Serial.flush();
#endif
  }
