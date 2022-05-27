#include <Arduino.h>
/*
      Remote: ESPNOW, DS3231 - Send outside temp & pressure inch of Hg
      1) Remote Sensor:
            Use BME280 sensor, store sensor data in a struct "cond" conditions
            Use espnow: broadcast struct "cond" at sleep_us interval (user adjustable)
            attic esp returns the struct with the sleep_us
            and if the current sleep_us is different we update cond.sleepTime & RTC memory.
            We wait for the response from the "Attic Controller" if after 7 seconds no response:
            this mcu initiates deep sleep

      2) uncomment //#define dbg enables Serial prints at several key locations in the code including the MAC

      3)    esp01 for the remote MCU: requires solder a connection from GPIO16 to RST.
            deepSleep wakes up the MCU by output of signal at GPIO16.
      
      NOTE:
            the attic sensor/controller esp's now_Msg struct must have the exact same definition for the cond struct

  int(4), uint(4), uint32(4), long(4), uLong(4), uLongLong(8), short(2), ushort(2), float(4), double(8)
*/
// extern "C" {
//   #include "user_interface.h" // 
// }

//#define dbg
#ifdef dbg
  u32 ms=millis();
  #define trace Serial.printf("%i %lums, ", __LINE__, millis()-ms);ms=millis(); Serial.flush();
#else
  #define trace ;
#endif

#include <espnow.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>

#include <Wire.h>  // loaded in TinyBME280.h - should be called: i2c.h or soft_i2c.h  imho

#define WiFiChannel 7

ADC_MODE(ADC_VCC);

//u8 attic_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};// MAC: MultiCast"
//u8 attic_mac[6] = {0xE8,0xDB,0x84,0xA9,0x13,0x3A};   // latest ESP01
u8 attic_mac[6] = {0x5C,0xCF,0x7F,0xE1,0x9B,0x8B};   // D1 mini

u32 startMillis=millis();
u16 vcc_correct = -90; // vcc == voltage/1000  : We add correction and divide by 1000
u64 sleep_us = 4000002; // default to 4sec 4us
u8 *p = (u8*)&sleep_us;    // pointer to sleep_us to ease handle by byte

// BME280 Values & Variables
#define SDA 0
#define SCL 2
#define BME280address 0x76

/******  PROTOTYPES  *******/
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len);
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus);

// BME280
void BME280setup();
void BME280sleep ();
int32_t BME280temperature();
uint32_t BME280pressure();
uint32_t BME280humidity();
uint8_t BME_ID();

typedef struct remote_Data{
  int temp;
  u32 press;
  u32 humid;
  u64 sleep_us = 4000000;
  u16 vcc;

  String toStr(){
    char c[120];
    int n = sprintf(c, "{\"temp\":%i,\"press\":%u,\"humid\",:%u,\"vcc:%hu,\"sleep_us\":%llu}"
      , temp, press, humid, vcc, sleep_us);
    c[n]=0;
    return String(c);
  }
}remote_data_t;
remote_data_t cond;

  void setup() {
    system_update_cpu_freq(SYS_CPU_160MHZ);
    system_deep_sleep_set_option (2);   // No RF cal on wake-up
    // 1: Vdd and TX power, 18 ms // 2: Vdd only, 2 ms // 3: Full cal, 200ms
    system_phy_set_powerup_option (2);
    system_phy_set_rfoption (2);
#ifdef dbg
    Serial.begin(74880);
    Serial.setTimeout(2000);
    while(!Serial){}
#endif
    EEPROM.begin(8);
    for(u8 i=0;i<8;i++)p[i] = EEPROM.read(i);
    EEPROM.end();
    cond.sleep_us = sleep_us;
trace;
    WiFi.setPhyMode(WIFI_PHY_MODE_11B);
    WiFi.setOutputPower(20.5);  // range: 0dbm - 20.5dbm; default 18.5
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
trace;
    esp_now_init();
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    esp_now_add_peer(attic_mac, ESP_NOW_ROLE_COMBO, WiFiChannel, NULL, 0);
trace;
    Wire.begin(SDA, SCL);
    BME280setup();
trace;
    // get the readings once, during startup (here!)
    cond.temp  = BME280temperature(); // .018 x temp +32  : done at index.html
    cond.press = BME280pressure();    // p x 0.00029529983071445 = in Hg  done int index.html
    cond.humid = BME280humidity();    // humidity / 100 done in index.html
    BME280sleep();
    cond.vcc   = ESP.getVcc() + vcc_correct; // below 3.0volts shutdown; battery needs to be charged
trace;
    esp_now_send(attic_mac, (u8*)&cond, sizeof(cond));
    delay(45); // takes 45-51ms to get here. timeout after ~100ms -> goto sleep
#ifdef dbg
    Serial.printf("\n%i %s %lu\n", __LINE__, __FUNCTION__, millis());Serial.flush();
    Serial.printf("%i %s %lums, ", __LINE__, cond.toStr().c_str(), millis());Serial.flush();
#endif
     ESP.deepSleepInstant(sleep_us, WAKE_RFCAL);
  }
  void loop(){}
  void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len){
trace;
    for(u8 i=0;i<len;i++)p[i] = incomingData[i];
trace;
    if(cond.sleep_us ^ sleep_us && sleep_us < ESP.deepSleepMax()){
trace;
      EEPROM.begin(8);
trace;
      for(u8 i=0;i<8;i++)EEPROM.write(i, p[i]);
trace;
      EEPROM.commit();
trace;
      EEPROM.end();
trace;
    }
trace;
    if(cond.vcc<(u16)3000){ESP.deepSleepInstant(0, WAKE_RF_DISABLED);}
#ifdef dbg
    Serial.printf("%i %s %lu max: %llu, ", __LINE__, __FUNCTION__, millis(), ESP.deepSleepMax());Serial.flush();
#endif
    ESP.deepSleepInstant(sleep_us, WAKE_NO_RFCAL); // Pin 16 must attach to rst for wakeup
  }
  void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus){}
/* TinyBME280 Library v2

   David Johnson-Davies - www.technoblogy.com - 22nd June 2019
   
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
*/
int16_t T[4], P[10], H[7]; // DON'T TOUCH -> TinyBME use
int32_t BME280t_fine;      // DON'T TOUCH -> TinyBME use

int16_t read16 () {
  uint8_t lo, hi;
  lo = Wire.read(); hi = Wire.read();
  return hi<<8 | lo;
}
int32_t read32 () {
  uint8_t msb, lsb, xlsb;
  msb = Wire.read(); lsb = Wire.read(); xlsb = Wire.read();
  return (uint32_t)msb<<12 | (uint32_t)lsb<<4 | (xlsb>>4 & 0x0F);
}
uint8_t BME_ID(){
  delay(2);
  // Set the mode to Normal, no upsampling
  Wire.beginTransmission(BME280address);
  Wire.write(0xD0);
  Wire.endTransmission();
  Wire.requestFrom(BME280address, 1);
  return((uint8_t)Wire.read());
}
// Must be called once at start
void BME280setup () {
  delay(2);
  // Set the mode to Normal, no upsampling
  Wire.beginTransmission(BME280address);
  Wire.write(0xF2);        // ctrl_hum
  Wire.write(0b00000001);  // Oversample H: 1x
  Wire.write(0xF4);        // ctrl_meas
  Wire.write(0b00100101);  // Oversample T: 1x; P: 1x : Force Mode 

  Wire.write(0xF5);         // config
  Wire.write(0b10100000);  // 101 inactive 1000ms, IIR 125ms, I2C mode

  // Read the chip calibrations.
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(BME280address, 26);
  for (int i=1; i<=3; i++) T[i] = read16();     // Temperature
  for (int i=1; i<=9; i++) P[i] = read16();     // Pressure
  Wire.read();  // Skip 0xA0
  H[1] = (uint8_t)Wire.read();                  // Humidity
  //
  Wire.beginTransmission(BME280address);
  Wire.write(0xE1);
  Wire.endTransmission();
  Wire.requestFrom(BME280address, 7);
  H[2] = read16();
  H[3] = (uint8_t)Wire.read();
  uint8_t e4 = Wire.read(); uint8_t e5 = Wire.read();
  H[4] = ((int16_t)((e4 << 4) + (e5 & 0x0F)));
  H[5] = ((int16_t)((Wire.read() << 4) + ((e5 >> 4) & 0x0F)));
  H[6] = ((int8_t)Wire.read()); // 0xE7
  // Read the temperature to set BME280t_fine
  BME280temperature();
}
// Can be called if sensor can sleep to save energy
void BME280sleep () {
  delay(2);
  // Set the mode to Normal, no upsampling
  Wire.beginTransmission(BME280address);
  Wire.write(0xF4);                             // ctrl_meas
  Wire.write(0b00000000);                       // what can be better than sleep?!
  Wire.endTransmission();
}
// Returns temperature in DegC, resolution is 0.01 DegC
// Output value of “5123” equals 51.23 DegC
int32_t BME280temperature () {
  Wire.beginTransmission(BME280address);
  Wire.write(0xFA);
  Wire.endTransmission();
  Wire.requestFrom(BME280address, 3);
  int32_t adc = read32();
  // Compensate
  int32_t var1, var2, t; 
  var1 = ((((adc>>3) - ((int32_t)((uint16_t)T[1])<<1))) * ((int32_t)T[2])) >> 11;
  var2 = ((((adc>>4) - ((int32_t)((uint16_t)T[1]))) * ((adc>>4) - ((int32_t)((uint16_t)T[1])))) >> 12);
  var2 = (var2 * ((int32_t)T[3])) >> 14;
  BME280t_fine = var1 + var2;
  return (BME280t_fine*5+128)>>8;
}
// Returns pressure in Pa as unsigned 32 bit integer
// Output value of “96386” equals 96386 Pa = 963.86 hPa
uint32_t BME280pressure () {
  Wire.beginTransmission(BME280address);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(BME280address, 3);
  int32_t adc = read32();
  // Compensate
  int32_t var1, var2;
  uint32_t p;
  var1 = (((int32_t)BME280t_fine)>>1) - (int32_t)64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)P[6]);
  var2 = var2 + ((var1*((int32_t)P[5]))<<1);
  var2 = (var2>>2) + (((int32_t)P[4])<<16);
  var1 = (((P[3] * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)P[2]) * var1)>>1))>>18;
  var1 = ((((32768+var1))*((int32_t)((uint16_t)P[1])))>>15);
  if (var1 == 0) return 0;
  p = (((uint32_t)(((int32_t)1048576) - adc) - (var2>>12)))*3125;
  if (p < 0x80000000) p = (p << 1) / ((uint32_t)var1);
  else p = (p / (uint32_t)var1) * 2;
  var1 = (((int32_t)P[9]) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((int32_t)(p>>2)) * ((int32_t)P[8]))>>13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + P[7]) >> 4));
  return p;
}
// Humidity in %RH, resolution is 0.01%RH
// Output value of “4653” represents 46.53 %RH
uint32_t BME280humidity () {
  Wire.beginTransmission(BME280address);
  Wire.write(0xFD);
  Wire.endTransmission();
  Wire.requestFrom(BME280address, 2);
  uint8_t hi = Wire.read(); uint8_t lo = Wire.read();
  int32_t adc = (uint16_t)(hi<<8 | lo);
  // Compensate
  int32_t var1; 
  var1 = (BME280t_fine - ((int32_t)76800));
  var1 = (((((adc << 14) - (((int32_t)H[4]) << 20) - (((int32_t)H[5]) * var1)) +
  ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)H[6])) >> 10) * (((var1 *
  ((int32_t)H[3])) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
  ((int32_t)H[2]) + 8192) >> 14));
  var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)H[1])) >> 4));
  var1 = (var1 < 0 ? 0 : var1);
  var1 = (var1 > 419430400 ? 419430400 : var1);
  return (uint32_t)((var1>>12)*25)>>8;
}
