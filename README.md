"# attic_fan_remote_sensor" 
Deep Sleep for "deepSleep_us" Value provided by Attic Fan Controller via ESPNOW

On Wake up this unit reads values from a BME280 and its internal VCC and then sends a, 32bytes (JSON string), using ESPNOW
to the Attic Fan Controller of temp/humid/press/vcc.  The Attic Fan Controller updates its struct with the remote values and sends
a JSON string to the Web Page it hosts using WebSockets.  This ESP01 supports ESPNOW only - no WiFi.

The first Remote Sensor iteration of this remote sensor is using two 18650 2300 mah LIPO Cells.

All LEDs on the ESP01 have been removed.
This ESP01 uses 18 ua (18 Micro Amp) in deep sleep and an average of 66 ma for 212 ms during its awake & transmit period.
With a 2 minute deep sleep cycle we could see extended periods of battery life perhaps several years.  (TBD)

If the Attic Fan Controller unit sends an updated sleep_us value this ESP01 power consuption jumps to 385ma during the communication rcx cycle ~45ms. yikes!
