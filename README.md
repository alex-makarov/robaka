1. Clone SimplePID to lib/
2. Change LSGD20 values as written in the header
3. Change Wire to Wire1 in LSM303 impl
4. ArduinoHardware: #if defined(USBCON) and !(defined(USE_USBCON))
      /* Leonardo support */
//      iostream = &Serial1;
      iostream = &Serial;
5. Patch ydlidar node: https://github.com/EAIBOT/ydlidar/pull/19/files
 

to debug rosserial:
`sudo interceptty -s 'ispeed 57600 ospeed 57600' -l /dev/ttyACM0 /dev/ttyDUMMY | interceptty-nicedump`


5. To use USB serial:
#elif defined(_SAM3XA_)
  #include <UARTClass.h>  // Arduino Due
//  #define SERIAL_CLASS UARTClass
-->>>  #define SERIAL_CLASS Serial_
#elif defined(USE_USBCON)
  // Arduino Leonardo USB Serial Port
  #define SERIAL_CLASS Serial_
#else 
  #include <HardwareSerial.h>  // Arduino AVR
  #define SERIAL_CLASS HardwareSerial
#endif

class ArduinoHardware {
  public:
    ArduinoHardware(SERIAL_CLASS* io , long baud= 57600){
      iostream = io;
      baud_ = baud;
    }
    ArduinoHardware()
    {
#if defined(USBCON) and !(defined(USE_USBCON))
      /* Leonardo support */
//      iostream = &Serial1;
      //iostream = &Serial;
-->>>      iostream = &SerialUSB;


---------------------

ESP8266 -> UART to WiFi code:
https://raw.githubusercontent.com/esp8266/Arduino/master/libraries/ESP8266WiFi/examples/WiFiTelnetToSerial/WiFiTelnetToSerial.ino

OLD:
link https://github.com/jeelabs/esp-link
flashing https://www.xgadget.de/anleitung/esp-01-esp8266-programmer-so-funktioniert-der-flashvorgang/
esp-link-v2.2.3
esptool.py --port /dev/ttyUSB0 --baud 115200 erase_flash
esptool.py --port /dev/ttyUSB0 --baud 57600 write_flash -fs 4m -ff 40m     0x00000 boot_v1.5.bin 0x1000 user1.bin 0x7E000 blank.bin
3.0.14 works: https://github.com/jeelabs/esp-link/releases/tag/V3.0.14
esptool.py --port /dev/ttyUSB0 --baud 57600 write_flash -fs 4m -ff 40m 0x00000 boot_v1.6.bin 0x1000 user1.bin 0x7C000 esp_init_data_default.bin 0x7E000 blank.bin
