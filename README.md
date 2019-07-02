1. Clone SimplePID to lib/
2. Change LSGD20 values as written in the header
3. Change Wire to Wire1 in LSM303 impl
4. ArduinoHardware: #if defined(USBCON) and !(defined(USE_USBCON))
      /* Leonardo support */
//      iostream = &Serial1;
      iostream = &Serial;


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
