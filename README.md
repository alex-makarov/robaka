# ROBAKA
![Alt text](design/images/IMG_0014.JPG?raw=true "Robaka")

Robaka is my testbed for ROS and SLAM. Software runs on Arduino Due (this repo) and Jetson Nano.

# SETUP
1. Clone SimplePID to lib: git@github.com:merose/SimplePID.git
2. Change Adafruit_L3GD20_U.h as following
```
  !! NOTE that Gyro is using non-standard address 0x69 instead of 0x6B and D3 id instead of D4 or D7
    // #define L3GD20_ADDRESS           (0x6B)        // 1101011
    #define L3GD20_ADDRESS           (0x69)        // 1101001
    ...
    // #define L3GD20_ID                (0xD4)
    // #define L3GD20H_ID               (0xD7)
    #define L3GD20_ID                (0xD3)
    #define L3GD20H_ID               (0xD7)
```
3. Change Wire to Wire1 in LSM303 implementation.
4. Make ROS use the SerialUSB of Arduino Due; in ArduinoHardware.h: 
```
#if defined(USBCON) and !(defined(USE_USBCON))
      /* Leonardo support */
//      iostream = &Serial1;
      iostream = &SerialUSB;
```
5. Patch ydlidar node: https://github.com/EAIBOT/ydlidar/pull/19/files
6. Apply cartographer configuration in ydlidar:  https://msadowski.github.io/ydlidar-x2-review-ros-cartographer/
