
#ifndef _CHASSIS_H
#define _CHASSIS_H 1

class Chassis {
public:
    static Chassis* instance();

    bool init();
    void updateSensors();

    void moveMotor(int motorId, int direction, int speed); 

private:
    Chassis();
    ~Chassis();
    
    static Chassis* _instance;

    class HWImpl;
    HWImpl* impl;
};

#endif