
#ifndef _CHASSIS_H
#define _CHASSIS_H 1

enum Direction {
    Forward = 0,
    Backward
};

enum Wheel {
    FrontLeft = 0,
    FrontRight,
    RearLeft,
    RearRight
};

class Chassis {
public:
    static Chassis* instance();

    bool init();
    void updateSensors();

    void moveMotor (Wheel wheel, int speed);
    void moveMotor (Wheel wheel, Direction direction, int speed);

    int heading() const;
    float roll() const;
    float pitch() const;
    void gyro(float& x, float& y, float& z) const;
    int speed() const; // m/s, aggregated over last second
    int range(const int sonar=0) const;
    int encoderCount(Wheel wheel) const;

    unsigned long lastUpdateTs() const;
    bool isInitialized() const { return initialized; }

private:
    Chassis();
    ~Chassis();
    
    static Chassis* _instance;

    unsigned long lastUpdate;
    bool initialized;

    class HWImpl;
    HWImpl* impl;

    int wheelToEncoder(Wheel wheel) const;
};

#endif