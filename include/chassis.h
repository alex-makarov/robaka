
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

    void moveMotor (Wheel wheel, Direction direction, int speed);

    int heading() const;
    float roll() const;
    float pitch() const;
    void gyro(float& x, float& y, float& z) const;
    int speed() const; // m/s, aggregated over last second
    int range(const int sonar=0) const;
    int encoderCount(const int encoder=0) const;

    unsigned long lastUpdateTs() const;

private:
    Chassis();
    ~Chassis();
    
    static Chassis* _instance;

    unsigned long lastUpdate;

    class HWImpl;
    HWImpl* impl;
};

#endif