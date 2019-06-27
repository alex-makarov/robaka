#ifndef _CONTROL_H
#define _CONTROL_H

#include <functional>
#include "common.h"

class Chassis;
typedef const std::function<void ()> handler_t;

class Control {
public:
    Control(Chassis& chassis);

    // Movement
    Control& drive(Direction direction, unsigned long timeMillis, handler_t callback);
    Control& drive(Direction direction, float meters, handler_t callback);
    Control& driveForwardUntilObstacle(handler_t callback);
    Control& stop();

    // Rotation
    Control& turn(float degrees, handler_t callback);

    // Reset the command queue
    void reset();
};

#endif
