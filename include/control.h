#ifndef _CONTROL_H
#define _CONTROL_H

#include <functional>
#include <queue>
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

private:
    struct command {
        enum {
            Drive = 0,
            Turn,
            Stop
        } action;
        union {
            float angle;
            unsigned long timeMillis;
        };
    };

    std::queue<command> commands;
};

#endif
