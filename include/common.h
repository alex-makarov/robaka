#ifndef _COMMON_H
#define _COMMON_H

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

struct vector_t {
    float x;
    float y;
    float z;
};

#endif
