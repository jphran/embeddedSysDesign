#ifndef INS_TYPES_H
#define INS_TYPES_H
// TODO: Figure out where int32_t is defined, because including this whole thing is just stupid. Even if it has include guards.
#include "stm32f0xx_hal.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

//Q21.10 Types, used for Position.
typedef int32_t q21p10_t;

//Q3.28 Types, used for Rotation.
typedef int32_t q3p28_t;

#define FLT_MOD(x, y) FLT_INT(x) % (int32_t) y + (x - FLT_INT(x))

#define FLT_INT(x) ((int32_t)x)
#define FLT_DEC(x, y) ((int32_t)abs((x - FLT_INT(x)) * pow(10, y)))

// Converts a signed integer to a Q21.10 Fixed Point representation
#define TO_Q21P10(x) ((q21p10_t) (x << 10))

// Converts from a q21p10 Fixed Point to a signed integer (truncates).
#define FROM_Q21P10(x) ((int32_t) (x >> 10))

// Returns the fractional bits of a Q21P10 Number.
#define FRC_Q21P10(x) ((int32_t)((((int64_t) (x & 0x000003FF)) * 10) / (int64_t) (1 << 10)))

// Converts a signed integer to a Q3.28 Fixed Point representation
#define TO_Q3P28(x) ((q3p28_t) (x << 28))

// Converts from a q3p28 Fixed Point to a signed integer (truncates).
#define FROM_Q3P28(x) ((int32_t) (x >> 28))

// Returns the fractional bits of a Q3P28 Number.
#define FRC_Q3P28(x) ((int32_t)((((int64_t) (x & 0x0FFFFFFF)) * 10) / (int64_t) (1 << 28)))

#define FMT_R3_LEN(x) (1 + 1 + 10 + 1 + x + 2 + 10 + 1 +x + 2 + 10 + 1 + x + 1 + 1)


#define FMT_R3(r, buf, p) ( \
    snprintf(buf, FMT_R3_LEN(p), \
          "(%+ld.%0*ld, %+ld.%0*ld, %+ld.%0*ld)", \
          FLT_INT(r->x), (int) p, FLT_DEC(r->x, p), \
          FLT_INT(r->y), (int) p, FLT_DEC(r->y, p), \
          FLT_INT(r->z), (int) p, FLT_DEC(r->z, p)))

struct R3
{
  float x;
  float y;
  float z;
};

struct DMS
{
  uint32_t deg;
  uint32_t min;
  float    sec;
  char     dir;
};

// Represents an updatable position, in 3D Euclidean Space.
// Positions are:
//    Single Precision Floating Point
//    Offset Units: meters
//    Velocity Units: m/s
//    Acceleration Units: m/s^2
struct Position
{
  struct R3 o;
  struct R3 v;
  struct R3 a;
};

// Defines the length of a formatted position string.
// '(off: ' + o + ', vel: ' + v + ', acc: ' + a + ')' + NULL termination
#define POS_FMT_LEN (6 + FMT_R3_LEN(4) + 5 + FMT_R3_LEN(4) + 5 + FMT_R3_LEN(4) + 1 + 1)

// Writes a formatted form of the position into the buffer. All positions are printed in the following format:
// '(x, y, z)', where each element is at longest, a 10 digit number with a decimal. This max length is defined
// as POS_FMT_LEN
void pos_format(struct Position* p, char* buf);

// Rotations are:
//    Single Precision Floating Point
//    Offset Units: degrees.
//    Velocity Units: deg/s
//    Acceleration Units: deg/s^2
struct Rotation
{
  struct R3 o;
  struct R3 v;
  struct R3 a;
};

// Defines the length of a formatted rotation string.
// '(off: ' + o + ', vel: ' + v + ', acc: ' + a + ')' + NULL termination
#define ROT_FMT_LEN (6 + FMT_R3_LEN(8) + 5 + FMT_R3_LEN(8) + 5 + FMT_R3_LEN(8) + 1 + 1)

// Writes a formatted form of the rotation into the buffer. All rotations are printed in the following format:
// '(x, y, z)', where each element is at longest, a 10 digit number with a decimal. This max length is defined
// as ROT_FMT_LEN
void rot_format(struct Rotation* p, char* buf);

// Represents a position and orientation in 3D Euclidean Space. Contains both a position and an orientation.
struct Orientation
{
  struct Position pos;
  struct Rotation rot;
};

// Defines the length of a formatted rotation string.
// '(pos: ' + POS + ', rot: ' + ROT + ')' + NULL termination
#define ONT_FMT_LEN (6 + POS_FMT_LEN + 7 + ROT_FMT_LEN + 1 + 1)

// Writes a formatted form of the orientation into the buffer. All orientations are printed in the following format:
// '(pos: ' + POSITION + ', rot: ' + ROTATION + ')'
void ont_format(struct Orientation* p, char* buf);

#endif
