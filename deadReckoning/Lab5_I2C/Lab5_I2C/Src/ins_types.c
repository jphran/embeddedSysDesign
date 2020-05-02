#include <ins_types.h>

// Writes a formatted form of the position into the buffer. All positions are printed in the following format:
// '(x, y, z)', where each element is a 10 digit number with a decimal.
void pos_format(struct Position* p, char* buf)
{
  int32_t prec = 4;
  // Offset
  char off_str[FMT_R3_LEN(prec)];
  FMT_R3((&(p->o)), off_str, prec);
  // Velocity
  char vel_str[FMT_R3_LEN(prec)];
  FMT_R3((&(p->v)), vel_str, prec);
  // Acceleration
  char acc_str[FMT_R3_LEN(prec)];
  FMT_R3((&(p->a)), acc_str, prec);
  snprintf(buf, POS_FMT_LEN,
           "(off: %s, vel: %s, acc: %s)",
           off_str,
           vel_str,
           acc_str);
}

// Writes a formatted form of the position into the buffer. All positions are printed in the following format:
// '(x, y, z)', where each element is a 10 digit number with a decimal.
void rot_format(struct Rotation* r, char* buf)
{
  int32_t prec = 8;
  // Offset
  char off_str[FMT_R3_LEN(prec)];
  FMT_R3((&(r->o)), off_str, prec);
  // Velocity
  char vel_str[FMT_R3_LEN(prec)];
  FMT_R3((&(r->v)), vel_str, prec);
  // Acceleration
  char acc_str[FMT_R3_LEN(prec)];
  FMT_R3((&(r->a)), acc_str, prec);
  snprintf(buf, POS_FMT_LEN,
           "(off: %s, vel: %s, acc: %s)",
           off_str,
           vel_str,
           acc_str);
}

// Writes a formatted form of the orientation into the buffer. All orientations are printed in the following format:
// '(pos: ' + POSITION + ', rot: ' + ROTATION + ')'
void ont_format(struct Orientation* o, char* buf)
{
  char pos_str[POS_FMT_LEN];
  pos_format(&o->pos, pos_str);
  char rot_str[ROT_FMT_LEN];
  rot_format(&o->rot, rot_str);
  snprintf(buf, ONT_FMT_LEN, "(pos: %s, rot: %s)", pos_str, rot_str);
}
