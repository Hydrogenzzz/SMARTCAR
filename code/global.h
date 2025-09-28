#ifndef _global_h_
#define _global_h_

#include "common.h"
#include "server.h"
#include "screen.h"
#include "menu.h"
#include "key.h"
#include "my_flash.h"
#include "BlueTooth.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "image.h"

void allinitialize(void);
extern float speedleft;
extern float speedright;
extern float target_left;
extern float target_right;
extern float target_angle;
extern uint8_t bin_image;
#endif
