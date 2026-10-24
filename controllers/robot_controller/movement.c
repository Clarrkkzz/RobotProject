/*
 * File:          robot_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <math.h>
#include <stdbool.h> 
#define TIME_STEP 64

void move_forward_aligned(WbDeviceTag left, WbDeviceTag right, double speed);
bool turn_left(void);
bool turn_right(void);
bool move_backward(void);
void full_stop(void);
 
int main(int argc, char **argv) {
  wb_robot_init();
  
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);


  while (wb_robot_step(TIME_STEP) != -1) {
    move_forward_aligned(left_motor, right_motor, 2.0); 
  }

  wb_robot_cleanup();

  return 0;
}
test
test2
test3 
test4
void move_forward_aligned(WbDeviceTag left, WbDeviceTag right, double speed) {
   wb_motor_set_velocity(left, speed);
   wb_motor_set_velocity(right, speed);
}