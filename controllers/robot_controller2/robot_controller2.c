#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#define TIME_STEP 32
#define TARGET_THRESHOLD 80.0  // Above ~61-73 ambient noise, below ~100+ wall readings
#define MAX_SPEED 5.0
#define TURN_SPEED 2.5         // Slower pivot speed prevents overshooting corners

// Function prototypes
void moveForward(WbDeviceTag lm, WbDeviceTag rm);
void leftRotate(WbDeviceTag lm, WbDeviceTag rm);
void rightRotate(WbDeviceTag lm, WbDeviceTag rm);
void curveLeft(WbDeviceTag lm, WbDeviceTag rm);

bool getFrontWall(WbDeviceTag ps0, WbDeviceTag ps7);
bool getLeftWall(WbDeviceTag ps5);
bool getRightWall(WbDeviceTag ps2);

int main(int argc, char **argv) {
    wb_robot_init();

    // 1. Initialize Motors in velocity control mode
    WbDeviceTag lm = wb_robot_get_device("left wheel motor");
    WbDeviceTag rm = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(lm, INFINITY);
    wb_motor_set_position(rm, INFINITY);
    wb_motor_set_velocity(lm, 0.0);
    wb_motor_set_velocity(rm, 0.0);

    // 2. Initialize Distance Sensors
    WbDeviceTag ps[8];
    const char *ps_names[8] = {
        "ps0", "ps1", "ps2", "ps3",
        "ps4", "ps5", "ps6", "ps7"
    };

    for (int i = 0; i < 8; i++) {
        ps[i] = wb_robot_get_device(ps_names[i]);
        wb_distance_sensor_enable(ps[i], TIME_STEP);
    }

    // Main Control Loop
    while (wb_robot_step(TIME_STEP) != -1) {
        bool front_wall = getFrontWall(ps[0], ps[7]); // ps0 (Front-R) & ps7 (Front-L)
        bool left_wall  = getLeftWall(ps[5]);         // ps5 (Left side)
        bool left_corner = (wb_distance_sensor_get_value(ps[6]) > TARGET_THRESHOLD); // ps6 (Front-Left 45 deg)

        if (front_wall) {
            // Wall ahead: pivot right in place
            rightRotate(lm, rm);
        } else if (left_corner) {
            // Getting too close to the left wall at an angle: nudge right
            rightRotate(lm, rm);
        } else if (left_wall) {
            // Aligned with left wall: drive straight
            moveForward(lm, rm);
        } else {
            // Lost the left wall (opening/corner): arc forward-left to find it
            curveLeft(lm, rm);
        }
    }

    wb_robot_cleanup();
    return 0;
}

// --- Sensor Functions ---
bool getFrontWall(WbDeviceTag ps0, WbDeviceTag ps7) {
    double right_front = wb_distance_sensor_get_value(ps0);
    double left_front  = wb_distance_sensor_get_value(ps7);
    if (left_front > TARGET_THRESHOLD) {
        printf("Front wall detected (ps0: %.1f, ps7: %.1f)\n", right_front, left_front);
        return true;
    }
    return false;
}

bool getLeftWall(WbDeviceTag ps5) {
    double value = wb_distance_sensor_get_value(ps5);
    if (value > TARGET_THRESHOLD) {
        printf("Left wall detected (ps5: %.1f)\n", value);
        return true;
    }
    return false;
}

bool getRightWall(WbDeviceTag ps2) {
    double value = wb_distance_sensor_get_value(ps2);
    if (value > TARGET_THRESHOLD) {
        printf("Right wall detected (ps2: %.1f)\n", value);
        return true;
    }
    return false;
}

// --- Movement Functions ---
void moveForward(WbDeviceTag lm, WbDeviceTag rm) {
    wb_motor_set_velocity(lm, MAX_SPEED);
    wb_motor_set_velocity(rm, MAX_SPEED);
}

void rightRotate(WbDeviceTag lm, WbDeviceTag rm) {
    wb_motor_set_velocity(lm, TURN_SPEED);
    wb_motor_set_velocity(rm, -TURN_SPEED);
}

void leftRotate(WbDeviceTag lm, WbDeviceTag rm) {
    wb_motor_set_velocity(lm, -TURN_SPEED);
    wb_motor_set_velocity(rm, TURN_SPEED);
}

void curveLeft(WbDeviceTag lm, WbDeviceTag rm) {
    // Drives forward while turning left so the robot doesn't spin in place forever in open space
    wb_motor_set_velocity(lm, MAX_SPEED * 0.35);
    wb_motor_set_velocity(rm, MAX_SPEED);
}