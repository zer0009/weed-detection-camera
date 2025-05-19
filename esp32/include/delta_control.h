// Delta robot control functions for Weed Farm Robot
// delta_control.h

#ifndef DELTA_CONTROL_H
#define DELTA_CONTROL_H

// Initialize the delta robot control system
void init_delta_control(void);

// Move delta robot to specific position
// x, y, z: target position in mm
// returns: true if movement successful, false otherwise
bool move_delta_to_position(int x, int y, int z);

// Move delta robot to home position
// returns: true if movement successful, false otherwise
bool move_delta_to_home(void);

// Engage the gripper mechanism
// action: 0=open, 1=close
// returns: true if action successful, false otherwise
bool control_gripper(int action);

// Check if delta robot is currently moving
// returns: true if robot is in motion, false if stationary
bool is_delta_moving(void);

// Perform weeding sequence at given position
// x, y: position of weed in delta robot coordinates
// returns: true if weeding successful, false otherwise
bool execute_weeding_sequence(int x, int y);

#endif // DELTA_CONTROL_H