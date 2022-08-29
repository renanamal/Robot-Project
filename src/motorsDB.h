#ifndef SRC_MOTORSCHARACTERSDB_H_
#define SRC_MOTORSCHARACTERSDB_H_

#define SPEED_CONSTANT (6240)
#define SPEED_TO_TORQUE_GRAD (93200)
#define TORQUE_CONST (1.53)
#define DRIVE_MOTOR_GEAR_EFFICIENCY (0.76)
#define EFFICIENCY (0.63 * DRIVE_MOTOR_GEAR_EFFICIENCY)
#define GEAR_RATIO (17.0)
#define INV_GEAR_RATIO (1.0/GEAR_RATIO)

// control gains
// TODO - if needed define KI, KP for each motor
#define KI (0.01)
#define KP (0.1)

#endif /* SRC_MOTORSCHARACTERSDB_H_ */
