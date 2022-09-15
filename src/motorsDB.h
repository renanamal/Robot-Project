#ifndef SRC_MOTORSCHARACTERSDB_H_
#define SRC_MOTORSCHARACTERSDB_H_

#define MOTOR_SPEED_CONSTANT (25800)  // [RPM/V]
#define SPEED_TO_TORQUE_GRAD (53400) // [RPM/mNm]
#define TORQUE_CONST (0.37) // [mNm/A]
#define MOTOR_GEAR_EFFICIENCY (0.76)
#define MOTOR_EFFICIENCY (0.63 * MOTOR_GEAR_EFFICIENCY)
#define GEAR_RATIO (15.0)
#define INV_GEAR_RATIO (1.0/GEAR_RATIO)
#define POWER_SUPPLY_VOLTAGE (3.3)
#define STARTING_VOLTAGE (1.0)
// control gains
// TODO - if needed define KI, KP for each motor
#define KI (0.1)
#define KP (1.0)


#define MOTOR_SPEED_CONTROLLER_HZ (100.0) // TODO - change to the correct Hz

#endif /* SRC_MOTORSCHARACTERSDB_H_ */
