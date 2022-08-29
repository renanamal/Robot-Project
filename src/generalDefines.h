#ifndef SRC_GENERALDEFINES_H_
#define SRC_GENERALDEFINES_H_


// Units conversion
#define PI (3.1415926535897932384626433832795)
#define RAD_PER_INTERAPT (PI/3.0)
#define PI_ANTIWINDUP (100.0)
#define IS_ZERO_FLOAT(dataIn) ((dataIn < 0.00001 && dataIn > -0.00001)?true:false)

#endif /* SRC_GENERALDEFINES_H_ */
