#ifndef SRC_METRYHANDLE_H_
#define SRC_METRYHANDLE_H_

#include "generalPurposeFunctions.h"
#include "uartdrv.h"
#include "sl_uartdrv_instances.h"

#pragma pack(1)
typedef struct MetryDB
{
  uint8_t header;
  float angle;
  uint8_t footer;
}MetryDB;

void sendMetry();

#endif /* SRC_METRYHANDLE_H_ */
