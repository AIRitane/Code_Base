#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H
#include "ChassisTask.h"
#include "main.h"

extern float SupKp;
extern uint8_t CalculCount;
void ChassisPowerControl(ChassisCtrl_t *ChassisCtrl);

#endif
