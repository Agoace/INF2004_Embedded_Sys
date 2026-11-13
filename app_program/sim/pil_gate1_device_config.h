/* Approved Gate1-only forced include for devinit.o, not a template edit. */
#ifndef PIL_GATE1_DEVICE_CONFIG_H
#define PIL_GATE1_DEVICE_CONFIG_H
#if !APP_SIM_GATE1 || !APP_SIM
#error Gate1 device selection requires its explicit SIM image
#endif
#include "../../config/config_device.h"
#undef DEVCNF_USE_SER
#define DEVCNF_USE_SER 0
#endif
