#ifndef MQTT_TEST_TKERNEL_H
#define MQTT_TEST_TKERNEL_H
#include "../../mtk_stubs/tk/tkernel.h"
typedef struct { PRI tskpri; } T_RTSK;
ER tk_ref_tsk(ID id,T_RTSK *status);
#endif
