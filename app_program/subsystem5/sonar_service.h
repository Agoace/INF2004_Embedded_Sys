#ifndef SONAR_SERVICE_H
#define SONAR_SERVICE_H
#include <stdbool.h>
#include <stdint.h>
#include "ultrasonic.h"
typedef enum { SONAR_STOP,SONAR_WATCH,SONAR_ONCE,SONAR_ACK } sonar_mode_t;
typedef struct {
    uint32_t generation,id,deadline_ms;
    uint16_t interval_ms;
    int8_t angle;
    uint8_t mode;
} sonar_request_t;
typedef enum { SONAR_EMPTY,SONAR_ECHO,SONAR_DEADLINE,SONAR_CANCELLED } sonar_status_t;
typedef struct {
    uint32_t generation,id,ping_sequence,result_sequence,trigger_ms,timestamp_ms,deadline_ms;
    ultrasonic_result_t echo;
    int8_t angle;
    uint8_t status;
    bool pending,valid;
} sonar_snapshot_t;
int sonar_service_init(void);
int sonar_service_start(void);
/* Copies by value through MBF, TMO_POL; callers must retry or stop on failure. */
int sonar_request(const sonar_request_t *request);
int sonar_result_read(sonar_snapshot_t *out,bool *new_result);
#endif
