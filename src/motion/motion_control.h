/**
 * @file motion_control.h
 * @brief Buddy 2 Motion Control library (FR5, FR6, FR14, NFR4, NFR5, NFR13,
 *        NFR16). RTOS-agnostic core: the μT-Kernel Motion Control Task (or the
 *        bare-metal bench target) calls motion_tick() every 20 ms.
 *
 * Ownership rules (Week 6 design + Tribunal 1):
 *  - Only this module writes motor PWM (GP8-GP11).
 *  - Other tasks request motion only through motion_command() (the
 *    mbf_motion_cmd payload), motion_set_steer() and motion_set_slow().
 *  - STEER is a latest-value variable, not a queued message, so STOP never
 *    queues behind STEER.
 *  - STOP/ESTOP skip SETTLE. motion_request_stop() zeroes PWM in the calling
 *    context and latches a stop flag that is checked before every PWM write.
 *  - Events are read with motion_get_event() and forwarded to mbf_ctrl_evt.
 *
 * Thread safety: motion_command(), motion_request_stop(), motion_set_steer()
 * and motion_set_slow() may be called from any task. motion_request_stop()
 * is also safe from an ISR. motion_tick() and motion_get_event() belong to
 * the Motion Control Task only.
 */
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "robot_shared.h"

/** Runtime-tunable gains (Q8: 256 = 1.0). For the A4 PID tuning report. */
typedef struct
{
    int32_t ff_q8;
    int32_t ff_offset_permille;
    int32_t kp_q8;
    int32_t ki_q8;
    int32_t ksync_q8;
    int32_t accel_q8_per_tick;  /* 0 = step changes (step-response test). */
} motion_gains_t;

/** One record per tick for step-response and accuracy logs. */
typedef struct
{
    uint32_t tick;
    uint8_t  state;
    int16_t  setpoint_cpt_q8[MOTOR_COUNT];  /* After ramp. */
    int16_t  speed_cpt[MOTOR_COUNT];        /* Measured counts this tick. */
    int16_t  duty_permille[MOTOR_COUNT];
    int32_t  travel[MOTOR_COUNT];           /* Counts since move start.   */
} motion_log_t;

typedef struct
{
    uint32_t ticks;
    uint32_t tick_wcet_us;      /* Longest motion_tick() execution.       */
    uint32_t tick_last_us;
    uint32_t tick_overruns;     /* Gaps between ticks > 25 ms.            */
    uint32_t cmd_dropped;       /* Queue full.                            */
    uint32_t evt_dropped;       /* Event ring full (should stay 0).       */
    uint32_t stop_requests;
} motion_stats_t;

typedef void (*motion_notify_fn_t)(void);
typedef void (*motion_log_fn_t)(const motion_log_t * p_log);

void           motion_init(void);

/** Queue a discrete command. STOP/ESTOP take the fast path. Returns false if
 *  the queue is full (the caller should retry or escalate). */
bool           motion_command(const motion_cmd_t * p_cmd);

/** Fast stop: PWM 0 now, flush queued commands, report on the next tick.
 *  estop = true latches FAULT_ESTOP until MOTION_CMD_RESET. ISR-safe. */
void           motion_request_stop(bool estop);

/** Latest-value steering (-127..+127). +127 = hard right (right wheel
 *  stopped), -127 = hard left. Write at least every 100 ms in FOLLOW. */
void           motion_set_steer(int8_t steer);

/** flg_mode SLOW: scale the cruise speed to MOTION_SLOW_PERCENT. */
void           motion_set_slow(bool slow);

/** Run one 20 ms control period. */
void           motion_tick(void);

/** Between ticks: act on a STOP and select a newly queued command at once
 *  (NFR4 "apply a new motion command as soon as it is received") without
 *  moving the 20 ms PID deadline. Call from the Motion task only, never
 *  concurrently with motion_tick(). */
void           motion_poll_commands(void);

bool           motion_get_event(motion_evt_t * p_evt);

/** Packed snapshot words (layout in robot_shared.h). */
void           motion_get_snapshot(uint32_t * p_w1, uint32_t * p_w2);

motion_state_t motion_get_state(void);
uint32_t       motion_get_fault_bits(void);

/** Reset the run odometer (VC calls this on RUN start). */
void           motion_odometer_reset(void);

void           motion_get_gains(motion_gains_t * p_gains);
void           motion_set_gains(const motion_gains_t * p_gains);
void           motion_get_stats(motion_stats_t * p_stats);

/** Called after a command is queued, so an RTOS task can wake at once
 *  (tk_wup_tsk). NULL to disable. */
void           motion_set_notify(motion_notify_fn_t fn);

/** Called at the end of every tick in the tick context. Keep it short. */
void           motion_set_log_hook(motion_log_fn_t fn);

const char *   motion_state_name(motion_state_t state);

#endif /* MOTION_CONTROL_H */
