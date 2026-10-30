/**
 * @file line_producer.h
 * @brief The Line component: samples the sensors, publishes the observation and
 *        owns the learned calibration.
 *
 * This is the piece the approved plan calls "Line component". It owns
 * `lp_t`, the health check and the 1 kHz cadence, and it never calls the
 * Manager or Motion: each step hands back one packed observation and, at most,
 * one discrete edge for the caller to publish.
 *
 * Calibration is driven by the Manager's commands. BEGIN calls
 * `lp_calib_begin()`, which invalidates the usable calibration first, so a
 * sweep that fails or is cancelled can never fall back to the old endpoints.
 *
 * Host-testable: depends only on the `line_adc.h` seam.
 */
#ifndef LINE_PRODUCER_H
#define LINE_PRODUCER_H

#include <stdbool.h>
#include <stdint.h>

#include "line_adc.h"
#include "line_health.h"
#include "line_position.h"
#include "line_sampler.h"
#include "line_types.h"

/**
 * Wheel-count differential that counts as a turn for the side hint.
 *
 * Upstream YX supplies the hint at 20 counts, measured on the *accumulated*
 * left-minus-right heading rather than on a single sample, so the value and the
 * accumulation both carry over.
 */
#define LINE_HINT_COUNTS 20

/**
 * Wheel travel that ends a junction straight-through.
 *
 * Upstream measures signed mean-wheel travel from the junction hold's rising
 * edge, at 450 counts. Signed, so reversing during a junction cannot spend the
 * budget early. After it the Manager resumes ordinary position steering instead
 * of driving straight for an unbounded junction.
 */
#define LINE_JUNCTION_STRAIGHT_COUNTS 450

typedef struct
{
    lp_t           lp;
    line_health_t  health;
    line_sampler_t sampler;
    uint32_t       last_sample_ms;
    bool           have_sample;
    uint8_t        fault_mask;
    uint16_t       raw_last[LINE_ADC_SENSORS];

    /* Reader-owned odometer marks, for the side hint and the junction bound. */
    int32_t        last_left_counts;
    int32_t        last_right_counts;
    int32_t        mean_counts;     /**< signed mean wheel travel right now    */
    bool           have_counts;

    /**
     * Left-minus-right counts accumulated since the line was centred.
     *
     * Upstream applies the hint threshold to this total, not to one sample, so a
     * slow drift of a couple of counts per millisecond still registers. Cleared
     * whenever the line is centred and whenever a FOUND edge ends a search.
     */
    int32_t        heading_counts;

    /** Signed mean travel at the junction hold's rising edge. */
    int32_t        junction_from;
    bool           junction_active;
} line_producer_t;

typedef struct
{
    line_observation_t observation;
    bool               got_observation;   /**< publish it this step          */
    line_event_t       event;
    bool               has_event;         /**< post the edge this step       */
    uint32_t           overruns;          /**< cumulative missed slots       */
} line_producer_out_t;

void line_producer_init(line_producer_t * p_producer, uint32_t now_ms);

/**
 * @brief One 1 kHz tick.
 *
 * Reads the sensors only when the cadence says a slot is due, so a caller that
 * runs faster than 1 kHz costs nothing.
 */
void line_producer_step(line_producer_t * p_producer, uint32_t now_ms,
                        line_producer_out_t * p_out);

/**
 * @brief Pack the current line state, without reading the ADC.
 *
 * A caller that services a command which changes the packed state in the same
 * cycle (a finished calibration, for example) must re-pack after the command,
 * otherwise it publishes a word that is already out of date.
 */
void line_producer_observation(const line_producer_t * p_producer,
                               line_observation_t * p_obs);

/**
 * @brief Feed the reader-owned wheel odometer.
 *
 * Takes the two wheel totals (32-bit and monotonic) and derives the YX side
 * hint: a differential of at least LINE_HINT_COUNTS means the chassis is
 * rotating, so the line is moving the other way. Upstream's `steer_test.c`
 * supplies exactly this input, and without it the learner has no way to know
 * which way the robot is turning when an outer sensor goes dark.
 *
 * Call once per sample, before line_producer_step(), from a NON-consuming
 * odometer, so the IMU's own travel feed keeps its marks.
 */
void line_producer_update_travel(line_producer_t * p_producer,
                                 int32_t left_counts, int32_t right_counts);

/* ---- Calibration service (driven by the Manager) ----------------------- */

/** Begin a sweep. Invalidates the usable calibration first. */
void line_producer_calib_begin(line_producer_t * p_producer);

/** Fold one raw reading into the sweep. */
void line_producer_calib_add(line_producer_t * p_producer,
                             const uint16_t raw[LINE_ADC_SENSORS]);

/**
 * @brief The raw readings from the most recent sample.
 *
 * Lets the calibration collect the same sample the observation used, instead of
 * reading the ADC twice in one millisecond.
 */
const uint16_t * line_producer_last_raw(const line_producer_t * p_producer);

/** Finish the sweep and build the outcome event. */
void line_producer_calib_finish(line_producer_t * p_producer,
                                line_calib_event_t * p_out);

#endif /* LINE_PRODUCER_H */
