#ifndef BUDDY1_COMMAND_H
#define BUDDY1_COMMAND_H

/* Provisional bench protocol. Not approved for live robot commands.
 * No function in this module accepts or executes a Vehicle Controller action.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define B1_COMMAND_BYTES 7u
#define B1_COMMAND_VERSION 1u
#define B1_START 0u
#define B1_STOP 1u
#define B1_ESTOP 2u
#define B1_RESET 3u
#define B1_COMMAND_INVALID UINT8_MAX

/* Working values only: never transmit this C structure. */
typedef struct {
    uint32_t session;
    uint16_t sequence;
    uint8_t opcode;
} b1_command_t;

typedef enum {
    B1_OK = 0,
    B1_MORE,
    B1_BAD_ARGUMENT,
    B1_BAD_LENGTH,
    B1_BAD_VERSION,
    B1_BAD_PADDING,
    B1_BAD_COMMAND,
    B1_BAD_STATE
} b1_result_t;

/* Buffer lengths must be exactly seven. Output is invalidated on decode
 * failure. Encode writes nothing on failure. Input/output must not overlap.
 */
b1_result_t b1_command_encode(const b1_command_t *command,
                              uint8_t *output, size_t length);
b1_result_t b1_command_decode(const uint8_t *data, size_t length,
                              b1_command_t *output);

/* Caller-owned state. Initialize with {0}; do not edit fields directly.
 * These objects require one owner or external synchronization.
 */
typedef struct {
    uint8_t data[B1_COMMAND_BYTES];
    uint8_t used;
    bool active;
} b1_assembler_t;

/* Call begin once for each publication, including rejected publications.
 * Begin abandons any preceding partial payload. Feed never retains pointers.
 * Any error closes the assembly until the next begin. Only B1_OK supplies a
 * command; B1_MORE and errors invalidate output. No queue is provided.
 */
b1_result_t b1_assembler_begin(b1_assembler_t *assembler, size_t total_length);
b1_result_t b1_assembler_feed(b1_assembler_t *assembler,
                            const uint8_t *data, size_t length, bool last,
                            b1_command_t *output);
void b1_assembler_abort(b1_assembler_t *assembler);

typedef enum {
    B1_SEQUENCE_NEW = 0,
    B1_SEQUENCE_DUPLICATE,
    B1_SEQUENCE_CONFLICT,
    B1_SEQUENCE_OLD,
    B1_SEQUENCE_WRONG_SESSION,
    B1_SEQUENCE_INVALID
} b1_sequence_result_t;

typedef struct {
    uint32_t session;
    uint16_t last_sequence;
    uint8_t last_opcode;
    bool has_last;
} b1_sequence_t;

/* First sequence may be any uint16_t; later sequences strictly increase.
 * Gaps are allowed; wrap is rejected. Only the latest recorded command can
 * be classified as DUPLICATE/CONFLICT; earlier commands are OLD.
 * The caller supplies a session. This module does not create fresh sessions.
 */
void b1_sequence_init(b1_sequence_t *state, uint32_t session);
b1_sequence_result_t b1_sequence_check(const b1_sequence_t *state,
                                       const b1_command_t *command);
/* Explicit history update, with a fresh check. Returns NEW only if recorded.
 * The caller must decide when recording is justified. This is not controller
 * acceptance, an acknowledgement, a reservation, or an execution operation.
 */
b1_sequence_result_t b1_sequence_record(b1_sequence_t *state,
                                        const b1_command_t *command);

#endif
