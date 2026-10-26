#include "buddy1_command.h"
#include <string.h>

static void invalidate_command(b1_command_t *output)
{
    if (output != NULL) {
        output->session = 0;
        output->sequence = 0;
        output->opcode = B1_COMMAND_INVALID;
    }
}

b1_result_t b1_command_encode(const b1_command_t *command,
                              uint8_t *output, size_t length)
{
    if (command == NULL || output == NULL) {
        return B1_BAD_ARGUMENT;
    }
    if (length != B1_COMMAND_BYTES) {
        return B1_BAD_LENGTH;
    }
    if (command->opcode > B1_RESET) {
        return B1_BAD_COMMAND;
    }
    output[0] = (uint8_t)((B1_COMMAND_VERSION << 6) |
                          ((unsigned int)command->opcode << 4));
    output[1] = (uint8_t)(command->sequence >> 8);
    output[2] = (uint8_t)command->sequence;
    output[3] = (uint8_t)(command->session >> 24);
    output[4] = (uint8_t)(command->session >> 16);
    output[5] = (uint8_t)(command->session >> 8);
    output[6] = (uint8_t)command->session;
    return B1_OK;
}

b1_result_t b1_command_decode(const uint8_t *data, size_t length,
                              b1_command_t *output)
{
    invalidate_command(output);
    if (data == NULL || output == NULL) {
        return B1_BAD_ARGUMENT;
    }
    if (length != B1_COMMAND_BYTES) {
        return B1_BAD_LENGTH;
    }
    if ((data[0] >> 6) != B1_COMMAND_VERSION) {
        return B1_BAD_VERSION;
    }
    if ((data[0] & 0x0fu) != 0) {
        return B1_BAD_PADDING;
    }
    output->opcode = (uint8_t)((data[0] >> 4) & 0x03u);
    output->sequence = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
    output->session = ((uint32_t)data[3] << 24) | ((uint32_t)data[4] << 16) |
                      ((uint32_t)data[5] << 8) | data[6];
    return B1_OK;
}

void b1_assembler_abort(b1_assembler_t *assembler)
{
    if (assembler != NULL) {
        memset(assembler->data, 0, sizeof(assembler->data));
        assembler->used = 0;
        assembler->active = false;
    }
}

b1_result_t b1_assembler_begin(b1_assembler_t *assembler, size_t total_length)
{
    if (assembler == NULL) {
        return B1_BAD_ARGUMENT;
    }
    b1_assembler_abort(assembler);
    if (total_length != B1_COMMAND_BYTES) {
        return B1_BAD_LENGTH;
    }
    assembler->active = true;
    return B1_OK;
}

b1_result_t b1_assembler_feed(b1_assembler_t *assembler,
                            const uint8_t *data, size_t length, bool last,
                            b1_command_t *output)
{
    invalidate_command(output);
    if (assembler == NULL || output == NULL || (data == NULL && length != 0)) {
        b1_assembler_abort(assembler);
        return B1_BAD_ARGUMENT;
    }
    if (!assembler->active || assembler->used > B1_COMMAND_BYTES) {
        b1_assembler_abort(assembler);
        return B1_BAD_STATE;
    }
    if (length > B1_COMMAND_BYTES - assembler->used) {
        b1_assembler_abort(assembler);
        return B1_BAD_LENGTH;
    }
    if (length != 0) {
        memcpy(assembler->data + assembler->used, data, length);
        assembler->used = (uint8_t)(assembler->used + length);
    }
    if (!last) {
        return B1_MORE;
    }
    b1_result_t result = b1_command_decode(assembler->data, assembler->used, output);
    b1_assembler_abort(assembler);
    return result;
}

void b1_sequence_init(b1_sequence_t *state, uint32_t session)
{
    if (state != NULL) {
        state->session = session;
        state->last_sequence = 0;
        state->last_opcode = B1_COMMAND_INVALID;
        state->has_last = false;
    }
}

b1_sequence_result_t b1_sequence_check(const b1_sequence_t *state,
                                       const b1_command_t *command)
{
    if (state == NULL || command == NULL || command->opcode > B1_RESET) {
        return B1_SEQUENCE_INVALID;
    }
    if (command->session != state->session) {
        return B1_SEQUENCE_WRONG_SESSION;
    }
    if (!state->has_last || command->sequence > state->last_sequence) {
        return B1_SEQUENCE_NEW;
    }
    if (command->sequence < state->last_sequence) {
        return B1_SEQUENCE_OLD;
    }
    return command->opcode == state->last_opcode ? B1_SEQUENCE_DUPLICATE :
                                                 B1_SEQUENCE_CONFLICT;
}

b1_sequence_result_t b1_sequence_record(b1_sequence_t *state,
                                        const b1_command_t *command)
{
    b1_sequence_result_t result = b1_sequence_check(state, command);
    if (result == B1_SEQUENCE_NEW) {
        state->last_sequence = command->sequence;
        state->last_opcode = command->opcode;
        state->has_last = true;
    }
    return result;
}
