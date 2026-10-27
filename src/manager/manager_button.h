/**
 * @file manager_button.h
 * @brief Short-press detection shared by the bare-metal and muT-Kernel
 *        adapters. Platform independent: the caller supplies the edge and the
 *        millisecond timestamp, so the host tests drive it directly.
 *
 * The GPIO interrupt only reports edges. A press shorter than
 * MANAGER_BUTTON_DEBOUNCE_MS is contact bounce and is dropped; a press longer
 * than MANAGER_BUTTON_SHORT_MAX_MS is a hold, not a toggle. Only the release
 * of a press inside that window counts.
 */
#ifndef MANAGER_BUTTON_H
#define MANAGER_BUTTON_H

#include <stdbool.h>
#include <stdint.h>

#include "manager_types.h"

typedef struct
{
    bool     down;          /* Raw level after the last edge.                 */
    uint32_t edge_ms;       /* Timestamp of the press edge.                   */
} manager_button_state_t;

/** Seed the filter with the level at boot so a held button is not a press. */
void manager_button_init(manager_button_state_t * p_state, bool down,
                         uint32_t now_ms);

/**
 * @brief Record one GPIO edge.
 * @param down true for a press edge, false for a release edge.
 * @return true when this release completes a debounced short press.
 */
bool manager_button_edge(manager_button_state_t * p_state, bool down,
                         uint32_t now_ms);

#endif /* MANAGER_BUTTON_H */
