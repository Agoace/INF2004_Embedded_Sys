#include <assert.h>
#include <stdint.h>
#include "../app_program/vehicle/vehicle_board.h"
int main(void) {
    /* Columns: idle/uncalibrated, idle/calibrated, idle/log-busy,
     * active, action-pending. ESTOP/DUMP ignore those state restrictions. */
    static const uint8_t expected[6][5]={
        {5u,5u,5u,5u,5u},{0u,0u,0u,0u,0u},{0u,0u,0u,0u,0u},
        {0u,0u,3u,1u,1u},{2u,0u,3u,1u,1u},{5u,5u,5u,5u,5u}};
    for (unsigned int opcode=0u;opcode<256u;++opcode) {
        for (unsigned int flags=0u;flags<16u;++flags) {
            const bool active=(flags&1u)!=0u,pending=(flags&2u)!=0u;
            const bool busy=(flags&4u)!=0u,calibrated=(flags&8u)!=0u;
            const unsigned int column=active ? 3u : pending ? 4u : busy ? 2u : calibrated ? 1u : 0u;
            const uint8_t wanted=expected[opcode<6u ? opcode : 5u][column];
            assert(vehicle_remote_policy((uint8_t)opcode,active,pending,busy,calibrated)==wanted);
        }
    }
    return 0;
}
