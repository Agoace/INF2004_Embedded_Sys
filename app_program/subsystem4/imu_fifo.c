/* LSM303DLHC FIFO_SRC_REG_A fields: datasheet DocID018771 Rev 2, 7.1.13,
 * Table 39. Count/OVRN meaning: AN3308 Rev 1 (LIS3DH), 9.2.3, Table 38. */
#include "imu_fifo.h"
#include <stddef.h>

bool imu_fifo_count(uint8_t status, unsigned int *count)
{
    if (count == NULL) {
        return false;
    }
    *count = status & 0x1fu;
    if ((status & IMU_FIFO_OVERRUN) != 0u) {
        *count = 32u;
        return false;
    }
    if ((status & IMU_FIFO_EMPTY) != 0u) {
        return *count == 0u;
    }
    return *count > 0u;
}

void imu_fifo_words(const uint8_t bytes[6], int16_t words[3])
{
    for (unsigned int i = 0u; i < 3u; ++i) {
        const uint16_t word = (uint16_t)((uint16_t)bytes[2u * i] |
                              ((uint16_t)bytes[2u * i + 1u] << 8));
        const int32_t signed_word = word >= 32768u ?
                                   (int32_t)word - 65536 : (int32_t)word;

        words[i] = (int16_t)signed_word;
    }
}

bool imu_fifo_mean(const int16_t words[][3], unsigned int count,
                    float mean[3])
{
    if (words == NULL || mean == NULL || count == 0u ||
        count > IMU_FIFO_MAX_BATCH) {
        return false;
    }
    for (unsigned int axis = 0u; axis < 3u; ++axis) {
        int32_t sum = 0;

        for (unsigned int i = 0u; i < count; ++i) {
            const uint16_t word = (uint16_t)words[i][axis];
            const int32_t raw = (int32_t)(word / 16u) -
                                ((word & 0x8000u) != 0u ? 4096 : 0);

            sum += raw;
        }
        mean[axis] = (float)sum / (float)count;
    }
    return true;
}
