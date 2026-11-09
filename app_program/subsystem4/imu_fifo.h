/* Pure FIFO status/word decode. No SDK, kernel or driving decisions. */
#ifndef IMU_FIFO_H
#define IMU_FIFO_H
#include <stdbool.h>
#include <stdint.h>
#define IMU_FIFO_MAX_BATCH 8u
#define IMU_FIFO_ODR_HZ 400u
#define IMU_FIFO_EMPTY 0x20u
#define IMU_FIFO_OVERRUN 0x40u
/* Conservative: full/overrun wins over contradictory empty/count bits. */
bool imu_fifo_count(uint8_t status, unsigned int *count);
/* Preserve the exact left-aligned 16-bit bus words for TEST capture. */
void imu_fifo_words(const uint8_t bytes[6], int16_t words[3]);
/* HR +/-2 g: discard the low 4 bits before averaging; keep fractional mg. */
bool imu_fifo_mean(const int16_t words[][3], unsigned int count,
                    float mean[3]);
#endif
