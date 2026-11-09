#ifndef TEST_IMU_SERVICE_HOOK_H
#define TEST_IMU_SERVICE_HOOK_H
void test_imu_board_hook(unsigned int index);
#define APP_BOARD_READ_HOOK(sequence, index) test_imu_board_hook(index)
#endif
