#include <assert.h>
#include <stddef.h>
#include "../app_program/app.h"
#include "../app_program/subsystem4/imu_service.h"

int main(void)
{
    imu_snapshot_t out = {0};

    assert(imu_service_init() == E_OK);
    assert(imu_service_start() == E_OK);
    assert(imu_snapshot_try_read(NULL) == E_PAR);
    assert(imu_snapshot_try_read(&out) == E_OBJ);
    assert(imu_request_mark() == 0u);
    return 0;
}
