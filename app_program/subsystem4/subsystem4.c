#include "subsystem4.h"
#include "imu_service.h"

EXPORT ER subsystem4_init(void)
{
    return (ER)imu_service_init();
}

EXPORT ER subsystem4_start(void)
{
    return (ER)imu_service_start();
}
