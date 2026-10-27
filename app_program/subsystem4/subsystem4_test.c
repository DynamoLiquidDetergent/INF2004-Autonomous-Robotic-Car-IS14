#include "subsystem4.h"
#include "../test_report.h"
#if APP_TEST
EXPORT void subsystem4_test(void)
{
    T_RTSK task;
    TEST_INIT();
    TEST_CHECK("S4-U03",
               tk_ref_tsk(tid_imu_terrain, &task) == E_OK &&
               task.tskpri == S4_PRIORITY,
               "IMU task exists at priority 18; no hardware yet");
    TEST_SUMMARY("subsystem4");
}
#endif
