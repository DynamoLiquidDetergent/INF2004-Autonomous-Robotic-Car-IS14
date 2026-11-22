/* S4-U09: compile the actual input implementation with minimal host API types.
 * These stubs test its contract, not RTOS scheduling or SMP synchronisation. */
#include <stdio.h>
#include <stdbool.h>
#define APP_H
#define IMPORT extern
#define EXPORT
#define FALSE 0
#define E_OK 0
#define E_OBJ (-41)
typedef int ID;
typedef int ER;
typedef int INT;
typedef int BOOL;
typedef unsigned int UW;
#include "s4_input.c"
ID tid_imu_terrain=0;
static unsigned checks,failures;
#define CHECK(x) do { ++checks; if(!(x)) { ++failures; \
    printf("FAIL line %d: %s\n",__LINE__,#x); } } while(0)
static bool good(s4_odometry_t *out)
{s4_odometry_decode_word(0x12340020U,true,out);return true;}
static bool rejected(s4_odometry_t *out)
{out->valid=true;return false;}
int main(void)
{
    s4_odometry_t out;
    CHECK(!s4_read_odometry(&out)&&!out.valid);
    CHECK(!s4_read_odometry(NULL));
    CHECK(subsystem4_bind_odometry(good)==E_OK);
    CHECK(s4_read_odometry(&out)&&out.valid&&out.distance_mm==0x1234U);
    tid_imu_terrain=1;CHECK(subsystem4_bind_odometry(NULL)==E_OBJ);
    CHECK(s4_read_odometry(&out)&&out.valid);
    tid_imu_terrain=-1;CHECK(subsystem4_bind_odometry(NULL)==E_OBJ);
    tid_imu_terrain=0;CHECK(subsystem4_bind_odometry(rejected)==E_OK);
    CHECK(!s4_read_odometry(&out)&&!out.valid);
    CHECK(subsystem4_bind_odometry(NULL)==E_OK);
    CHECK(!s4_read_odometry(&out)&&!out.valid);
    printf("S4-U09: %s (%u checks, %u failures). Host callback contract; no RTOS/SMP claim.\n",
           failures?"FAIL":"PASS",checks,failures);
    return failures?1:0;
}
