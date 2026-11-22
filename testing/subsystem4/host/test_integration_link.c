/* S4-U11: actual S4 glue with minimal RTOS/S2 host stubs. No SMP claim. */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#define APP_H
#define IMPORT extern
#define EXPORT
#define Inline static inline
#define FALSE 0
#define TRUE 1
#define E_OK 0
#define E_OBJ (-41)
#define E_PAR (-17)
#define E_NOEXS (-42)
#define TP_PRC2 2
typedef int ID;
typedef int ER;
typedef int INT;
typedef int BOOL;
typedef unsigned int UW;
typedef unsigned int UINT;
typedef uint16_t UH;
typedef struct { void *next; } T_MSG;
typedef struct { UW hi,lo; } SYSTIM;
ER tk_get_tim(SYSTIM *time);
ER tk_set_flg(ID group,UINT bits);
#include "s4_input.c"
#include "s4_integration.c"
ID tid_imu_terrain=0;
static UW odometry,status;
static UINT now_ms,flags,flag_calls;
static ID last_group;
static bool changing;
static unsigned reads,checks,failures;
#define CHECK(x) do { ++checks; if(!(x)){++failures;printf("FAIL %d: %s\n",__LINE__,#x);} } while(0)
UW s2_get_odometry(void){++reads;return odometry+((changing&&reads%2U==0U)?1U:0U);}
UW s2_get_status(void){return status;}
ER tk_get_tim(SYSTIM *time){time->lo=now_ms;time->hi=0;return E_OK;}
ER tk_set_flg(ID group,UINT bits){last_group=group;flags|=bits;++flag_calls;return E_OK;}
int main(void)
{
    s4_odometry_t out;s4_height_t height;
    CHECK(subsystem4_connect_motion(0,4U)==E_PAR);
    CHECK(subsystem4_connect_motion(17,3U)==E_PAR);
    CHECK(subsystem4_connect_motion(17,4U)==E_OK);
    CHECK(subsystem4_highest_hump_mm()==-1);
    status=s2_status_pack(0,0,S2_STATE_TIMEOUT,FALSE,0);
    CHECK(!s4_read_odometry(&out));
    now_ms=20;odometry=s2_odo_pack(5,20);
    CHECK(s4_read_odometry(&out)&&out.valid&&out.forward&&out.distance_mm==5U);
    changing=true;reads=0;CHECK(!s4_read_odometry(&out)&&reads==2U);changing=false;
    now_ms=40;odometry=s2_odo_pack(10,40);
    status=s2_status_pack(100,-100,S2_STATE_TURNING,FALSE,0);
    CHECK(s4_read_odometry(&out)&&!out.forward);
    s4_height_init(&height);CHECK(s4_integration_publish(&height,FALSE,0U)==E_OK);
    CHECK(subsystem4_highest_hump_mm()==-1&&flag_calls==0U);
    height.highest_valid=true;height.highest_mm=63.4F;
    CHECK(s4_integration_publish(&height,FALSE,0U)==E_OK&&subsystem4_highest_hump_mm()==63);
    flags=2U;CHECK(s4_integration_publish(&height,TRUE,0U)==E_OK);
    CHECK(last_group==17&&flags==6U&&flag_calls==1U); /* preserved another owner's bit */
    CHECK(s4_integration_publish(&height,FALSE,0U)==E_OK&&flags==6U); /* no fault clearing */
    height.highest_mm=100.8F;s4_integration_publish(&height,FALSE,0U);
    CHECK(subsystem4_highest_hump_mm()==101);
    CHECK(subsystem4_completed_humps()==0U);
    s4_integration_publish(&height,FALSE,3U);
    CHECK(subsystem4_completed_humps()==3U&&subsystem4_highest_hump_mm()==101);
    tid_imu_terrain=1;CHECK(subsystem4_connect_motion(18,4U)==E_OBJ);
    printf("S4-U11: %s (%u checks, %u failures); host glue/flag stubs only.\n",failures?"FAIL":"PASS",checks,failures);
    return failures?1:0;
}
