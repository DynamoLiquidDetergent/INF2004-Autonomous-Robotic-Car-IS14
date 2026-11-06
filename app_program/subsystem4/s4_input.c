#include "subsystem4.h"

static s4_read_odometry_fn read_odometry=NULL;

EXPORT ER subsystem4_bind_odometry(s4_read_odometry_fn read_fn)
{
    /* S5 init may bind/unbind before subsystem4_start. No runtime rebinding:
     * on SMP, another core may already be executing the S4 callback. */
    if(tid_imu_terrain!=0){return E_OBJ;}
    read_odometry=read_fn;
    return E_OK;
}
EXPORT BOOL s4_read_odometry(s4_odometry_t *out)
{
    bool received;
    if(out==NULL){return FALSE;}
    *out=(s4_odometry_t){0};
    received=(read_odometry!=NULL)&&read_odometry(out);
    if(!received){out->valid=false;}
    return received&&out->valid;
}
