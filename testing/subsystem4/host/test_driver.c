#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "drv_motion_sensor.h"
static unsigned char regs[2][256];
static int fail;
static int io(void *c,uint8_t a,uint8_t r,uint8_t *b,s4_size_t n,bool read)
{
    unsigned i; unsigned dev = (a == 0x19U) ? 0U : 1U;
    (void)c;
    if (fail) { return -1; }
    if (dev == 0U) { r &= 0x7fU; }
    for (i=0U;i<n;++i)
    { if(read){b[i]=regs[dev][r+i];}else{regs[dev][r+i]=b[i];} }
    return 0;
}
int main(void)
{
    s4_bus_t bus={io,0}; s4_sample_t s={0};
    memcpy(&regs[1][10],"H43",3);
    assert(s4_sensor_init(&bus)==0);
    assert(regs[0][0x20]==0x47U && regs[0][0x23]==0x98U);
    assert(regs[1][0]==0x18U);
    regs[0][0x27]=8; regs[0][0x2d]=0x1f; regs[0][0x2c]=0x40;
    regs[1][9]=1; regs[1][3]=0x04; regs[1][4]=0x4c;
    regs[1][5]=0x03; regs[1][6]=0xd4;
    assert(s4_sensor_read(&bus,&s)==0 && s.accel_fresh && s.magnetic_fresh);
    assert(s.accel_g[2]>0.999F && s.accel_g[2]<1.001F);
    assert(s.magnetic_gauss[0]>0.999F && s.magnetic_gauss[2]>0.999F);
    regs[1][3]=0xf0; regs[1][4]=0x00;
    assert(s4_sensor_read(&bus,&s)==0 && !s.saturated && !s.magnetic_fresh);
    regs[0][0x27]=0; regs[1][9]=0;
    assert(s4_sensor_read(&bus,&s)==0 && !s.accel_fresh);
    fail=1; assert(s4_sensor_read(&bus,&s)<0);
    fail=0; regs[1][10]='X'; assert(s4_sensor_init(&bus)<0);
    puts("driver: PASS (configuration, endian/order, freshness, overflow, I/O error)");
    return 0;
}
