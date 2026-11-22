/* S4-U07: synthetic inputs exercise the real height code, not sensor accuracy. */
#include <stdio.h>
#include "test_math.h"
#include "s4_odometry.h"
#include "s4_terrain.h"

static unsigned checks,failures;
#define CHECK(x) do { ++checks; if(!(x)) { ++failures; \
    printf("FAIL line %d: %s\n",__LINE__,#x); } } while(0)
typedef struct
{
    s4_height_t height;
    s4_odometry_t odometry;
    s4_orientation_t angle;
    uint32_t us,ms;
} fixture;
static void stamp(fixture *f)
{
    f->odometry.timestamp=(f->odometry.clock_format==S4_ODOM_MS16)?
                           (uint16_t)f->ms:f->us;
}
static void step(fixture *f,float pitch,uint8_t state,bool completed,bool aborted,
                  bool fresh,bool clock_ok)
{
    f->angle.pitch_deg=pitch;
    s4_height_step(&f->height,&f->odometry,&f->angle,fresh,clock_ok,
                  f->us,f->ms,state,2.0F,completed,aborted);
}
static void advance(fixture *f,unsigned mm,unsigned ms)
{
    f->ms+=ms;f->us+=ms*1000U;
    f->odometry.distance_mm=(uint16_t)(f->odometry.distance_mm+mm);stamp(f);
}
static void init(fixture *f,unsigned format,uint16_t distance,uint32_t ms,uint32_t us)
{
    *f=(fixture){0};s4_height_init(&f->height);
    f->us=us;f->ms=ms;f->odometry.clock_format=(uint8_t)format;
    f->odometry.distance_mm=distance;f->odometry.valid=true;
    f->odometry.forward=true;f->angle.orientation_valid=true;stamp(f);
    step(f,0.0F,S4_LEVEL,false,false,true,true);
}
static void up(fixture *f)
{advance(f,10U,20U);step(f,30.0F,S4_CLIMBING,false,false,true,true);}
static void finish(fixture *f)
{
    advance(f,10U,20U);step(f,-30.0F,S4_DESCENDING,false,false,true,true);
    advance(f,0U,20U);step(f,0.0F,S4_LEVEL,true,false,true,true);
}
static float profile(fixture *f,float pitch,unsigned mm)
{
    unsigned i;float expected=0.0F,previous=0.0F;
    /* Independent reference sine from the host C library. Deliberately
     * synthetic: 20 ten-mm uphill steps, a stopped crest, then downhill. */
    for(i=0U;i<20U;++i)
    {
        float sine=test_sin(pitch*0.01745329252F);
        expected+=(float)mm*0.5F*(previous+sine);previous=sine;
        advance(f,mm,20U);step(f,pitch,S4_CLIMBING,false,false,true,true);
    }
    advance(f,0U,20U);step(f,0.0F,S4_CLIMBING,false,false,true,true);
    for(i=0U;i<20U;++i)
    {advance(f,mm,20U);step(f,-pitch,S4_DESCENDING,false,false,true,true);}
    advance(f,0U,20U);step(f,0.0F,S4_LEVEL,true,false,true,true);
    return expected;
}
int main(void)
{
    fixture f;s4_odometry_t decoded;float expected,highest,before;
    unsigned format,i;
    s4_odometry_decode_word(0x1234abcdU,true,&decoded);
    CHECK(decoded.distance_mm==0x1234U&&decoded.timestamp==0xabcdU);
    CHECK(decoded.clock_format==S4_ODOM_MS16&&decoded.valid&&decoded.forward);
    s4_odometry_decode_word(0U,false,&decoded);CHECK(!decoded.forward);
    for(format=0U;format<=1U;++format)
    {
        init(&f,format,65500U,65500U,0xffff0000U);
        expected=profile(&f,30.0F,10U);
        CHECK(f.height.last_valid&&f.height.highest_valid);
        CHECK(test_abs(expected-97.5F)<0.001F);
        CHECK(test_abs(f.height.last_mm-expected)<0.01F);
        highest=f.height.highest_mm;
        expected=profile(&f,15.0F,10U);
        CHECK(f.height.last_valid&&test_abs(f.height.last_mm-expected)<0.01F);
        CHECK(f.height.highest_mm==highest);
        expected=profile(&f,45.0F,10U);
        CHECK(f.height.last_valid&&test_abs(f.height.highest_mm-expected)<0.01F);
    }
    init(&f,S4_ODOM_US32,0U,0U,0U);profile(&f,30.0F,0U);
    CHECK(!f.height.last_valid&&!f.height.highest_valid);
    CHECK(f.height.reason==S4_HEIGHT_NO_MOVEMENT);

    /* Input failures permanently invalidate the current traversal. */
    for(i=0U;i<8U;++i)
    {
        init(&f,S4_ODOM_US32,0U,0U,0U);up(&f);advance(&f,10U,20U);
        if(i==0U){f.odometry.valid=false;}
        if(i==1U){f.odometry.forward=false;}
        if(i==2U){f.angle.orientation_valid=false;}
        if(i==3U){f.odometry.timestamp=f.us-41000U;}
        if(i==4U){f.odometry.timestamp=f.us+1U;}
        if(i==5U){f.odometry.clock_format=9U;}
        step(&f,30.0F,S4_CLIMBING,false,false,i!=6U,i!=7U);
        CHECK(!f.height.chain_valid);
        f.odometry.valid=true;f.odometry.forward=true;
        f.angle.orientation_valid=true;f.odometry.clock_format=S4_ODOM_US32;stamp(&f);
        finish(&f);CHECK(!f.height.last_valid&&!f.height.highest_valid);
    }
    init(&f,S4_ODOM_US32,0U,0U,0U);up(&f);advance(&f,10U,80U);
    step(&f,30.0F,S4_CLIMBING,false,false,true,true);
    CHECK(f.height.reason==S4_HEIGHT_GAP);finish(&f);CHECK(!f.height.last_valid);
    init(&f,S4_ODOM_US32,0U,0U,0U);up(&f);advance(&f,51U,20U);
    step(&f,30.0F,S4_CLIMBING,false,false,true,true);
    CHECK(f.height.reason==S4_HEIGHT_JUMP);finish(&f);CHECK(!f.height.last_valid);
    init(&f,S4_ODOM_US32,0U,0U,0U);up(&f);
    f.odometry.clock_format=S4_ODOM_MS16;advance(&f,10U,20U);
    step(&f,30.0F,S4_CLIMBING,false,false,true,true);
    CHECK(f.height.reason==S4_HEIGHT_FORMAT);

    /* A duplicate must not integrate twice or replace its original pitch. */
    init(&f,S4_ODOM_US32,0U,0U,0U);up(&f);before=f.height.current_mm;
    f.ms+=20U;f.us+=20000U;
    step(&f,60.0F,S4_CLIMBING,false,false,true,true);
    CHECK(f.height.current_mm==before&&f.height.chain_valid);
    advance(&f,10U,20U);step(&f,30.0F,S4_CLIMBING,false,false,true,true);
    CHECK(test_abs(f.height.current_mm-before-5.0F)<0.001F);
    f.odometry.distance_mm+=1U;
    step(&f,30.0F,S4_CLIMBING,false,false,true,true);
    CHECK(!f.height.chain_valid&&f.height.reason==S4_HEIGHT_JUMP);

    init(&f,S4_ODOM_US32,0U,0U,0U);
    advance(&f,10U,20U);step(&f,3.0F,S4_LEVEL,false,false,true,true);
    CHECK(f.height.active);
    advance(&f,0U,20U);step(&f,0.0F,S4_LEVEL,false,false,true,true);
    CHECK(!f.height.active&&!f.height.last_valid&&!f.height.highest_valid);
    init(&f,S4_ODOM_US32,0U,0U,0U);
    advance(&f,10U,20U);step(&f,-30.0F,S4_LEVEL,false,false,true,true);
    CHECK(!f.height.active&&!f.height.highest_valid);
    init(&f,S4_ODOM_US32,0U,0U,0U);f.height.have_previous=false;up(&f);finish(&f);
    CHECK(!f.height.last_valid&&f.height.reason==S4_HEIGHT_BASELINE);
    init(&f,S4_ODOM_US32,0U,0U,0U);up(&f);
    advance(&f,0U,20U);step(&f,0.0F,S4_LEVEL,false,true,true,true);
    CHECK(!f.height.active&&!f.height.last_valid);
    init(&f,S4_ODOM_US32,0U,0U,0U);up(&f);
    advance(&f,0U,20U);step(&f,0.0F,S4_FAULT,false,false,true,true);
    CHECK(!f.height.active&&!f.height.last_valid);
    init(&f,S4_ODOM_US32,0U,0U,0U);
    for(i=0U;i<502U;++i)
    {advance(&f,0U,20U);step(&f,3.0F,S4_LEVEL,false,false,true,true);}
    CHECK(!f.height.active&&!f.height.last_valid&&!f.height.highest_valid);

    /* Same discrete trace through the actual terrain FSM, not a hand-labelled
     * completion event: debounce must produce one complete height. */
    {
        s4_terrain_t terrain;unsigned humps=0U;
        init(&f,S4_ODOM_MS16,0U,0U,0U);s4_terrain_init(&terrain,0U);
        for(i=0U;i<50U;++i)
        {
            float pitch=(i<5U||i>=45U)?0.0F:((i<25U)?30.0F:-30.0F);
            advance(&f,(i>=5U&&i<45U)?10U:0U,20U);f.angle.pitch_deg=pitch;
            s4_terrain_step(&terrain,&f.angle,true,true,true,f.us);
            if(terrain.hump_event){++humps;}
            s4_height_step(&f.height,&f.odometry,&f.angle,true,true,f.us,f.ms,
                s4_terrain_state(&terrain),2.0F,terrain.hump_event,terrain.traversal_aborted);
        }
        CHECK(humps==1U&&f.height.last_valid);
        CHECK(test_abs(f.height.last_mm-97.5F)<0.01F);
    }
    printf("S4-U07: %s (%u checks, %u failures). Synthetic odometry/angles only; not physical accuracy.\n",
           failures?"FAIL":"PASS",checks,failures);
    return failures?1:0;
}
