/* S4-U15: hump-height test driven by a physical car/hump model.
 *
 * Unlike S4-U07/U13/U14 (hand-written pitch values), this test simulates the
 * car itself: a rigid chassis with the drive wheels (encoders) and the caster
 * S4_TEST_WHEELBASE_MM apart, rolling over a cosine-shaped hump. From that
 * geometry it generates what the real inputs would be:
 *   - LSM303DLHC accelerometer samples (50 Hz ODR, slightly slower than the
 *     task so NO_NEW_SAMPLE gaps occur, 2 mg quantisation, vibration noise)
 *   - S2 IF-17 odometry words (cumulative mm | kernel ms, published every 20 ms)
 * and feeds them through the REAL S4 pipeline, in the same order as
 * imu_terrain_task.c: s4_process -> s4_terrain_step -> s4_motion_snapshot ->
 * s4_height_step_quality.
 *
 * Requirements checked (Week 6 design review):
 *   FR32  detect speed humps                 (cases A, C, F)
 *   FR33  estimate height, retain highest    (cases B, C)
 *   NFR24 height error <= +/-10%             (case A)
 *   NFR26 no false hump on flat + vibration  (case D)
 *   guard reject reversing wheel on the hump (case E)
 *
 * The estimator gets the same wheelbase as the model (S4_HEIGHT_WHEELBASE_MM
 * on the car). A host pass is NOT the physical H10 test. Measure your car and hump and
 * set the macros below; the model has point contacts and no wheel slip.
 * Deterministic: fixed-seed RNG, no rand(). */
#include <stdio.h>
#include <math.h>
#include "s4_processing.h"
#include "s4_terrain.h"
#include "s4_odometry.h"
#include "s4_motion_input.h"

/* ---- Measure these on the real car / hump, then update ---- */
#ifndef S4_TEST_WHEELBASE_MM
#define S4_TEST_WHEELBASE_MM   110.0   /* drive axle to caster contact */
#endif
#ifndef S4_TEST_SPEED_MMPS
#define S4_TEST_SPEED_MMPS     50.0   /* VC_LF_SPEED when centred */
#endif
#define TEST_STILL_NOISE_G     0.004   /* sensor noise, car at rest */
#define TEST_DRIVE_NOISE_G     0.010   /* light vibration while driving */
#define TEST_VIBRATION_G       0.040   /* moderate motor vibration */
#define TEST_TOLERANCE         0.10    /* NFR24 */

#define TEST_PI 3.14159265358979
#define MAX_HUMPS 3

static unsigned checks, failures;
#define CHECK(name,cond,...) do{++checks; \
    if(cond){printf("  PASS %-34s ",name);} \
    else{++failures;printf("  FAIL %-34s ",name);} \
    printf(__VA_ARGS__);printf("\n");}while(0)

typedef struct { double start_mm, height_mm, length_mm; } hump_t;
typedef struct
{
    hump_t humps[MAX_HUMPS]; unsigned hump_count;
    double drive_noise_g;      /* vibration while the wheels turn */
    double settle_noise_g;     /* vibration on flat ground after the last hump */
    double settle_s;           /* time on flat ground after the last hump */
    double block_from_mm, block_to_mm; /* S2 reports a reversing wheel here */
    double flat_run_mm;        /* used when hump_count == 0 */
    unsigned seed;
} scenario_t;
typedef struct
{
    unsigned events;           /* terrain hump_event count */
    bool event_valid[8]; float event_mm[8];
    bool last_valid; float last_mm; bool highest_valid; float highest_mm;
    uint8_t reason; bool calibrated; uint8_t final_state;
} result_t;

/* ---- deterministic noise ---- */
static uint32_t rng;
static double uniform(void){rng=rng*1664525U+1013904223U;return ((rng>>8)+0.5)/16777216.0;}
static double gauss(void){return sqrt(-2.0*log(uniform()))*cos(2.0*TEST_PI*uniform());}
static float quantise(double g){return (float)(floor(g/0.002+0.5)*0.002);}

/* ---- hump surface and chassis geometry ---- */
static const scenario_t *sc;
static double surface(double x)
{
    double h=0.0; unsigned i;
    for(i=0;i<sc->hump_count;++i)
    {
        const hump_t *p=&sc->humps[i];
        if(x>p->start_mm&&x<p->start_mm+p->length_mm)
        {h+=p->height_mm*0.5*(1.0-cos(2.0*TEST_PI*(x-p->start_mm)/p->length_mm));}
    }
    return h;
}
static double slope(double x){return (surface(x+0.05)-surface(x-0.05))/0.1;}
/* Caster contact: the point ahead on the surface exactly one wheelbase away. */
static double front_x(double xr)
{
    double lo=xr, hi=xr+S4_TEST_WHEELBASE_MM, hr=surface(xr); int i;
    for(i=0;i<50;++i)
    {
        double m=0.5*(lo+hi);
        if(hypot(m-xr,surface(m)-hr)<S4_TEST_WHEELBASE_MM){lo=m;}else{hi=m;}
    }
    return lo;
}

static result_t run(const scenario_t *s)
{
    s4_processing_config_t cfg=s4_processing_defaults();
    s4_processor_t p; s4_terrain_t t; s4_height_t h; s4_motion_reader_t reader;
    s4_orientation_t o={0}; s4_odometry_t od={0}; result_t r={0};
    double xr=0.0, path=0.0, ax=0.0, ay=0.0, az=1.0, next_sample=7.3, end_mm;
    uint32_t ms, stop_ms=0U; bool have_sample=false;
    sc=s; rng=s->seed*2654435761U+12345U;
    s4_processor_init(&p,&cfg); s4_terrain_init(&t,0U); s4_height_init(&h);
    h.wheelbase_mm=(uint16_t)S4_TEST_WHEELBASE_MM; /* as imu_terrain_task.c does */
    t.config.traversal_timeout_us=20000000U;h.max_candidate_ms=20000U; /* matches slow diagnostic image */
    s4_motion_reader_init(&reader);
    end_mm=s->hump_count?s->humps[s->hump_count-1U].start_mm+
                         s->humps[s->hump_count-1U].length_mm+300.0:s->flat_run_mm;
    for(ms=1U;ms<120000U;++ms)
    {
        bool moving=(ms>3000U)&&(xr<end_mm);
        double pitch, noise;
        if(moving)
        {
            double step=S4_TEST_SPEED_MMPS/1000.0; /* path length per ms */
            xr+=step*cos(atan(slope(xr))); path+=step;
        }
        else if(ms>3000U&&stop_ms==0U){stop_ms=ms;}
        if(stop_ms!=0U&&(ms-stop_ms)>(uint32_t)(s->settle_s*1000.0)){break;}
        pitch=atan2(surface(front_x(xr))-surface(xr),front_x(xr)-xr);
        noise=moving?s->drive_noise_g:(stop_ms?s->settle_noise_g:TEST_STILL_NOISE_G);
        if((double)ms>=next_sample) /* ODR 1% slower than the 20 ms task */
        {
            next_sample+=20.0/0.99; have_sample=true;
            /* Sensor frame after S4 axis mapping: x forward, z up. */
            ax=sin(pitch)+noise*gauss(); ay=noise*gauss(); az=cos(pitch)+noise*gauss();
        }
        if(ms%20U==0U)
        {
            s4_sample_t smp={0}; uint8_t q, state;
            bool fresh=have_sample, blocked;
            uint32_t word=((uint32_t)(uint16_t)(long)path<<16U)|(uint16_t)(ms-15U);
            have_sample=false;
            if(fresh)
            {
                smp.accel_fresh=true; smp.accel_g[0]=quantise(ax);
                smp.accel_g[1]=quantise(ay); smp.accel_g[2]=quantise(az);
                (void)s4_process(&p,&smp,ms*1000U,&o);
            }
            else {o.orientation_valid=false;}
            /* Same quality ladder as imu_terrain_task.c */
            if(!fresh){q=S4_IMU_WAIT;}
            else if(!p.calibrated){q=S4_IMU_FAULT;}
            else if(o.dynamic_g>=t.config.impact_g){q=S4_IMU_FAULT;}
            else if(!o.orientation_valid){q=S4_IMU_DYNAMIC;}
            else {q=S4_IMU_GOOD;}
            s4_terrain_step(&t,&o,p.calibrated,true,fresh,ms*1000U);
            state=s4_terrain_state(&t);
            blocked=(path>=s->block_from_mm)&&(path<s->block_to_mm);
            (void)s4_motion_snapshot(&reader,word,0U,blocked,ms,&od);
            s4_height_step_quality(&h,&od,&o,q,true,ms*1000U,ms,state,
                                   t.config.exit_deg,t.hump_event,t.traversal_aborted);
            if(t.hump_event&&r.events<8U)
            {r.event_valid[r.events]=h.last_valid;r.event_mm[r.events]=h.last_mm;++r.events;}
        }
    }
    r.last_valid=h.last_valid; r.last_mm=h.last_mm;
    r.highest_valid=h.highest_valid; r.highest_mm=h.highest_mm;
    r.reason=h.reason; r.calibrated=p.calibrated; r.final_state=s4_terrain_state(&t);
    return r;
}

static scenario_t one_hump(double height,double length,unsigned seed)
{
    scenario_t s={0};
    s.humps[0]=(hump_t){300.0,height,length}; s.hump_count=1U;
    s.drive_noise_g=TEST_DRIVE_NOISE_G; s.settle_noise_g=TEST_STILL_NOISE_G;
    s.settle_s=1.0; s.block_from_mm=-1.0; s.block_to_mm=-1.0; s.seed=seed;
    return s;
}
static double err_pct(float est,double ref){return ((double)est-ref)/ref*100.0;}

int main(void)
{
    static const double lengths[]={250.0,300.0,400.0,600.0};
    char name[64]; unsigned i, seed;
    result_t r; scenario_t s;
    printf("S4-U15 physical-model hump height (wheelbase %.0f mm, %.0f mm/s)\n",
           S4_TEST_WHEELBASE_MM,S4_TEST_SPEED_MMPS);

    printf("A  NFR24 accuracy, 30 mm hump, light vibration, 3 seeds each\n");
    for(i=0;i<sizeof(lengths)/sizeof(lengths[0]);++i)
    {
        for(seed=1U;seed<=3U;++seed)
        {
            s=one_hump(30.0,lengths[i],seed); r=run(&s);
            snprintf(name,sizeof name,"A 30x%.0f mm seed %u",lengths[i],seed);
            CHECK(name,r.events==1U&&r.event_valid[0]&&
                  fabs(err_pct(r.event_mm[0],30.0))<=TEST_TOLERANCE*100.0,
                  "events=%u valid=%d est=%.1f mm err=%+.1f%% reason=%s",
                  r.events,(int)r.event_valid[0],(double)r.event_mm[0],
                  r.event_valid[0]?err_pct(r.event_mm[0],30.0):0.0,
                  s4_height_reason_name(r.reason));
        }
    }

    printf("B  FR33 height kept after the hump while the car vibrates on flat ground\n");
    for(seed=1U;seed<=3U;++seed)
    {
        s=one_hump(30.0,400.0,seed); s.settle_noise_g=TEST_VIBRATION_G; s.settle_s=3.0;
        r=run(&s);
        snprintf(name,sizeof name,"B last height retained seed %u",seed);
        CHECK(name,r.events==1U&&r.event_valid[0]&&r.last_valid&&
              r.last_mm==r.event_mm[0],
              "event=%.1f mm -> 3 s later valid=%d last=%.1f mm reason=%s",
              (double)r.event_mm[0],(int)r.last_valid,(double)r.last_mm,
              s4_height_reason_name(r.reason));
        snprintf(name,sizeof name,"B highest retained seed %u",seed);
        CHECK(name,r.highest_valid&&r.highest_mm==r.event_mm[0],
              "highest_valid=%d highest=%.1f mm",(int)r.highest_valid,(double)r.highest_mm);
    }

    printf("C  FR33 highest of three humps in one run (20, 30, 15 mm)\n");
    s=one_hump(20.0,400.0,1U);
    s.humps[1]=(hump_t){1200.0,30.0,400.0}; s.humps[2]=(hump_t){2100.0,15.0,300.0};
    s.hump_count=3U; r=run(&s);
    CHECK("C three humps detected",r.events==3U,"events=%u",r.events);
    {
        float best=-1.0F; unsigned k;
        for(k=0;k<r.events;++k){if(r.event_valid[k]&&r.event_mm[k]>best){best=r.event_mm[k];}}
        CHECK("C highest = largest event",r.highest_valid&&best>0.0F&&r.highest_mm==best,
              "events %.1f/%.1f/%.1f mm, highest=%.1f mm",(double)r.event_mm[0],
              (double)r.event_mm[1],(double)r.event_mm[2],(double)r.highest_mm);
        CHECK("C highest within 10% of 30 mm",r.highest_valid&&
              fabs(err_pct(r.highest_mm,30.0))<=TEST_TOLERANCE*100.0,
              "highest=%.1f mm err=%+.1f%%",(double)r.highest_mm,err_pct(r.highest_mm,30.0));
    }

    printf("D  NFR26 flat ground with motor vibration: no hump, no height\n");
    for(seed=1U;seed<=3U;++seed)
    {
        s=one_hump(0.0,1.0,seed); s.hump_count=0U; s.flat_run_mm=1500.0;
        s.drive_noise_g=TEST_VIBRATION_G; s.settle_noise_g=TEST_VIBRATION_G;
        r=run(&s);
        snprintf(name,sizeof name,"D flat run seed %u",seed);
        CHECK(name,r.calibrated&&r.events==0U&&!r.highest_valid,
              "calibrated=%d events=%u highest_valid=%d",(int)r.calibrated,
              r.events,(int)r.highest_valid);
    }

    printf("E  guard: a wheel reverses (S2 speed < 0) during the climb\n");
    s=one_hump(30.0,400.0,1U); s.block_from_mm=400.0; s.block_to_mm=440.0; r=run(&s);
    CHECK("E traversal rejected",r.events==1U&&!r.event_valid[0]&&!r.highest_valid&&
          r.reason==S4_HEIGHT_REVERSE,"events=%u valid=%d reason=%s",
          r.events,(int)r.event_valid[0],s4_height_reason_name(r.reason));

    printf("F  FR32 detection of a lower hump (15 mm x 300 mm)\n");
    s=one_hump(15.0,300.0,1U); r=run(&s);
    CHECK("F 15 mm hump detected",r.events==1U&&r.event_valid[0],
          "events=%u valid=%d est=%.1f mm",r.events,(int)r.event_valid[0],(double)r.event_mm[0]);

    /* Measured geometry/demo speed, synthetic 23 mm humps. Real hump
     * length, caster shape, chassis contact and slip are not modelled. */
    for(seed=1U;seed<=3U;++seed)
    {
        s=one_hump(23.0,250.0,seed);r=run(&s);
        CHECK("23 mm model at 50 mm/s",r.events==1U&&r.event_valid[0]&&
              fabs(err_pct(r.event_mm[0],23.0))<=10.0,
              "events=%u valid=%d est=%.1f mm",r.events,(int)r.event_valid[0],(double)r.event_mm[0]);
    }
    printf("S4-U15: %s (%u checks, %u failures); physical model only, not H10 evidence.\n",
           failures?"FAIL":"PASS",checks,failures);
    return failures?1:0;
}
