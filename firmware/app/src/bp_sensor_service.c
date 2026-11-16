#include <zephyr/kernel.h>
#include <errno.h>
#include <string.h>
#include "bp_sensor_service.h"
#include "bp_iqs9151_runtime.h"
#include "bp_touch_policy.h"
#include "bp_tune.h"
/* IQS9151 sensor worker. Owns the bus, chip start-up, re-ATI, host tuning and raw captures.
 * BLE/main only consume queued processed samples (unchanged interface). */
volatile bp_tune_control backpad_tune_control;
volatile bp_tune_capture backpad_tune_capture={.magic=0x42505443,.version=BP_TUNE_CAPTURE_VERSION_IQS9151};
/* v0.20.8: production mounting is the phone back (pad facing out, FPC top-right seen from the back; confirmed on the
 * board 09:35): BP_TOUCH_FLIP_X, which rotation 270 turns into a screen-Y mirror. Host tests keep BP_TOUCH_PARAMS_DEFAULT
 * (desk orientation); the tuning page can still clear it at run time. */
#define BP_TOUCH_PARAMS_PRODUCTION {0u, 0u, 0u, BP_TOUCH_ENTER_AREA, BP_TOUCH_EXIT_AREA, 256u, \
                                    BP_TOUCH_ENTER_DENSITY, BP_TOUCH_EXIT_DENSITY, BP_TOUCH_FLIP_X}
volatile bp_touch_params backpad_iqs_params=BP_TOUCH_PARAMS_PRODUCTION;
volatile bp_iqs_write backpad_iqs_write;
volatile bp_iqs_read backpad_iqs_read;
volatile bp_iqs_diag backpad_iqs_diag;
bp_touch_log backpad_touch_log; /* last BP_TOUCH_LOG_N contact lifetimes, read over J-Link for tap tuning */
/* Position conditioning, tunable like backpad_iqs_params: write the fields, then a new seq. */
volatile bp_touch_filter_params backpad_filter_params=BP_TOUCH_FILTER_DEFAULT;
static bp_touch_filter_params filter_params=BP_TOUCH_FILTER_DEFAULT;
static uint32_t filter_seq;
/* state: 1 init/retry, 3 streaming, 4 host operation */
static bp_sensor_health health, published;
volatile struct { uint32_t seq; bp_sensor_health value; } backpad_sensor_health;
static struct k_spinlock health_lock;
K_MSGQ_DEFINE(samples,sizeof(bp_sample),16,8);
static bool ready,capturing,ever_attempted;
/* op2 waits for release, then captures from the first chip-reported contact. */
static bool capture_on_touch,capture_wait_zero;
static bool capture_timed;
static bp_trace_encoder trace_encoder;
static bp_touch_filter_state touch_filter;
static bp_touch_params params=BP_TOUCH_PARAMS_PRODUCTION;
static uint32_t params_seq,last_request,missed,bad_frames,chip_resets;
static uint64_t retry_at,last_data,last_finger,capture_start;
static unsigned backoff_ms=250;
static bp_iqs9151_frame last_frame;

static void health_publish(void) {
 ++backpad_sensor_health.seq;__DMB();backpad_sensor_health.value=health;__DMB();++backpad_sensor_health.seq;
 k_spinlock_key_t key=k_spin_lock(&health_lock);published=health;k_spin_unlock(&health_lock,key);
}
void bp_sensor_health_get(bp_sensor_health *out) {
 k_spinlock_key_t key=k_spin_lock(&health_lock);*out=published;k_spin_unlock(&health_lock,key);
}
static void diag_publish(void) {
 const bp_iqs9151_bus *b=bp_iqs9151_bus_state();
 bp_iqs_diag d; memset(&d,0,sizeof(d));
 d.step=b->step;d.ati_error=b->ati_error;d.tp_settings=b->tp_settings;d.fingers=last_frame.fingers_reported;
 d.last_error=b->last_error?b->last_error:health.last_error;
 d.product=b->product;d.major=b->major;d.minor=b->minor;d.x_res=b->x_res;d.y_res=b->y_res;
 d.info=b->last_info;d.tp=b->last_tp;d.session=b->session;d.sequence=b->sequence;
 d.dead_channels=b->dead_channels;d.dead_handled=b->dead_handled;
 d.frames=health.frames;d.missed_windows=missed;d.bad_frames=bad_frames;d.chip_resets=chip_resets;
 d.init_calls=health.init_calls;d.force_requests=bp_iqs9151_force_requests();d.ati_ms=b->ati_ms;
 d.admitted_frames=touch_filter.admitted_frames;d.suppressed_frames=touch_filter.suppressed_frames;
 d.uptime_ms=(uint32_t)k_uptime_get();
 for(unsigned k=0;k<3;k++){const bp_iqs9151_finger *g=&last_frame.finger[k];
  d.finger[k].x=g->x;d.finger[k].y=g->y;d.finger[k].strength=g->strength;d.finger[k].area=g->area;}
 memcpy(d.block,b->last_block,sizeof(d.block));
 uint32_t s=backpad_iqs_diag.seq|1u;backpad_iqs_diag.seq=s;__DMB();
 d.seq=s;memcpy((void *)&backpad_iqs_diag,&d,sizeof(d));__DMB();backpad_iqs_diag.seq=s+1u;
}
bool bp_sensor_take(bp_sample *out) { return k_msgq_get(&samples,out,K_NO_WAIT)==0; }
static void cancel_output(void) {
 touch_filter.log=&backpad_touch_log;touch_filter.fp=&filter_params;
 bp_touch_filter_reset(&touch_filter);
 bp_sample bad={.kind=BP_BAD_FRAME};k_msgq_purge(&samples);(void)k_msgq_put(&samples,&bad,K_NO_WAIT);
}
static void emit(const bp_sample *sample) {
 if(k_msgq_put(&samples,sample,K_NO_WAIT)) {cancel_output();(void)k_msgq_put(&samples,sample,K_NO_WAIT);}
}
static void finish_capture(int error) {
 backpad_tune_capture.error=error;backpad_tune_capture.end_ms=(uint32_t)k_uptime_get();
 backpad_tune_capture.state=error?3u:2u;__DMB();++backpad_tune_capture.snapshot_seq;
 capturing=false;capture_on_touch=false;capture_wait_zero=false;capture_timed=false;backpad_tune_control.result=error;backpad_tune_control.state=error?3u:2u;
 __DMB();backpad_tune_control.ack=last_request;
}
static void fault(int error,bool immediate) {
 ++health.faults;health.last_error=error;health.state=1;
 ready=false;bp_iqs9151_cancel();cancel_output();
 if(capturing) finish_capture(error);
 retry_at=(uint64_t)k_uptime_get()+(immediate?0u:backoff_ms);
 if(!immediate && backoff_ms<4000u)backoff_ms*=2u;
}
static int initialize(void) {
 bp_touch_filter_reset(&touch_filter);
 health.state=1;health_publish();
 int r=0;
 if(ever_attempted) {
  ++health.recoveries;health.bus_error=bp_iqs9151_recover_bus();
  if(health.bus_error && health.bus_error!=-ENOSYS) r=health.bus_error;
 }
 ever_attempted=true;
 if(!r) {++health.init_calls;r=bp_iqs9151_init();}
 if(r) {fault(r,false);return r;}
 ++health.calibrations; /* start-up always ends with a trackpad ATI */
 ready=true;health.state=3;health.last_error=0;backoff_ms=250;
 last_data=last_finger=k_uptime_get();
 return 0;
}
static void snapshot_config(volatile uint8_t *c) {
 const bp_iqs9151_bus *b=bp_iqs9151_bus_state();
 uint8_t v[64]={0};
 v[0]=(uint8_t)b->product;v[1]=(uint8_t)(b->product>>8);v[2]=(uint8_t)b->major;v[3]=(uint8_t)(b->major>>8);
 v[4]=(uint8_t)b->minor;v[5]=(uint8_t)(b->minor>>8);v[6]=b->tp_settings;
 v[7]=(uint8_t)b->x_res;v[8]=(uint8_t)(b->x_res>>8);v[9]=(uint8_t)b->y_res;v[10]=(uint8_t)(b->y_res>>8);
 v[11]=b->ati_error;v[12]=(uint8_t)b->ati_ms;v[13]=(uint8_t)(b->ati_ms>>8);v[14]=(uint8_t)(b->ati_ms>>16);v[15]=(uint8_t)(b->ati_ms>>24);
 (void)bp_iqs9151_read_reg(BP_IQS9151_REG_MAPPING,v+16,25);
 v[41]=b->dead_channels;v[42]=b->dead_handled;v[43]=b->removed_tx;v[44]=b->removed_rx;
 for(unsigned i=0;i<64;i++)c[i]=v[i];
}
/* Tuning registers the host may change at runtime. Geometry, mapping, comms and I2C address are
 * deliberately excluded. */
static bool write_allowed(uint16_t reg,unsigned len) {
 static const uint16_t ranges[][2]={
  {0x1196u,0x1197u}, /* trackpad ATI target (apply with op5) */
  {0x119Cu,0x11A0u}, /* negative/positive-delta re-ATI levels, reference drift limit */
  {0x1246u,0x1275u}, /* trackpad channel disables, Tx rows 0..11 (16 bytes per write; apply with op5) */
  {0x117Au,0x117Bu}, /* trackpad ATI multipliers/dividers (global; apply with op5) */
  {0x11A2u,0x11B5u}, /* sampling periods and mode timeouts (first-touch latency vs power) */
  {0x11CCu,0x11CDu}, /* touch set / clear threshold multipliers */
  {0x11E2u,0x11E2u}, /* trackpad settings: flip/switch, IIR, static, jitter, area filter */
  {0x11E5u,0x11E5u}, /* max multi-touches */
  {0x11D8u,0x11E1u}, /* trackpad conversion frequency and hardware settings (apply with op5) */
  {0x11EAu,0x11F5u}, /* dynamic filter speeds/beta, static beta, stationary, split, trims, jitter, confidence */
 };
 if(!len || len>16u) return false;
 for(unsigned i=0;i<sizeof(ranges)/sizeof(ranges[0]);i++)
  if(reg>=ranges[i][0] && reg+len-1u<=ranges[i][1]) return true;
 return false;
}
static void command(void) {
 uint32_t req=backpad_tune_control.request;__DMB();
 if(!req || req==last_request || capturing)return;
 uint32_t op=backpad_tune_control.op,arg=backpad_tune_control.arg;
 __DMB();if(req!=backpad_tune_control.request)return;
 last_request=req;int r=-EINVAL;
 backpad_tune_control.state=1;
 if((((op==1u || op==2u) && arg>=1u && arg<=BP_TUNE_MAX_FRAMES) ||
     (op==6u && arg>=1000u && arg<=40000u)) && ready) {
  ++backpad_tune_capture.snapshot_seq;__DMB();
  capture_timed=op==6u;
  backpad_tune_capture.version=capture_timed?3u:BP_TUNE_CAPTURE_VERSION_IQS9151;
  memset(&trace_encoder,0,sizeof(trace_encoder));
  for(unsigned j=0;j<4u;j++)backpad_tune_capture.reserved[j]=0;
  if(capture_timed) {
   backpad_tune_capture.reserved[1]=arg;
   backpad_tune_capture.reserved[2]=bp_iqs9151_bus_state()->session;
  }
  backpad_tune_capture.request=req;backpad_tune_capture.state=1;
  backpad_tune_capture.count=0;backpad_tune_capture.target=arg;backpad_tune_capture.error=0;
  capture_start=k_uptime_get();backpad_tune_capture.start_ms=(uint32_t)capture_start;
  backpad_tune_capture.end_ms=0;snapshot_config(backpad_tune_capture.config);
  capture_on_touch=op==2u;
  /* Always observe a zero-contact frame after arming: a held finger cannot trigger op2. */
  capture_wait_zero=capture_on_touch;
  capturing=true;return;
 } else if(op==3u) {
  cancel_output();bp_iqs9151_cancel();ready=false;r=initialize();
 } else if(op==4u && ready) {
  uint16_t reg=backpad_iqs_read.reg;unsigned len=backpad_iqs_read.len;
  static uint8_t buf[BP_IQS9151_MAX_READ];
  if(!len || len>BP_IQS9151_MAX_READ) r=-EINVAL;
  else {r=bp_iqs9151_read_reg(reg,buf,len);
   if(!r)for(unsigned i=0;i<len;i++)backpad_iqs_read.data[i]=buf[i];
   backpad_iqs_read.result=r;backpad_iqs_read.done_ms=(uint32_t)k_uptime_get();}
 } else if(op==5u && ready) {
  if(k_uptime_get()-last_finger<300u) r=-EBUSY;
  else {cancel_output();health.state=4;health_publish();r=bp_iqs9151_reati();
   if(r==BP_IQS9151_EATI)r=0; /* ATI error is reported through diag.ati_error, streaming continues */
   if(r)fault(r,false);else {++health.calibrations;health.state=3;last_data=k_uptime_get();}}
 } else if(op==9u && ready) {
  uint16_t reg=backpad_iqs_write.reg;unsigned len=backpad_iqs_write.len;uint8_t data[16];
  if(!write_allowed(reg,len)) r=-EPERM;
  else {for(unsigned i=0;i<len;i++)data[i]=backpad_iqs_write.data[i];
   cancel_output();health.state=4;health_publish();
   r=bp_iqs9151_write_verify(reg,data,len);
   if(r)fault(r,false);else {health.state=3;last_data=k_uptime_get();}}
 } else if((op==1u || op==2u || op==4u || op==5u || op==6u || op==9u) && !ready) r=-EAGAIN;
 backpad_tune_control.result=r;backpad_tune_control.state=r?3u:2u;
 __DMB();backpad_tune_control.ack=req;
}
static void apply_filter_params(void) {
 uint32_t s=backpad_filter_params.seq;
 if(s==filter_seq) return;
 __DMB();
 bp_touch_filter_params p={s,backpad_filter_params.fmin_chz,backpad_filter_params.beta_milli,
  backpad_filter_params.dcut_chz,backpad_filter_params.string_um,backpad_filter_params.settle_pct,
  backpad_filter_params.lift_pct,backpad_filter_params.tap_window_ms,backpad_filter_params.flags,
  backpad_filter_params.dwell_ms,backpad_filter_params.stop_um};
 if(!p.fmin_chz) p.fmin_chz=1u;
 if(!p.dcut_chz) p.dcut_chz=1u;
 filter_params=p;filter_seq=s;
}
static void apply_params(void) {
 apply_filter_params();
 uint32_t s=backpad_iqs_params.seq;
 if(s==params_seq) return;
 __DMB();
 bp_touch_params p={s,backpad_iqs_params.enter_strength,backpad_iqs_params.exit_strength,
                    backpad_iqs_params.enter_area,backpad_iqs_params.exit_area,
                    backpad_iqs_params.strength_scale_q8,backpad_iqs_params.enter_density,
                    backpad_iqs_params.exit_density,backpad_iqs_params.orient};
 if(!p.strength_scale_q8) p.strength_scale_q8=256u;
 if(p.exit_strength>p.enter_strength) p.exit_strength=p.enter_strength;
 if(p.exit_area>p.enter_area) p.exit_area=p.enter_area;
 if(p.exit_density>p.enter_density) p.exit_density=p.enter_density;
 params=p;params_seq=s;
}
static void record(const bp_iqs9151_bus *b,const bp_iqs9151_window *w) {
 unsigned i=backpad_tune_capture.count;
 if(capture_timed) {
  if(b->session!=backpad_tune_capture.reserved[2]){finish_capture(-ECONNRESET);return;}
  uint8_t xy[36];
  for(unsigned j=0;j<4u;j++)xy[j]=b->last_block[8u+j];
  for(unsigned j=0;j<28u;j++)xy[4u+j]=b->last_block[12u+j];
  for(unsigned j=0;j<4u;j++)xy[32u+j]=b->last_block[j];
  if(!bp_trace_append(&trace_encoder,backpad_tune_capture.compressed,BP_TRACE_BYTES,
                       (uint32_t)w->ready_ms,b->sequence,xy)){finish_capture(-ENOSPC);return;}
  if(!i)backpad_tune_capture.start_ms=(uint32_t)w->ready_ms;
  backpad_tune_capture.reserved[0]=trace_encoder.used;
  backpad_tune_capture.count=i+1u;
  if((uint32_t)w->ready_ms-backpad_tune_capture.start_ms>=backpad_tune_capture.target)finish_capture(0);
  return;
 }
 volatile bp_tune_record *rec=&backpad_tune_capture.records[i];
 rec->ms=(uint32_t)w->ready_ms;rec->session=b->session;rec->sequence=b->sequence;
 for(unsigned j=0;j<4;j++)rec->xy[j]=b->last_block[8+j];
 for(unsigned j=0;j<28;j++)rec->xy[4+j]=b->last_block[12+j];
 for(unsigned j=0;j<4;j++)rec->xy[32+j]=b->last_block[j];
 backpad_tune_capture.count=i+1;
 if(backpad_tune_capture.count>=backpad_tune_capture.target)finish_capture(0);
}
static void sensor_step(void) {
 command();
 uint64_t now=k_uptime_get();
 if(!ready) {
  if(now>=retry_at)(void)initialize();
  health_publish();diag_publish();
  if(!ready)k_sleep(K_MSEC(10));
  return;
 }
 (void)bp_iqs9151_wait_ready(20u);
 bp_iqs9151_frame frame;bp_iqs9151_window w={0};
 int r=bp_iqs9151_read(&frame,&w);
 now=k_uptime_get();
 if(r==-EAGAIN) {
  ++health.no_data;++missed;
  if(now-last_data>=BP_SENSOR_STALL_MS)fault(-ETIMEDOUT,false);
 } else if(r==-ECONNRESET) {
  ++chip_resets;fault(r,true); /* the chip lost its settings: configure again at once */
 } else if(r==-EBADMSG) {
  ++bad_frames;
  if(now-last_data>=BP_SENSOR_STALL_MS)fault(r,false);
 } else if(r) {
  fault(r,false);
 } else {
  const bp_iqs9151_bus *b=bp_iqs9151_bus_state();
  ++health.frames;last_data=now;last_frame=frame;
  health.raw_count=frame.fingers_reported;
  if(frame.fingers_reported)last_finger=now;
  if(capturing) {
   if(capture_on_touch && capture_wait_zero) {
    if(!frame.fingers_reported)capture_wait_zero=false;
   } else if(capture_on_touch && frame.fingers_reported) {
    capture_on_touch=false;
    capture_start=now;backpad_tune_capture.start_ms=(uint32_t)w.ready_ms;
    record(b,&w); /* keep this first raw contact, even if admission rejects it */
   } else if(!capture_on_touch)record(b,&w);
  }
  apply_params();
  bp_sample sample;
  bp_touch_geometry geo={b->total_rx,b->total_tx,b->orig_rx,b->orig_tx,b->col_phys,b->row_phys};
  bp_touch_convert(&touch_filter,&params,&frame,b->x_res,b->y_res,&geo,b->session,b->sequence,w.ready_ms,&sample);
  health.filtered_count=sample.count;
  emit(&sample);
 }
 /* Arming must allow time to read the prompt; recording retains the original 10s bound. */
 if(capturing && (uint64_t)k_uptime_get()-capture_start>=
    (capture_timed?backpad_tune_capture.target+2000u:capture_on_touch?30000u:10000u))finish_capture(-ETIMEDOUT);
 health_publish();diag_publish();
}
static void worker(void *a,void *b,void *c) {
 ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
 cancel_output();
 for(;;) sensor_step();
}
K_THREAD_DEFINE(sensor_worker,4096,worker,NULL,NULL,NULL,7,0,SYS_FOREVER_MS);
void bp_sensor_start(void) {k_thread_start(sensor_worker);}

