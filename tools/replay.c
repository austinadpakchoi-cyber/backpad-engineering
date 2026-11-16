#include "bp_experiment_flags.h"
#include "bp_touch_policy.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Offline algorithm replay, not a claim of exact live state at clip start. */
static unsigned wheel_abs, wheel_reports, motion_reports, clicks;
static int last_wheel;
static unsigned event_ms,previous_buttons,left_presses,right_presses;
static bp_tx_result sink(void *ctx, const bp_report *r) {
    (void)ctx;
    if((r->buttons&1u) && !(previous_buttons&1u)) ++left_presses;
    if((r->buttons&2u) && !(previous_buttons&2u)) ++right_presses;
    previous_buttons=r->buttons;
    fprintf(stderr,"REPORT {\"ms\":%u,\"buttons\":%u,\"dx\":%d,\"dy\":%d,\"wheel\":%d}\n",event_ms,r->buttons,r->dx,r->dy,r->wheel);
    if (r->wheel) {wheel_abs += (unsigned)abs(r->wheel); ++wheel_reports; last_wheel += r->wheel;}
    if (r->dx || r->dy) ++motion_reports;
    if (r->buttons) ++clicks;
    return BP_TX_ACCEPTED;
}
static unsigned u16(const unsigned char *p) {return p[0] | ((unsigned)p[1]<<8);}
static unsigned u32(const unsigned char *p) {return u16(p) | (u16(p+2)<<16);}
static void load(const char *name, void *buf, size_t size) {
    FILE *f=fopen(name,"rb");
    if(!f || fread(buf,1,size,f)!=size) {fprintf(stderr,"read failed: %s\n",name);exit(2);}
    fclose(f);
}
static void load_capture(const char *name, unsigned char *cap, size_t capacity) {
    FILE *f=fopen(name,"rb");if(!f)exit(2);
    if(fseek(f,0,SEEK_END)){fclose(f);exit(2);}long size=ftell(f);
    if(size<128 || (unsigned long)size>capacity || (size-128)%48){fclose(f);exit(2);}
    rewind(f);if(fread(cap,1,(size_t)size,f)!=(size_t)size){fclose(f);exit(2);}fclose(f);
    unsigned count=u32(cap+20);
    if(!count || count>20000 || 128u+count*48u>(unsigned long)size)exit(2);
}
int main(int argc, char **argv) {
    if(argc!=5) return 2;
    static unsigned char cap[960128]; load_capture(argv[1],cap,sizeof cap);
    bp_touch_params params; load(argv[2],&params,sizeof params);
    bp_touch_filter_params fp; load(argv[3],&fp,sizeof fp);
    bp_config config=bp_default_config();
/* Configuration is always supplied explicitly by the public runner. */
    _Static_assert(sizeof(bp_config)==96,"Live configuration ABI must match");
    if(argc==5)load(argv[4],&config,sizeof config);
    if(argc==6) {
        if(!strcmp(argv[4],"fmin"))fp.fmin_chz=(uint16_t)atoi(argv[5]);
        else if(!strcmp(argv[4],"exit"))params.exit_density=(uint16_t)atoi(argv[5]);
        else if(!strcmp(argv[4],"step"))config.scroll_step=(uint16_t)atoi(argv[5]);
        else return 2;
    }
    const unsigned char original_map[]={7,8,12,11,10,9,6,2,3,4,5,1,0,34,35,37,38,41,42,36,40,43,45,39,33};
    const unsigned char *cfg=cap+64;
    uint16_t xr=(uint16_t)u16(cfg+7),yr=(uint16_t)u16(cfg+9);
    uint8_t cols[13],rows[12],nx=(uint8_t)(xr/256+1),ny=(uint8_t)(yr/256+1);
    if(nx>13 || ny>12) return 3;
    for(unsigned k=0;k<nx;k++) {unsigned j=0;while(j<13 && original_map[j]!=cfg[16+k])++j;if(j==13)return 3;cols[k]=(uint8_t)j;}
    for(unsigned k=0;k<ny;k++) {unsigned j=0;while(j<12 && original_map[13+j]!=cfg[16+nx+k])++j;if(j==12)return 3;rows[k]=(uint8_t)j;}
    bp_touch_geometry geo={nx,ny,13,12,cols,rows};
    bp_touch_filter_state tf={0};tf.fp=&fp;
    bp_pointer p; if(bp_init(&p,&config,sink,NULL)!=BP_OK)return 3;
    unsigned first_ms=u32(cap+128),first_seq=u32(cap+132),session=u32(cap+136);
    event_ms=first_ms-5;
    (void)bp_connect(&p,1,first_ms-5);
    (void)bp_flush(&p,first_ms-5);
    bp_sample zero={0};zero.kind=BP_DATA;zero.session=session;zero.sequence=first_seq-1;zero.time_ms=first_ms-4;
    (void)bp_process(&p,&zero,zero.time_ms);(void)bp_flush(&p,zero.time_ms);
    unsigned admitted[4]={0},raw_counts[4]={0},modes[8]={0};
    puts("ms,raw_count,filtered_count,mode,wheel_delta,weak_bits,px0,py0,px1,py1,motion_reports_total,button_reports_total,tap_spikes,cancellations,lift_holds,intent,moving_now");
    for(unsigned i=0;i<u32(cap+20);++i) {
        const unsigned char *v=cap+128+i*48,*xy=v+12;
        bp_iqs9151_frame f={0};f.gestures1=(uint16_t)u16(xy);f.gestures2=(uint16_t)u16(xy+2);
        f.info=(uint16_t)u16(xy+4);f.tp=(uint16_t)u16(xy+6);f.fingers_reported=(uint8_t)(f.tp&15);
        for(unsigned k=0;k<3;k++) {const unsigned char *g=xy+8+k*8;
            f.finger[k].x=(uint16_t)u16(g);f.finger[k].y=(uint16_t)u16(g+2);
            f.finger[k].strength=(uint16_t)u16(g+4);f.finger[k].area=(uint16_t)u16(g+6);
            f.finger[k].valid=f.finger[k].x!=65535 && f.finger[k].y!=65535 && (f.finger[k].strength||f.finger[k].area);
        }
        bp_sample s;bp_touch_convert(&tf,&params,&f,xr,yr,&geo,u32(v+8),u32(v+4),u32(v),&s);
        event_ms=(unsigned)s.time_ms;last_wheel=0;(void)bp_process(&p,&s,s.time_ms);(void)bp_tick(&p,s.time_ms);(void)bp_flush(&p,s.time_ms);
        ++admitted[s.count];++raw_counts[f.fingers_reported];++modes[p.mode];
        printf("%u,%u,%u,%u,%d,%u,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u\n",u32(v),f.fingers_reported,s.count,p.mode,last_wheel,s.weak_contacts,
            s.count?s.contacts[0].x:0,s.count?s.contacts[0].y:0,s.count>1?s.contacts[1].x:0,s.count>1?s.contacts[1].y:0,motion_reports,clicks,p.tap_spikes,p.cancellations,tf.lift_holds,tf.intent[0].active,tf.moving_now[0]);
    }
    fprintf(stderr,"raw=%u/%u/%u/%u filtered=%u/%u/%u/%u wheel=%u reports=%u motion=%u clicks=%u cancellations=%u scroll_frames=%u\n",
        raw_counts[0],raw_counts[1],raw_counts[2],raw_counts[3],admitted[0],admitted[1],admitted[2],admitted[3],wheel_abs,wheel_reports,motion_reports,clicks,p.cancellations,modes[BP_SCROLL]);
    fprintf(stderr,"STATE {\"left_presses\":%u,\"right_presses\":%u,\"rejoin_window_open\":%u,\"rejoin_deadline_ms\":%llu,\"last_input_ms\":%u}\n",left_presses,right_presses,p.tap_rejoin_pending,(unsigned long long)p.tap_rejoin_deadline,event_ms);
    return 0;
}

