#ifndef BP_POLICY_SOURCE
#define BP_POLICY_SOURCE "bp_touch_policy.c"
#endif
#include BP_POLICY_SOURCE
#include <stdio.h>
#include <stdlib.h>
static bp_touch_filter_state st;static bp_touch_filter_params fp=BP_TOUCH_FILTER_DEFAULT;static bp_touch_params tp=BP_TOUCH_PARAMS_DEFAULT;static bp_pointer core;static bp_config cfg;
static unsigned ms,seq,buttons,left,cadence=5,area=5;static int um,hx,hy;
static bp_tx_result sink(void*c,const bp_report*r){(void)c;hx+=r->dx;hy+=r->dy;if((r->buttons&1)&&!(buttons&1))left++;buttons=r->buttons;return BP_TX_ACCEPTED;}
static void frame(unsigned strength,bool present){bp_iqs9151_frame f={0};bp_sample s;if(present){f.fingers_reported=1;f.finger[0]=(bp_iqs9151_finger){.valid=true,.x=(uint16_t)(4000+lroundf(um/1000.0f/MM_X)),.y=5000,.strength=strength,.area=area};}ms+=cadence;bp_touch_convert(&st,&tp,&f,BP_COORD_MAX,BP_COORD_MAX,NULL,1,++seq,ms,&s);(void)bp_process(&core,&s,ms);(void)bp_tick(&core,ms);(void)bp_flush(&core,ms);}
static void reset(void){memset(&st,0,sizeof st);st.fp=&fp;ms=1000;seq=buttons=left=0;hx=hy=um=0;area=5;(void)bp_init(&core,&cfg,sink,0);(void)bp_connect(&core,1,ms-5);(void)bp_flush(&core,ms-5);for(unsigned i=0;i<20;i++)frame(0,false);}
int main(int argc,char**argv){if(argc!=2)return 2;FILE*f=fopen(argv[1],"rb");if(!f||fread(&cfg,1,sizeof cfg,f)!=96)return 2;fclose(f);
 puts("cadence,tail_um,tail_ms,hid_tail_counts,left,final_buttons");
 for(cadence=5;cadence<=10;cadence+=5)for(unsigned dist=500;dist<=1500;dist+=500)for(unsigned dur=60;dur<=180;dur+=60){reset();for(unsigned i=0;i<300/cadence;i++){um+=10*cadence;frame(3000,true);}for(unsigned i=0;i<800/cadence;i++)frame(3000,true);int x0=hx,y0=hy;
  for(unsigned i=1;i<=dur/cadence;i++){um=3000+dist*i/(dur/cadence);frame(3000-2400*i/(dur/cadence),true);}for(unsigned i=0;i<100/cadence;i++)frame(0,false);
  printf("%u,%u,%u,%.3f,%u,%u\n",cadence,dist,dur,(double)hypotf((float)(hx-x0),(float)(hy-y0)),left,buttons);
 }return 0;}
