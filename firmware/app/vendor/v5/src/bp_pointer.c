#include "bp_pointer.h"
#include <limits.h>
#include <string.h>

/* Independent motion-confidence opt-in. Candidate not yet hardware validated. */
#ifndef BP_EXPERIMENT_MOTION_CONFIDENCE
#define BP_EXPERIMENT_MOTION_CONFIDENCE 1
#endif
#ifndef BP_MOTION_CONFIDENCE_ENTER
#define BP_MOTION_CONFIDENCE_ENTER 450u
#endif
#ifndef BP_MOTION_CONFIDENCE_EXIT
#define BP_MOTION_CONFIDENCE_EXIT 350u
#endif
#ifndef BP_MOTION_CONFIDENCE_DRAG_MS
#define BP_MOTION_CONFIDENCE_DRAG_MS 60u
#endif
_Static_assert(BP_MOTION_CONFIDENCE_EXIT>0 && BP_MOTION_CONFIDENCE_ENTER>=BP_MOTION_CONFIDENCE_EXIT,"motion confidence thresholds");

/* Independent of discard_motion: tap silence and spatial rebases are not
 * new physical contact lifetimes. Only loss/zero/fault/selected-ID changes
 * revoke permission. Keep an existing loss deadline across debounced zeros. */
static bool movement_pending_intent(const bp_pointer *p) {
    return p->second_tap && p->movement_confidence_seen && !p->scroll_latched &&
        p->mode!=BP_SCROLL && p->mode!=BP_PAIR_RELEASE;
}
static bool movement_enabled(const bp_pointer *p) {
    return BP_EXPERIMENT_MOTION_CONFIDENCE && p->config.motion_confidence_gate;
}
static void movement_invalidate(bp_pointer *p,uint64_t ms,bool lost) {
    if(!movement_enabled(p))return;
    p->movement_confidence_open=false;p->movement_confidence_id=0;p->movement_qualified_since=0;
    bool pending_qualified=movement_pending_intent(p);
    if(lost && (p->mode==BP_DRAG || p->lock_rejoin || p->local_buttons || pending_qualified) && !p->movement_confidence_since)
        p->movement_confidence_since=ms;
}
static void movement_reset(bp_pointer *p) {
    p->movement_confidence_open=false;p->movement_confidence_seen=false;p->movement_initial_eligible=true;
    p->movement_confidence_id=0;p->movement_confidence_since=0;p->movement_qualified_since=0;
    p->movement_drag_x=0;p->movement_drag_y=0;
    memset(&p->movement_drag_median,0,sizeof(p->movement_drag_median));
}

/* Ordinary initial300..449 strength growth is not a recovery boundary.
 * Actual zero, suppression, selected-weak or ID transfer is a boundary and
 * can never regain the initial-landing exception on this contact lifetime. */
static void movement_boundary(bp_pointer *p,uint64_t ms,bool lost) {
    if(!movement_enabled(p))return;
    p->movement_initial_eligible=false;movement_invalidate(p,ms,lost);
}
static bp_result entry(const bp_pointer *p) {
    if(p==NULL || !p->initialized) return BP_INVALID;
    return p->in_send ? BP_REENTRANT:BP_OK;
}
static bool report_new(bp_pointer *p,bp_report *r,uint8_t buttons,int16_t dx,int16_t dy) {
    if(p->next_report_id==UINT64_MAX) { p->online=false; return false; }
    r->id=++p->next_report_id; r->link_epoch=p->link_epoch;
    r->buttons=buttons; r->dx=dx; r->dy=dy; r->wheel=0; return true;
}
static void reset_speed(bp_pointer *p) {
    p->adaptive_speed_q8=0;
    p->speed_ms=0;p->speed_count=0;p->speed_index=0;
    p->speed_last_dx=0;p->speed_last_dy=0;
    memset(p->speed_samples,0,sizeof(p->speed_samples));
}
/* keep_wheel (scroll_incremental): unsent wheel-only reports survive, so a scroll pause, the lone-member path or
 * the lift do not delete detents the link has not taken yet (the firmware flushes once per ~2 sensor frames) */
static void discard_motion_ex(bp_pointer *p,bool keep_wheel) {
    memset(&p->motion_median,0,sizeof(p->motion_median));
    memset(&p->guard,0,sizeof(p->guard));memset(&p->still,0,sizeof(p->still));memset(&p->precise_still,0,sizeof(p->precise_still));
    reset_speed(p);p->motion_started_ms=0;p->resume_ms=0;memset(&p->soft,0,sizeof(p->soft));
    p->position_ready=false;p->position_ms=0;
    p->velocity_x_q8=0;p->velocity_y_q8=0;p->direction_x=0;p->direction_y=0;
    p->motion_pending=0;p->remainder_x=0;p->remainder_y=0;
    memset(p->motion_delay,0,sizeof(p->motion_delay));
    /* Local queue entries have not been accepted by the transport. Purge
     * motion at a boundary while preserving ordered pure button transitions. */
    unsigned keep=0;
    for(unsigned i=0;i<p->queued;i++) {
        bp_report r=p->queue[(p->head+i)%BP_QUEUE_CAP];
        if(r.dx || r.dy || (r.wheel && !keep_wheel)) {++p->dropped_unsent_reports;continue;}
        p->queue[(p->head+keep++)%BP_QUEUE_CAP]=r;
    }
    p->queued=keep;
}
static void discard_motion(bp_pointer *p) { discard_motion_ex(p,false); }
static void cancel_at(bp_pointer *p,unsigned line) {
    p->last_cancel_line=line;
    movement_reset(p);
    if(p->tap_tracking) {++p->tap_reject[3];p->tap_tracking=false;}
    p->release_frames=0;p->hold_eligible=false;p->pair_pause_active=false;
    p->primary_confidence_paused=false;p->primary_confidence_since=0;
    memset(&p->pair_landing,0,sizeof(p->pair_landing));
    p->tap_rejoin_pending=false;p->second_tap=false;p->lock_rejoin=false;p->scroll_latched=false;p->scroll_paused=false;p->scroll_confirmed=false;p->scroll_started=false;
    memset(&p->scroll_repair_candidate,0,sizeof(p->scroll_repair_candidate));
    ++p->cancellations; p->dropped_unsent_reports+=(uint32_t)p->queued;
    p->head=0u; p->queued=0u; p->mode=BP_WAIT_ZERO; p->local_buttons=0u;
    p->remainder_x=0; p->remainder_y=0; p->have_fresh=false; p->moved=false;
    p->pair_tap_possible=false; p->single_only=false;p->primary_missing=false; p->pending_pair_frames=0;memset(p->extras,0,sizeof(p->extras));
    discard_motion(p);p->contact_ms=0;p->landing_frames=0;p->motion_active=false;p->landing_ready=false;
    /* Dedicated slot: release cannot be crowded out by motion or click queue. */
    if(!p->release_pending) {
        p->release_pending=true;
        if(!report_new(p,&p->release_report,0u,0,0)) p->release_report.id=0u;
    }
}
#define cancel(p) cancel_at((p),__LINE__)
static bool clock_ok(bp_pointer *p,uint64_t now) {
    if(p->clock_known && now<p->last_call_ms) { cancel(p); return false; }
    p->clock_known=true; p->last_call_ms=now; return true;
}
static void lose_link(bp_pointer *p) {
    cancel(p); p->online=false; p->accepted_buttons_known=false;
}
static bool queue_one(bp_pointer *p,uint8_t buttons,int16_t dx,int16_t dy) {
    /* Motion coalescing: when the link drains slower than the sensor frames arrive (IQS9151 at ~150 Hz over a
     * 7.5 ms connection interval), add this step into the newest unsent motion-only report with the same
     * buttons instead of growing the queue. Queued reports are unsent (bp_flush dequeues on acceptance), so
     * order and button edges are preserved; the queue used to fill in ~0.6 s of fast travel, which cancelled
     * the gesture into BP_WAIT_ZERO until the finger lifted (board 1, v0.15: cursor dead mid-stroke). */
    if(p->config.coalesce_motion && p->queued && (dx||dy)) {
        bp_report *tail=&p->queue[(p->head+p->queued-1u)%BP_QUEUE_CAP];
        if(tail->buttons==buttons && tail->wheel==0 && (tail->dx||tail->dy)) {
            int32_t nx=tail->dx+dx,ny=tail->dy+dy;
            if(nx>=-127 && nx<=127 && ny>=-127 && ny<=127) {tail->dx=(int16_t)nx;tail->dy=(int16_t)ny;++p->coalesced_reports;return true;}
        }
    }
    if(p->queued>=BP_QUEUE_CAP) return false;
    unsigned index=(p->head+p->queued)%BP_QUEUE_CAP;
    if(!report_new(p,&p->queue[index],buttons,dx,dy)) return false;
    ++p->queued; return true;
}
/* One immediate cursor step of (ux,uy) micrometres at the configured gain, bypassing the motion delay and the
 * position filter (used to repay evidenced travel at once, so none of it trails into a later reversal). */
static bool emit_step_um(bp_pointer *p,int32_t ux,int32_t uy) {
    int64_t ax=(int64_t)ux*p->config.gain_q8*256,ay=(int64_t)uy*p->config.gain_q8*256;
    int32_t dx=(int32_t)(ax/65536000),dy=(int32_t)(ay/65536000);
    while(dx!=0 || dy!=0) {
        int16_t sx=(int16_t)(dx>127?127:dx < -127?-127:dx),sy=(int16_t)(dy>127?127:dy < -127?-127:dy);
        if(!queue_one(p,p->local_buttons,sx,sy)) return false;
        dx-=sx;dy-=sy;
    }
    return true;
}
static bool click_batch(bp_pointer *p,unsigned clicks) {
    if(clicks>2u || p->queued>BP_QUEUE_CAP-2u*clicks)return false;
    for(unsigned i=0;i<clicks;i++) {
        if(!queue_one(p,1u,0,0) || !queue_one(p,0u,0,0))return false;
        ++p->tap_left;
    }
    return true;
}
static bp_result finish_drag(bp_pointer *p) {
    discard_motion(p);
    if(!queue_one(p,0u,0,0)) {cancel(p);return BP_BACKPRESSURE;}
    p->local_buttons=0;p->mode=BP_IDLE;p->second_tap=false;p->lock_rejoin=false;
    p->tap_tracking=false;p->tap_rejoin_pending=false;p->motion_active=false;return BP_OK;
}
/* The first tap has already been sent. Abandon only the second contact's drag
 * ownership; strength recovery must neither replay that click nor resurrect drag. */
static bp_result movement_settle_first(bp_pointer *p) {
    if(p->lock_rejoin){bp_result r=finish_drag(p);if(r!=BP_OK)return r;}
    ++p->movement_confidence_intent_cancels;p->movement_confidence_since=0;
    p->second_tap=false;p->lock_rejoin=false;p->mode=BP_MOVING;
    p->tap_tracking=false;p->hold_eligible=false;p->moved=true;
    p->motion_active=true;p->landing_ready=true;discard_motion(p);return BP_OK;
}
/* A lost motion qualification cannot hold an already pressed button forever.
 * This uses the existing ordered-up path and never turns strength decline into
 * a click. Actual zero still uses the original release debounce. */
static bp_result movement_timeout(bp_pointer *p,uint64_t now) {
    if(!movement_enabled(p) || !p->movement_confidence_since ||
       now-p->movement_confidence_since<BP_MOTION_CONFIDENCE_DRAG_MS)return BP_OK;
    if(!(p->mode==BP_DRAG || p->lock_rejoin || p->local_buttons)) {
        if(movement_pending_intent(p))
            return movement_settle_first(p);
        return BP_OK;
    }
    bp_result r=finish_drag(p);if(r!=BP_OK)return r;
    ++p->movement_confidence_drag_releases;p->movement_confidence_since=0;
    p->mode=BP_MOVING;p->moved=true;p->tap_tracking=false;p->hold_eligible=false;
    p->motion_active=true;p->landing_ready=true;return BP_OK;
}
static int32_t magnitude(int32_t v) { return v<0 ? -v:v; }
static bool outside(int32_t x,int32_t y,int32_t ox,int32_t oy,uint16_t radius) {
    int64_t dx=(int64_t)x-ox,dy=(int64_t)y-oy;
    return dx*dx+dy*dy>(int64_t)radius*radius;
}
/* Distance in physical sensor coordinates (36 x 61.6mm), before rotation. */
static void remember_primary(bp_pointer *p,const bp_contact *c,uint64_t ms,bool reset) {
    if(reset) {p->primary_history_count=0;p->primary_history_next=0;}
    unsigned i=p->primary_history_next;
    p->primary_history[i].x=c->x;p->primary_history[i].y=c->y;p->primary_history[i].ms=ms;
    p->primary_history_next=(i+1u)%4u;if(p->primary_history_count<4u) p->primary_history_count++;
}
static bool continuous_contact(const bp_pointer *p,const bp_sample *s,const bp_contact *c) {
    if(s->time_ms<=p->contact_ms || s->time_ms-p->contact_ms>(p->primary_missing?BP_PRIMARY_MISSING_MS:BP_CONTACT_CONTINUITY_MS)) return false;
    /* A disappearing ID's last coordinates can already be a split artefact.
     * Match only a unique replacement near the recent <=60ms physical trail;
     * reanchor on transfer, never emit the old-to-new offset. */
    int64_t radius=(int64_t)BP_CONTACT_CONTINUITY_MM*100000;
    for(unsigned i=0;i<p->primary_history_count;i++) {
        if(s->time_ms<=p->primary_history[i].ms || s->time_ms-p->primary_history[i].ms>BP_CONTACT_CONTINUITY_MS) continue;
        int64_t dx=((int64_t)c->x-p->primary_history[i].x)*(BP_PHYSICAL_X_UM/100);
        int64_t dy=((int64_t)c->y-p->primary_history[i].y)*(BP_PHYSICAL_Y_UM/100);
        if(dx*dx+dy*dy<=radius*radius) return true;
    }
    return false;
}
/* Integer Euclidean length keeps equal physical speeds axis-independent. */
static uint32_t length_um(int32_t dx,int32_t dy) {
    uint64_t n=(uint64_t)((int64_t)dx*dx+(int64_t)dy*dy),r=0,bit=UINT64_C(1)<<62;
    while(bit>n) bit>>=2;
    while(bit) {if(n>=r+bit) {n-=r+bit;r=(r>>1)+bit;} else r>>=1;bit>>=2;}
    return (uint32_t)r;
}
static uint16_t motion_gain(bp_pointer *p,int32_t dx,int32_t dy,uint64_t ms) {
    if(p->config.adaptive_motion_gain) {
        /* Drag retains its original mapping. Never carry velocity across
         * release/identity/weakness boundaries (reset_speed owns those). */
        if(p->local_buttons || p->mode==BP_DRAG || p->mode==BP_DRAG_LOCK)return 256u;
        uint64_t age=ms-p->speed_ms;
        bool fresh=!p->speed_ms || ms<=p->speed_ms || age>BP_CONTACT_CONTINUITY_MS;
        uint32_t dt=fresh?10u:(uint32_t)age;
        uint32_t v=(uint32_t)((uint64_t)length_um(dx,dy)*256u/dt);
        if(v>400u*256u)v=400u*256u;
        bool reverse=(int64_t)dx*p->speed_last_dx+(int64_t)dy*p->speed_last_dy<0;
        if(fresh || reverse)p->adaptive_speed_q8=v;
        else {
            /* Only smooth the gain estimate, never defer a displacement.
             * Fast decay prevents a travel gain lingering into fine aiming. */
            uint32_t tau=v>p->adaptive_speed_q8?12u:4u;
            p->adaptive_speed_q8=(uint32_t)((int64_t)p->adaptive_speed_q8+
                ((int64_t)v-p->adaptive_speed_q8)*dt/(tau+dt));
        }
        p->speed_ms=ms;p->speed_last_dx=dx;p->speed_last_dy=dy;
        uint32_t speed=p->adaptive_speed_q8;
        uint32_t lo,hi;uint16_t a,b;
        if(speed<=8u*256u)return 230u; /*0.90x: no low-speed dead zone*/
        if(speed>=100u*256u)return 448u; /*1.75x: bounded travel gain*/
        if(speed<25u*256u){lo=8u*256u;hi=25u*256u;a=230u;b=256u;}
        else {lo=25u*256u;hi=100u*256u;a=256u;b=448u;}
        uint32_t t=(speed-lo)*256u/(hi-lo);
        uint32_t smooth=t*t*(768u-2u*t)/65536u;
        return (uint16_t)(a+(b-a)*smooth/256u);
    }
    if(p->config.fixed_motion_gain)return 256u;
    bool startup=p->config.responsive_startup && ms-p->motion_started_ms<BP_START_UNITY_MS;
    uint16_t fine=p->config.responsive_startup?BP_START_FINE_Q8:BP_FINE_MIN_Q8;
    uint32_t fine_speed=p->config.responsive_startup?BP_START_FINE_LOW_MM_S:BP_FINE_LOW_MM_S;
    uint64_t previous=p->speed_ms;
    bool reversal=(dx||dy) && (p->speed_last_dx||p->speed_last_dy) &&
        (int64_t)dx*p->speed_last_dx+(int64_t)dy*p->speed_last_dy<=0;
    if(!previous || ms<=previous || ms-previous>BP_CONTACT_CONTINUITY_MS || reversal) {
        reset_speed(p);p->speed_ms=ms;p->speed_last_dx=dx;p->speed_last_dy=dy;return p->config.pointer_acceleration&&!startup?fine:256u;
    }
    p->speed_ms=ms;
    if(dx||dy) {p->speed_last_dx=dx;p->speed_last_dy=dy;}
    p->speed_samples[p->speed_index].dx=dx;
    p->speed_samples[p->speed_index].dy=dy;
    p->speed_samples[p->speed_index].dt_ms=(uint32_t)(ms-previous);
    p->speed_index=(p->speed_index+1u)%BP_SPEED_WINDOW;
    if(p->speed_count<BP_SPEED_WINDOW) ++p->speed_count;
    uint64_t dt=0;int32_t sum_x=0,sum_y=0;
    /* Time-windowed NET velocity: back-and-forth noise is not travel speed. */
    for(unsigned n=0;n<p->speed_count;n++) {
        unsigned i=(p->speed_index+BP_SPEED_WINDOW-1u-n)%BP_SPEED_WINDOW;
        sum_x+=p->speed_samples[i].dx;sum_y+=p->speed_samples[i].dy;
        dt+=p->speed_samples[i].dt_ms;
        if(dt>=BP_SPEED_SPAN_MS) break;
    }
    if(!p->config.pointer_acceleration || startup) return 256u;
    if(p->speed_count<3u || !dt) return fine;
    uint64_t speed=length_um(sum_x,sum_y)/dt;
    if(speed<=fine_speed) return fine;
    if(speed<=BP_ACCEL_LOW_MM_S) return (uint16_t)(fine+(speed-fine_speed)*(256u-fine)/(BP_ACCEL_LOW_MM_S-fine_speed));
    if(speed>=BP_ACCEL_HIGH_MM_S) return BP_ACCEL_MAX_Q8;
    return (uint16_t)(256u+(speed-BP_ACCEL_LOW_MM_S)*(BP_ACCEL_MAX_Q8-256u)/(BP_ACCEL_HIGH_MM_S-BP_ACCEL_LOW_MM_S));
}
/* Per-axis hysteresis holds the last extreme until a real reversal exceeds
 * 0.3mm. Discard the deadband instead of emitting a catch-up jump. */
static int32_t reverse_gate(int32_t value,int32_t *last,int8_t *direction,uint32_t deadband) {
    int32_t delta=value-*last;
    int8_t sign=delta>0?1:delta<0?-1:0;
    if(sign && *direction && sign!=*direction) {
        if((uint32_t)magnitude(delta)<=deadband) return 0;
        *last=value;*direction=sign;
        return delta-sign*(int32_t)deadband;
    }
    *last=value;if(sign) *direction=sign;return delta;
}
static void smooth_motion(bp_pointer *p,int32_t x,int32_t y,uint64_t ms,int32_t *dx,int32_t *dy) {
    if(!p->position_ready || ms<=p->position_ms || ms-p->position_ms>BP_CONTACT_CONTINUITY_MS) {
        p->position_ready=true;
        p->position_x_q8=(int64_t)p->anchor_x*256;p->position_y_q8=(int64_t)p->anchor_y*256;
        p->position_out_x=p->anchor_x;p->position_out_y=p->anchor_y;
        p->velocity_x_q8=0;p->velocity_y_q8=0;p->direction_x=0;p->direction_y=0;
        p->position_ms=ms>15?ms-15:0;
    }
    uint32_t dt=(uint32_t)(ms-p->position_ms);if(!dt) dt=1;
    int32_t vx=(int32_t)((int64_t)(x-p->anchor_x)*256/dt);
    int32_t vy=(int32_t)((int64_t)(y-p->anchor_y)*256/dt);
    uint32_t velocity_tau=BP_SPEED_SPAN_MS;
    if(p->config.responsive_startup && length_um(vx,vy)>length_um(p->velocity_x_q8,p->velocity_y_q8)) velocity_tau=BP_RISING_SPEED_TAU_MS;
    p->velocity_x_q8+=(int32_t)((int64_t)(vx-p->velocity_x_q8)*dt/(velocity_tau+dt));
    p->velocity_y_q8+=(int32_t)((int64_t)(vy-p->velocity_y_q8)*dt/(velocity_tau+dt));
    uint32_t v=length_um(p->velocity_x_q8,p->velocity_y_q8)/256;
    uint32_t slow_tau=p->config.motion_median?BP_MICRO_SLOW_TAU_MS:p->config.high_resolution?BP_HIGH_RES_SLOW_TAU_MS:BP_POSITION_SLOW_TAU_MS;
    uint32_t fast_tau=p->config.high_resolution?BP_HIGH_RES_FAST_TAU_MS:BP_POSITION_FAST_TAU_MS;
    uint32_t tau=slow_tau;
    if(v>=BP_ACCEL_HIGH_MM_S) tau=fast_tau;
    else if(v>BP_ACCEL_LOW_MM_S) tau-=(v-BP_ACCEL_LOW_MM_S)*(slow_tau-fast_tau)/(BP_ACCEL_HIGH_MM_S-BP_ACCEL_LOW_MM_S);
    p->position_x_q8+=((int64_t)x*256-p->position_x_q8)*dt/(tau+dt);
    p->position_y_q8+=((int64_t)y*256-p->position_y_q8)*dt/(tau+dt);
    *dx=reverse_gate((int32_t)(p->position_x_q8/256),&p->position_out_x,&p->direction_x,(p->config.continuous_gain || p->config.zero_reverse_deadband)?0u:p->config.high_resolution?BP_HIGH_RES_REVERSE_UM:BP_REVERSE_DEADBAND_UM);
    *dy=reverse_gate((int32_t)(p->position_y_q8/256),&p->position_out_y,&p->direction_y,(p->config.continuous_gain || p->config.zero_reverse_deadband)?0u:p->config.high_resolution?BP_HIGH_RES_REVERSE_UM:BP_REVERSE_DEADBAND_UM);
    p->position_ms=ms;
}
/* Reanchor without replaying either a rejected jump or filter/tail residue. */
static void motion_rebase_preserve_input(bp_pointer *p,int32_t x,int32_t y) {
    p->anchor_x=x;p->anchor_y=y;p->position_ready=false;
    p->resume_ms=0;memset(&p->soft,0,sizeof(p->soft));memset(&p->precise_still,0,sizeof(p->precise_still));
    p->motion_pending=0;p->remainder_x=0;p->remainder_y=0;
    memset(p->motion_delay,0,sizeof(p->motion_delay));reset_speed(p);
    memset(&p->still,0,sizeof(p->still));
}
/* True relocation discards pre-relocation coordinates. Coherent fast travel
 * uses the internal rebase without losing its already validated input history. */
static void motion_rebase(bp_pointer *p,int32_t x,int32_t y) {
    memset(&p->motion_median,0,sizeof(p->motion_median));
    motion_rebase_preserve_input(p,x,y);
}
/* Observe only the final selected single primary. Suppressed frames that
 * return earlier explicitly invalidate above; pair/scroll decisions do not
 * consume these fields. Tap median/slop processing must still run when false. */
static bool movement_observe(bp_pointer *p,const bp_sample *s,int32_t x,int32_t y) {
    if(!movement_enabled(p) || !s->strengths[0])return true;
    uint16_t strength=s->strengths[0];uint64_t id=s->contacts[0].id;
    if(p->movement_confidence_id!=id)p->movement_confidence_open=false;
    p->movement_confidence_id=id;
    if(p->movement_confidence_open && strength>=BP_MOTION_CONFIDENCE_EXIT)return true;
    if(strength>=BP_MOTION_CONFIDENCE_ENTER) {
        bool initial_landing=p->second_tap && !p->movement_confidence_seen &&
            p->movement_initial_eligible && s->time_ms-p->down_ms<=BP_START_QUIET_MS;
        p->movement_confidence_open=true;p->movement_confidence_seen=true;p->movement_confidence_since=0;
        /* Normal early strength growth within the existing50ms quiet landing
         * may preserve tap-drag intent. Long weak waits and all true recovery
         * boundaries always start a new clock/origin. XY still rebases below. */
        p->movement_qualified_since=initial_landing?p->down_ms:s->time_ms;
        p->movement_drag_x=initial_landing?p->down_x:x;p->movement_drag_y=initial_landing?p->down_y:y;
        /* Drag travel has its own post-qualification history. The original
         * tap median must retain earlier travel/slop evidence across recovery. */
        p->movement_drag_median.next=0;
        for(unsigned k=0;k<3;k++){p->movement_drag_median.x[k]=x;p->movement_drag_median.y[k]=y;}
        ++p->movement_confidence_rebases;discard_motion(p);motion_rebase(p,x,y);
        return false; /* first qualified observation is a new origin */
    }
    movement_invalidate(p,s->time_ms,true);p->movement_confidence_id=id;
    ++p->movement_confidence_suppressed;discard_motion(p);motion_rebase(p,x,y);
    return false;
}
static uint32_t guarded_speed(const bp_pointer *p,int32_t *hx,int32_t *hy) {
    uint64_t travel=0,dt=0;*hx=0;*hy=0;
    for(unsigned i=0;i<p->guard.count;i++) {
        travel+=length_um(p->guard.history[i].dx,p->guard.history[i].dy);
        dt+=p->guard.history[i].dt;
        *hx+=p->guard.history[i].dx;*hy+=p->guard.history[i].dy;
    }
    return dt?(uint32_t)(travel*256/dt):0;
}
/* A repeated new position is a reanchor, never a catch-up jump. Coherent
 * continued travel confirms acceleration after one rejected observation. */
static bool coordinate_guard(bp_pointer *p,int32_t x,int32_t y,uint64_t ms,uint32_t *cap) {
    *cap=UINT32_MAX;
    if(!p->config.spike_guard) return true;
    if(!p->guard.ready || ms<=p->guard.ms || ms-p->guard.ms>BP_CONTACT_CONTINUITY_MS) {
        memset(&p->guard,0,sizeof(p->guard));p->guard.ready=true;
        p->guard.x=p->anchor_x;p->guard.y=p->anchor_y;p->guard.ms=ms>15?ms-15:0;
    }
    uint32_t dt=(uint32_t)(ms-p->guard.ms);if(!dt)dt=1;
    int32_t dx=x-p->guard.x,dy=y-p->guard.y,hx,hy;
    uint32_t speed=guarded_speed(p,&hx,&hy),distance=length_um(dx,dy);
    uint32_t expected=(uint32_t)((uint64_t)speed*dt/256);
    if(p->config.absolute_spike) {
        /* 1.5mm flags an isolated jump. Two coherent observations can establish
         * fast travel; never repay the first rejected displacement. No low-speed
         * floor or recent-speed multiplier is used to reject ordinary motion. */
        bool accepted=distance<=BP_SPIKE_ABSOLUTE_UM;
        if(p->guard.pending && !accepted) {
            int32_t nx=x-p->guard.px,ny=y-p->guard.py;
            int32_t ox=p->guard.px-p->guard.x,oy=p->guard.py-p->guard.y;
            uint32_t nd=length_um(nx,ny),od=length_um(ox,oy);
            int64_t dot=(int64_t)nx*ox+(int64_t)ny*oy;
            if(nd<=BP_SPIKE_CONFIRM_UM) {
                motion_rebase(p,x,y);memset(&p->guard,0,sizeof(p->guard));++p->spike_reanchors;return false;
            }
            if(dot>0 && (uint64_t)dot*10u>=(uint64_t)nd*od*8u && nd*4u>=od && nd<=od*4u) {
                uint64_t previous_ms=p->guard.pms;
                motion_rebase_preserve_input(p,p->guard.px,p->guard.py);
                dx=nx;dy=ny;dt=(uint32_t)(ms-previous_ms);if(!dt)dt=1;
                p->guard.count=1;p->guard.next=1;
                p->guard.history[0].dx=ox;p->guard.history[0].dy=oy;
                p->guard.history[0].dt=(uint32_t)(previous_ms-p->guard.ms);
                ++p->spike_reanchors;accepted=true;
            }
        } else if(!accepted && p->guard.count>=2u) {
            /* Continuing fast motion must agree with the last confirmed step.
             * A reversal above the threshold gets its own two-frame confirmation. */
            unsigned j=(p->guard.next+3u)%4u;
            int32_t ox=p->guard.history[j].dx,oy=p->guard.history[j].dy;
            uint32_t od=length_um(ox,oy),ot=p->guard.history[j].dt;
            int64_t dot=(int64_t)dx*ox+(int64_t)dy*oy;
            if(ot && dot>0 && (uint64_t)dot*10u>=(uint64_t)distance*od*8u &&
               (uint64_t)distance*ot<=(uint64_t)od*dt*3u)accepted=true;
        }
        if(!accepted) {
            p->guard.pending=true;p->guard.px=x;p->guard.py=y;p->guard.pms=ms;
            p->motion_pending=0;p->remainder_x=0;p->remainder_y=0;++p->spike_rejected;return false;
        }
        unsigned i=p->guard.next;p->guard.history[i].dx=dx;p->guard.history[i].dy=dy;p->guard.history[i].dt=dt;
        p->guard.next=(i+1u)%4u;if(p->guard.count<4u)++p->guard.count;
        p->guard.pending=false;p->guard.x=x;p->guard.y=y;p->guard.ms=ms;return true;
    }
    uint32_t limit=4u*expected+BP_SPIKE_MARGIN_UM;
    if(limit<BP_SPIKE_FLOOR_UM)limit=BP_SPIKE_FLOOR_UM;
    bool oppose=(int64_t)dx*hx+(int64_t)dy*hy<0;
    bool grace=BP_EXP_RESUME_GUARD_GRACE && p->resume_ms && ms>=p->resume_ms && ms-p->resume_ms<BP_RESUME_GRACE_MS;
    bool suspicious=grace ? distance>BP_SPIKE_ABSOLUTE_UM : p->guard.count>=1 ? distance>limit ||
        (oppose && distance>BP_SPIKE_FLOOR_UM && distance>2u*expected+BP_SPIKE_MARGIN_UM) : distance>BP_SPIKE_ABSOLUTE_UM;
    if(grace && p->guard.pending) {
        /* Absolute jumps during resume need a return or a stable new anchor.
         * Do not let the ordinary coherent-acceleration path repay a jump. */
        if(distance<=BP_SPIKE_ABSOLUTE_UM)p->guard.pending=false;
        else if(length_um(x-p->guard.px,y-p->guard.py)<=BP_SPIKE_CONFIRM_UM) {
            motion_rebase(p,x,y);memset(&p->guard,0,sizeof(p->guard));
            ++p->spike_reanchors;return false;
        } else {
            p->guard.px=x;p->guard.py=y;p->guard.pms=ms;
            p->motion_pending=0;p->remainder_x=0;p->remainder_y=0;
            ++p->spike_rejected;return false;
        }
    }
    if(p->guard.pending && !grace) {
        int32_t nx=x-p->guard.px,ny=y-p->guard.py;
        uint32_t nd=length_um(nx,ny);
        uint32_t interval=(uint32_t)(ms-p->guard.pms);if(!interval)interval=1;
        uint32_t return_radius=(uint32_t)((uint64_t)speed*interval/256)+BP_SPIKE_MARGIN_UM;
        if(return_radius<BP_SPIKE_CONFIRM_UM)return_radius=BP_SPIKE_CONFIRM_UM;
        /* Never let elapsed time since rejection make the original offset
         * look like acceptable slow motion on the next observation. */
        if(distance<=return_radius) {p->guard.pending=false;suspicious=false;}
        else if(nd<=BP_SPIKE_CONFIRM_UM) {
            motion_rebase(p,x,y);memset(&p->guard,0,sizeof(p->guard));
            p->guard.ready=true;p->guard.x=x;p->guard.y=y;p->guard.ms=ms;
            ++p->spike_reanchors;return false;
        } else {
            int32_t ox=p->guard.px-p->guard.x,oy=p->guard.py-p->guard.y;
            uint32_t od=length_um(ox,oy);
            int64_t dot=(int64_t)nx*ox+(int64_t)ny*oy;
            if(dot>0 && (uint64_t)dot*2u>(uint64_t)nd*od && nd<=od*3u) {
                motion_rebase_preserve_input(p,p->guard.px,p->guard.py);
                dx=nx;dy=ny;dt=(uint32_t)(ms-p->guard.pms);if(!dt)dt=1;
                expected=nd;p->guard.count=0;p->guard.next=0;p->guard.pending=false;
                ++p->spike_reanchors;
                suspicious=false; /* preserve new coherent step, discard original offset */
            } else suspicious=true;
        }
    }
    if(suspicious) {
        p->guard.pending=true;p->guard.px=x;p->guard.py=y;p->guard.pms=ms;
        p->motion_pending=0;p->remainder_x=0;p->remainder_y=0;
        ++p->spike_rejected;return false;
    }
    /* Limit this frame using prior accepted travel; excess is discarded, not banked. */
    *cap=grace ? BP_SPIKE_ABSOLUTE_UM : p->guard.count>=1 ? 3u*expected+BP_SPIKE_MARGIN_UM : BP_SPIKE_ABSOLUTE_UM;
    if(*cap<200u)*cap=200u;
    unsigned i=p->guard.next;
    p->guard.history[i].dx=dx;p->guard.history[i].dy=dy;p->guard.history[i].dt=dt;
    p->guard.next=(i+1u)%4u;if(p->guard.count<4u)++p->guard.count;
    p->guard.x=x;p->guard.y=y;p->guard.ms=ms;
    return true;
}
static int32_t median3(int32_t a,int32_t b,int32_t c) {
    if(a>b){int32_t t=a;a=b;b=t;}if(b>c)b=c;return a>b?a:b;
}
static bool stationary_gate(bp_pointer *p,int32_t x,int32_t y,uint64_t ms) {
    if(!p->config.stationary_guard) return false;
    int32_t raw_x=x,raw_y=y;
    unsigned h=p->still.history_next;p->still.history_x[h]=x;p->still.history_y[h]=y;
    p->still.history_next=(h+1u)%3u;if(p->still.history_count<3u)++p->still.history_count;
    if(p->still.history_count==3u) {
        x=median3(p->still.history_x[0],p->still.history_x[1],p->still.history_x[2]);
        y=median3(p->still.history_y[0],p->still.history_y[1],p->still.history_y[2]);
    }
    if(!p->still.ready) {
        p->still.ready=true;p->still.x=x;p->still.y=y;p->still.ms=ms;
        p->still.anchor_x=x;p->still.anchor_y=y;return false;
    }
    if(p->still.locked) {
        int32_t dx=x-p->still.anchor_x,dy=y-p->still.anchor_y;
        uint32_t radius=length_um(dx,dy);
        uint32_t unlock=p->config.micro_resume?BP_MICRO_UNLOCK_UM:BP_STILL_UNLOCK_UM;
        if(radius>unlock) {
            bool same_direction=(int64_t)dx*p->still.unlock_x+(int64_t)dy*p->still.unlock_y>0;
            bool monotonic=radius>=length_um(p->still.unlock_x,p->still.unlock_y);
            bool confirmed=p->config.micro_resume?(p->still.confirmations>=2u && same_direction && monotonic):p->still.confirmations!=0u;
            if(confirmed && ms-p->still.unlock_since>=20u &&
               (int64_t)dx*p->still.unlock_x+(int64_t)dy*p->still.unlock_y>0) {
                /* No repayment of displacement accumulated while stationary. */
                p->hold_eligible=false;
                motion_rebase(p,BP_EXP_RESUME_RAW_ANCHOR?raw_x:x,BP_EXP_RESUME_RAW_ANCHOR?raw_y:y);
                p->resume_ms=ms;p->guard.ready=false;return true;
            }
            if(!p->still.confirmations || (int64_t)dx*p->still.unlock_x+(int64_t)dy*p->still.unlock_y<=0)p->still.unlock_since=ms;
            p->still.confirmations=p->config.micro_resume && same_direction && monotonic?p->still.confirmations+1u:1u;
            p->still.unlock_x=dx;p->still.unlock_y=dy;
        } else p->still.confirmations=0;
        return true;
    }
    if(p->config.precise_stationary) {
        /* Net displacement over a real time window, not per-frame speed.
         * Coherent creep remains motion even below the nominal still threshold. */
        if(!p->precise_still.ready || ms<=p->precise_still.ms) {
            p->precise_still.ready=true;p->precise_still.ms=ms;
            p->precise_still.x=p->precise_still.last_x=x;
            p->precise_still.y=p->precise_still.last_y=y;p->precise_still.travel=0;return false;
        }
        p->precise_still.travel+=length_um(x-p->precise_still.last_x,y-p->precise_still.last_y);
        p->precise_still.last_x=x;p->precise_still.last_y=y;
        if(ms-p->precise_still.ms<BP_PRECISE_STILL_MS)return false;
        uint32_t net=length_um(x-p->precise_still.x,y-p->precise_still.y);
        bool coherent=p->precise_still.travel && (uint64_t)net*10u>=p->precise_still.travel*9u;
        uint64_t travel=p->precise_still.travel;
        p->precise_still.ms=ms;p->precise_still.x=x;p->precise_still.y=y;p->precise_still.travel=0;
        if(net>=BP_PRECISE_STILL_NET_UM || coherent || (p->config.travel_still && travel>=BP_PRECISE_STILL_TRAVEL_UM))return false;
        p->still.locked=true;p->still.anchor_x=x;p->still.anchor_y=y;
        p->motion_pending=0;p->remainder_x=0;p->remainder_y=0;++p->stationary_entries;return true;
    }
    uint64_t dt=ms-p->still.ms;
    bool slow=dt && length_um(x-p->still.x,y-p->still.y)<BP_STILL_SPEED_MM_S*dt;
    p->still.x=x;p->still.y=y;p->still.ms=ms;
    if(!slow || length_um(x-p->still.anchor_x,y-p->still.anchor_y)>BP_STILL_RADIUS_UM) {
        p->still.since=ms;p->still.anchor_x=x;p->still.anchor_y=y;return false;
    }
    if(!p->still.since)p->still.since=ms;
    if(ms-p->still.since>=BP_STILL_TIME_MS) {
        p->still.locked=true;p->still.anchor_x=x;p->still.anchor_y=y;
        p->motion_pending=0;p->remainder_x=0;p->remainder_y=0;++p->stationary_entries;
        return true;
    }
    return false;
}

static int32_t median5(const int32_t *values) {
    int32_t a[5];memcpy(a,values,sizeof(a));
    for(unsigned i=1;i<5;i++){int32_t v=a[i];unsigned j=i;while(j && a[j-1]>v){a[j]=a[j-1];--j;}a[j]=v;}
    return a[2];
}
static uint16_t resume_gain(const bp_pointer *p,uint64_t ms) {
    if(!BP_EXP_RESUME_RAMP || !p->resume_ms || ms<p->resume_ms || ms-p->resume_ms>=BP_RESUME_RAMP_MS)return 256u;
    return BP_RESUME_START_Q8+(uint16_t)((ms-p->resume_ms)*(256u-BP_RESUME_START_Q8)/BP_RESUME_RAMP_MS);
}
static uint16_t soft_gain(bp_pointer *p,int32_t x,int32_t y,uint64_t ms) {
    if(p->config.adaptive_motion_gain)return 256u;
    if(p->config.fixed_motion_gain)return 256u;
    if(!p->config.soft_low_speed && !p->config.continuous_gain)return 256u;
    unsigned i=p->soft.next;p->soft.x[i]=x;p->soft.y[i]=y;
    p->soft.next=(i+1u)%5u;if(p->soft.count<5u)++p->soft.count;
    if(p->soft.count<5u)return p->config.continuous_gain?102u:256u;
    int32_t mx=median5(p->soft.x),my=median5(p->soft.y);
    uint64_t dt=ms-p->soft.ms;
    if(!p->soft.ms || ms<=p->soft.ms || dt>BP_CONTACT_CONTINUITY_MS) {
        p->soft.ms=ms;p->soft.last_x=mx;p->soft.last_y=my;p->soft.vx_q8=0;p->soft.vy_q8=0;return p->config.continuous_gain?102u:256u;
    }
    int32_t vx=(int32_t)((int64_t)(mx-p->soft.last_x)*256/(int64_t)dt);
    int32_t vy=(int32_t)((int64_t)(my-p->soft.last_y)*256/(int64_t)dt);
    p->soft.vx_q8+=(int32_t)((int64_t)(vx-p->soft.vx_q8)*(int64_t)dt/(int64_t)(BP_SPEED_SPAN_MS+dt));
    p->soft.vy_q8+=(int32_t)((int64_t)(vy-p->soft.vy_q8)*(int64_t)dt/(int64_t)(BP_SPEED_SPAN_MS+dt));
    p->soft.ms=ms;p->soft.last_x=mx;p->soft.last_y=my;
    if(!p->config.continuous_gain && (ms-p->motion_started_ms<BP_START_UNITY_MS || (p->resume_ms && ms-p->resume_ms<BP_RESUME_GRACE_MS)))return 256u;
    uint32_t speed=length_um(p->soft.vx_q8,p->soft.vy_q8);
    if(p->config.continuous_gain) {
        if(speed<=2u*256u)return 102u;
        if(speed<20u*256u)return (uint16_t)(102u+(speed-2u*256u)*154u/(18u*256u));
        if(speed<60u*256u)return (uint16_t)(256u+(speed-20u*256u)*154u/(40u*256u));
        return 410u;
    }
    if(speed<=BP_SOFT_LOW_MM_S*256u)return BP_SOFT_MIN_Q8;
    if(speed>=BP_SOFT_HIGH_MM_S*256u)return 256u;
    uint32_t t=(speed-BP_SOFT_LOW_MM_S*256u)/(BP_SOFT_HIGH_MM_S-BP_SOFT_LOW_MM_S);
    uint32_t smooth=t*t*(768u-2u*t)/65536u;
    return (uint16_t)(BP_SOFT_MIN_Q8+(256u-BP_SOFT_MIN_Q8)*smooth/256u);
}
/* Keep the configured number of fresh displacements behind the sensor. Release, weakness
 * and identity changes discard these frames; flush/tick never drain them. */
static bool emit_motion(bp_pointer *p,int32_t x,int32_t y,uint64_t ms) {
    /* Median input is shared by velocity, position, and stationary decisions.
     * No strength-based gain or stored unlock displacement is introduced. */
    if(p->config.motion_median) {
        if(!p->motion_median.count || ms<=p->motion_median.ms || ms-p->motion_median.ms>BP_CONTACT_CONTINUITY_MS) {
            for(unsigned k=0;k<3;k++){p->motion_median.x[k]=p->anchor_x;p->motion_median.y[k]=p->anchor_y;}
            p->motion_median.count=3;p->motion_median.next=0;
        }
        unsigned k=p->motion_median.next;
        p->motion_median.x[k]=x;p->motion_median.y[k]=y;p->motion_median.next=(k+1u)%3u;p->motion_median.ms=ms;
        x=median3(p->motion_median.x[0],p->motion_median.x[1],p->motion_median.x[2]);
        y=median3(p->motion_median.y[0],p->motion_median.y[1],p->motion_median.y[2]);
    }

    /* A spike rebase must never clear the stationary latch. While locked,
     * only the temporal median/directional confirmation may unlock it. */
    if(p->config.stationary_guard && p->still.locked) {
        (void)stationary_gate(p,x,y,ms);return true;
    }
    uint32_t frame_cap;
    if(!coordinate_guard(p,x,y,ms,&frame_cap)) return true;
    if(!p->motion_started_ms) p->motion_started_ms=ms;
    unsigned delay=p->config.responsive_startup?BP_START_DELAY_FRAMES:BP_MOTION_DELAY_FRAMES;
    int32_t incoming_x=x-p->anchor_x,incoming_y=y-p->anchor_y;
    if(p->config.motion_smoothing) smooth_motion(p,x,y,ms,&incoming_x,&incoming_y);
    p->anchor_x=x;p->anchor_y=y;
    int32_t fx=p->config.motion_smoothing?(int32_t)(p->position_x_q8/256):x;
    int32_t fy=p->config.motion_smoothing?(int32_t)(p->position_y_q8/256):y;
    if(stationary_gate(p,fx,fy,ms)) return true;
    uint32_t travel=length_um(incoming_x,incoming_y);
    if(travel>frame_cap) {
        incoming_x=(int32_t)((int64_t)incoming_x*frame_cap/travel);
        incoming_y=(int32_t)((int64_t)incoming_y*frame_cap/travel);
        /* Position filter retains no unreported large displacement to leak later. */
        p->position_x_q8=(int64_t)x*256;p->position_y_q8=(int64_t)y*256;
        p->position_out_x=x;p->position_out_y=y;++p->motion_clipped;
    }
    uint16_t gain=motion_gain(p,incoming_x,incoming_y,ms);
    if(!p->config.adaptive_motion_gain && !p->config.continuous_gain && !p->config.fixed_motion_gain)gain=(uint16_t)((uint32_t)gain*resume_gain(p,ms)/256u);
    gain=(uint16_t)((uint32_t)gain*soft_gain(p,x,y,ms)/256u);
    if(p->motion_pending<delay) {
        p->motion_delay[p->motion_pending].dx=incoming_x;
        p->motion_delay[p->motion_pending].dy=incoming_y;
        p->motion_delay[p->motion_pending++].gain_q8=gain;return true;
    }
    int32_t mature_x=p->motion_delay[0].dx,mature_y=p->motion_delay[0].dy;
    uint16_t mature_gain=p->motion_delay[0].gain_q8;
    for(unsigned i=1;i<delay;i++) p->motion_delay[i-1]=p->motion_delay[i];
    p->motion_delay[delay-1].dx=incoming_x;p->motion_delay[delay-1].dy=incoming_y;p->motion_delay[delay-1].gain_q8=gain;
    int64_t ax=(int64_t)mature_x*p->config.gain_q8*mature_gain+p->remainder_x;
    int64_t ay=(int64_t)mature_y*p->config.gain_q8*mature_gain+p->remainder_y;
    /* Nearest HID count keeps reversal error within half a count without
     * changing gain. Half-count ties stay in the residual (otherwise a
     * constant input can alternate +1/-1). Carry the signed residual. */
    int32_t qx=(int32_t)((ax+(ax>=0?32767999:-32767999))/65536000);
    int32_t qy=(int32_t)((ay+(ay>=0?32767999:-32767999))/65536000);
    int32_t dx=qx,dy=qy;
    unsigned needed=(unsigned)((magnitude(dx)>magnitude(dy)?magnitude(dx):magnitude(dy))+126)/127u;
    if(p->queued>BP_QUEUE_CAP-2u || needed>BP_QUEUE_CAP-2u-p->queued) return false;
    /* A committed nonzero cursor step prevents a later synthetic tap, even
     * when the entire contact remained inside the2mm tap displacement slop. */
    if(dx || dy) {p->moved=true;if(p->mode==BP_CANDIDATE) p->mode=BP_MOVING;}
    while(dx!=0 || dy!=0) {
        int16_t sx=(int16_t)(dx>127?127:dx < -127?-127:dx);
        int16_t sy=(int16_t)(dy>127?127:dy < -127?-127:dy);
        if(!queue_one(p,p->local_buttons,sx,sy)) return false;
        dx-=sx;dy-=sy;
    }
    p->remainder_x=(int32_t)(ax-(int64_t)qx*65536000);
    p->remainder_y=(int32_t)(ay-(int64_t)qy*65536000);return true;
}
static void transform(const bp_config *c,const bp_contact *in,int32_t *x,int32_t *y) {
    int32_t a=(int32_t)(((int64_t)in->x*BP_PHYSICAL_X_UM+BP_COORD_MAX/2)/BP_COORD_MAX);
    int32_t b=(int32_t)(((int64_t)in->y*BP_PHYSICAL_Y_UM+BP_COORD_MAX/2)/BP_COORD_MAX);
    int32_t width=BP_PHYSICAL_X_UM,height=BP_PHYSICAL_Y_UM;
    switch(c->rotation) {
    case 90: *x=height-b;*y=a;break;
    case 180: *x=width-a;*y=height-b;break;
    case 270: *x=b;*y=width-a;break;
    default: *x=a;*y=b;break;
    }
    if(c->rotation==90 || c->rotation==270) {width=BP_PHYSICAL_Y_UM;height=BP_PHYSICAL_X_UM;}
    if(c->invert_x) *x=width-*x;
    if(c->invert_y) *y=height-*y;
}
/* An IQS ID is not proof of spatial continuity: preserve the last trusted
 * position when the same ID teleports. During landing the quiet guard already
 * suppresses output. Afterwards use the existing physical8mm continuity bound;
 * a same-ID discontinuity is an invalid motion frame, not an extra finger. */
static bool primary_position_ok(const bp_pointer *p,const bp_sample *s,const bp_contact *c) {
    if(s->time_ms-p->down_ms<(p->config.responsive_startup?BP_START_QUIET_MS:BP_LANDING_QUIET_MS)) return true;
    int64_t dx=((int64_t)c->x-p->last_raw_x)*(BP_PHYSICAL_X_UM/100);
    int64_t dy=((int64_t)c->y-p->last_raw_y)*(BP_PHYSICAL_Y_UM/100);
    int64_t radius=(int64_t)BP_CONTACT_CONTINUITY_MM*100000;
    return dx*dx+dy*dy<=radius*radius;
}
static unsigned primary_index(const bp_pointer *p,const bp_sample *s) {
    for(unsigned i=0;i<s->count;i++) if(s->contacts[i].id==p->contact_id) return primary_position_ok(p,s,&s->contacts[i])?i:5;
    unsigned found=5,n=0;
    for(unsigned i=0;i<s->count;i++) if(continuous_contact(p,s,&s->contacts[i])) {found=i;n++;}
    return n==1?found:5;
}
static bp_result primary_absent(bp_pointer *p,const bp_sample *s) {
    movement_boundary(p,s->time_ms,true);
    if(p->tap_tracking && s->kind==BP_SUPPRESSED) {++p->tap_reject[4];p->tap_tracking=false;}
    p->hold_eligible=false;discard_motion(p);p->pair_tap_possible=false;p->moved=true;
    if(!p->primary_missing) {p->primary_missing=true;p->primary_missing_since=s->time_ms;}
    if(s->time_ms-p->primary_missing_since>=BP_PRIMARY_MISSING_MS) {cancel(p);return BP_FAULT;}
    return BP_WAITING;
}
static void degrade_primary(bp_pointer *p,const bp_sample *s,unsigned i) {
    movement_boundary(p,s->time_ms,true);
    /* This branch explicitly abandons tap eligibility. Record the reason
     * here rather than leaving a tracking flag alive until some later lift. */
    if(p->tap_tracking) {++p->tap_reject[6];p->tap_tracking=false;}
    int32_t x,y;transform(&p->config,&s->contacts[i],&x,&y);
    bool dragging=p->mode==BP_DRAG && p->local_buttons==1u;
    p->hold_eligible=false;discard_motion(p);p->mode=dragging?BP_DRAG:BP_MOVING;p->single_only=true;p->pair_tap_possible=false;
    p->moved=true;p->pending_pair_frames=0;memset(p->extras,0,sizeof(p->extras));
    remember_primary(p,&s->contacts[i],s->time_ms,true);
    p->contact_id=s->contacts[i].id;p->last_raw_x=s->contacts[i].x;p->last_raw_y=s->contacts[i].y;p->contact_ms=s->time_ms;
    p->anchor_x=x;p->anchor_y=y;p->down_x=x;p->down_y=y;
    p->motion_active=false;p->primary_missing=false;
    /* Degradation has selected and rebased this single primary. A verified
     * strong return clears a prior loss deadline here before the early return;
     * weak return retains that deadline and cannot regain the onset exception. */
    if(!(s->weak_contacts&(1u<<i))) {
        bp_sample selected=*s;selected.count=1;selected.contacts[0]=s->contacts[i];
        selected.strengths[0]=s->strengths[i];selected.weak_contacts=0;
        (void)movement_observe(p,&selected,x,y);
    }
}
static bool valid_sample(const bp_sample *s) {
    if((s->kind!=BP_DATA && s->kind!=BP_SUPPRESSED) ||
       (s->kind==BP_SUPPRESSED && s->count!=0u) || s->count>5u || s->session==0u ||
       (s->weak_contacts & ~((1u<<s->count)-1u))) return false;
    if(s->motion_intent_id && (s->count!=1u || s->weak_contacts ||
       s->motion_intent_id!=s->contacts[0].id)) return false;
    for(unsigned i=0u;i<s->count;++i) {
        if(s->contacts[i].id==0u || s->contacts[i].x<0 || s->contacts[i].x>BP_COORD_MAX ||
           s->contacts[i].y<0 || s->contacts[i].y>BP_COORD_MAX) return false;
        for(unsigned j=0u;j<i;++j) if(s->contacts[i].id==s->contacts[j].id) return false;
    }
    return true;
}
/* Confidence loss never creates a tap. The confirmed first tap was already
 * emitted; only release or abandon the current drag/second-contact ownership. */
static bp_result confidence_timeout(bp_pointer *p,uint64_t now) {
    if(!p->config.primary_confidence_gate || !p->primary_confidence_paused ||
       !p->primary_confidence_since || now-p->primary_confidence_since<p->config.primary_confidence_drag_ms)
        return BP_OK;
    if(p->mode==BP_DRAG || (p->second_tap && p->lock_rejoin)) {
        bp_result r=finish_drag(p);if(r!=BP_OK)return r;
        ++p->primary_confidence_drag_releases;
    } else if(p->mode==BP_CANDIDATE && p->second_tap) {
        p->second_tap=false;
    } else return BP_OK;
    /* The same weak contact cannot arm another click/drag on recovery. */
    p->mode=BP_MOVING;p->moved=true;p->tap_tracking=false;p->hold_eligible=false;
    p->motion_active=true;p->landing_ready=true;return BP_OK;
}
/* Called only for the single-pointer path. A zero strength is unspecified
 * synthetic input and keeps its historical behavior. Real admitted A000
 * contacts carry nonzero strength. Pair/scroll confirmation remains separate. */
static bp_result primary_confidence(bp_pointer *p,const bp_sample *s,unsigned i,
                                    uint64_t now,bool *handled) {
    *handled=false;
    if(!p->config.primary_confidence_gate || p->scroll_latched ||
       p->mode==BP_SCROLL || p->mode==BP_PAIR_RELEASE || p->mode==BP_WAIT_ZERO)return BP_OK;
    bool suppressed=s->kind==BP_SUPPRESSED;
    uint16_t strength=suppressed?0u:s->strengths[i];
    if(!suppressed && !strength)return BP_OK;
    if(p->mode==BP_IDLE || p->mode==BP_DRAG_LOCK) {
        if(!suppressed && strength>=p->config.primary_confidence_enter)return BP_OK;
        movement_boundary(p,s->time_ms,true);
        ++p->primary_confidence_suppressed;discard_motion(p);*handled=true;return BP_IGNORED;
    }
    if(!suppressed && !p->primary_confidence_paused && strength>=p->config.primary_confidence_exit)return BP_OK;
    *handled=true;
    if(!suppressed && p->primary_confidence_paused && strength>=p->config.primary_confidence_enter) {
        int32_t x,y;transform(&p->config,&s->contacts[i],&x,&y);
        /* An expired/far lifetime cannot inherit a held button or tap. It is
         * still a valid strong observation: release in queue order and start
         * a fresh movement anchor, without waiting for a complete lift. */
        if(s->contacts[i].id!=p->contact_id && !continuous_contact(p,s,&s->contacts[i])) {
            if(p->mode==BP_DRAG || p->lock_rejoin || p->local_buttons) {
                bp_result r=finish_drag(p);if(r!=BP_OK)return r;
                ++p->primary_confidence_drag_releases;
            } else if(p->second_tap) {
                p->second_tap=false;
            }
            if(p->tap_tracking)++p->tap_reject[6];
            p->tap_tracking=false;p->tap_ambiguous=false;p->tap_second_seen=false;
            p->moved=true;p->mode=BP_MOVING;p->motion_active=false;p->landing_ready=false;
            p->single_only=false;p->pair_tap_possible=false;p->pair_pause_active=false;
            p->pending_pair_frames=0;memset(p->extras,0,sizeof(p->extras));
            p->down_ms=s->time_ms;p->down_x=x;p->down_y=y;
            p->landing_frames=1;p->landing_last_ms=s->time_ms;
        }
        /* A nearby verified lifetime rebirth after a zero is not tap travel.
         * The original tap origin/slop still decides whether it relocated. */
        bool relocated=outside(x,y,p->down_x,p->down_y,p->config.tap_slop);
        if(relocated && p->mode==BP_CANDIDATE) {
            if(p->second_tap) {
                if(p->lock_rejoin){bp_result r=finish_drag(p);if(r!=BP_OK)return r;}
                p->second_tap=false;
            }
            if(p->tap_tracking)++p->tap_reject[1];
            p->tap_tracking=false;p->moved=true;p->mode=BP_MOVING;
        }
        discard_motion(p);motion_rebase(p,x,y);
        p->contact_id=s->contacts[i].id;p->contact_ms=s->time_ms;
        p->last_raw_x=s->contacts[i].x;p->last_raw_y=s->contacts[i].y;
        remember_primary(p,&s->contacts[i],s->time_ms,true);
        p->primary_missing=false;p->hold_eligible=false;
        p->primary_confidence_paused=false;p->primary_confidence_since=0;
        ++p->primary_confidence_rebases;
        /* This early recovery return already has the verified single primary.
         * Requalify independent movement here so a recovered strong drag does
         * not retain the old weak-signal release deadline until its next frame. */
        (void)movement_observe(p,s,x,y);return BP_OK;
    }
    if(!p->primary_confidence_paused || !p->primary_confidence_since) {
        p->primary_confidence_since=s->time_ms;++p->primary_confidence_pauses;
    }
    movement_boundary(p,s->time_ms,true);
    p->primary_confidence_paused=true;++p->primary_confidence_suppressed;
    p->hold_eligible=false;discard_motion(p);
    if(!suppressed)transform(&p->config,&s->contacts[i],&p->anchor_x,&p->anchor_y);
    bp_result r=confidence_timeout(p,now);return r==BP_OK?BP_IGNORED:r;
}
/* scroll_incremental: the running scroll position. A member with a previous position adds its own travel; a lone
 * member adds twice its travel (the pair sum moves by both fingers); a one-frame step over BP_SCROLL_JUMP_UM (the
 * chip merging or splitting the pair, a slot swap) restarts that member without travel. */
/* start-hysteresis evidence (v0.20.6): each member's own travel, weak frames included. The lighter finger at ATI 250 is
 * weak in many frames; counting only its trusted frames left the coherent start unreached and the push waited for the
 * BP_SCROLL_START_UM sum (220..340 ms to the first detent on the board at 08:25). Positions only, never wheel. */
static void scroll_coherence(bp_pointer *p,bool known_lo,int32_t ylo,bool known_hi,int32_t yhi) {
    if(known_lo && p->scroll_qok[0] && magnitude(ylo-p->scroll_qy[0])<=BP_SCROLL_JUMP_UM)p->scroll_mt[0]+=ylo-p->scroll_qy[0];
    if(known_hi && p->scroll_qok[1] && magnitude(yhi-p->scroll_qy[1])<=BP_SCROLL_JUMP_UM)p->scroll_mt[1]+=yhi-p->scroll_qy[1];
    if(known_lo)p->scroll_qy[0]=ylo;
    if(known_hi)p->scroll_qy[1]=yhi;
    p->scroll_qok[0]=known_lo;p->scroll_qok[1]=known_hi;
}
static int32_t scroll_track(bp_pointer *p,bool have_lo,int32_t ylo,bool have_hi,int32_t yhi) {
    bool dl=have_lo && p->scroll_pok[0] && magnitude(ylo-p->scroll_py[0])<=BP_SCROLL_JUMP_UM;
    bool dh=have_hi && p->scroll_pok[1] && magnitude(yhi-p->scroll_py[1])<=BP_SCROLL_JUMP_UM;
    int32_t d=0;
    if(dl && dh) d=(ylo-p->scroll_py[0])+(yhi-p->scroll_py[1]);
    else if(dl) d=2*(ylo-p->scroll_py[0]);
    else if(dh) d=2*(yhi-p->scroll_py[1]);
    if(have_lo) p->scroll_py[0]=ylo;
    if(have_hi) p->scroll_py[1]=yhi;
    p->scroll_pok[0]=have_lo; p->scroll_pok[1]=have_hi;
    p->scroll_vsum+=d; return p->scroll_vsum;
}
static bp_result scroll_emit(bp_pointer *p,int32_t cur,uint64_t now);
static bp_result scroll(bp_pointer *p,const bp_sample *s) {
    uint64_t a=s->contacts[0].id,b=s->contacts[1].id;
    uint64_t low=a<b?a:b,high=a<b?b:a;
    int32_t x1,y1,x2,y2;
    transform(&p->config,&s->contacts[0],&x1,&y1);
    transform(&p->config,&s->contacts[1],&x2,&y2);
    if(p->mode!=BP_SCROLL) {
        bool joining=(p->mode==BP_CANDIDATE || p->mode==BP_MOVING) &&
            (a==p->contact_id || b==p->contact_id);
        /* A late join may scroll, but cannot become a right tap. Movement of
         * the original finger on the confirmation frame also vetoes a tap. */
        if(joining && !p->moved) {
            int32_t ox=a==p->contact_id?x1:x2,oy=a==p->contact_id?y1:y2;
            p->moved=outside(ox,oy,p->down_x,p->down_y,p->config.tap_slop);
        }
        if(p->mode!=BP_IDLE && !joining) { cancel(p); return BP_FAULT; }
        p->hold_eligible=false;p->mode=BP_SCROLL;p->scroll_latched=p->config.scroll_latch; p->pair_low=low; p->pair_high=high;
        p->pair_down_ms=joining?p->down_ms:s->time_ms;
        p->pair_tap_possible=p->config.two_finger_tap && !p->moved &&
            (!joining || p->pending_pair_ms-p->down_ms<=p->config.scroll_join_ms);
        /* Pointer confidence may postpone its own landing. A confirmed pair
         * keeps the earlier admitted TOUCH history, so the pointer threshold
         * cannot delay scrolling or strand its fresh-frame count below3. */
        p->scroll_landing_ms=p->down_ms;
        if(p->config.primary_confidence_gate && p->pair_landing.frames &&
           (p->pair_landing.id==a || p->pair_landing.id==b)) {
            p->scroll_landing_ms=p->pair_landing.first_ms;
            if(p->landing_frames<p->pair_landing.frames)p->landing_frames=p->pair_landing.frames;
        }
        discard_motion(p);p->motion_active=false;p->pending_pair_frames=0;
        p->pair_low_x=a==low?x1:x2; p->pair_low_y=a==low?y1:y2;
        p->pair_high_x=a==high?x1:x2; p->pair_high_y=a==high?y1:y2;
        p->pair_scroll_frames=0;p->scroll_anchor_sum=y1+y2;
        p->scroll_last_sum=y1+y2;p->scroll_ms=s->time_ms;p->scroll_velocity_q8=0;p->scroll_remainder_q8=0;
        p->scroll_py[0]=a==low?y1:y2;p->scroll_py[1]=a==low?y2:y1;p->scroll_pok[0]=p->scroll_pok[1]=true;
        p->scroll_activity_sum=y1+y2;p->scroll_activity_ms=s->time_ms;
        p->scroll_vsum=y1+y2;p->scroll_confirmed=false;p->scroll_ps[0]=s->strengths[a==low?0:1];p->scroll_ps[1]=s->strengths[a==low?1:0];
        p->scroll_started=false;p->scroll_origin=y1+y2;p->scroll_mt[0]=p->scroll_mt[1]=0;
        p->scroll_qy[0]=p->scroll_py[0];p->scroll_qy[1]=p->scroll_py[1];p->scroll_qok[0]=p->scroll_qok[1]=true;return BP_OK;
    }
    if(low!=p->pair_low || high!=p->pair_high) { cancel(p); return BP_FAULT; }
    if(s->time_ms-p->pair_down_ms>p->config.tap_ms ||
       outside(a==low?x1:x2,a==low?y1:y2,p->pair_low_x,p->pair_low_y,p->config.tap_slop) ||
       outside(a==high?x1:x2,a==high?y1:y2,p->pair_high_x,p->pair_high_y,p->config.tap_slop))
        p->pair_tap_possible=false;
    int32_t cur=y1+y2;
    bool weak_hold=s->weak_contacts!=0;
    if(p->config.scroll_incremental) {   /* a weak member contributes no travel; it no longer rebases the pair */
        bool lw=(s->weak_contacts>>(a==low?0u:1u))&1u,hw=(s->weak_contacts>>(a==low?1u:0u))&1u;
        scroll_coherence(p,true,a==low?y1:y2,true,a==low?y2:y1);
        cur=scroll_track(p,!lw,a==low?y1:y2,!hw,a==low?y2:y1);weak_hold=lw && hw;
    }
    if(weak_hold || s->time_ms-p->scroll_landing_ms<(p->config.responsive_startup?BP_START_QUIET_MS:BP_LANDING_QUIET_MS) || p->landing_frames<BP_LANDING_FRESH_FRAMES) {
        discard_motion_ex(p,p->config.scroll_incremental);p->scroll_anchor_sum=cur;p->scroll_last_sum=cur;p->scroll_ms=s->time_ms;p->scroll_velocity_q8=0;p->scroll_remainder_q8=0;
        if(!p->scroll_started){p->scroll_origin=cur;p->scroll_mt[0]=p->scroll_mt[1]=0;}return BP_OK;
    }
    if(p->config.tap_guard && !(p->config.scroll_incremental && p->scroll_confirmed)) {
        uint16_t lo=s->strengths[0]<s->strengths[1]?s->strengths[0]:s->strengths[1];
        uint16_t hi=s->strengths[0]>s->strengths[1]?s->strengths[0]:s->strengths[1];
        bool strong=!lo || !hi || (uint32_t)lo*100u>=(uint32_t)hi*BP_SECONDARY_RELATIVE_PERCENT;
        if(!strong)p->pair_scroll_frames=0;
        else if(p->pair_scroll_frames<BP_PAIR_CONFIRM_FRAMES)++p->pair_scroll_frames;
        if(p->pair_scroll_frames<BP_PAIR_CONFIRM_FRAMES) {p->scroll_anchor_sum=cur;p->scroll_last_sum=cur;p->scroll_ms=s->time_ms;p->scroll_velocity_q8=0;p->scroll_remainder_q8=0;if(!p->scroll_started){p->scroll_origin=cur;p->scroll_mt[0]=p->scroll_mt[1]=0;}return BP_OK;}
    }
    /* incremental: once confirmed, an unequal frame (lighter finger under 20 %) no longer restarts confirmation */
    p->scroll_confirmed=true;
    return scroll_emit(p,cur,s->time_ms);
}
static bp_result scroll_emit(bp_pointer *p,int32_t cur,uint64_t now) {
    int32_t step=2*(int32_t)p->config.scroll_step;
    if(p->config.scroll_incremental) {
        /* Rearm on physical inactivity, never on missing wheel output. A slow
         * but coherent stroke needs longer than 600 ms to accumulate a detent. */
        if(p->scroll_started && now-p->scroll_activity_ms>BP_SCROLL_REARM_MS) {
            p->scroll_started=false;p->scroll_origin=cur;p->scroll_anchor_sum=cur;p->scroll_last_sum=cur;p->scroll_remainder_q8=0;p->scroll_mt[0]=p->scroll_mt[1]=0;
            p->scroll_activity_sum=cur;p->scroll_activity_ms=now;
        }
        /* Pair-sum 200 um = 0.1 mm of shared travel. Tiny filtered jitter
         * does not perpetually renew an otherwise stationary gesture. */
        if(magnitude(cur-p->scroll_activity_sum)>=200u) {
            p->scroll_activity_sum=cur;p->scroll_activity_ms=now;
        }
        if(!p->scroll_started) {
            bool coherent=(int64_t)p->scroll_mt[0]*p->scroll_mt[1]>0 &&
                          magnitude(p->scroll_mt[0])>=BP_SCROLL_COHERENT_UM && magnitude(p->scroll_mt[1])>=BP_SCROLL_COHERENT_UM;
            if(!coherent && magnitude(cur-p->scroll_origin)<BP_SCROLL_START_UM) return BP_OK;
            p->scroll_started=true;p->scroll_detent_ms=now;   /* the anchor stays at the origin: the travel is paid now */
        }
    }
    int32_t delta=cur-p->scroll_anchor_sum,detents=delta/step;
    if(p->config.scroll_acceleration) {
        uint64_t dt=now-p->scroll_ms;
        int32_t change=cur-p->scroll_last_sum;
        if(!dt || dt>BP_CONTACT_CONTINUITY_MS) {
            p->scroll_velocity_q8=0;p->scroll_remainder_q8=0;change=0;
        } else {
            int32_t speed=(int32_t)((int64_t)change*256/(2*(int64_t)dt));
            if((int64_t)speed*p->scroll_velocity_q8<0)p->scroll_velocity_q8=0;
            p->scroll_velocity_q8+=(int32_t)(((int64_t)speed-p->scroll_velocity_q8)*(int64_t)dt/(50+(int64_t)dt));
        }
        unsigned speed=(unsigned)magnitude(p->scroll_velocity_q8)/256u,gain=256u;
        if(speed>=BP_SCROLL_ACCEL_HIGH_MM_S)gain=BP_SCROLL_ACCEL_MAX_Q8;
        else if(speed>BP_SCROLL_ACCEL_LOW_MM_S)gain+=(speed-BP_SCROLL_ACCEL_LOW_MM_S)*(BP_SCROLL_ACCEL_MAX_Q8-256u)/(BP_SCROLL_ACCEL_HIGH_MM_S-BP_SCROLL_ACCEL_LOW_MM_S);
        p->scroll_remainder_q8+=(int64_t)change*gain;
        detents=(int32_t)(p->scroll_remainder_q8/(step*256));
        p->scroll_remainder_q8-=(int64_t)detents*step*256;
        p->scroll_last_sum=cur;p->scroll_ms=now;
    }
    if(p->config.scroll_report_cap && (detents>1 || detents< -1)) {
        int32_t bounded=detents>0?1:-1;
        p->scroll_anchor_sum+=(detents-bounded)*step;detents=bounded;
    }
    if(!p->config.scroll_enabled) detents=0;
    if(detents!=0) {p->pair_tap_possible=false;p->scroll_detent_ms=now;}
    /* incremental scroll: add the detents into the newest unsent wheel-only report (as queue_one does for motion);
     * at 1 mm per detent a fast push can outrun a 15 ms link, and a full queue cancelled the gesture */
    bp_report *wtail=p->config.scroll_incremental && p->config.coalesce_motion && p->queued && detents ?
        &p->queue[(p->head+p->queued-1u)%BP_QUEUE_CAP] : NULL;
    int32_t wsum=wtail ? (int32_t)wtail->wheel+(p->config.invert_scroll?detents:-detents) : 0;
    bool wmerge=wtail && !wtail->buttons && !wtail->dx && !wtail->dy && wtail->wheel && wsum>=-127 && wsum<=127;
    unsigned needed=(unsigned)(magnitude(detents)+126)/127u;
    if(!wmerge && (p->queued>BP_QUEUE_CAP-2u || needed>BP_QUEUE_CAP-2u-p->queued)) {
        cancel(p); return BP_BACKPRESSURE;
    }
    p->scroll_anchor_sum+=detents*step;
    if(wmerge) { wtail->wheel=(int16_t)wsum; ++p->coalesced_reports; return BP_OK; }
    /* Screen y grows down; default wheel is positive toward the top. */
    if(!p->config.invert_scroll) detents=-detents;
    while(detents!=0) {
        int16_t part=(int16_t)(detents>127?127:detents < -127?-127:detents);
        if(!queue_one(p,0u,0,0)) { cancel(p); return BP_BACKPRESSURE; }
        p->queue[(p->head+p->queued-1u)%BP_QUEUE_CAP].wheel=part;
        detents-=part;
    }
    return BP_OK;
}
static bp_result pair_lift(bp_pointer *p,const bp_sample *s) {
    if(!p->pair_tap_possible || s->time_ms-p->pair_down_ms>p->config.tap_ms ||
       (p->mode==BP_PAIR_RELEASE && s->time_ms-p->pair_lift_ms>p->config.pair_release_ms)) {
        if(p->tap_tracking)++p->tap_reject[5];
        cancel(p); return BP_WAITING;
    }
    if(s->count==0u) {
        if(p->config.release_debounce && s->time_ms-p->pair_down_ms<BP_TAP_MIN_MS) {
            ++p->tap_reject[0];p->tap_tracking=false;p->mode=BP_IDLE;discard_motion(p);return BP_OK;
        }
        ++p->tap_right;p->tap_tracking=false;
        if(p->queued>BP_QUEUE_CAP-2u || !queue_one(p,2u,0,0) || !queue_one(p,0u,0,0)) {
            cancel(p); return BP_BACKPRESSURE;
        }
        p->mode=BP_IDLE; p->pair_tap_possible=false; p->local_buttons=0u;
        p->remainder_x=0; p->remainder_y=0;
        discard_motion(p);p->contact_ms=0;p->landing_frames=0;p->motion_active=false;p->landing_ready=false;return BP_OK;
    }
    uint64_t id=s->contacts[0].id;
    if(s->count!=1u || (id!=p->pair_low && id!=p->pair_high) ||
       (p->mode==BP_PAIR_RELEASE && id!=p->pair_remaining)) { cancel(p); return BP_FAULT; }
    int32_t x,y; transform(&p->config,&s->contacts[0],&x,&y);
    if(outside(x,y,id==p->pair_low?p->pair_low_x:p->pair_high_x,
                  id==p->pair_low?p->pair_low_y:p->pair_high_y,p->config.tap_slop)) {
        cancel(p); return BP_WAITING;
    }
    if(p->mode!=BP_PAIR_RELEASE) {
        p->pair_lift_ms=s->time_ms; p->pair_remaining=id; p->mode=BP_PAIR_RELEASE;
    }
    return BP_WAITING;
}
/* Rebuild by identity on every fresh frame: absent and weak contacts lose
 * their streak, while unrelated extra contacts cannot reset surviving IDs. */
static unsigned confirm_extras(bp_pointer *p,const bp_sample *s,uint64_t ignore_a,uint64_t ignore_b) {
    bp_contact_confirmation next[4]={{0}};unsigned n=0,confirmed=0;
    for(unsigned i=0;i<s->count;i++) {
        uint64_t id=s->contacts[i].id;if(id==ignore_a || id==ignore_b) continue;
        bp_contact_confirmation track={.id=id,.first_ms=s->time_ms,.last_ms=s->time_ms};
        uint16_t primary_strength=0;
        for(unsigned q=0;q<s->count;q++) if(s->contacts[q].id==p->contact_id) primary_strength=s->strengths[q];
        unsigned percent=!p->config.confirmed_secondary && p->config.tap_guard && p->mode==BP_CANDIDATE && !p->moved &&
            s->time_ms-p->down_ms<=p->config.tap_ms ? BP_SECONDARY_EARLY_PERCENT:BP_SECONDARY_RELATIVE_PERCENT;
        bool relative_ok=(!primary_strength || !s->strengths[i] ||
            (uint32_t)s->strengths[i]*100u >= (uint32_t)primary_strength*percent) &&
            (!p->config.confirmed_secondary || s->strengths[i]>=80u);
        if(p->config.tap_guard && p->mode==BP_CANDIDATE) {
            if(!relative_ok || (s->weak_contacts&(1u<<i)))++p->pair_weak_frames;
            else if(s->time_ms-p->down_ms>p->config.scroll_join_ms)++p->pair_late_frames;
            else if(!p->config.confirmed_secondary)p->tap_second_seen=true;
        }
        if(!(s->weak_contacts&(1u<<i)) && relative_ok) {
            track.frames=1;
            for(unsigned k=0;k<4;k++) {
                bp_contact_confirmation *old=&p->extras[k];
                if(old->id==id && old->frames && s->time_ms>old->last_ms &&
                   s->time_ms-old->last_ms<=BP_CONTACT_CONTINUITY_MS) {
                    track.first_ms=old->first_ms;
                    track.frames=old->frames<BP_PAIR_CONFIRM_FRAMES?old->frames+1u:old->frames;
                    break;
                }
            }
        } else if(p->config.scroll_incremental) {
            /* a weak frame of a tracked extra (the lighter finger of a pair at ATI 250) pauses its streak instead of
             * restarting it: 3 good frames over BP_PAIR_CONFIRM_MS are still required */
            for(unsigned k=0;k<4;k++) {
                bp_contact_confirmation *old=&p->extras[k];
                if(old->id==id && old->frames && s->time_ms>old->last_ms && s->time_ms-old->last_ms<=BP_CONTACT_CONTINUITY_MS) {
                    track.first_ms=old->first_ms;track.frames=old->frames;break;
                }
            }
        }
        if(track.frames>=BP_PAIR_CONFIRM_FRAMES && s->time_ms-track.first_ms>=BP_PAIR_CONFIRM_MS) {++confirmed;if(p->config.confirmed_secondary && p->mode==BP_CANDIDATE)p->tap_second_seen=true;}
        next[n++]=track;
    }
    memcpy(p->extras,next,sizeof(next));return confirmed;
}
static bool same_frame(const bp_sample *a,const bp_sample *b) {
    if(a->motion_intent_id!=b->motion_intent_id) return false;
    if(a->session!=b->session || a->sequence!=b->sequence || a->time_ms!=b->time_ms || a->count!=b->count || a->weak_contacts!=b->weak_contacts) return false;
    for(unsigned i=0u;i<a->count;++i)
        if(a->contacts[i].id!=b->contacts[i].id || a->contacts[i].x!=b->contacts[i].x || a->contacts[i].y!=b->contacts[i].y || a->strengths[i]!=b->strengths[i]) return false;
    return true;
}
/* A confirmed scroll stays pointer-silent after ID churn. Re-establish only
 * one unambiguous, strong pair with fresh temporal evidence, never whichever
 * two of three candidates happen to be strongest on a single frame. */
static bool repair_scroll_pair(bp_pointer *p,const bp_sample *s,unsigned *lo,unsigned *hi) {
    unsigned a=5,b=5,n=0;uint16_t strongest=0;
    for(unsigned i=0;i<s->count;i++)if(!(s->weak_contacts&(1u<<i)) && s->strengths[i]>strongest)strongest=s->strengths[i];
    for(unsigned i=0;i<s->count;i++) {
        if((s->weak_contacts&(1u<<i)) || s->strengths[i]<80u ||
           (uint32_t)s->strengths[i]*100u<(uint32_t)strongest*BP_SECONDARY_RELATIVE_PERCENT)continue;
        if(!n)a=i;else if(n==1)b=i;++n;
    }
    if(n!=2u)goto reset;
    int32_t ax,ay,bx,by;transform(&p->config,&s->contacts[a],&ax,&ay);transform(&p->config,&s->contacts[b],&bx,&by);
    uint32_t gap=length_um(ax-bx,ay-by);if(gap<8000u || gap>40000u)goto reset;
    uint64_t low=s->contacts[a].id<s->contacts[b].id?s->contacts[a].id:s->contacts[b].id;
    uint64_t high=s->contacts[a].id<s->contacts[b].id?s->contacts[b].id:s->contacts[a].id;
    if(p->scroll_repair_candidate.low!=low || p->scroll_repair_candidate.high!=high) {
        p->scroll_repair_candidate.low=low;p->scroll_repair_candidate.high=high;
        p->scroll_repair_candidate.since=s->time_ms;p->scroll_repair_candidate.frames=1;return false;
    }
    if(p->scroll_repair_candidate.frames<BP_PAIR_CONFIRM_FRAMES)++p->scroll_repair_candidate.frames;
    if(p->scroll_repair_candidate.frames<BP_PAIR_CONFIRM_FRAMES ||
       s->time_ms-p->scroll_repair_candidate.since<BP_PAIR_CONFIRM_MS)return false;
    p->pair_low=low;p->pair_high=high;p->pair_tap_possible=false;
    *lo=s->contacts[a].id==low?a:b;*hi=s->contacts[a].id==high?a:b;
    if(p->contact_id!=low && p->contact_id!=high)p->contact_id=low;
    p->scroll_paused=true; /* Existing resume path rebases without a wheel. */
    memset(&p->scroll_repair_candidate,0,sizeof(p->scroll_repair_candidate));return true;
reset:
    memset(&p->scroll_repair_candidate,0,sizeof(p->scroll_repair_candidate));return false;
}
bp_config bp_default_config(void) {
    bp_config c={.tap_ms=180u,.hold_ms=300u,.stale_ms=200u,.tap_slop=2000u,.deadzone=1000u,
        .gain_q8=BP_BASE_GAIN_Q8,.hold_drag=true,.scroll_step=BP_SCROLL_STEP_UM,
        .scroll_join_ms=100u,.pair_release_ms=80u,.pointer_acceleration=true,
        .primary_confidence_enter=BP_PRIMARY_CONFIDENCE_ENTER,
        .primary_confidence_exit=BP_PRIMARY_CONFIDENCE_EXIT,
        .primary_confidence_drag_ms=BP_PRIMARY_CONFIDENCE_DRAG_MS}; return c;
}
bp_result bp_init(bp_pointer *p,const bp_config *c,bp_send send,void *context) {
    if(p==NULL) return BP_INVALID;
    memset(p,0,sizeof(*p));
    if(c==NULL || send==NULL || c->tap_ms==0u || c->hold_ms<c->tap_ms || c->stale_ms==0u ||
       c->gain_q8==0u || c->gain_q8>16384u || c->tap_slop>BP_PHYSICAL_Y_UM || c->deadzone>c->tap_slop ||
       (c->rotation!=0u && c->rotation!=90u && c->rotation!=180u && c->rotation!=270u) ||
       c->scroll_step==0u || c->scroll_step>BP_PHYSICAL_Y_UM || c->scroll_join_ms>c->hold_ms ||
       (c->two_finger_tap && (c->pair_release_ms==0u || c->pair_release_ms>c->tap_ms)) ||
       (c->primary_confidence_gate && (!c->primary_confidence_exit ||
        c->primary_confidence_exit>c->primary_confidence_enter || !c->primary_confidence_drag_ms))) return BP_INVALID;
    p->config=*c; p->send=send; p->send_context=context; p->mode=BP_WAIT_ZERO; p->initialized=true; return BP_OK;
}
bp_result bp_connect(bp_pointer *p,uint32_t epoch,uint64_t now) {
    bp_result r=entry(p); if(r!=BP_OK) return r;
    if(!clock_ok(p,now)) return BP_FAULT;
    if(epoch==0u || (p->linked_once && epoch<=p->link_epoch)) return BP_INVALID;
    p->link_epoch=epoch; p->linked_once=true; p->online=true; p->have_frame=false;
    p->accepted_buttons_known=false; p->release_pending=false; cancel(p);
    return BP_WAITING;
}
bp_result bp_disconnect(bp_pointer *p,uint64_t now) {
    bp_result r=entry(p); if(r!=BP_OK) return r;
    bool clock_valid=clock_ok(p,now); lose_link(p);
    return clock_valid?BP_LINK_LOST:BP_FAULT;
}
bp_result bp_tick(bp_pointer *p,uint64_t now) {
    bp_result r=entry(p); if(r!=BP_OK) return r;
    if(!clock_ok(p,now)) return BP_FAULT;
    if(p->have_fresh && p->mode!=BP_WAIT_ZERO && now-p->last_fresh_ms>=p->config.stale_ms) {
        cancel(p); return BP_FAULT;
    }
    if(p->online && !p->release_pending) {
        bp_result confidence_result=confidence_timeout(p,now);
        if(confidence_result!=BP_OK)return confidence_result;
        bp_result movement_result=movement_timeout(p,now);
        if(movement_result!=BP_OK)return movement_result;
        if(p->config.tap_drag && p->tap_rejoin_pending && p->mode==BP_IDLE && now>=p->tap_rejoin_deadline) {
            /* Expire second-landing eligibility, not an unsent first click. */
            p->tap_rejoin_pending=false;
        }
        if(p->mode==BP_DRAG_LOCK && now>=p->drag_lock_deadline)return finish_drag(p);
    }
    return p->online?BP_OK:BP_LINK_LOST;
}
bp_result bp_process(bp_pointer *p,const bp_sample *s,uint64_t now) {
    bp_result r=entry(p); if(r!=BP_OK) return r;
    r=bp_tick(p,now); if(r==BP_FAULT || r==BP_LINK_LOST) return r;
    if(s==NULL || (s->kind!=BP_NO_DATA && !valid_sample(s))) { cancel(p); return BP_FAULT; }
    if(s->kind==BP_NO_DATA) return BP_IGNORED;
    if(s->time_ms>now || now-s->time_ms>=p->config.stale_ms) { cancel(p); return BP_FAULT; }
    if(p->have_frame) {
        if(s->session!=p->last_frame.session) {
            /* A session change is a cancellation boundary, even if zero. */
            if(s->session>p->last_frame.session) { p->last_frame=*s; }
            cancel(p); return BP_FAULT;
        }
        uint32_t diff=s->sequence-p->last_frame.sequence;
        if(diff==0u) {
            if(same_frame(s,&p->last_frame)) return BP_IGNORED;
            cancel(p); return BP_FAULT;
        }
        if(diff>=0x80000000u || s->time_ms<p->last_frame.time_ms) { cancel(p); return BP_FAULT; }
    }
    p->last_frame=*s; p->have_frame=true;
    if(p->release_pending) return BP_WAITING;
    p->last_fresh_ms=s->time_ms; p->have_fresh=true;
    bp_sample relevant;
    if(p->config.ignore_weak_secondary && s->count>1u && !p->scroll_latched) {
        unsigned primary=0;
        for(unsigned i=0;i<s->count;i++)if(s->contacts[i].id==p->contact_id && p->mode!=BP_IDLE)primary=i;
        uint16_t strength=s->strengths[primary];relevant=*s;relevant.count=0;relevant.weak_contacts=0;
        for(unsigned i=0;i<s->count;i++) {
            if(i!=primary && strength && s->strengths[i] && (uint32_t)s->strengths[i]*100u<(uint32_t)strength*BP_SECONDARY_EARLY_PERCENT)continue;
            unsigned j=relevant.count++;relevant.contacts[j]=s->contacts[i];relevant.strengths[j]=s->strengths[i];
            if(s->weak_contacts&(1u<<i))relevant.weak_contacts|=(uint8_t)(1u<<j);
        }
        s=&relevant;
    }
    if(p->config.primary_confidence_gate && !p->scroll_latched &&
       p->mode!=BP_SCROLL && p->mode!=BP_WAIT_ZERO && s->kind==BP_DATA && s->count) {
        bool same=false;
        for(unsigned i=0;i<s->count;i++)if(s->contacts[i].id==p->pair_landing.id)same=true;
        if(!same) {
            p->pair_landing.id=s->contacts[0].id;p->pair_landing.first_ms=s->time_ms;
            p->pair_landing.last_ms=s->time_ms;p->pair_landing.frames=1;
        } else if(s->time_ms>p->pair_landing.last_ms) {
            if(p->pair_landing.frames<BP_LANDING_FRESH_FRAMES)++p->pair_landing.frames;
            p->pair_landing.last_ms=s->time_ms;
        }
    }
    bp_sample confirmed_release;
    bool interrupted_release=false;
    if(s->kind==BP_DATA && !s->count) {
        /* Purge transport-unaccepted cursor/wheel entries on the FIRST zero,
         * while the existing debounce still owns tap/button release timing. */
        movement_boundary(p,s->time_ms,false);
        if(p->config.purge_motion_on_zero)discard_motion(p);
        if(p->config.primary_confidence_gate &&
           (p->mode==BP_CANDIDATE || p->mode==BP_MOVING || p->mode==BP_DRAG)) {
            /* Pure zeros retain normal release debounce. Once weak TOUCH has
             * started a confidence deadline, zeros cannot restart it: an
             * alternating weak/zero stream must not hold a drag indefinitely. */
            if(!p->primary_confidence_paused)p->primary_confidence_since=0;
            p->primary_confidence_paused=true;
        }
    }
    if(p->scroll_latched && !s->count)memset(&p->scroll_repair_candidate,0,sizeof(p->scroll_repair_candidate));
    if(p->config.release_debounce) {
        if(s->kind==BP_DATA && s->count==0u && p->mode!=BP_IDLE && p->mode!=BP_WAIT_ZERO && p->mode!=BP_DRAG_LOCK) {
            if(!p->release_frames)p->release_first_ms=s->time_ms;
            if(p->release_frames<BP_RELEASE_CONFIRM_FRAMES)++p->release_frames;
            p->motion_pending=0;p->remainder_x=0;p->remainder_y=0;
            if(p->release_frames<BP_RELEASE_CONFIRM_FRAMES || s->time_ms-p->release_first_ms<BP_RELEASE_CONFIRM_MS)return BP_WAITING;
            /* Count physical contact time to the FIRST zero, not debounce completion. */
            confirmed_release=*s;confirmed_release.time_ms=p->release_first_ms;s=&confirmed_release;
            p->release_frames=0;
        } else if(p->release_frames) {
            interrupted_release=true;++p->release_bounces;p->release_frames=0;p->hold_eligible=false;
        }
    }
    if(s->kind==BP_DATA && !s->count)memset(&p->pair_landing,0,sizeof(p->pair_landing));
    if(s->kind==BP_SUPPRESSED && p->scroll_latched){p->scroll_paused=true;memset(&p->scroll_repair_candidate,0,sizeof(p->scroll_repair_candidate));return BP_WAITING;}
    if(s->kind==BP_SUPPRESSED) {
        bool handled=false;bp_result confidence_result=primary_confidence(p,s,0u,now,&handled);
        if(handled)return confidence_result;
        /* A weak approach must not lock an already armed idle pointer until
         * lift. Never use it to arm on connection or to finish an active tap. */
        if(p->mode==BP_IDLE) return BP_IGNORED;
        if(p->mode==BP_WAIT_ZERO) return BP_WAITING;
        if(p->config.tap_guard && p->mode==BP_CANDIDATE && !p->moved && s->time_ms-p->down_ms<=p->config.tap_ms) {
            if(!p->tap_weak_since)p->tap_weak_since=s->time_ms;
            if(s->time_ms-p->tap_weak_since<60u) {p->motion_pending=0;return BP_IGNORED;}
        }
        return primary_absent(p,s);
    }
    if(p->mode==BP_WAIT_ZERO) {
        if(s->count==0u) { p->mode=BP_IDLE; p->wait_zero_since=0; return BP_OK; }
        /* v0.17: a cancel used to block output until every finger left (Codex: cancelled mid-stroke by a
         * missing primary or a position jump, then 500..700 ms of a valid finger ignored). A single steady
         * contact now resumes as a fresh landing without tap eligibility. */
        /* v0.18 (Codex): the candidate must be the SAME contact id throughout, moving continuously (no step
         * beyond the continuity radius); any other observation restarts the timer. */
        if(p->config.wait_zero_recover_ms && s->kind==BP_DATA && s->count==1u && !s->weak_contacts) {
            const bp_contact *c=&s->contacts[0];
            int64_t jx=((int64_t)c->x-p->recover_x)*(BP_PHYSICAL_X_UM/100),jy=((int64_t)c->y-p->recover_y)*(BP_PHYSICAL_Y_UM/100);
            int64_t radius=(int64_t)BP_CONTACT_CONTINUITY_MM*100000;
            bool same=p->wait_zero_since && c->id==p->recover_id && jx*jx+jy*jy<=radius*radius;
            p->recover_id=c->id;p->recover_x=c->x;p->recover_y=c->y;
            if(!same) {p->wait_zero_since=s->time_ms?s->time_ms:1u;return BP_WAITING;}
            if(s->time_ms-p->wait_zero_since<p->config.wait_zero_recover_ms) return BP_WAITING;
            p->wait_zero_since=0;p->mode=BP_IDLE;p->recovering=true;++p->wait_zero_recoveries;
        } else { p->wait_zero_since=0; return BP_WAITING; }
    }
    if(s->count==1u) {
        bool handled=false;bp_result confidence_result=primary_confidence(p,s,0u,now,&handled);
        if(handled)return confidence_result;
    }
    if(p->config.tap_drag && p->mode==BP_IDLE && p->tap_rejoin_pending && s->count) {
        p->tap_rejoin_pending=false;
        if(s->count==1u && !s->weak_contacts) {
            int32_t tx,ty;transform(&p->config,&s->contacts[0],&tx,&ty);
            /* A clutch lift can land elsewhere within the temporal window.
             * Only a nearby second landing owns tap-drag; double taps still
             * each click at release, and nearby slide/hold drag is unchanged. */
            p->second_tap=!outside(tx,ty,p->down_x,p->down_y,BP_TAP_DRAG_REJOIN_UM);
        }
    }
    if(p->mode==BP_DRAG_LOCK) {
        if(!s->count)return BP_WAITING;
        if(s->count!=1u || s->weak_contacts){(void)finish_drag(p);return BP_WAITING;}
        p->second_tap=true;p->lock_rejoin=true;p->mode=BP_IDLE;
    }
    if(p->scroll_latched) {
        if(s->kind==BP_SUPPRESSED){p->scroll_paused=true;memset(&p->scroll_repair_candidate,0,sizeof(p->scroll_repair_candidate));return BP_WAITING;}
        if(!s->count) {
            p->scroll_latched=false;p->scroll_paused=false;
            memset(&p->scroll_repair_candidate,0,sizeof(p->scroll_repair_candidate));
            if(p->pair_tap_possible)return pair_lift(p,s);
            discard_motion_ex(p,p->config.scroll_incremental);p->mode=BP_IDLE;p->tap_tracking=false;p->pair_tap_possible=false;return BP_OK;
        }
        unsigned lo=5,hi=5;
        for(unsigned i=0;i<s->count;i++) {if(s->contacts[i].id==p->pair_low)lo=i;if(s->contacts[i].id==p->pair_high)hi=i;}
        if(p->config.scroll_repair && !p->pair_tap_possible &&
           (lo==5 || hi==5 || (s->weak_contacts&((1u<<lo)|(1u<<hi)))))
            (void)repair_scroll_pair(p,s,&lo,&hi);
        else memset(&p->scroll_repair_candidate,0,sizeof(p->scroll_repair_candidate));
        if(p->config.scroll_incremental && lo==5 && hi==5 && s->count==1u && !(s->weak_contacts&1u) &&
           (p->scroll_confirmed || s->time_ms-p->scroll_landing_ms>=BP_START_QUIET_MS)) {
            /* the chip merged the pair into one contact the policy started as a new lifetime (its centroid is ~10 mm
             * from either finger): without a lift in between it carries the scroll as the lone member */
            p->pair_low=s->contacts[0].id;lo=0;p->scroll_pok[0]=p->scroll_pok[1]=false;
            p->scroll_ps[0]=p->scroll_ps[0]>p->scroll_ps[1]?p->scroll_ps[0]:p->scroll_ps[1];
            memset(&p->scroll_repair_candidate,0,sizeof(p->scroll_repair_candidate));
        }
        if(p->config.scroll_incremental && s->time_ms>p->landing_last_ms) {   /* the latch returns before the generic count below */
            if(p->landing_frames<BP_LANDING_FRESH_FRAMES) ++p->landing_frames;
            p->landing_last_ms=s->time_ms;
        }
        if(lo==5 || hi==5 || (s->weak_contacts&((1u<<lo)|(1u<<hi)))) {
            if(p->pair_tap_possible && s->count==1u && s->time_ms-p->pair_down_ms<=p->config.tap_ms &&
               (p->mode!=BP_PAIR_RELEASE || s->time_ms-p->pair_lift_ms<=p->config.pair_release_ms)) {
                p->scroll_paused=true;return pair_lift(p,s);
            }
            p->pair_tap_possible=false;
            bool gl=lo!=5 && !(s->weak_contacts&(1u<<lo)),gh=hi!=5 && !(s->weak_contacts&(1u<<hi));
            bool settled=s->time_ms-p->scroll_landing_ms>=(p->config.responsive_startup?BP_START_QUIET_MS:BP_LANDING_QUIET_MS) &&
                         p->landing_frames>=BP_LANDING_FRESH_FRAMES;
            /* a lone member much stronger than it was in the pair is the chip's merged blob: its centroid wanders between
             * the two fingers as their strengths change, so it pauses like before instead of scrolling (twice its travel) */
            bool merged=(gl && !gh && p->scroll_ps[0] && (uint32_t)s->strengths[lo]*10u>(uint32_t)p->scroll_ps[0]*14u) ||
                        (gh && !gl && p->scroll_ps[1] && (uint32_t)s->strengths[hi]*10u>(uint32_t)p->scroll_ps[1]*14u);
            if(p->config.scroll_incremental && (p->scroll_confirmed || settled) && (gl || gh) && !merged) {
                /* one member weak or missing (or the chip merged the pair into one contact): keep scrolling with the
                 * other, twice its travel. The latch already keeps the pointer silent; this only adds the wheel. */
                if(!p->scroll_confirmed){p->scroll_confirmed=true;p->scroll_paused=true;}
                if(p->scroll_paused){p->scroll_pok[0]=p->scroll_pok[1]=false;p->scroll_paused=false;}
                int32_t xl=0,yl=0,xh=0,yh=0;
                if(lo!=5)transform(&p->config,&s->contacts[lo],&xl,&yl);
                if(hi!=5)transform(&p->config,&s->contacts[hi],&xh,&yh);
                scroll_coherence(p,lo!=5,yl,hi!=5,yh);
                p->mode=BP_SCROLL;
                return scroll_emit(p,scroll_track(p,gl,yl,gh,yh),s->time_ms);
            }
            if(p->config.scroll_incremental)p->scroll_pok[0]=p->scroll_pok[1]=false;
            p->scroll_paused=true;discard_motion_ex(p,p->config.scroll_incremental);return BP_WAITING;
        }
        bp_sample pair=*s;pair.count=2;pair.contacts[0]=s->contacts[lo];pair.contacts[1]=s->contacts[hi];
        pair.strengths[0]=s->strengths[lo];pair.strengths[1]=s->strengths[hi];pair.weak_contacts=0;
        p->scroll_ps[0]=s->strengths[lo];p->scroll_ps[1]=s->strengths[hi];
        if(p->mode==BP_PAIR_RELEASE)p->pair_tap_possible=false;
        p->mode=BP_SCROLL;
        if(p->scroll_paused) {
            int32_t x1,y1,x2,y2;transform(&p->config,&pair.contacts[0],&x1,&y1);transform(&p->config,&pair.contacts[1],&x2,&y2);
            if(p->config.scroll_incremental) {   /* restart both members; the travel since the last detent is kept */
                p->scroll_py[0]=y1;p->scroll_py[1]=y2;p->scroll_pok[0]=p->scroll_pok[1]=true;
                p->scroll_last_sum=p->scroll_vsum;p->scroll_ms=s->time_ms;p->scroll_paused=false;return BP_OK;
            }
            p->scroll_anchor_sum=p->scroll_last_sum=y1+y2;p->scroll_ms=s->time_ms;p->scroll_velocity_q8=0;p->scroll_remainder_q8=0;p->scroll_paused=false;return BP_OK;
        }
        return scroll(p,&pair);
    }
    if(s->count && p->mode!=BP_IDLE && s->time_ms>p->landing_last_ms) {
        if(p->landing_frames<BP_LANDING_FRESH_FRAMES) ++p->landing_frames;
        p->landing_last_ms=s->time_ms;
    }
    if(p->config.tap_guard && !p->config.median_tap && p->mode==BP_CANDIDATE && !p->moved && s->count &&
       s->time_ms-p->down_ms<=p->config.tap_ms) {
        unsigned chosen=5;
        for(unsigned i=0;i<s->count;i++)if(s->contacts[i].id==p->contact_id)chosen=i;
        if(chosen==5 && s->count==1u)chosen=0;
        if(chosen!=5) {
            /* Do not let the generic same-ID spatial fault cancel a tap
             * on an isolated sensor teleport. No click or motion is emitted
             * from this observation; a real zero or valid return is required. */
            if(!primary_position_ok(p,s,&s->contacts[chosen])) {
                ++p->tap_spikes;return BP_IGNORED;
            }
            int32_t x,y;transform(&p->config,&s->contacts[chosen],&x,&y);
            uint32_t cap,reanchors=p->spike_reanchors;
            bool trusted=coordinate_guard(p,x,y,s->time_ms,&cap);
            if(p->spike_reanchors!=reanchors && !trusted) {
                /* A settled relocation is not evidence of a valid tap. Keep
                 * its bounded decision window silent, without fabricating
                 * either a click or motion from the discarded offset. */
                if(p->tap_tracking) {++p->tap_reject[1];p->tap_tracking=false;}
                p->tap_ambiguous=true;p->moved=true;return BP_IGNORED;
            }
            if(!trusted) {++p->tap_spikes;return BP_IGNORED;}
        }
        p->tap_weak_since=0;
    }
    if(p->config.tap_guard && p->tap_ambiguous && s->count) {
        if(s->time_ms-p->down_ms<=p->config.tap_ms) return BP_IGNORED;
        p->tap_ambiguous=false;
        /* The normal continuity checks still apply when observation resumes. */
    }
    bool handover=false;
    bp_sample selected_sample;
    if(p->primary_missing && s->count) {
        unsigned i=primary_index(p,s);
        if(i==5) return primary_absent(p,s);
        if(s->time_ms-p->primary_missing_since>=BP_PRIMARY_MISSING_MS) {cancel(p);return BP_FAULT;}
        degrade_primary(p,s,i);return BP_OK;
    }
    if(p->mode==BP_PAIR_RELEASE) {
        if(!s->count) return pair_lift(p,s);
        if(s->count==1u && s->contacts[0].id==p->pair_remaining) {
            int32_t x,y;transform(&p->config,&s->contacts[0],&x,&y);
            int32_t ox=p->pair_remaining==p->pair_low?p->pair_low_x:p->pair_high_x;
            int32_t oy=p->pair_remaining==p->pair_low?p->pair_low_y:p->pair_high_y;
            if(p->pair_tap_possible && s->time_ms-p->pair_down_ms<=p->config.tap_ms &&
               s->time_ms-p->pair_lift_ms<=p->config.pair_release_ms &&
               !outside(x,y,ox,oy,p->config.tap_slop)) return pair_lift(p,s);
        }
        unsigned i=primary_index(p,s);if(i==5) return primary_absent(p,s);
        degrade_primary(p,s,i);return BP_OK;
    }
    if(p->mode==BP_SCROLL && s->count>=2u) {
        unsigned low=5,high=5;
        for(unsigned i=0;i<s->count;i++) {
            if(s->contacts[i].id==p->pair_low) low=i;
            if(s->contacts[i].id==p->pair_high) high=i;
        }
        /* An extra point may be transient. A missing established pair member
         * cannot be replaced by an arbitrary additional contact. */
        if(low==5 || high==5) {
            unsigned i=primary_index(p,s);if(i==5) return primary_absent(p,s);
            degrade_primary(p,s,i);return BP_OK;
        }
        unsigned confirmed=confirm_extras(p,s,p->pair_low,p->pair_high);
        if(confirmed) {
            unsigned i=primary_index(p,s);if(i==5) return primary_absent(p,s);
            degrade_primary(p,s,i);return BP_OK;
        }
        selected_sample=*s;selected_sample.count=2;
        selected_sample.contacts[0]=s->contacts[low];selected_sample.contacts[1]=s->contacts[high];
        selected_sample.strengths[0]=s->strengths[low];selected_sample.strengths[1]=s->strengths[high];
        unsigned primary=p->contact_id==p->pair_low?low:high;
        remember_primary(p,&s->contacts[primary],s->time_ms,false);
        p->last_raw_x=s->contacts[primary].x;p->last_raw_y=s->contacts[primary].y;p->contact_ms=s->time_ms;
        selected_sample.weak_contacts=(uint8_t)(((s->weak_contacts>>low)&1u)|(((s->weak_contacts>>high)&1u)<<1));
        return scroll(p,&selected_sample);
    }
    if(p->mode==BP_SCROLL) {
        memset(p->extras,0,sizeof(p->extras));
        if(!s->count) return pair_lift(p,s);
        if(p->pair_tap_possible && s->time_ms-p->pair_down_ms<=p->config.tap_ms && s->count==1u) {
            uint64_t id=s->contacts[0].id;
            int32_t x,y;transform(&p->config,&s->contacts[0],&x,&y);
            if((id==p->pair_low || id==p->pair_high) &&
               !outside(x,y,id==p->pair_low?p->pair_low_x:p->pair_high_x,
                            id==p->pair_low?p->pair_low_y:p->pair_high_y,p->config.tap_slop)) return pair_lift(p,s);
        }
        unsigned i=primary_index(p,s);if(i==5) return primary_absent(p,s);
        degrade_primary(p,s,i);return BP_OK;
    }
    if(s->count>1u) {
        unsigned selected=0;
        if(p->mode!=BP_IDLE) {
            bool found=false;
            for(unsigned i=0;i<s->count;i++) if(s->contacts[i].id==p->contact_id) {
                bool confidence_rebase=p->config.primary_confidence_gate &&
                    (p->primary_confidence_paused || (s->strengths[i] && s->strengths[i]<p->config.primary_confidence_exit));
                if(!confidence_rebase && !primary_position_ok(p,s,&s->contacts[i])) {
                    if(p->config.median_tap && p->mode==BP_CANDIDATE && s->time_ms-p->down_ms<=p->config.tap_ms){++p->tap_spikes;return BP_IGNORED;}
                    cancel(p);return BP_FAULT;
                }
                selected=i;found=true;break;
            }
            if(!found) {
                unsigned nearby=0;
                for(unsigned i=0;i<s->count;i++) if(continuous_contact(p,s,&s->contacts[i])) {selected=i;++nearby;}
                if(nearby!=1u) return primary_absent(p,s);
                p->contact_id=s->contacts[selected].id;handover=true;
            }
        }
        unsigned confirmed=p->single_only?0:confirm_extras(p,s,s->contacts[selected].id,0);
        if(confirmed && p->second_tap) {
            p->second_tap=false;
            if(p->lock_rejoin){(void)finish_drag(p);return BP_WAITING;}
        }
        /* Each extra earns its own streak. A transient third cannot erase a
         * genuine second finger's history or bypass the confirmation delay. */
        if(confirmed>1u) {degrade_primary(p,s,selected);return BP_OK;}
        unsigned secondary=5;bp_contact_confirmation *candidate=NULL;
        p->pending_pair_frames=0;
        for(unsigned k=0;k<4;k++) if(p->extras[k].id) {
            bp_contact_confirmation *track=&p->extras[k];
            bool ready=track->frames>=BP_PAIR_CONFIRM_FRAMES && s->time_ms-track->first_ms>=BP_PAIR_CONFIRM_MS;
            if(ready || (!candidate && track->frames)) candidate=track;
            if(ready) break;
        }
        if(candidate) {
            p->pending_pair_ms=candidate->first_ms;p->pending_pair_last_ms=candidate->last_ms;
            p->pending_pair_frames=candidate->frames;
            if(confirmed) for(unsigned i=0;i<s->count;i++) if(s->contacts[i].id==candidate->id) secondary=i;
        }
        if(confirmed) {
            if(!(p->config.scroll_enabled || p->config.two_finger_tap) || secondary==5) {degrade_primary(p,s,selected);return BP_OK;}
            if(p->config.primary_confidence_gate && p->mode==BP_IDLE) {
                /* A weak single approach stayed idle, but the existing pair
                 * confirmation has now independently qualified two contacts.
                 * Carry its real landing history into scroll; do not require
                 * either finger to meet the pointer-only enter threshold. */
                p->down_ms=p->pending_pair_ms;p->landing_frames=p->pending_pair_frames;
                p->landing_last_ms=s->time_ms;p->moved=false;p->tap_tracking=false;
                p->contact_id=s->contacts[selected].id;p->contact_ms=s->time_ms;
                p->last_raw_x=s->contacts[selected].x;p->last_raw_y=s->contacts[selected].y;
                remember_primary(p,&s->contacts[selected],s->time_ms,true);
                transform(&p->config,&s->contacts[selected],&p->down_x,&p->down_y);
                p->anchor_x=p->down_x;p->anchor_y=p->down_y;
                p->primary_confidence_paused=false;p->primary_confidence_since=0;
            }
            selected_sample=*s;selected_sample.count=2;
            selected_sample.contacts[0]=s->contacts[selected];selected_sample.contacts[1]=s->contacts[secondary];
            selected_sample.strengths[0]=s->strengths[selected];selected_sample.strengths[1]=s->strengths[secondary];
            selected_sample.weak_contacts=0;
            bp_result result=scroll(p,&selected_sample);
            if(result==BP_OK) for(unsigned k=0;k<4;k++)
                if(p->extras[k].id==p->pair_low || p->extras[k].id==p->pair_high) memset(&p->extras[k],0,sizeof(p->extras[k]));
            return result;
        }
        if(p->config.scroll_preconfirm_pause && ((p->config.pair_pause_30 && p->mode==BP_CANDIDATE && !p->moved) || (candidate && candidate->frames)) && p->mode!=BP_IDLE && p->mode!=BP_DRAG) {
            bool strong_pair=false;int32_t px,py;transform(&p->config,&s->contacts[selected],&px,&py);
            unsigned pause_percent=p->config.pair_pause_30 && p->mode==BP_CANDIDATE && !p->moved?BP_SECONDARY_EARLY_PERCENT:BP_SECONDARY_RELATIVE_PERCENT;
            for(unsigned i=0;i<s->count;i++)if(i!=selected && !(s->weak_contacts&(1u<<i)) &&
               s->strengths[i]>=80u && (!s->strengths[selected] || (uint32_t)s->strengths[i]*100u>=(uint32_t)s->strengths[selected]*pause_percent)) {
                int32_t qx,qy;transform(&p->config,&s->contacts[i],&qx,&qy);
                uint32_t distance=length_um(qx-px,qy-py);if(distance>=8000u && distance<=40000u)strong_pair=true;
            }
            if(strong_pair) {
                if(!p->pair_pause_active){p->pair_pause_active=true;p->pair_pause_since=s->time_ms;}
                if(s->time_ms-p->pair_pause_since<200u){
                    /* Silencing pointer output must not erase physical travel.
                     * Keep the same median tap evidence while a possible pair
                     * is pending; one transient coordinate spike still cannot
                     * veto a tap. Sustained travel cannot become a click after
                     * the extra contact disappears and the primary returns. */
                    if(p->mode==BP_CANDIDATE && p->config.median_tap) {
                        unsigned j=p->tap_median.next;
                        p->tap_median.x[j]=px;p->tap_median.y[j]=py;
                        p->tap_median.next=(j+1u)%3u;
                        if(p->tap_median.frames<65535u)++p->tap_median.frames;
                        if(s->strengths[selected]>p->tap_median.peak)p->tap_median.peak=s->strengths[selected];
                        int32_t tx=median3(p->tap_median.x[0],p->tap_median.x[1],p->tap_median.x[2]);
                        int32_t ty=median3(p->tap_median.y[0],p->tap_median.y[1],p->tap_median.y[2]);
                        if(outside(tx,ty,p->down_x,p->down_y,p->config.tap_slop)) {
                            if(p->tap_tracking)++p->tap_reject[1];
                            p->tap_tracking=false;p->moved=true;
                        }
                    }
                    remember_primary(p,&s->contacts[selected],s->time_ms,false);
                    p->last_raw_x=s->contacts[selected].x;p->last_raw_y=s->contacts[selected].y;
                    p->contact_ms=s->time_ms;motion_rebase(p,px,py);return BP_WAITING;
                }
            }
        }
        selected_sample=*s;selected_sample.count=1;selected_sample.contacts[0]=s->contacts[selected];
        selected_sample.strengths[0]=s->strengths[selected];
        selected_sample.weak_contacts=(uint8_t)((s->weak_contacts>>selected)&1u);s=&selected_sample;
    } else {
        p->pair_pause_active=false;p->pending_pair_frames=0;
        /* incremental: keep the extras' streaks over a frame the chip dropped the second finger from; they expire by
         * BP_CONTACT_CONTINUITY_MS in confirm_extras */
        if(!p->config.scroll_incremental || !s->count)memset(p->extras,0,sizeof(p->extras));
    }
    if(s->count==0u) {
        if(p->config.tap_guard && p->mode==BP_CANDIDATE && p->tap_second_seen && !p->moved) {
            ++p->pair_unconfirmed;++p->tap_reject[5];p->tap_tracking=false;p->moved=true;
        }
        if(p->mode==BP_DRAG) {
            if(p->config.drag_lock) {discard_motion(p);p->mode=BP_DRAG_LOCK;p->drag_lock_deadline=now+BP_DRAG_LOCK_MS;return BP_WAITING;}
            return finish_drag(p);
        }
        if(p->mode==BP_CANDIDATE && !p->moved && s->time_ms-p->down_ms<=p->config.tap_ms && (!(p->config.tap_guard || p->config.release_debounce) || s->time_ms-p->down_ms>=(p->config.short_tap?20u:BP_TAP_MIN_MS)) && (!p->config.short_tap || (p->tap_median.frames>=2u && p->tap_median.peak>=80u))) {
            p->tap_tracking=false;
            if(p->second_tap) {
                if(p->lock_rejoin)return finish_drag(p);
                p->second_tap=false;
                /* First click was emitted on its release; this is only click2. */
                if(!click_batch(p,1u)){cancel(p);return BP_BACKPRESSURE;}
            } else {
                if(!click_batch(p,1u)){cancel(p);return BP_BACKPRESSURE;}
                /* Preserve tap-to-drag/double-tap matching without delaying click1. */
                if(p->config.tap_drag) {
                    p->tap_rejoin_pending=true;
                    p->tap_rejoin_deadline=s->time_ms+(p->config.extended_drag_window?250u:BP_TAP_DRAG_WINDOW_MS);
                }
            }
        }
        if(p->second_tap) {
            p->second_tap=false;
            if(p->lock_rejoin)return finish_drag(p);
        }
        if(p->tap_tracking) {++p->tap_reject[p->moved?2:0];p->tap_tracking=false;}
        p->mode=BP_IDLE;p->single_only=false;p->primary_missing=false; p->local_buttons=0u; p->remainder_x=0; p->remainder_y=0;
        p->primary_confidence_paused=false;p->primary_confidence_since=0;
        movement_reset(p);
        discard_motion(p);p->contact_ms=0;p->landing_frames=0;p->motion_active=false;p->landing_ready=false;return BP_OK;
    }
    /* Multi-contact processing above can select a single primary after
     * leaving genuine secondary/scroll confirmation unchanged. */
    {
        bool handled=false;bp_result confidence_result=primary_confidence(p,s,0u,now,&handled);
        if(handled)return confidence_result;
    }
    int32_t x,y; transform(&p->config,&s->contacts[0],&x,&y);
    if(p->mode==BP_IDLE) {
        p->primary_confidence_paused=false;p->primary_confidence_since=0;
        movement_reset(p);
        memset(&p->tap_median,0,sizeof(p->tap_median));
        for(unsigned i=0;i<3;i++){p->tap_median.x[i]=x;p->tap_median.y[i]=y;}
        p->tap_median.frames=1;p->tap_median.peak=s->strengths[0];p->pair_pause_active=false;
        p->tap_tracking=true;p->tap_ambiguous=false;p->tap_second_seen=false;p->hold_eligible=true;p->tap_spike_since=0;p->tap_weak_since=0;
        p->mode=BP_CANDIDATE;p->single_only=false;p->primary_missing=false; p->contact_id=s->contacts[0].id;
        p->down_ms=s->time_ms; p->down_x=x; p->down_y=y; p->anchor_x=x; p->anchor_y=y;
        p->remainder_x=0; p->remainder_y=0; p->moved=false;
        discard_motion(p);p->motion_active=false;p->landing_ready=false;p->landing_frames=1;p->landing_last_ms=s->time_ms;
        if(p->recovering) {   /* resumed after a cancel: track only, no tap, no hold (after the generic init, Codex) */
            p->recovering=false;p->tap_tracking=false;p->hold_eligible=false;p->moved=true;p->mode=BP_MOVING;
            p->motion_active=true;p->landing_ready=true;
        }
        remember_primary(p,&s->contacts[0],s->time_ms,true);
        p->last_raw_x=s->contacts[0].x;p->last_raw_y=s->contacts[0].y;p->contact_ms=s->time_ms;
        (void)movement_observe(p,s,x,y);return BP_OK;
    }
    if(s->contacts[0].id==p->contact_id && !primary_position_ok(p,s,&s->contacts[0])) {if(p->config.median_tap && p->mode==BP_CANDIDATE && s->time_ms-p->down_ms<=p->config.tap_ms){++p->tap_spikes;return BP_IGNORED;}cancel(p);return BP_FAULT;}
    if(s->contacts[0].id!=p->contact_id) {
        if(!continuous_contact(p,s,&s->contacts[0])) return primary_absent(p,s);
        p->contact_id=s->contacts[0].id;handover=true;
    }
    if(!(s->weak_contacts&1u)) remember_primary(p,&s->contacts[0],s->time_ms,handover);
    p->last_raw_x=s->contacts[0].x;p->last_raw_y=s->contacts[0].y;p->contact_ms=s->time_ms;
    if(handover) {
        p->hold_eligible=false;
        p->down_x+=x-p->anchor_x;p->down_y+=y-p->anchor_y;
        discard_motion(p);p->anchor_x=x;p->anchor_y=y;
        movement_boundary(p,s->time_ms,false);(void)movement_observe(p,s,x,y);return BP_OK;
    }
    if(s->weak_contacts&1u) {
        movement_boundary(p,s->time_ms,true);
        p->hold_eligible=false;discard_motion(p);p->anchor_x=x;p->anchor_y=y;return BP_OK;
    }
    if(interrupted_release) {motion_rebase(p,x,y);(void)movement_observe(p,s,x,y);return BP_OK;}
    bool movement_allowed=movement_observe(p,s,x,y);
    if(p->mode==BP_CANDIDATE && !(s->weak_contacts&1u)) {
        unsigned i=p->tap_median.next;p->tap_median.x[i]=x;p->tap_median.y[i]=y;
        p->tap_median.next=(i+1u)%3u;if(p->tap_median.frames<65535u)++p->tap_median.frames;
        if(s->strengths[0]>p->tap_median.peak)p->tap_median.peak=s->strengths[0];
    }
    /* A degraded primary may retain second-tap ownership in MOVING. Its new
     * drag history and loss deadline follow that ownership, not a mode label. */
    if(movement_enabled(p) && p->second_tap && p->movement_confidence_open) {
        unsigned m=p->movement_drag_median.next;
        p->movement_drag_median.x[m]=x;p->movement_drag_median.y[m]=y;
        p->movement_drag_median.next=(m+1u)%3u;
    }
    if(p->second_tap) {
        int32_t tx=x,ty=y;
        if(p->config.median_tap){tx=median3(p->tap_median.x[0],p->tap_median.x[1],p->tap_median.x[2]);ty=median3(p->tap_median.y[0],p->tap_median.y[1],p->tap_median.y[2]);}
        bool drag_move=outside(tx,ty,p->down_x,p->down_y,BP_TAP_DRAG_MOVE_UM);
        bool drag_hold=s->time_ms-p->down_ms>=BP_TAP_DRAG_HOLD_MS;
        if(movement_enabled(p) && s->strengths[0]) {
            if(!p->movement_confidence_open) {
                /* Never-qualified light second taps retain the original tap
                 * window. Lost qualification has a separate bounded deadline. */
                if(drag_move || s->time_ms-p->down_ms>p->config.tap_ms)
                    return movement_settle_first(p);
                motion_rebase(p,x,y);return BP_OK;
            }
            /* Requalification never spends old unqualified time or travel.
             * Median tap/slop above still tracks the physical tap lifetime. */
            if(!movement_allowed){motion_rebase(p,x,y);return BP_OK;}
            int32_t mx=median3(p->movement_drag_median.x[0],p->movement_drag_median.x[1],p->movement_drag_median.x[2]);
            int32_t my=median3(p->movement_drag_median.y[0],p->movement_drag_median.y[1],p->movement_drag_median.y[2]);
            drag_move=outside(mx,my,p->movement_drag_x,p->movement_drag_y,BP_TAP_DRAG_MOVE_UM);
            drag_hold=s->time_ms-p->movement_qualified_since>=BP_TAP_DRAG_HOLD_MS;
        }
        if(drag_move || drag_hold) {
            discard_motion(p);p->anchor_x=x;p->anchor_y=y;
            if(!p->lock_rejoin && !queue_one(p,1u,0,0)){cancel(p);return BP_BACKPRESSURE;}
            p->local_buttons=1;p->mode=BP_DRAG;p->second_tap=false;p->lock_rejoin=false;
            p->tap_tracking=false;p->hold_eligible=false;p->moved=true;p->motion_active=true;p->landing_ready=true;
            return BP_OK;
        }
        motion_rebase(p,x,y);return BP_OK;
    }

    if(outside(x,y,p->down_x,p->down_y,BP_STILL_UNLOCK_UM))p->hold_eligible=false;
    if(movement_allowed && p->config.guarded_hold && p->config.hold_drag && p->hold_eligible &&
       (p->still.locked || p->config.hold_without_lock) &&
       (p->mode==BP_MOVING || (p->config.hold_without_lock && p->mode==BP_CANDIDATE)) && s->time_ms-p->down_ms>=p->config.hold_ms) {
        if(!queue_one(p,1u,0,0)) {cancel(p);return BP_BACKPRESSURE;}
        p->mode=BP_DRAG;p->local_buttons=1;p->hold_eligible=false;
        p->motion_pending=0;p->remainder_x=0;p->remainder_y=0;return BP_OK;
    }
    /* Only an ordinary single candidate may consume sensor movement evidence (Codex micro-intent); the
     * second-tap path above keeps drag ownership. With tap_catch_up the step is repaid from the landing
     * origin, so an evidenced short flick moves its full distance instead of clicking or being dropped. */
    if(movement_allowed && p->mode==BP_CANDIDATE && !p->second_tap && !p->lock_rejoin &&
       s->motion_intent_id!=0u && s->motion_intent_id==p->contact_id) {
        p->moved=true;p->tap_tracking=false;p->hold_eligible=false;p->mode=BP_MOVING;
        p->motion_active=true;p->landing_ready=true;
        if(p->config.tap_catch_up) {
            /* repay the evidenced trajectory from its own settled start (never the landing point, whose
             * wander made the cursor jump on a press in v0.13), in one report now (v0.18: through the
             * position filter it trailed ~80 ms and showed the old direction after a reversal) */
            int32_t ox,oy;bp_contact o={.id=s->contacts[0].id,.x=s->motion_origin_x,.y=s->motion_origin_y};
            transform(&p->config,&o,&ox,&oy);
            discard_motion(p);p->anchor_x=x;p->anchor_y=y;
            memset(&p->guard,0,sizeof(p->guard));p->guard.ready=true;p->guard.x=x;p->guard.y=y;p->guard.ms=s->time_ms;
            if(!emit_step_um(p,x-ox,y-oy)) {cancel(p);return BP_BACKPRESSURE;}
        }
    }
    if(p->config.tap_guard && p->mode==BP_CANDIDATE && !p->moved) {
        int32_t tx=x,ty=y;
        if(p->config.median_tap){tx=median3(p->tap_median.x[0],p->tap_median.x[1],p->tap_median.x[2]);ty=median3(p->tap_median.y[0],p->tap_median.y[1],p->tap_median.y[2]);}
        if(s->time_ms-p->down_ms<=p->config.tap_ms && !outside(tx,ty,p->down_x,p->down_y,p->config.tap_slop)) {
            /* Relative HID clicks occur at the existing desktop cursor, not sensor coordinates. */
            motion_rebase(p,x,y);return BP_OK;
        }
        ++p->tap_reject[s->time_ms-p->down_ms>p->config.tap_ms?0:1];
        p->tap_tracking=false;p->moved=true;p->mode=BP_MOVING;
        discard_motion(p);p->anchor_x=x;p->anchor_y=y;p->motion_active=true;p->landing_ready=true;
        if(p->config.stationary_guard && !p->config.precise_stationary && !outside(x,y,p->down_x,p->down_y,p->config.tap_slop)) {
            p->still.ready=true;p->still.locked=true;p->still.anchor_x=x;p->still.anchor_y=y;
            p->still.x=x;p->still.y=y;p->still.ms=s->time_ms;++p->stationary_entries;
        }
        return BP_OK;
    }
    if(outside(x,y,p->down_x,p->down_y,p->config.tap_slop)) {
        p->moved=true; if(p->mode==BP_CANDIDATE) p->mode=BP_MOVING;
    }
    if(movement_allowed && p->mode==BP_CANDIDATE && !p->moved && p->config.hold_drag && !p->config.guarded_hold && s->time_ms-p->down_ms>=p->config.hold_ms) {
        if(p->queued>=BP_QUEUE_CAP-2u || !queue_one(p,1u,0,0)) { cancel(p); return BP_BACKPRESSURE; }
        p->mode=BP_DRAG; p->local_buttons=1u;
    }
    if(!movement_allowed) {motion_rebase(p,x,y);return BP_OK;}
    if(s->time_ms-p->down_ms<(p->config.responsive_startup?BP_START_QUIET_MS:BP_LANDING_QUIET_MS) || p->landing_frames<BP_LANDING_FRESH_FRAMES) {
        /* Certified tap catch-up may already have queued a bounded step.
         * Keep that step through the quiet window while tracking its anchor. */
        if(p->mode==BP_MOVING && p->landing_ready && p->guard.ready) {
            p->guard.x=x;p->guard.y=y;p->guard.ms=s->time_ms;
        } else discard_motion(p);
        p->anchor_x=x;p->anchor_y=y;return BP_OK;
    }
    if(!p->landing_ready) {
        /* The first qualified post-landing observation defines the origin.
         * Never activate from a noisy point retained inside the quiet window. */
        p->landing_ready=true;discard_motion(p);p->anchor_x=x;p->anchor_y=y;return BP_OK;
    }
    if(!p->motion_active && p->config.stationary_guard) {
        /* Initial stationary origin is set after the existing landing guard. */
        int32_t dx=x-p->anchor_x,dy=y-p->anchor_y;
        if(length_um(dx,dy)>BP_STILL_UNLOCK_UM) {
            if(p->still.confirmations && (int64_t)dx*p->still.unlock_x+(int64_t)dy*p->still.unlock_y>0) {
                p->motion_active=true;discard_motion(p);p->anchor_x=x;p->anchor_y=y;
            } else {p->still.confirmations=1;p->still.unlock_x=dx;p->still.unlock_y=dy;}
        } else p->still.confirmations=0;
        return BP_OK;
    }
    if(!p->motion_active) {
        if(outside(x,y,p->anchor_x,p->anchor_y,p->config.responsive_startup?BP_START_DEADZONE_UM:p->config.deadzone)) {
            p->motion_active=true;p->anchor_x=x;p->anchor_y=y;discard_motion(p);
        }
        return BP_OK;
    }
    if(!emit_motion(p,x,y,s->time_ms)) { cancel(p); return BP_BACKPRESSURE; }
    return BP_OK;
}
bp_result bp_flush(bp_pointer *p,uint64_t now) {
    bp_result r=bp_tick(p,now); if(r==BP_REENTRANT || r==BP_INVALID) return r;
    if(!p->online) return BP_LINK_LOST;
    for(unsigned n=0u;n<BP_FLUSH_LIMIT;++n) {
        if(!p->release_pending && p->queued==0u) return BP_OK;
        bp_report *report=p->release_pending?&p->release_report:&p->queue[p->head];
        if(report->id==0u) { lose_link(p); return BP_FAULT; }
        p->in_send=true; bp_tx_result tx=p->send(p->send_context,report); p->in_send=false;
        if(tx==BP_TX_BUSY) return BP_BACKPRESSURE;
        if(tx!=BP_TX_ACCEPTED) {
            lose_link(p); return tx==BP_TX_UNKNOWN?BP_AMBIGUOUS:BP_LINK_LOST;
        }
        p->last_accepted_buttons=report->buttons; p->accepted_buttons_known=true;
        if(p->release_pending) p->release_pending=false;
        else { p->head=(p->head+1u)%BP_QUEUE_CAP; --p->queued; }
    }
    return p->queued!=0u?BP_BACKPRESSURE:BP_OK;
}
bp_result bp_encode_mouse3(const bp_report *r,uint8_t out[3]) {
    if(r==NULL || out==NULL || r->buttons>1u || r->dx < -127 || r->dx>127 || r->dy < -127 || r->dy>127 || r->wheel!=0) return BP_INVALID;
    out[0]=r->buttons; out[1]=(uint8_t)r->dx; out[2]=(uint8_t)r->dy; return BP_OK;
}

