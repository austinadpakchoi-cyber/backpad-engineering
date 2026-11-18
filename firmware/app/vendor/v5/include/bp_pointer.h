#ifndef BP_POINTER_H
#define BP_POINTER_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define BP_COORD_MAX 10000
#define BP_QUEUE_CAP 16u
#define BP_FLUSH_LIMIT 8u
/* Input API remains normalized. Internal distances are uniform micrometres. */
/* Distance between the first and last electrode centres of the sensor, set per board at build time. */
#ifndef BP_PHYSICAL_X_UM
#define BP_PHYSICAL_X_UM 36000
#endif
#ifndef BP_PHYSICAL_Y_UM
#define BP_PHYSICAL_Y_UM 61600
#endif
#define BP_PAIR_CONFIRM_FRAMES 3u
#define BP_PAIR_CONFIRM_MS 40u
#define BP_CONTACT_CONTINUITY_MS 60u
#define BP_PRIMARY_MISSING_MS 100u
#ifndef BP_SECONDARY_RELATIVE_PERCENT
#define BP_SECONDARY_RELATIVE_PERCENT 50u  /* IQS550 ghost defence; the IQS9151 build sets 20 (see CMakeLists) */
#endif
/* the 30 % early-pair allowances never exceed the general floor */
#define BP_SECONDARY_EARLY_PERCENT (BP_SECONDARY_RELATIVE_PERCENT < 30u ? BP_SECONDARY_RELATIVE_PERCENT : 30u)
#define BP_CONTACT_CONTINUITY_MM 8u
#define BP_LANDING_QUIET_MS 50u
#define BP_LANDING_FRESH_FRAMES 3u
#define BP_START_QUIET_MS 50u /* 25ms failed recorded tap zero-motion regression */
#define BP_START_DEADZONE_UM 500u
#define BP_START_DELAY_FRAMES 1u
#define BP_START_UNITY_MS 150u
#define BP_START_FINE_LOW_MM_S 10u
#define BP_START_FINE_Q8 179u /* approximately 0.7 */
#define BP_RISING_SPEED_TAU_MS 20u
#define BP_MOTION_DELAY_FRAMES 2u
#define BP_BASE_GAIN_Q8 (20u*256u)
#define BP_FINE_LOW_MM_S 15u
#define BP_FINE_MIN_Q8 128u /* half gain for precision positioning */
#define BP_ACCEL_LOW_MM_S 30u
#define BP_ACCEL_HIGH_MM_S 150u
#define BP_ACCEL_MAX_Q8 640u /* 2.5x */
#define BP_SPEED_WINDOW 64u
#define BP_SPEED_SPAN_MS 50u
#define BP_POSITION_SLOW_TAU_MS 40u
#define BP_POSITION_FAST_TAU_MS 15u
#define BP_REVERSE_DEADBAND_UM 300u
#define BP_HIGH_RES_GAIN_Q8 (40u*256u)
#define BP_HIGH_RES_REVERSE_UM 100u
#define BP_HIGH_RES_SLOW_TAU_MS 15u
#define BP_HIGH_RES_FAST_TAU_MS 10u
/* Independent A/B switches. B1 enables the three resume fixes; B2 adds soft gain. */
#ifndef BP_EXP_RESUME_RAW_ANCHOR
#define BP_EXP_RESUME_RAW_ANCHOR 1
#endif
#ifndef BP_EXP_RESUME_RAMP
#define BP_EXP_RESUME_RAMP 1
#endif
#ifndef BP_EXP_RESUME_GUARD_GRACE
#define BP_EXP_RESUME_GUARD_GRACE 1
#endif
#ifndef BP_EXP_SOFT_LOW_SPEED
#define BP_EXP_SOFT_LOW_SPEED 0
#endif
#define BP_RESUME_RAMP_MS 100u
#define BP_RESUME_START_Q8 77u /* 0.30x, after stationary unlock only */
#define BP_RESUME_GRACE_MS 150u
#define BP_SOFT_LOW_MM_S 2u
#define BP_SOFT_HIGH_MM_S 8u
#define BP_SOFT_MIN_Q8 38u /* 0.15x, B2 only */
/* Physical, time-scaled coordinate guard. Separate from HID gain. */
#define BP_SPIKE_ABSOLUTE_UM 1500u
#define BP_SPIKE_FLOOR_UM 450u
#define BP_SPIKE_MARGIN_UM 150u
#define BP_SPIKE_CONFIRM_UM 250u
#define BP_STILL_SPEED_MM_S 4u
#define BP_STILL_TIME_MS 100u
#define BP_STILL_RADIUS_UM 150u
#define BP_STILL_UNLOCK_UM 400u
#define BP_TAP_MIN_MS 40u
#define BP_RELEASE_CONFIRM_FRAMES 4u /* reject 1-3 missing sensor frames */
#define BP_RELEASE_CONFIRM_MS 30u
#define BP_SCROLL_ACCEL_LOW_MM_S 20u
#define BP_SCROLL_ACCEL_HIGH_MM_S 120u
#define BP_SCROLL_ACCEL_MAX_Q8 768u
#define BP_TAP_DRAG_WINDOW_MS 150u
#define BP_TAP_DRAG_HOLD_MS 200u
#define BP_TAP_DRAG_MOVE_UM 500u
#define BP_TAP_DRAG_REJOIN_UM 3000u /* a remote re-landing is a fresh navigation contact */
#define BP_DRAG_LOCK_MS 250u
/* Trial pointer confidence only; not IQS/secondary/scroll admission thresholds. */
#define BP_PRIMARY_CONFIDENCE_ENTER 300u
#define BP_PRIMARY_CONFIDENCE_EXIT 220u
#define BP_PRIMARY_CONFIDENCE_DRAG_MS 60u
#define BP_PRECISE_STILL_MS 250u
#define BP_PRECISE_STILL_NET_UM 100u
#define BP_PRECISE_STILL_TRAVEL_UM 150u
#define BP_MICRO_UNLOCK_UM 200u
#define BP_MICRO_SLOW_TAU_MS 25u
#define BP_SCROLL_STEP_UM 3000u /* 3mm per legacy wheel detent */
#ifndef BP_SCROLL_START_UM
#define BP_SCROLL_START_UM 8000 /* incremental scroll: the pair sum (both fingers' travel) must move this far from where it rested
                                 * before the first detent, again after BP_SCROLL_REARM_MS without one; the travel is then paid
                                 * at once. A resting pair whose chip merges/splits it wanders a few mm and used to scroll itself */
#endif
#define BP_SCROLL_COHERENT_UM 1000 /* v0.20.6 (was 1500) ... unless both members moved this far the same way (a real two-finger push starts at once;
                                     * a resting pair's merge/split wander moves one member at a time) */
#define BP_SCROLL_REARM_MS 600u /* v0.20.6 (was 300): a slow push (1 detent per >300 ms) re-armed between detents and stuttered */
#define BP_SCROLL_JUMP_UM 3000 /* incremental scroll: a larger one-frame step of a pair member is a discontinuity, not travel */
_Static_assert(BP_PAIR_CONFIRM_FRAMES>=3u,"pair confirmation requires three frames");
_Static_assert(BP_MOTION_DELAY_FRAMES==2u,"motion tail guard requires two frames");

typedef enum { BP_OK, BP_IGNORED, BP_WAITING, BP_INVALID, BP_FAULT,
               BP_BACKPRESSURE, BP_LINK_LOST, BP_AMBIGUOUS, BP_REENTRANT } bp_result;
/* SUPPRESSED: valid raw TOUCH exists but no contact passes admission strength.
 * It is neither a physical release nor a transport/decode error. */
typedef enum { BP_DATA, BP_NO_DATA, BP_BAD_FRAME, BP_SUPPRESSED } bp_sample_kind;
typedef struct { uint64_t id; int32_t x,y; } bp_contact;
typedef struct {
    bp_sample_kind kind;
    uint32_t session, sequence; /* host window epoch, NOT a sensor sequence */
    uint64_t time_ms;
    uint8_t count;
    bp_contact contacts[5];
    uint16_t strengths[5]; /* physical representatives; zero means unavailable in synthetic input */
    uint8_t weak_contacts; /* admission-held below enter strength; cannot confirm a new pair */
    /* Optional single-contact movement evidence (Codex micro-intent). Zero preserves legacy behaviour;
     * coordinates remain filtered. This is never a button or a drag request. */
    uint64_t motion_intent_id;
    /* With motion_intent_id: where the evidenced trajectory began (the settled baseline after landing,
     * core coordinates). The core may repay the travel from here, never from the landing point. */
    int32_t motion_origin_x, motion_origin_y;
    bool motion_only; /* Low-area swipe evidence cannot acquire any button. */
} bp_sample;
typedef struct {
    uint32_t tap_ms, hold_ms, stale_ms;
    uint16_t tap_slop, deadzone; /* physical micrometres */
    uint16_t gain_q8; /* linear HID counts/mm in Q8; default20*256 */
    uint16_t rotation; /* 0/90/180/270 applied to output coordinates */
    bool invert_x, invert_y, hold_drag;
    bool scroll_enabled, invert_scroll;
    uint16_t scroll_step; /* physical micrometres per wheel detent */
    uint32_t scroll_join_ms; /* maximum join delay eligible for right tap */
    bool two_finger_tap; /* optional right click; never combined with wheel */
    uint32_t pair_release_ms; /* maximum stagger between the two lift events */
    bool motion_smoothing; /* adaptive physical-position filter and reversal gate */
    bool high_resolution; /* light filter, 0.1mm reversal gate; host acceleration */
    bool responsive_startup; /* shorter landing/onset/tail with unity startup gain */
    bool pointer_acceleration; /* cursor only; never amplifies wheel */
    bool tap_guard;
    bool release_debounce, guarded_hold, scroll_acceleration;
    bool spike_guard, stationary_guard; /* independently selectable for A/B replay */
    bool confirmed_secondary,median_tap,ignore_weak_secondary,short_tap,extended_drag_window,scroll_preconfirm_pause,scroll_report_cap;
    bool tap_drag,drag_lock,scroll_latch,continuous_gain,absolute_spike,precise_stationary,zero_reverse_deadband,fixed_motion_gain;
    /* When sensor movement evidence (motion_intent_id) ends a tap candidate, repay the evidenced trajectory
     * from its settled origin (motion_origin_x/y). Off by default (IQS550 builds unchanged). */
    bool tap_catch_up;
    /* Press-and-hold drag without the stationary latch: a contact that stays within BP_STILL_UNLOCK_UM of its
     * landing point for hold_ms presses the left button; moving then drags, lifting releases. The IQS9151
     * policy already holds the position still below its HOLD radius. Off by default. */
    bool hold_without_lock;
    /* Merge consecutive unsent motion-only reports with the same buttons (see queue_one). Off by default. */
    bool coalesce_motion;
    /* After a cancel (BP_WAIT_ZERO) a single non-weak contact that stays for this long resumes as a fresh
     * MOVING contact (no tap/hold eligibility) instead of waiting for the finger to leave. 0 = off. */
    uint32_t wait_zero_recover_ms;
    bool motion_median,travel_still,micro_resume;
    bool scroll_repair,pair_pause_30;
    bool soft_low_speed; /* explicit precision-profile opt-in, B2 only */
    bool motion_confidence_gate; /* opt-in: tap eligibility is unchanged */
    bool purge_motion_on_zero,primary_confidence_gate; /* independent opt-ins */
    uint16_t primary_confidence_enter,primary_confidence_exit;
    uint32_t primary_confidence_drag_ms; /* maximum weak-signal button hold */
    /* Incremental two-finger scroll (see scroll_track): each pair member's own frame-to-frame travel is summed,
     * twice a lone member's while the other is weak or missing, so the lighter finger's weak or dropped frames
     * neither pause the scroll nor discard the travel since the last detent. Off by default. Appended last:
     * the tuning service reads earlier config fields by fixed offset. */
    bool scroll_incremental;
    /* Cursor-only smooth velocity gain. Appended in the existing96B padding;
     * all preceding tuning offsets and legacy profiles retain their meaning. */
    bool adaptive_motion_gain;
} bp_config;
typedef struct {
    uint64_t id; /* local id for trace/correlation, not HID payload */
    uint32_t link_epoch;
    uint8_t buttons;
    int16_t dx,dy; /* each component limited to -127..127 */
    int16_t wheel;
} bp_report;
typedef enum {
    BP_TX_ACCEPTED, /* exactly one ordered local acceptance; NOT proof of delivery */
    BP_TX_BUSY,     /* definitely not accepted, safe to retry same id */
    BP_TX_DOWN,    /* disconnected; no remote-state assertion */
    BP_TX_UNKNOWN, /* ambiguous acceptance: NEVER retry this report */
    BP_TX_ERROR    /* fatal endpoint fault: require explicit fresh link */
} bp_tx_result;
typedef bp_tx_result (*bp_send)(void *,const bp_report *);
typedef enum { BP_WAIT_ZERO, BP_IDLE, BP_CANDIDATE, BP_MOVING, BP_DRAG, BP_SCROLL, BP_PAIR_RELEASE, BP_DRAG_LOCK } bp_mode;
typedef struct {
    uint64_t id,first_ms,last_ms;
    unsigned frames;
} bp_contact_confirmation;
typedef struct {
    struct {
        bool ready,pending;
        int32_t x,y,px,py;
        uint64_t ms,pms;
        unsigned count,next;
        struct {int32_t dx,dy;uint32_t dt;} history[4];
    } guard;
    struct {
        bool ready,locked;
        uint64_t ms,since,unlock_since;
        int32_t history_x[3],history_y[3];unsigned history_count,history_next;
        int32_t x,y,anchor_x,anchor_y,unlock_x,unlock_y;
        unsigned confirmations;
    } still;
    struct {int32_t x[3],y[3];unsigned count,next;uint64_t ms;} motion_median;
    /* First click is already emitted. Rejoin window recognizes click2/drag only;
     * keep the original state field types/order and config ABI unchanged. */
    bool tap_rejoin_pending,second_tap,lock_rejoin,scroll_latched,scroll_paused;
    struct {uint64_t low,high,since;unsigned frames;} scroll_repair_candidate;
    uint64_t tap_rejoin_deadline,drag_lock_deadline;
    struct {bool ready;uint64_t ms;int32_t x,y,last_x,last_y;uint64_t travel;} precise_still;
    uint64_t resume_ms;
    struct {
        int32_t x[5],y[5],last_x,last_y;unsigned count,next;
        uint64_t ms;int32_t vx_q8,vy_q8;
    } soft;
    struct {int32_t x[3],y[3];unsigned next,frames;uint16_t peak;} tap_median;
    uint64_t pair_pause_since;bool pair_pause_active;
    bool tap_tracking, tap_ambiguous, tap_second_seen, hold_eligible;
    unsigned release_frames;
    uint64_t release_first_ms;
    uint32_t release_bounces,pair_weak_frames,pair_late_frames,pair_unconfirmed;
    int32_t scroll_last_sum,scroll_velocity_q8;
    int64_t scroll_remainder_q8;
    uint64_t scroll_ms;
    uint32_t tap_reject[7],tap_left,tap_right,tap_spikes,last_cancel_line;
    uint64_t tap_spike_since,tap_weak_since;
    unsigned pair_scroll_frames;
    uint32_t spike_rejected,spike_reanchors,motion_clipped,stationary_entries;
    /* Private motion qualification; not part of the published diagnostics ABI. */
    bool movement_confidence_open,movement_confidence_seen;
    bool movement_initial_eligible;
    uint64_t movement_confidence_id,movement_confidence_since,movement_qualified_since;
    int32_t movement_drag_x,movement_drag_y;
    struct {int32_t x[3],y[3];unsigned next;} movement_drag_median;
    uint32_t movement_confidence_suppressed,movement_confidence_rebases;
    uint32_t movement_confidence_drag_releases,movement_confidence_intent_cancels;
    bool primary_confidence_paused;
    uint64_t primary_confidence_since; /* zero: zero-contact recovery, not weak TOUCH */
    uint32_t primary_confidence_suppressed,primary_confidence_pauses;
    uint32_t primary_confidence_rebases,primary_confidence_drag_releases;
    /* Passive TOUCH history for pair landing; never arms a pointer tap. */
    struct {uint64_t id,first_ms,last_ms;unsigned frames;} pair_landing;
    uint64_t scroll_landing_ms;
    bp_config config;
    bp_send send; void *send_context;
    bool initialized, online, linked_once, release_pending, clock_known, in_send;
    bool have_frame, have_fresh, moved;
    uint32_t link_epoch;
    uint64_t last_call_ms, last_fresh_ms, down_ms, contact_id, next_report_id;
    bp_mode mode;
    uint8_t local_buttons, last_accepted_buttons;
    bool accepted_buttons_known;
    int32_t down_x,down_y,anchor_x,anchor_y;
    int32_t remainder_x,remainder_y; /* signed fractions, denominator65536000 */
    int32_t last_raw_x,last_raw_y;
    struct {int32_t x,y;uint64_t ms;} primary_history[4];
    unsigned primary_history_count,primary_history_next;
    uint64_t contact_ms,landing_last_ms;
    unsigned landing_frames,motion_pending;
    bool motion_active,landing_ready;
    struct {int32_t dx,dy;uint16_t gain_q8;} motion_delay[BP_MOTION_DELAY_FRAMES];
    uint64_t motion_started_ms;
    uint64_t speed_ms;
    uint32_t adaptive_speed_q8;
    int32_t speed_last_dx,speed_last_dy;
    unsigned speed_count,speed_index;
    struct {int32_t dx,dy;uint32_t dt_ms;} speed_samples[BP_SPEED_WINDOW];
    bool position_ready;
    uint64_t position_ms;
    int64_t position_x_q8,position_y_q8;
    int32_t position_out_x,position_out_y;
    int32_t velocity_x_q8,velocity_y_q8;
    int8_t direction_x,direction_y;
    uint64_t pending_pair_low,pending_pair_high,pending_pair_ms,pending_pair_last_ms;
    unsigned pending_pair_frames;
    bp_contact_confirmation extras[4];
    bool single_only,primary_missing;
    uint64_t primary_missing_since;
    uint64_t pair_low,pair_high;
    int32_t scroll_anchor_sum;
    bool pair_tap_possible;
    uint64_t pair_down_ms,pair_lift_ms,pair_remaining;
    int32_t pair_low_x,pair_low_y,pair_high_x,pair_high_y;
    bp_sample last_frame;
    bp_report queue[BP_QUEUE_CAP], release_report;
    unsigned head, queued;
    uint32_t cancellations, dropped_unsent_reports, coalesced_reports, wait_zero_recoveries;
    uint64_t wait_zero_since, recover_id; int32_t recover_x, recover_y; bool recovering;
    /* scroll_incremental state (appended last, see bp_config) */
    int32_t scroll_py[2],scroll_vsum; bool scroll_pok[2],scroll_confirmed;
    uint16_t scroll_ps[2];   /* each member's strength the last time both were present */
    bool scroll_started; int32_t scroll_origin; uint64_t scroll_detent_ms;   /* start hysteresis, see BP_SCROLL_START_UM */
    int32_t scroll_mt[2];   /* each member's own travel since the origin */
    int32_t scroll_qy[2]; bool scroll_qok[2];   /* v0.20.6: last known position of each member, weak frames included */
    /* Physical activity for scroll rearm; earlier field offsets remain unchanged. */
    int32_t scroll_activity_sum; uint64_t scroll_activity_ms;
} bp_pointer;

bp_config bp_default_config(void); /* provisional development parameters */
bp_result bp_init(bp_pointer *,const bp_config *,bp_send,void *);
/* All APIs are serialized by caller; callbacks cannot reenter. Connect requires
 * a new monotonically increasing epoch, and the backend must quiesce old-epoch
 * submissions. CONNECT never asserts that an old remote button was released. */
bp_result bp_connect(bp_pointer *,uint32_t epoch,uint64_t now_ms);
bp_result bp_disconnect(bp_pointer *,uint64_t now_ms);
bp_result bp_process(bp_pointer *,const bp_sample *,uint64_t now_ms);
bp_result bp_tick(bp_pointer *,uint64_t now_ms);
bp_result bp_flush(bp_pointer *,uint64_t now_ms);
/* Logical HID-style 3-byte mouse payload (buttons, signed x, signed y).
 * No BLE characteristic, Report ID, GATT descriptor, or OS acceptance implied. */
bp_result bp_encode_mouse3(const bp_report *,uint8_t out[3]);
#endif

