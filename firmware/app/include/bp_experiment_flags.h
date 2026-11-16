#ifndef BP_EXPERIMENT_FLAGS_H
#define BP_EXPERIMENT_FLAGS_H
/* Independently disable one experiment and rebuild with app/tools/build.py. */
#define BP_EXP_TAP_GUARD 1
#define BP_EXP_SPIKE_GUARD 1
#define BP_EXP_STATIONARY_GUARD 1
#define BP_EXP_RELEASE_DEBOUNCE 1
#define BP_EXP_HOLD_DRAG 0
#define BP_EXP_SCROLL_ACCELERATION 0
#define BP_EXP_TAP_DRAG 1
#define BP_EXP_DRAG_LOCK 0
#define BP_EXP_SCROLL_LATCH 1
#define BP_EXP_CONTINUOUS_GAIN 0
#define BP_EXP_ZERO_REVERSE_DEADBAND 1
#define BP_EXP_FIXED_MOTION_GAIN 1
#define BP_EXP_ABSOLUTE_SPIKE 1
#define BP_EXP_PRECISE_STATIONARY 1
#define BP_EXP_CONFIRMED_SECONDARY 1
#define BP_EXP_MEDIAN_TAP 1
#define BP_EXP_IGNORE_WEAK_SECONDARY 1
#define BP_EXP_SHORT_TAP 0 /* withdrawn: retain 40ms */
#define BP_EXP_EXTENDED_DRAG_WINDOW 1
#define BP_EXP_SCROLL_PRECONFIRM_PAUSE 1
#define BP_EXP_SCROLL_REPORT_CAP 1
#define BP_EXP_SCROLL_TWO_MM 1
#define BP_EXP_MOTION_MEDIAN 1
#define BP_EXP_TRAVEL_STILL 1
#define BP_EXP_MICRO_RESUME 1
#define BP_EXP_SCROLL_REPAIR 1
#define BP_EXP_PAIR_PAUSE_30 1 /* landing only; paired with paused-tap travel validation */
#define BP_EXP_PURGE_MOTION_ON_ZERO 1
/* IQS9151: both gates use IQS550 strength units (300/220, 450/350). Off until the IQS9151 strength
 * scale is measured on the board; hover rejection is the chip threshold plus the runtime
 * admission floor in bp_touch_policy (backpad_iqs_params). */
#define BP_EXP_PRIMARY_CONFIDENCE 0
#ifndef BP_EXP_MOTION_CONFIDENCE
#define BP_EXP_MOTION_CONFIDENCE 0
#endif
#endif

