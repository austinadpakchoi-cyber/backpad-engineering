#ifndef BP_TOUCH_POLICY_H
#define BP_TOUCH_POLICY_H
#include "bp_iqs9151.h"
#include "bp_pointer.h"
/* IQS9151 frame -> gesture-core sample. The chip already tracks fingers (slot = identity), applies
 * its own set/clear touch thresholds and reports per-finger strength and area. This layer adds a
 * runtime-tunable admission gate (enter, then held down to exit like a Schmitt trigger) and converts
 * coordinates to the core's 0..BP_COORD_MAX space.
 * Hover gate = density, strength / area: the mean delta over the touched channels. Measured on board 1
 * under 3 mm acrylic + glass (target 600): hover at 5 mm <= 650, hover at 2 mm <= 738 (area up to 10,
 * strength up to 7379), light touch 1000..1156 (8000..9250 / 8), normal press 1214..1313. Strength
 * or area alone cannot separate 2 mm hover from a light touch.
 * A contact that falls below the exit floor for BP_TOUCH_RELEASE_FRAMES frames is RELEASED (a lift),
 * not suppressed: every real lift passes through the hover range before the chip lets go, and a
 * suppressed frame would cancel the tap (bp_pointer primary_absent). Re-admission starts a new
 * lifetime, so the core never stitches the two touches. */
#define BP_TOUCH_MAX_CONTACTS 5u
#define BP_SENSOR_STALL_MS 1000u
#define BP_TOUCH_RELEASE_FRAMES 5u      /* v0.14 (was 3): an 8 s slow stroke on v0.13 split into 12 lifetimes (chip
                                         * dropouts plus light-slide density dips), each restarting the core's landing
                                         * and tap candidate: stick-slip, and 72 ms fragments became clicks */
/* Web-page runs on v0.6 (board 1, 12x10 mapping, strongest finger, area >= 4): hover at 2 mm up to 682
 * with area 10..13, touching while sliding 5th percentile 961..1028, held press >= 1143. At 650/480 a
 * 2 mm hover moved the cursor; at 850/650 hover 5 mm and 2 mm gave 0 motion and 0 clicks while light
 * and normal strokes moved normally. Hover blips of 1..2 channels reach 700..1089 (density = one
 * channel's delta), so a new contact also needs an area of 4 or more. */
/* v0.19: an admitted contact this large is a pressed flat finger (hover never exceeded area 15 in any recorded
 * hover test); it stays down while its density is >= BP_TOUCH_BIG_EXIT_DENSITY. Real circles drawn on board 1
 * (00:27): area 40..57 at density 530..700, i.e. at the 520 exit gate. */
#define BP_TOUCH_BIG_AREA 20u
#define BP_TOUCH_BIG_EXIT_DENSITY 400u
/* v0.19: admitted contacts are matched to the chip's reported contacts by position every frame (chip slot numbers
 * are not trusted): a match must lie within this distance of the contact's last position (chip counts). */
#define BP_TOUCH_HANDOVER_COUNTS 460   /* ~8 mm at 17.6 um/count */
/* A new contact this close to an admitted one must be seen in BP_TOUCH_SPLIT_FRAMES consecutive frames before it
 * is admitted (board 1 circles at 00:27: a large fast finger threw a one-frame blob 15 mm away, which became a
 * second finger and latched the core into scroll until lift). */
#define BP_TOUCH_SPLIT_COUNTS 1140     /* ~20 mm */
#define BP_TOUCH_SPLIT_FRAMES 3u
/* v0.20 electrode-gap handling. Board 1 runs with two Tx rows removed from the mapping (a 9 mm hole in the pad centre
 * and one at the edge); over such a gap the chip's centroid flips to either side of the hole every frame (00:26 light
 * taps in the centre: chip y 17 <-> 24 mm, i.e. physical 26 <-> 36 mm), and One Euro, just initialised, lets a third of
 * the jump through, which breaks the 1.5 mm tap slop. Inside a gap zone (within one line pitch of a removed interior
 * line, from geo->row_phys / col_phys) the policy: feeds the mean of the last two raw frames, runs One Euro at its
 * minimum cutoff (no speed opening), and admits one frame later so the core's landing point is the mean. */
#define BP_TOUCH_GAP_ZONE_Q8 384u       /* 1.5 pitches each side of the gap centre, in 1/256 pitch */
#ifndef BP_TOUCH_GAP_BETA_Q8
#define BP_TOUCH_GAP_BETA_Q8 128u       /* One Euro speed coefficient scale inside a gap zone (256 = unchanged; v0.20 sweep: 256→jitter 160, 128→118, 64→90 but −21% fast-stroke path in the zone, 0→28 but −78%) */
#endif
/* Generic alternation latch: three consecutive raw steps >= BP_TOUCH_ZIGZAG_MM, each reversing direction, cannot be a
 * finger; the fed position is then the 2-frame mean for BP_TOUCH_ZIGZAG_HOLD_MS. An isolated out-and-back spike (two
 * steps) does not latch: the core's spike guard already drops those, and halving them would let them through. */
#ifndef BP_TOUCH_PREDICT
#define BP_TOUCH_PREDICT 1          /* association prefers the candidate ahead along consistent motion (0: nearest only) */
#endif
#ifndef BP_TOUCH_HOLDABLE_MATCH
#define BP_TOUCH_HOLDABLE_MATCH 1   /* only contacts satisfying the hold gate may carry an admitted lifetime */
#endif
#ifndef BP_TOUCH_EURO_SHARED
#define BP_TOUCH_EURO_SHARED 1      /* One Euro cutoff from the vector speed (0: per axis) */
#endif
/* v0.20 lift tail: a STOPPED finger whose strength falls below BP_TOUCH_LIFT_PCT of its recent level (EMA over ~50 ms)
 * is leaving; the contact peels off one side first and the centroid drifts 0.5..1.5 mm in the lift direction during
 * the last 30..60 ms (user: "lifting at an angle moves the cursor, a Magic Trackpad does not"). The position is held
 * until the contact ends, the strength recovers (re-press), or BP_TOUCH_LIFT_HOLD_MS pass (a deliberate lightening).
 * A moving finger is never held (that froze a re-landed stroke in v0.19). */
#define BP_TOUCH_LIFT_PCT 80u
#define BP_TOUCH_LIFT_RECOVER_PCT 92u
#define BP_TOUCH_LIFT_HOLD_MS 120u
#define BP_TOUCH_ZIGZAG_MM 1.5f
#define BP_TOUCH_ZIGZAG_HOLD_MS 120u
#ifndef BP_TOUCH_ENTER_DENSITY_OVERRIDE   /* -D overrides: replay captures recorded at another ATI target with their own gates */
#define BP_TOUCH_ENTER_DENSITY 400u   /* v0.20.3 (v0.20.2 550, was 850 at ATI 600): at ATI 250 66 of 95 landings start below 550 and a light circle sits
                                         * at 350..550 for 45 % of frames, so 550 delayed the start; the chip reports no hover at 2 mm (0 of 81 samples) */
#else
#define BP_TOUCH_ENTER_DENSITY BP_TOUCH_ENTER_DENSITY_OVERRIDE
#endif
#ifndef BP_TOUCH_EXIT_DENSITY_OVERRIDE
#define BP_TOUCH_EXIT_DENSITY 260u   /* v0.20.3 (v0.20.2 350, was 520): 8.0 % of mid-contact samples dipped below 350 (felt as dropouts), 3.2 % below 260 */
#else
#define BP_TOUCH_EXIT_DENSITY BP_TOUCH_EXIT_DENSITY_OVERRIDE
#endif
#ifndef BP_TOUCH_SECOND_ENTER_DENSITY
#define BP_TOUCH_SECOND_ENTER_DENSITY 250u   /* second-finger gate (bp_touch_policy.c); 0 disables. v0.20.4: 300 -> 250, the scroll
                                               * simulation 131 -> 138/240 and pointer leak at 24 mm 25..32 -> 8..11 HID per push */
#endif
#define BP_TOUCH_SECOND_MIN_COUNTS 570   /* ~10 mm */
#define BP_TOUCH_SECOND_MAX_COUNTS 2270  /* ~40 mm */
#define BP_TOUCH_ENTER_AREA 4u
#define BP_TOUCH_EXIT_AREA 3u
#define BP_TOUCH_FLIP_X 1u   /* sensor X mirror; with the production rotation 270 this is the phone-back mount */
#define BP_TOUCH_FLIP_Y 2u
/* Runtime parameters, written by the host through RAM (backpad_iqs_params) and applied when seq
 * changes. Defaults: no strength/area floor, density 850/700. strength_scale_q8 maps IQS9151 strength
 * into the gesture core's legacy strength units (its secondary-finger floor is 80). */
typedef struct {
    uint32_t seq;
    uint16_t enter_strength, exit_strength;
    uint16_t enter_area, exit_area;
    uint16_t strength_scale_q8;
    uint16_t enter_density, exit_density;  /* strength per area unit; 0 = off; area 0 counts as 0 */
    uint16_t orient;                       /* BP_TOUCH_FLIP_X / BP_TOUCH_FLIP_Y, applied after geometry */
} bp_touch_params;
#define BP_TOUCH_PARAMS_DEFAULT {0u, 0u, 0u, BP_TOUCH_ENTER_AREA, BP_TOUCH_EXIT_AREA, 256u, \
                                 BP_TOUCH_ENTER_DENSITY, BP_TOUCH_EXIT_DENSITY, 0u}
/* Electrode geometry in use: chip coordinates span (n-1) pitches of the lines in use; phys[i] is the
 * settings-table index of line i, so lines taken out of the mapping get their physical spacing back. */
typedef struct {
    uint8_t n_cols, n_rows, orig_cols, orig_rows;
    const uint8_t *col_phys, *row_phys;
} bp_touch_geometry;
/* Position conditioning, chosen by replaying full-rate raw captures from board 1 (chip IIR off; held
 * finger: y std 0.5 mm, frame-to-frame p90 1.2..1.4 mm; taps: centroid moves 1..2 mm while the strength
 * is still rising and 2..5 mm once it falls; single-frame dropouts while sliding):
 * - landing settle: a new contact is admitted once its strength stops rising by > settle_pct per frame
 *   (at most BP_TOUCH_SETTLE_MAX frames), so the tap's down position is the settled one;
 * - lift hold: within tap_window_ms of admission, a frame below lift_pct of the contact's peak strength
 *   keeps the last trusted position (the finger is leaving);
 * - dropout: an admitted contact the chip misses for < BP_TOUCH_RELEASE_FRAMES frames keeps its
 *   lifetime and position (a new lifetime would restart the core's landing guard mid-stroke);
 * - One Euro filter (fmin, beta per mm/s, derivative cutoff dcut) then a string deadband of string_um.
 * End-to-end replay of those captures through this policy and the production core (test 13): 9 real taps
 * 0 clicks without conditioning -> 9 with it (tap_ms 350), held-finger cursor travel 449 -> ~50 HID over
 * 3 x 1.6 s; a 0.25 mm string left 361. Synthetic 150 mm/s stroke on real noise: ~1.4 mm behind after
 * 120 ms. All runtime-tunable (seq last). */
typedef struct {
    uint32_t seq;
    uint16_t fmin_chz, beta_milli, dcut_chz, string_um;  /* 1.00 Hz, 0.100 Hz per mm/s, 1.00 Hz, HOLD radius 600 um */
    uint16_t settle_pct, lift_pct, tap_window_ms, flags;  /* 15 %, 85 %, 250 ms, BP_TF_* */
    uint16_t dwell_ms, stop_um;   /* 60 ms: HOLD ends only after the position stays outside it this long;
                                   * 400 um: TRACK returns to HOLD once the position stays within it (0 = BP_TOUCH_STOP_UM).
                                   * Replay sweep (BP_SWEEP3): clean 1 mm forward/back 39/-38 HID, 9/9 raw taps, held
                                   * back-and-forth 84 (538 unconditioned); a 2.5 mm flick no longer clicks (it is lost).
                                   * HOLD also ends at once beyond twice its radius: noise excursions are short. */
} bp_touch_filter_params;
#define BP_TF_EURO 1u
#define BP_TF_STRING 2u
#define BP_TF_SETTLE 4u
#define BP_TF_LIFT 8u
#define BP_TF_HOLDTRACK 16u  /* hold/track instead of the continuous trailing string (v0.13; off: stick-slip on slow strokes) */
#define BP_TOUCH_FILTER_DEFAULT {0u, 100u, 100u, 100u, 400u, 15u, 85u, 250u, BP_TF_EURO | BP_TF_STRING | BP_TF_SETTLE | BP_TF_LIFT, 0u, BP_TOUCH_MOVING_STRING_UM}
#define BP_TOUCH_SETTLE_MAX 3u
/* Hold / track (replaces a trailing string, which always stopped one string length short and needed two to
 * reverse: Codex replay 1 mm forward 23 HID, 1 mm back 6). HOLD: the output stays put while the filtered
 * position is within string_um of it. Leaving it for dwell_ms (or by twice string_um at once) switches to
 * TRACK, which outputs the filtered position itself (the whole movement, no trailing gap, reversals at once).
 * TRACK returns to HOLD once the position has stayed within BP_TOUCH_STOP_UM for BP_TOUCH_STOP_MS. */
#define BP_TOUCH_STOP_UM 150u
#define BP_TOUCH_STOP_MS 80u
/* TRACK trails the position by at most this much (absorbs noise while moving); entering HOLD closes the gap. */
#define BP_TOUCH_TRACK_STRING_UM 150u
/* Continuous string (default): string_um while the finger has been still for BP_TOUCH_REST_MS, else this much,
 * so a reversal mid-adjustment costs 0.24 mm instead of 0.8 mm (Codex: 0.5 mm reversal after a pause gave 0). */
#define BP_TOUCH_MOVING_STRING_UM 120u
/* Established single-finger corrections, not landing/tap or paired scrolling. */
#define BP_TOUCH_FINE_STRING_UM 6u
#define BP_TOUCH_FINE_INTENT_MM 0.04f
/* Cap of the displacement repaid when movement evidence ends a tap candidate. */
#define BP_TOUCH_REPAY_CAP_MM 1.0f   /* v0.18: emitted in one report by the core, so it no longer trails */
#define BP_TOUCH_REST_MS 600u
#ifndef BP_TOUCH_MOVING_NET_MM
#define BP_TOUCH_MOVING_NET_MM 0.4f   /* net filtered travel over ~400 ms that counts as "moving" */
#endif
#ifndef BP_TOUCH_JUMP_MM
#define BP_TOUCH_JUMP_MM 4.0f   /* v0.20.4: a lifetime's raw centroid stepping further than this in one frame is the chip merging or
                                 * splitting two fingers (or swapping slots), not motion (~2 mm per frame at 300 mm/s): the step is
                                 * carried as an input offset, so the output neither jumps nor glides through it */
#endif
#ifndef BP_TOUCH_MOVING_SHORT_MM
#define BP_TOUCH_MOVING_SHORT_MM 0.6f /* v0.20.4: ... or over ~100..250 ms (a stroke that just started) */
#endif
/* One finished contact lifetime (admission to release), for tuning taps on the board. Positions in
 * core coordinates (0..BP_COORD_MAX over the sensor, before rotation). */
typedef struct {
    uint32_t end_ms;
    uint16_t dur_ms, frames;
    int16_t dx, dy;          /* last trusted position - admission position */
    uint16_t max_dev;        /* largest distance from the admission position (max of |dx|,|dy|) */
    uint16_t peak_density, start_density, weak_end; /* weak_end: 1 released by the gate, 0 chip let go */
} bp_touch_log_entry;
#define BP_TOUCH_LOG_N 32u
typedef struct { uint32_t seq, count; bp_touch_log_entry e[BP_TOUCH_LOG_N]; } bp_touch_log;
/* Movement-intent observer (ported from Codex's micro-intent candidate, 2026-10-24 22:20): a separate,
 * bounded evidence track decides "the finger is moving" for a young single contact, so a short flick leaves
 * the tap candidate instead of clicking. It never feeds coordinates to HID. */
#ifndef BP_TOUCH_MICRO_INTENT
#define BP_TOUCH_MICRO_INTENT 1
#endif
#define BP_TOUCH_INTENT_POINTS 64u   /* >= 400 ms at the chip's ~6.6 ms frames */
#ifndef BP_TOUCH_INTENT_MM
#define BP_TOUCH_INTENT_MM 0.45f      /* fitted travel that evidences a deliberate stroke */
#endif
typedef struct { uint64_t ms; float x, y, ox, oy; uint16_t strength, area; } bp_touch_intent_point;   /* ox/oy: output position then */
typedef struct {
    bp_touch_intent_point p[BP_TOUCH_INTENT_POINTS];
    unsigned n;
    uint64_t good_ms;
    float good_x, good_y;
    bool confirmed, active;
    bool pressure_safe; /* current strict fit, not a cached landing tail */
} bp_touch_intent;
/* Independent of held/filter output: a frozen centroid cannot prove that it is
 * moving. This bounded raw trajectory only vetoes lift-freezing; it never
 * requests a click, drag, or movement-intent catch-up. */
#define BP_LIFT_MOTION_POINTS 16u
typedef struct { uint64_t ms; int32_t x, y; uint16_t strength; } bp_lift_motion_point;
typedef struct { bp_lift_motion_point p[BP_LIFT_MOTION_POINTS]; unsigned n; } bp_lift_motion;
/* A separate history for the one raw contact spanning a weak landing. */
#define BP_LANDING_POINTS 12u
typedef struct { uint64_t ms; uint16_t x, y, strength, area; } bp_landing_point;
typedef struct {
    bp_landing_point p[BP_LANDING_POINTS];
    unsigned n, slot;
    uint64_t first_weak_ms;
    bool passed_entry, sustained_weak;
} bp_landing_motion;
typedef struct {
    uint64_t births[BP_IQS9151_MAX_FINGERS], next_birth;
    bool present[BP_IQS9151_MAX_FINGERS], admitted[BP_IQS9151_MAX_FINGERS];
    uint8_t weak_frames[BP_IQS9151_MAX_FINGERS];
    /* last position of the slot at or above the exit floor: a lifting finger's centroid drifts while
     * it weakens, so weak frames report this instead (taps stay inside the tap slop, lifts don't jump) */
    int32_t held_x[BP_IQS9151_MAX_FINGERS], held_y[BP_IQS9151_MAX_FINGERS];
    int32_t start_x[BP_IQS9151_MAX_FINGERS], start_y[BP_IQS9151_MAX_FINGERS];
    uint64_t start_ms[BP_IQS9151_MAX_FINGERS], last_ms[BP_IQS9151_MAX_FINGERS];
    uint16_t frames[BP_IQS9151_MAX_FINGERS], max_dev[BP_IQS9151_MAX_FINGERS];
    uint16_t peak_density[BP_IQS9151_MAX_FINGERS], start_density[BP_IQS9151_MAX_FINGERS];
    bp_touch_log *log;        /* optional, owned by the caller; survives bp_touch_filter_reset */
    const bp_touch_filter_params *fp; /* optional, owned by the caller (NULL = defaults); survives reset */
    uint16_t prev_strength[BP_IQS9151_MAX_FINGERS], peak_strength[BP_IQS9151_MAX_FINGERS], last_strength[BP_IQS9151_MAX_FINGERS];
    uint8_t settle[BP_IQS9151_MAX_FINGERS];
    bool euro_init[BP_IQS9151_MAX_FINGERS];
    float ex[BP_IQS9151_MAX_FINGERS], ey[BP_IQS9151_MAX_FINGERS];   /* One Euro output, mm */
    float edx[BP_IQS9151_MAX_FINGERS], edy[BP_IQS9151_MAX_FINGERS]; /* filtered speed, mm/s */
    float ax[BP_IQS9151_MAX_FINGERS], ay[BP_IQS9151_MAX_FINGERS];   /* string anchor, mm */
    uint64_t trusted_ms[BP_IQS9151_MAX_FINGERS], outside_ms[BP_IQS9151_MAX_FINGERS], moved_ms[BP_IQS9151_MAX_FINGERS];
    bool tracking[BP_IQS9151_MAX_FINGERS];
    bp_touch_intent intent[BP_IQS9151_MAX_FINGERS];
    /* Short relanding after proven movement still needs fresh coherent evidence;
     * it must not re-prove the full 0.45mm stroke. No context survives taps/pairs. */
    uint64_t recent_motion_ms;
    bool resume_hint[BP_IQS9151_MAX_FINGERS];
    bp_lift_motion lift_motion[BP_IQS9151_MAX_FINGERS];
    bp_landing_motion landing;
    uint8_t intent_streak[BP_IQS9151_MAX_FINGERS];
    uint16_t raw_x[BP_IQS9151_MAX_FINGERS], raw_y[BP_IQS9151_MAX_FINGERS];   /* last chip coordinates per slot */
    int16_t vel_x[BP_IQS9151_MAX_FINGERS], vel_y[BP_IQS9151_MAX_FINGERS];     /* last chip step per frame (v0.20: association predicts) */
    int16_t pvel_x[BP_IQS9151_MAX_FINGERS], pvel_y[BP_IQS9151_MAX_FINGERS];   /* the step before: prediction only when both agree */
    float hx[BP_IQS9151_MAX_FINGERS][5], hy[BP_IQS9151_MAX_FINGERS][5]; uint64_t hms[BP_IQS9151_MAX_FINGERS][5]; uint8_t hn[BP_IQS9151_MAX_FINGERS];   /* filtered position every ~100 ms */
    uint32_t slot_swaps, slot_handovers;
    uint8_t hwmap[BP_IQS9151_MAX_FINGERS];   /* chip slot -> lifetime slot for contacts not yet admitted */
    bool hwmap_set;
    uint8_t life_hw[BP_IQS9151_MAX_FINGERS];  /* chip slot an admitted lifetime was last matched to */
    uint8_t near_frames[BP_IQS9151_MAX_FINGERS];
    /* zig-zag suppressor (v0.20): last two raw positions (mm) and the time of the last detected alternation */
    float zx[BP_IQS9151_MAX_FINGERS][3], zy[BP_IQS9151_MAX_FINGERS][3]; uint8_t zn[BP_IQS9151_MAX_FINGERS]; uint64_t zz_ms[BP_IQS9151_MAX_FINGERS];
    uint32_t zigzags, gap_frames; bool gap_now;
    bool moving_now[BP_IQS9151_MAX_FINGERS];           /* condition(): the string is in its moving state */
    bool pair_seen[BP_IQS9151_MAX_FINGERS]; /* keep single-stroke recovery out of scrolling lifetimes */
    uint16_t recent_strength[BP_IQS9151_MAX_FINGERS];  /* EMA of the strength over trusted frames (~50 ms) */
    uint64_t lift_hold_ms[BP_IQS9151_MAX_FINGERS];     /* 0: not holding; else when the lift hold started */
    uint32_t lift_holds;
    uint16_t gap_x[BP_IQS9151_MAX_FINGERS], gap_y[BP_IQS9151_MAX_FINGERS]; bool gap_have[BP_IQS9151_MAX_FINGERS];   /* previous valid frame's raw centroid */
    uint64_t gap_defer_ms[BP_IQS9151_MAX_FINGERS];   /* time of the frame whose admission was deferred over a gap (0: none) */
    float sx[BP_IQS9151_MAX_FINGERS], sy[BP_IQS9151_MAX_FINGERS];   /* TRACK: where the position last settled */
    uint32_t session;
    uint32_t admitted_frames, suppressed_frames;
    /* v0.20.4 jump absorption (BP_TOUCH_JUMP_MM): input offset carried by each lifetime after a one-frame jump */
    float jox[BP_IQS9151_MAX_FINGERS], joy[BP_IQS9151_MAX_FINGERS];
    uint32_t jumps;
} bp_touch_filter_state;
void bp_touch_filter_reset(bp_touch_filter_state *);
/* Fills *out (always kind BP_DATA; fingers below the gate are left out, i.e. not touching).
 * x_res/y_res: chip resolution. geo NULL: uniform lines over the whole resolution. */
void bp_touch_convert(bp_touch_filter_state *, const bp_touch_params *, const bp_iqs9151_frame *,
                      uint16_t x_res, uint16_t y_res, const bp_touch_geometry *geo, uint32_t session,
                      uint32_t sequence, uint64_t time_ms, bp_sample *out);
/* One axis: chip coordinate -> core coordinate 0..BP_COORD_MAX over the settings-table lines. */
int32_t bp_touch_axis(uint16_t v, uint16_t res, uint8_t n, uint8_t orig, const uint8_t *phys);
#endif

