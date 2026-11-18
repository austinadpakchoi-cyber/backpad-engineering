#include "bp_touch_policy.h"
#include <math.h>
#include <string.h>

static const bp_touch_filter_params default_filter = BP_TOUCH_FILTER_DEFAULT;
#define MM_X ((float)BP_PHYSICAL_X_UM / 1000.0f / (float)BP_COORD_MAX)
#define MM_Y ((float)BP_PHYSICAL_Y_UM / 1000.0f / (float)BP_COORD_MAX)

void bp_touch_filter_reset(bp_touch_filter_state *state) {
    uint64_t next = state->next_birth;
    bp_touch_log *log = state->log;
    const bp_touch_filter_params *fp = state->fp;
    memset(state, 0, sizeof(*state));
    state->next_birth = next; /* lifetimes never repeat across resets */
    state->log = log;
    state->fp = fp;
}

static void log_release(bp_touch_filter_state *state, unsigned k, bool weak) {
    if (!state->intent[k].confirmed || state->pair_seen[k]) state->recent_motion_ms = 0;
    state->resume_hint[k] = false;
    bp_touch_log *log = state->log;
    if (!log) return;
    bp_touch_log_entry *e = &log->e[log->count % BP_TOUCH_LOG_N];
    uint64_t dur = state->last_ms[k] - state->start_ms[k];
    e->end_ms = (uint32_t)state->last_ms[k];
    e->dur_ms = (uint16_t)(dur > 0xFFFFu ? 0xFFFFu : dur);
    e->frames = state->frames[k];
    e->dx = (int16_t)(state->held_x[k] - state->start_x[k]);
    e->dy = (int16_t)(state->held_y[k] - state->start_y[k]);
    e->max_dev = state->max_dev[k];
    e->peak_density = state->peak_density[k];
    e->start_density = state->start_density[k];
    e->weak_end = weak ? 1u : 0u;
    ++log->count;
    ++log->seq;
}

static uint16_t scaled(uint16_t strength, uint16_t q8) {
    uint32_t v = ((uint32_t)strength * q8 + 128u) / 256u;
    return (uint16_t)(v > 0xFFFFu ? 0xFFFFu : (v ? v : 1u)); /* 0 means "unavailable" to the core */
}

int32_t bp_touch_axis(uint16_t v, uint16_t res, uint8_t n, uint8_t orig, const uint8_t *phys) {
    if (!res) return 0;
    if (!phys || n < 2u || orig < 2u) return (int32_t)(((uint32_t)v * BP_COORD_MAX + res / 2u) / res);
    /* position over the lines in use, in 1/256 pitch, then the same fraction between their table lines */
    uint32_t q = (uint32_t)v * (n - 1u) * 256u / res, i = q / 256u, frac = q % 256u;
    if (i >= n - 1u) { i = n - 2u; frac = 256u; }
    uint32_t p = (uint32_t)phys[i] * 256u + (uint32_t)(phys[i + 1u] - phys[i]) * frac;
    uint32_t span = (orig - 1u) * 256u;
    return (int32_t)((p * BP_COORD_MAX + span / 2u) / span);
}

static float euro_alpha(float fc, float dt) {
    float tau = 1.0f / (6.2831853f * fc);
    return 1.0f / (1.0f + tau / dt);
}

/* Association reference for lifetime L: the last raw position plus one step of velocity, but only while the last
 * two steps agree in direction (consistent motion). A big flat blob's centroid jitters 5..12 mm per frame on the
 * circle captures; predicting from that jitter threw the true blob outside the match radius (v0.20 regression). */
static int32_t assoc_cost(const bp_touch_filter_state *state, unsigned L, const bp_iqs9151_finger *g, int32_t *d2_out) {
    int32_t dx = (int32_t)g->x - state->raw_x[L], dy = (int32_t)g->y - state->raw_y[L], d2 = dx * dx + dy * dy;
    *d2_out = d2;
    int32_t dot = (int32_t)state->vel_x[L] * state->pvel_x[L] + (int32_t)state->vel_y[L] * state->pvel_y[L];
    bool consistent = BP_TOUCH_PREDICT && dot > 0;
    /* v0.20: a predicted-position match shrank the effective radius and lost a jittery big blob (circle replays);
     * so the distance stays from the last position and only the CHOICE is biased: behind consistent motion costs
     * extra (the trailing half of a split fast blob, the "rubber band") */
    if (consistent && (int32_t)dx * state->vel_x[L] + (int32_t)dy * state->vel_y[L] < 0) d2 += 4 * BP_TOUCH_HANDOVER_COUNTS * BP_TOUCH_HANDOVER_COUNTS;
    return d2;
}
/* a contact may carry an admitted lifetime only if it satisfies the hold gate (Codex gap-932 audit: an area-1 ghost
 * next to the finger took the finger's identity, which ended the lifetime and left a 90 ms output gap) */
static bool holdable(const bp_iqs9151_finger *g, const bp_touch_params *params) {
    if (!BP_TOUCH_HOLDABLE_MATCH) return g->valid;
    /* plausibility only, NOT the hold gate: a contact in a weak frame (below the gate for < 5 frames) must keep
     * carrying its lifetime so the position keeps tracking (requiring the gate here lost 40 % of a circle
     * replay: the blob re-passed the gate > 8 mm from the last matched position). An area-1 sliver cannot. */
    return g->valid && g->area >= 2u && g->strength >= params->exit_strength &&
           g->strength / g->area >= params->exit_density / 2u;
}

static bool hold_quality(const bp_iqs9151_finger *g, const bp_touch_params *p) {
    unsigned density = g->area ? g->strength / g->area : 0u;
    return g->valid && ((g->strength >= p->exit_strength && g->area >= p->exit_area && density >= p->exit_density) ||
           (g->area >= BP_TOUCH_BIG_AREA && density >= BP_TOUCH_BIG_EXIT_DENSITY));
}

/* Is chip coordinate v (0..res over n lines in use) within BP_TOUCH_GAP_ZONE_Q8 of a removed INTERIOR line? */
static bool in_gap_axis(uint16_t v, uint16_t res, uint8_t n, const uint8_t *phys) {
    if (!phys || n < 2u || !res) return false;
    uint32_t q = (uint32_t)v * (n - 1u) * 256u / res;   /* position in 1/256 of the lines in use */
    for (unsigned i = 0; i + 1u < n; i++) {
        if (phys[i + 1u] - phys[i] <= 1u) continue;     /* no gap between these two lines */
        uint32_t centre = i * 256u + 128u;             /* midway between lines i and i+1 */
        uint32_t d = q > centre ? q - centre : centre - q;
        if (d <= BP_TOUCH_GAP_ZONE_Q8) return true;
    }
    return false;
}

static void intent_clear(bp_touch_intent *m, bool all) {
    m->n = 0; m->good_ms = 0; m->active = false;
    if (all) m->confirmed = false;
}

/* Only trusted points of a single admitted contact accumulate movement evidence (Codex
 * micro-intent): landing requires the original BP_TOUCH_INTENT_MM fit over <=400ms.
 * Confirmed strokes use <=120ms and BP_TOUCH_FINE_INTENT_MM; >=85% explained variance,
 * strength within2:1, first confirmation >=80ms after admission; evidence kept <=30ms after the last good
 * fit unless the position moves > 0.2 mm from it. Gaps > 15 ms reset the window. */
static void intent_observe(bp_touch_filter_state *state, unsigned k, int32_t x, int32_t y,
                           uint16_t strength, uint16_t area, uint64_t ms, bool fresh) {
    bp_touch_intent *m = &state->intent[k];
    if (fresh) memset(m, 0, sizeof(*m));
    m->active = false; m->pressure_safe = false;
    if (!BP_TOUCH_MICRO_INTENT || (ms - state->start_ms[k] < 20u && !state->resume_hint[k])) return;
    if (m->n && (ms <= m->p[m->n - 1u].ms || ms - m->p[m->n - 1u].ms > 15u)) { m->n = 0; m->good_ms = 0; }
    /* Landing still needs the original long-stroke evidence. Once this single
     * lifetime has proved movement, a short correction must not prove another
     * 0.45 mm stroke or compete with 400 ms of the preceding rest/reversal. */
    bool resuming = !m->confirmed && state->resume_hint[k] && ms - state->start_ms[k] <= 250u;
    /* Keep the long window for a genuinely slow new stroke. A recent fast
     * coherent segment should not compete with the landing centroid trail.
     * Use its shorter window only after it already spans0.6mm; otherwise
     * shortening every window would erase slow1mm swipes. */
    unsigned window_ms = m->confirmed ? 120u : 400u;
    if (!m->confirmed && !resuming && ms - state->start_ms[k] <= 250u && m->n) {
        unsigned first = 0;
        while (first + 1u < m->n && ms - m->p[first].ms > 60u) ++first;
        float dx = x * MM_X - m->p[first].x, dy = y * MM_Y - m->p[first].y;
        if (dx * dx + dy * dy >= 0.6f * 0.6f) window_ms = 60u;
    }
    /* A shorter intent window can expire several points in the same frame. */
    unsigned expired = 0;
    while (expired < m->n && (ms - m->p[expired].ms > window_ms ||
                               m->n - expired >= BP_TOUCH_INTENT_POINTS)) {
        ++expired;
    }
    if (expired) {
        m->n -= expired;
        memmove(m->p, m->p + expired, m->n * sizeof(m->p[0]));
    }
    /* Recent movement is only a hint: a growing landing footprint is not a
     * deliberate short swipe. Require40ms of a settled area before using the
     * smaller continuation threshold; ordinary fresh and proved motion are unchanged. */
    if (resuming && m->n && area != m->p[m->n - 1u].area) { m->n = 0; m->good_ms = 0; }
    m->p[m->n++] = (bp_touch_intent_point){ms, (float)x * MM_X, (float)y * MM_Y, state->ax[k], state->ay[k], strength, area};
    if (m->n < 5u || ms - m->p[0].ms < 40u) return;
    float mt = 0.0f, mx = 0.0f, my = 0.0f;
    uint16_t lo = strength, hi = strength;
    for (unsigned i = 0; i < m->n; i++) {
        mt += (float)(m->p[i].ms - m->p[0].ms); mx += m->p[i].x; my += m->p[i].y;
        if (m->p[i].strength < lo) lo = m->p[i].strength;
        if (m->p[i].strength > hi) hi = m->p[i].strength;
    }
    mt /= (float)m->n; mx /= (float)m->n; my /= (float)m->n;
    float tt = 0.0f, tx = 0.0f, ty = 0.0f, total = 0.0f;
    for (unsigned i = 0; i < m->n; i++) {
        float t = (float)(m->p[i].ms - m->p[0].ms) - mt, dx = m->p[i].x - mx, dy = m->p[i].y - my;
        tt += t * t; tx += t * dx; ty += t * dy; total += dx * dx + dy * dy;
    }
    /* v0.18: pressure and area may change during a slide (Codex v0.17 review: a +4 % pressure step or area
     * 8 -> 10 at constant density turned a 1 mm slide into a click). Neither vetoes motion any more; a
     * landing tail is recognised by its decelerating path (below). The strength only has to stay within a
     * 2:1 band (a landing doubles it). */
    (void)area;
    if ((uint32_t)lo * 100u < (uint32_t)hi * 50u) { m->good_ms = 0; return; }
    if (!m->confirmed) {
        unsigned h = m->n / 2u;
        float ax1 = m->p[h].x - m->p[0].x, ay1 = m->p[h].y - m->p[0].y;
        float ax2 = m->p[m->n - 1u].x - m->p[h].x, ay2 = m->p[m->n - 1u].y - m->p[h].y;
        if (ax2 * ax2 + ay2 * ay2 < (resuming ? 0.64f : 0.25f) * (ax1 * ax1 + ay1 * ay1)) { m->good_ms = 0; return; }
    }
    bool fit = false;
    if (tt > 0.0f && total > 0.0f) {
        float bx = tx / tt, by = ty / tt, span = (float)(ms - m->p[0].ms);
        float explained = (tx * tx + ty * ty) / tt;
        /* v0.15: >= 0.6 mm fitted travel (Codex's 0.35 mm let 0.35..0.43 mm landing tails through); the
         * strength-growth and deceleration checks above reject tails that creep further. Losing a tap is
         * worse for the user than a lost sub-millimetre flick, so the bar favours the click. */
        float travel2 = (bx * bx + by * by) * span * span;
        float travel_mm = m->confirmed ? BP_TOUCH_FINE_INTENT_MM : (resuming ? 0.12f : BP_TOUCH_INTENT_MM);
        /* At a0.1mm correction, small quantisation noise can dominate the
         * variance ratio even for a straight stroke. For a previously proved
         * single lifetime only, also accept a bounded35um RMS fit residual
         * (including coordinate quantisation).
         * The travel floor still rejects quiet noise; new taps keep85%. */
        bool coherent = explained >= 0.85f * total ||
                        (m->confirmed && total - explained <= (float)m->n * 0.035f * 0.035f);
        fit = travel2 >= travel_mm * travel_mm && coherent;
        /* A frozen new contact must distinguish a coherent light slide from
         * a press/release centroid tail. This is a fresh fit over >=40ms,
         * >=95% explained variance and <=20% strength range; an old cached
         * fit cannot acquire this permission after pressure has fallen. */
        m->pressure_safe = fit && explained >= 0.95f * total &&
                           (uint32_t)lo * 100u >= (uint32_t)hi * 80u;
        if (fit) { m->good_ms = ms; m->good_x = m->p[m->n - 1u].x; m->good_y = m->p[m->n - 1u].y; }
    }
    if (!fit && m->good_ms) {
        float dx = m->p[m->n - 1u].x - m->good_x, dy = m->p[m->n - 1u].y - m->good_y;
        if (dx * dx + dy * dy > 0.20f * 0.20f) m->good_ms = 0;
    }
    if (resuming && !m->confirmed && ms - state->start_ms[k] < 80u) {
        /* A short continuation may start at40ms, but a falling-pressure
         * centroid tail is still a tap/lift. Do not cache early evidence
         * across this veto and spend it at the80ms boundary. */
        if ((uint32_t)strength * 100u < (uint32_t)hi * 95u) { m->good_ms = 0; return; }
    }
    if (m->good_ms && ms - m->good_ms <= 30u && ms - state->start_ms[k] >= (resuming ? 40u : 80u)) { m->confirmed = true; m->active = true; }
}

/* A coherent raw stroke may change pressure without being a lift. The normal
 * strength/area admission still applies. Gaps, weak points, jumps and the
 * decelerating tail of a tap cannot build this evidence. */
static bool lift_motion_observe(bp_touch_filter_state *state, unsigned k,
                                int32_t x, int32_t y, uint16_t strength,
                                uint64_t ms, bool fresh, bool eligible) {
    bp_lift_motion *m = &state->lift_motion[k];
    if (fresh || !eligible) m->n = 0;
    if (!eligible) return false;
    if (m->n) {
        bp_lift_motion_point *last = &m->p[m->n - 1u];
        float dx = (x - last->x) * MM_X, dy = (y - last->y) * MM_Y;
        if (ms <= last->ms || ms - last->ms > 15u || dx * dx + dy * dy > 1.5f * 1.5f) m->n = 0;
    }
    if (m->n == BP_LIFT_MOTION_POINTS) {
        --m->n; memmove(m->p, m->p + 1, m->n * sizeof(m->p[0]));
    }
    m->p[m->n++] = (bp_lift_motion_point){ms, x, y, strength};
    if (m->n < 5u || ms - m->p[0].ms < 35u) return false;
    uint16_t lo = strength, hi = strength;
    float path = 0.0f;
    for (unsigned i = 0; i < m->n; ++i) {
        if (m->p[i].strength < lo) lo = m->p[i].strength;
        if (m->p[i].strength > hi) hi = m->p[i].strength;
        if (i) {
            float dx = (m->p[i].x - m->p[i-1u].x) * MM_X;
            float dy = (m->p[i].y - m->p[i-1u].y) * MM_Y;
            path += sqrtf(dx * dx + dy * dy);
        }
    }
    /* Match the existing motion-intent 2:1 pressure band: a sliding finger's
     * pressure can dip considerably under a thick cover. */
    if ((uint32_t)lo * 100u < (uint32_t)hi * 50u) return false;
    /* Large falling-pressure centroid tails belong to release protection.
     * A stroke must still carry at least80% of this window's peak pressure. */
    if ((uint32_t)strength * 100u < (uint32_t)hi * 80u) return false;
    unsigned h = m->n / 2u;
    float dx = (x - m->p[0].x) * MM_X, dy = (y - m->p[0].y) * MM_Y;
    float ax = (m->p[h].x - m->p[0].x) * MM_X, ay = (m->p[h].y - m->p[0].y) * MM_Y;
    float bx = (x - m->p[h].x) * MM_X, by = (y - m->p[h].y) * MM_Y;
    float net2 = dx * dx + dy * dy;
    return net2 >= 2.5f * 2.5f && net2 >= 0.81f * path * path &&
           bx * bx + by * by >= 0.25f * (ax * ax + ay * ay);
}

/* The normal gates and settling delay still decide contact ownership. An
 * isolated raw contact may prove a swipe across that delay if its motion
 * continues in the same direction. A declining signal alone cannot prove it. */
static bool landing_motion_observe(bp_landing_motion *m, unsigned slot, bp_landing_point point,
                                   bool new_contact, bool passes_gate, float *dx_out, float *dy_out) {
    if (point.area < 2u) { m->n = 0; return false; }
    if (m->n) {
        const bp_landing_point *last = &m->p[m->n - 1u];
        float dx = ((int32_t)point.x - last->x) * MM_X;
        float dy = ((int32_t)point.y - last->y) * MM_Y;
        if (slot != m->slot || point.ms <= last->ms || point.ms - last->ms > 15u ||
            dx * dx + dy * dy > 1.8f * 1.8f) m->n = 0;
    }
    if (!m->n) {
        m->slot = slot;
        m->first_weak_ms = point.ms;
        m->passed_entry = passes_gate || !new_contact;
        m->sustained_weak = false;
    }
    if (!m->passed_entry && passes_gate) {
        m->passed_entry = true;
        m->sustained_weak = point.ms - m->first_weak_ms >= 22u;
    }
    if (m->n == BP_LANDING_POINTS) {
        --m->n;
        memmove(m->p, m->p + 1, m->n * sizeof(m->p[0]));
    }
    m->p[m->n++] = point;
    if (!passes_gate || !m->sustained_weak || m->n < 6u) return false;
    for (unsigned first = 0; first + 5u < m->n; ++first) {
        const bp_landing_point *a = &m->p[first];
        if (point.ms - a->ms < 30u || point.ms - a->ms > 60u) continue;
        float dx = ((int32_t)point.x - a->x) * MM_X;
        float dy = ((int32_t)point.y - a->y) * MM_Y;
        float net2 = dx * dx + dy * dy;
        if (net2 < 2.35f * 2.35f) continue;
        const bp_landing_point *mid = &m->p[first + (m->n - 1u - first) / 2u];
        float fa = ((int32_t)mid->x - a->x) * MM_X * dx + ((int32_t)mid->y - a->y) * MM_Y * dy;
        if (fa < net2 * 0.20f || fa > net2 * 0.69f) continue;
        float backtrack = 0.0f, path = 0.0f;
        uint16_t peak = a->strength;
        bool same_area = true;
        for (unsigned i = first + 1u; i < m->n; ++i) {
            const bp_landing_point *p = &m->p[i], *q = &m->p[i-1u];
            float sx = ((int32_t)p->x - q->x) * MM_X, sy = ((int32_t)p->y - q->y) * MM_Y;
            float advance = sx * dx + sy * dy;
            if (advance < 0.0f) backtrack -= advance;
            path += sqrtf(sx * sx + sy * sy);
            same_area &= p->area == a->area;
            if (p->strength > peak) peak = p->strength;
        }
        bool landing = a->area < point.area && backtrack <= net2 * 0.12f &&
                       point.strength * 100u >= peak * 85u;
        bool moving_light = same_area && net2 >= 0.90f * path * path &&
                            point.strength * 100u >= peak * 45u;
        if (landing || moving_light) { *dx_out = dx; *dy_out = dy; return true; }
    }
    return false;
}

/* One Euro filter step on one axis (mm, mm/s). */
static float euro(float *x, float *dx, float raw, float dt, const bp_touch_filter_params *fp) {
    float v = (raw - *x) / dt;
    *dx += euro_alpha((float)fp->dcut_chz / 100.0f, dt) * (v - *dx);
    float fc = (float)fp->fmin_chz / 100.0f + (float)fp->beta_milli / 1000.0f * fabsf(*dx);
    *x += euro_alpha(fc, dt) * (raw - *x);
    return *x;
}

/* One Euro on both axes with ONE cutoff from the vector speed (v0.20). Per-axis cutoffs (the previous euro() per
 * axis) filtered the slow axis of a diagonal stroke much harder than the fast one, so the output first ran along
 * the fast axis and the slow axis caught up later: "a diagonal stroke suddenly turns horizontal" (user, 01:50). */
static void euro2(bp_touch_filter_state *state, unsigned k, float *xm, float *ym, float dt, const bp_touch_filter_params *fp) {
    float vx = (*xm - state->ex[k]) / dt, vy = (*ym - state->ey[k]) / dt;
    float ad = euro_alpha((float)fp->dcut_chz / 100.0f, dt);
    state->edx[k] += ad * (vx - state->edx[k]); state->edy[k] += ad * (vy - state->edy[k]);
    float speed = sqrtf(state->edx[k] * state->edx[k] + state->edy[k] * state->edy[k]);
    float fc = (float)fp->fmin_chz / 100.0f + (float)fp->beta_milli / 1000.0f * speed;
    float a = euro_alpha(fc, dt);
    state->ex[k] += a * (*xm - state->ex[k]); state->ey[k] += a * (*ym - state->ey[k]);
    *xm = state->ex[k]; *ym = state->ey[k];
}

static void condition(bp_touch_filter_state *state, unsigned k, int32_t x, int32_t y, bool fresh, uint64_t time_ms,
                      const bp_touch_filter_params *fp) {
    float xm = (float)x * MM_X, ym = (float)y * MM_Y;
    if (fresh || !state->euro_init[k]) {
        state->ex[k] = state->ax[k] = xm; state->ey[k] = state->ay[k] = ym;
        state->edx[k] = state->edy[k] = 0.0f; state->euro_init[k] = true; state->outside_ms[k] = 0;
        state->moved_ms[k] = time_ms; state->hn[k] = 0; state->tracking[k] = false;
    } else {
        float dt = (float)(time_ms - state->trusted_ms[k]) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        if (fp->flags & BP_TF_EURO) {
            bp_touch_filter_params zone = *fp;
            /* A proved fine correction needs a short filter tail to reverse
             * within a small target. Gain is unchanged. New taps, pairs and
             * quiet input without recent coherent motion retain their filter. */
            if (state->intent[k].confirmed && state->intent[k].good_ms &&
                time_ms - state->intent[k].good_ms <= 150u &&
                zone.fmin_chz < 1000u) zone.fmin_chz = 1000u;
            if (state->gap_now) zone.beta_milli = (uint16_t)((uint32_t)fp->beta_milli * BP_TOUCH_GAP_BETA_Q8 / 256u);
            if (BP_TOUCH_EURO_SHARED) euro2(state, k, &xm, &ym, dt, &zone);
            else { xm = euro(&state->ex[k], &state->edx[k], xm, dt, &zone); ym = euro(&state->ey[k], &state->edy[k], ym, dt, &zone); }
        }
        if ((fp->flags & BP_TF_STRING) && !(fp->flags & BP_TF_HOLDTRACK)) {
            /* Continuous trailing string (v0.14): the output follows once the position is string_um away
             * (after dwell_ms if set), trailing by string_um; evidenced movement follows without the string.
             * v0.17: a shorter string while the finger is moving (string moved within BP_TOUCH_REST_MS). */
            /* "moving" = the filtered position travelled >= 0.4 mm net over the last ~400 ms (samples every
             * 100 ms); zero-mean noise never does, a creep at 1 mm/s does. The string then shrinks to the
             * moving length and the output closes the gap at once. */
            unsigned hn = state->hn[k];
            if (!hn || time_ms - state->hms[k][(hn - 1u) % 5u] >= 100u) {
                state->hx[k][hn % 5u] = xm; state->hy[k][hn % 5u] = ym; state->hms[k][hn % 5u] = time_ms; state->hn[k] = (uint8_t)(hn + 1u); hn++;
            }
            bool moving = state->intent[k].active;
            for (unsigned q = 1; q < 5u && q < hn && !moving; q++) {
                unsigned idx = (hn - 1u - q) % 5u; uint64_t age = time_ms - state->hms[k][idx];
                float nx_ = xm - state->hx[k][idx], ny_ = ym - state->hy[k][idx], n2 = nx_ * nx_ + ny_ * ny_;
                if (age >= 350u && age <= 550u) moving = n2 >= BP_TOUCH_MOVING_NET_MM * BP_TOUCH_MOVING_NET_MM;
                /* v0.20.4: a stroke that just started is moving too (the 350..550 ms sample still shows the rest
                 * point for the first 0.35 s; meanwhile a strength dip froze the finger as a "lift tail") */
                else if (age >= 90u && age <= 260u) moving = n2 >= BP_TOUCH_MOVING_SHORT_MM * BP_TOUCH_MOVING_SHORT_MM;
            }
            state->moving_now[k] = moving;
            /* moving string = stop_um in this mode (tunable from the page; 0 -> BP_TOUCH_MOVING_STRING_UM). Trade-off
             * on the held-finger replay: 400 um -> back-and-forth 54, 1 mm correction 24/-8; 120 um -> 166, 35/-30 */
            unsigned moving_um = fp->stop_um ? fp->stop_um : BP_TOUCH_MOVING_STRING_UM;
            /* Only coherent corrections of an already moving SINGLE lifetime
             * get the short string. Stationary noise, new taps, and pairs keep
             * their existing radius; do not globally lower the quiet gate. */
            /* Settle this filter's tail without extending movement intent or
             * moving_now: lift detection must retain its original time bound. */
            bool fine = state->intent[k].confirmed && state->intent[k].good_ms &&
                        time_ms - state->intent[k].good_ms <= 150u;
            unsigned radius_um = moving ? moving_um : fp->string_um;
            if (fine && radius_um > BP_TOUCH_FINE_STRING_UM) radius_um = BP_TOUCH_FINE_STRING_UM;
            float r = (float)radius_um / 1000.0f;
            float dxm = xm - state->ax[k], dym = ym - state->ay[k], d = sqrtf(dxm * dxm + dym * dym);
            if (d > r) {
                if (!state->outside_ms[k]) state->outside_ms[k] = time_ms ? time_ms : 1u;
                if (r == 0.0f || d > 2.0f * r || time_ms - state->outside_ms[k] >= fp->dwell_ms) {
                    state->ax[k] += dxm * (1.0f - r / d); state->ay[k] += dym * (1.0f - r / d);
                    state->moved_ms[k] = time_ms;
                }
            } else {
                state->outside_ms[k] = 0;
            }
        } else if (fp->flags & BP_TF_STRING) {
            float r = (float)fp->string_um / 1000.0f;
            if (!state->tracking[k] && state->intent[k].active) {   /* evidenced movement: follow now */
                state->tracking[k] = true; state->outside_ms[k] = 0;
                state->sx[k] = xm; state->sy[k] = ym; state->moved_ms[k] = time_ms;
            }
            if (!state->tracking[k]) {          /* HOLD */
                float dxm = xm - state->ax[k], dym = ym - state->ay[k], d2 = dxm * dxm + dym * dym;
                if (d2 > r * r) {
                    if (!state->outside_ms[k]) state->outside_ms[k] = time_ms ? time_ms : 1u;
                    if (d2 > 4.0f * r * r || time_ms - state->outside_ms[k] >= fp->dwell_ms) {
                        state->tracking[k] = true; state->outside_ms[k] = 0;
                        state->sx[k] = xm; state->sy[k] = ym; state->moved_ms[k] = time_ms;
                    }
                } else {
                    state->outside_ms[k] = 0;
                }
            }
            if (state->tracking[k]) {           /* TRACK */
                float tx = xm - state->ax[k], ty = ym - state->ay[k], td = sqrtf(tx * tx + ty * ty);
                float tr = (float)BP_TOUCH_TRACK_STRING_UM / 1000.0f;
                if (td > tr) { state->ax[k] += tx * (1.0f - tr / td); state->ay[k] += ty * (1.0f - tr / td); }
                float ex = xm - state->sx[k], ey = ym - state->sy[k];
                float s_ = (float)(fp->stop_um ? fp->stop_um : BP_TOUCH_STOP_UM) / 1000.0f;
                if (ex * ex + ey * ey > s_ * s_) { state->sx[k] = xm; state->sy[k] = ym; state->moved_ms[k] = time_ms; }
                else if (time_ms - state->moved_ms[k] >= BP_TOUCH_STOP_MS) {
                    state->tracking[k] = false; state->ax[k] = xm; state->ay[k] = ym;   /* stop where it stopped */
                }
            }
        } else {
            state->ax[k] = xm; state->ay[k] = ym;
        }
    }
    state->trusted_ms[k] = time_ms;
    state->held_x[k] = (int32_t)lroundf(state->ax[k] / MM_X);
    state->held_y[k] = (int32_t)lroundf(state->ay[k] / MM_Y);
}

static void emit(bp_touch_filter_state *state, const bp_touch_params *params, unsigned k, uint16_t strength,
                 bp_sample *out) {
    if (out->count >= BP_TOUCH_MAX_CONTACTS) return;
    unsigned i = out->count++;
    out->contacts[i].id = (state->births[k] << 3) | (k + 1u);
    out->contacts[i].x = state->held_x[k];
    out->contacts[i].y = state->held_y[k];
    out->strengths[i] = scaled(strength, params->strength_scale_q8);
    /* Schmitt behaviour: once admitted at the enter floor, a contact moves normally down to the
     * exit floor (the core stops motion for weak contacts, which would freeze the cursor during
     * a light stroke). Only a never-admitted contact could be weak, and admission requires the
     * enter floor, so weak_contacts stays clear; the floor itself is the hover defence. */
}

void bp_touch_convert(bp_touch_filter_state *state, const bp_touch_params *params, const bp_iqs9151_frame *f,
                      uint16_t x_res, uint16_t y_res, const bp_touch_geometry *geo, uint32_t session,
                      uint32_t sequence, uint64_t time_ms, bp_sample *out) {
    memset(out, 0, sizeof(*out));
    out->kind = BP_DATA;
    out->session = session;
    out->sequence = sequence;
    out->time_ms = time_ms;
    if (state->session != session) {
        bp_touch_filter_reset(state);
        state->session = session;
    }
    const bp_touch_filter_params *fp = state->fp ? state->fp : &default_filter;
    /* Contact association (v0.19). Chip slot numbers are not trusted: the chip hands one finger between its slots
     * (board 1 circles at 00:27: every ~100 ms, ending the lifetime each time, so the core saw two contacts, lost
     * its primary and cancelled into wait-for-lift) and swaps two fingers' slots (Codex, two-finger hold frames
     * 191->193). Every frame each admitted lifetime takes the reported contact nearest its last position within
     * BP_TOUCH_HANDOVER_COUNTS (best assignment over the 3! slot orders: most matches, then least distance); an
     * unmatched lifetime gets a dropout frame (position held); unmatched contacts go to free lifetime slots,
     * keeping the slot they had last frame where possible. */
    bp_iqs9151_frame swapped;
    {
        if (!state->hwmap_set) { for (unsigned h = 0; h < BP_IQS9151_MAX_FINGERS; h++) state->hwmap[h] = state->life_hw[h] = (uint8_t)h; state->hwmap_set = true; }
        static const uint8_t perms[6][3] = {{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
        const int32_t R2 = BP_TOUCH_HANDOVER_COUNTS * BP_TOUCH_HANDOVER_COUNTS;
        unsigned A[BP_IQS9151_MAX_FINGERS], na = 0;
        for (unsigned k = 0; k < BP_IQS9151_MAX_FINGERS; k++) if (state->admitted[k]) A[na++] = k;
        /* pick[i] = chip slot for admitted lifetime A[i] (0xFF: none). The exhaustive 3! search below only covers
         * three lifetimes over chip slots 0..2; with more lifetimes (dropout-held ones plus new contacts) or a valid
         * contact in slot >= 3 (max touches > 3) it indexed perms[][3] out of bounds (independent review F1, UBSan
         * "index 3 out of bounds"). Those frames use a greedy assignment with the same priority instead. */
        uint8_t pick[BP_IQS9151_MAX_FINGERS];
        for (unsigned i = 0; i < BP_IQS9151_MAX_FINGERS; i++) pick[i] = 0xFFu;
        bool small = na <= 3u;
        for (unsigned h = 3; h < BP_IQS9151_MAX_FINGERS; h++) if (f->finger[h].valid) small = false;
        int best_good = -1, best_n = -1; int64_t best_cost = 0; unsigned best = 0;
        for (unsigned q = 0; small && q < 6u; q++) {
            int good = 0, n = 0; int64_t cost = 0; bool keeps = true;
            for (unsigned i = 0; i < na; i++) {
                const bp_iqs9151_finger *g = &f->finger[perms[q][i]];
                if (perms[q][i] != state->life_hw[A[i]]) keeps = false;
                if (!holdable(g, params)) continue;
                /* v0.20: match against the velocity-predicted position. Matching the LAST position pulled a fast
                 * finger onto the trailing half when the chip split its blob (the last position lags the finger),
                 * i.e. a backward jump, then forward again when it merged: the "rubber band" (user, 01:50). */
                int32_t d2; int32_t c_ = assoc_cost(state, A[i], g, &d2);
                if (d2 <= R2) { n++; good += hold_quality(g, params); cost += c_; }
            }
            if (keeps) cost -= 1;   /* ties keep the previous slots */
            /* Complete hold quality wins; weak-only frames retain the original fallback. */
            if (good > best_good || (good == best_good && (n > best_n || (n == best_n && cost < best_cost)))) {
                best_good = good; best_n = n; best_cost = cost; best = q;
            }
        }
        if (small) {
            for (unsigned i = 0; i < na; i++) pick[i] = perms[best][i];
        } else {   /* greedy: repeatedly take the best remaining (hold quality first, then cost) lifetime/slot pair */
            bool li[BP_IQS9151_MAX_FINGERS] = {false}, hi[BP_IQS9151_MAX_FINGERS] = {false};
            for (;;) {
                int bi = -1, bh = -1, bg = -1; int32_t bc = 0;
                for (unsigned i = 0; i < na; i++) {
                    if (li[i]) continue;
                    for (unsigned h = 0; h < BP_IQS9151_MAX_FINGERS; h++) {
                        const bp_iqs9151_finger *g = &f->finger[h];
                        if (hi[h] || !holdable(g, params)) continue;
                        int32_t d2; int32_t c_ = assoc_cost(state, A[i], g, &d2);
                        if (d2 > R2) continue;
                        if (h == state->life_hw[A[i]]) c_ -= 1;
                        int gq = hold_quality(g, params);
                        if (gq > bg || (gq == bg && c_ < bc)) { bi = (int)i; bh = (int)h; bg = gq; bc = c_; }
                    }
                }
                if (bi < 0) break;
                li[bi] = true; hi[bh] = true; pick[bi] = (uint8_t)bh;
            }
        }
        bool used_h[BP_IQS9151_MAX_FINGERS] = {false}, used_l[BP_IQS9151_MAX_FINGERS] = {false};
        swapped = *f;
        for (unsigned k = 0; k < BP_IQS9151_MAX_FINGERS; k++) swapped.finger[k].valid = false;
        for (unsigned i = 0; i < na; i++) {
            unsigned L = A[i];
            used_l[L] = true;   /* matched or not, an admitted slot never takes another contact */
            if (pick[i] == 0xFFu) continue;
            unsigned h = pick[i];
            const bp_iqs9151_finger *g = &f->finger[h];
            if (!holdable(g, params)) continue;
            int32_t d2; (void)assoc_cost(state, L, g, &d2);
            if (d2 > R2) continue;
            swapped.finger[L] = *g; used_h[h] = true;
            if (state->life_hw[L] != h) { ++state->slot_swaps; state->life_hw[L] = (uint8_t)h; }
        }
        for (unsigned pass_ = 0; pass_ < 2u; pass_++)
            for (unsigned h = 0; h < BP_IQS9151_MAX_FINGERS; h++) {
                if (used_h[h] || !f->finger[h].valid) continue;
                unsigned L = state->hwmap[h];
                if (pass_ == 1u || used_l[L]) {   /* first pass: keep last frame's slot; second: any free slot */
                    if (pass_ == 0u) continue;
                    L = BP_IQS9151_MAX_FINGERS;
                    for (unsigned k = 0; k < BP_IQS9151_MAX_FINGERS; k++) if (!used_l[k]) { L = k; break; }
                    if (L == BP_IQS9151_MAX_FINGERS) continue;   /* more contacts than slots: dropped */
                }
                swapped.finger[L] = f->finger[h]; used_h[h] = used_l[L] = true;
                state->hwmap[h] = (uint8_t)L; state->life_hw[L] = (uint8_t)h;
            }
        f = &swapped;
    }
    bool raw_proven[BP_IQS9151_MAX_FINGERS] = {false};
    bool landing_proven[BP_IQS9151_MAX_FINGERS] = {false};
    float landing_dx = 0.0f, landing_dy = 0.0f;
    unsigned eligible = 0;   /* contacts that are or could be admitted this frame: intent is single-finger only */
    for (unsigned k = 0; k < BP_IQS9151_MAX_FINGERS; k++) {
        const bp_iqs9151_finger *g = &f->finger[k];
        if (g->valid && (state->admitted[k] || (g->area >= params->enter_area && g->area &&
            g->strength >= params->enter_strength && g->strength / g->area >= params->enter_density))) eligible++;
    }
    bool lone_raw = f->fingers_reported == 1u;
    if (!lone_raw) state->landing.n = 0;
    unsigned reported = 0;
    for (unsigned k = 0; k < BP_IQS9151_MAX_FINGERS; k++) {
        const bp_iqs9151_finger *g = &f->finger[k];
        if (!g->valid) {
            intent_clear(&state->intent[k], false);
            state->prev_strength[k] = 0;
            state->settle[k] = 0;
            state->near_frames[k] = 0;
            state->gap_have[k] = false; state->gap_defer_ms[k] = 0;
            if (!state->admitted[k]) {
                state->present[k] = false;
                state->weak_frames[k] = 0;
                continue;
            }
            /* dropout of an admitted contact: a weak frame with the position held */
            if (++state->weak_frames[k] >= BP_TOUCH_RELEASE_FRAMES) {
                log_release(state, k, false);
                state->present[k] = false;
                state->admitted[k] = false;
                state->weak_frames[k] = 0;
                continue;
            }
            ++state->frames[k]; state->last_ms[k] = time_ms;
            emit(state, params, k, state->last_strength[k], out);
            continue;
        }
        ++reported;
        state->present[k] = true;
        uint16_t prev_x = state->gap_x[k], prev_y = state->gap_y[k]; bool have_prev = state->gap_have[k];
        state->gap_x[k] = g->x; state->gap_y[k] = g->y; state->gap_have[k] = true;
        bool held = state->admitted[k];
        uint16_t density = (uint16_t)(g->area ? g->strength / g->area : 0u);
        bool pass = held ? hold_quality(g, params)
                         : g->strength >= params->enter_strength && g->area >= params->enter_area &&
                           density >= params->enter_density;
        bool watch_landing = lone_raw && (!held || (time_ms - state->start_ms[k] <= 130u &&
                              !state->pair_seen[k] && !state->intent[k].confirmed)) &&
                             (fp->flags & (BP_TF_EURO | BP_TF_STRING | BP_TF_SETTLE)) ==
                             (BP_TF_EURO | BP_TF_STRING | BP_TF_SETTLE);
        if (lone_raw && !watch_landing) state->landing.n = 0;
        /* v0.20.3 second-finger gate: next to an admitted, unweakened finger a real second finger shows density
         * ~170..530 (median ~360 over 20 min of ATI-250 use, 6 of 73 samples above 550): it may enter at
         * BP_TOUCH_SECOND_ENTER_DENSITY when 10..40 mm away with area >= 3 and >= 20 % of that finger's strength, for
         * BP_TOUCH_SPLIT_FRAMES consecutive frames. */
        bool second = false;
        if (!held && !pass && BP_TOUCH_SECOND_ENTER_DENSITY != 0u && g->area >= 3u && density >= BP_TOUCH_SECOND_ENTER_DENSITY) {
            for (unsigned j = 0; j < BP_IQS9151_MAX_FINGERS; j++) {
                if (j == k || !state->admitted[j] || state->weak_frames[j]) continue;
                int32_t dx = (int32_t)g->x - state->raw_x[j], dy = (int32_t)g->y - state->raw_y[j], d2 = dx * dx + dy * dy;
                if (d2 >= BP_TOUCH_SECOND_MIN_COUNTS * BP_TOUCH_SECOND_MIN_COUNTS && d2 <= BP_TOUCH_SECOND_MAX_COUNTS * BP_TOUCH_SECOND_MAX_COUNTS &&
                    (uint32_t)g->strength * 100u >= (uint32_t)state->last_strength[j] * 20u) { pass = true; second = true; }
            }
        }
        int32_t raw_mx = 0, raw_my = 0;
        if (watch_landing || held || pass) {
            raw_mx = bp_touch_axis(g->x, x_res, geo ? geo->n_cols : 0, geo ? geo->orig_cols : 0, geo ? geo->col_phys : NULL);
            raw_my = bp_touch_axis(g->y, y_res, geo ? geo->n_rows : 0, geo ? geo->orig_rows : 0, geo ? geo->row_phys : NULL);
        }
        if (watch_landing) {
            landing_proven[k] = landing_motion_observe(&state->landing, k,
                (bp_landing_point){time_ms, (uint16_t)raw_mx, (uint16_t)raw_my, g->strength, g->area},
                !held, pass, &landing_dx, &landing_dy);
        }
        bool fresh = false;
        uint16_t prev = state->prev_strength[k];
        state->prev_strength[k] = g->strength;
        if (held) {
            if (pass) state->weak_frames[k] = 0;
            else if (++state->weak_frames[k] >= BP_TOUCH_RELEASE_FRAMES) {
                log_release(state, k, true);
                state->admitted[k] = false; /* lifted: reported as gone from this frame on */
                state->weak_frames[k] = 0;
            }
        } else if (pass) {
            /* a new contact close to an admitted one must persist a 2nd frame (one-frame split blobs) */
            bool near = false;
            for (unsigned j = 0; j < BP_IQS9151_MAX_FINGERS; j++) {
                if (j == k || !state->admitted[j]) continue;
                int32_t dx = (int32_t)g->x - state->raw_x[j], dy = (int32_t)g->y - state->raw_y[j];
                if (dx * dx + dy * dy <= BP_TOUCH_SPLIT_COUNTS * BP_TOUCH_SPLIT_COUNTS) near = true;
            }
            if ((near || second) && state->near_frames[k] + 1u < BP_TOUCH_SPLIT_FRAMES) { ++state->near_frames[k]; continue; }
            /* over an electrode gap: one more frame, so the first position fed is the mean of two centroids */
            if (!have_prev && geo && (in_gap_axis(g->x, x_res, geo->n_cols, geo->col_phys) || in_gap_axis(g->y, y_res, geo->n_rows, geo->row_phys))) { state->gap_defer_ms[k] = time_ms ? time_ms : 1u; continue; }
            /* landing: wait while the strength still rises fast (the centroid is still moving) */
            if ((fp->flags & BP_TF_SETTLE) && prev && state->settle[k] < BP_TOUCH_SETTLE_MAX &&
                (uint32_t)g->strength * 100u > (uint32_t)prev * (100u + fp->settle_pct)) {
                ++state->settle[k];
                continue;
            }
            /* Admission starts a new lifetime, so the core never stitches two touches. */
            if (++state->next_birth >= (UINT64_MAX >> 3)) state->next_birth = 1;
            state->births[k] = state->next_birth;
            state->admitted[k] = true;
            state->settle[k] = 0; state->near_frames[k] = 0;
            fresh = true;
        } else {
            state->settle[k] = 0;
            if (!held) { state->gap_defer_ms[k] = 0; state->near_frames[k] = 0; }   /* below the entry gate: a later admission is a new landing; near/second frames must be consecutive */
        }
        if (!state->admitted[k]) continue;
        if (!fresh && have_prev) { int32_t sx = (int32_t)g->x - state->raw_x[k], sy = (int32_t)g->y - state->raw_y[k];
            /* a finger moves at most ~2 mm per frame (300 mm/s); a larger step is a chip glitch and must not steer the prediction */
            if (sx < -120) sx = -120; if (sx > 120) sx = 120; if (sy < -120) sy = -120; if (sy > 120) sy = 120;
            state->pvel_x[k] = state->vel_x[k]; state->pvel_y[k] = state->vel_y[k];
            state->vel_x[k] = (int16_t)sx; state->vel_y[k] = (int16_t)sy; } else { state->vel_x[k] = state->vel_y[k] = state->pvel_x[k] = state->pvel_y[k] = 0; }
        state->raw_x[k] = g->x; state->raw_y[k] = g->y;
        if (fresh) { state->jox[k] = 0.0f; state->joy[k] = 0.0f; }
        else if (have_prev && BP_TOUCH_JUMP_MM > 0.0f) {   /* jump absorption, see BP_TOUCH_JUMP_MM */
            float jx = (float)(geo ? bp_touch_axis(g->x, x_res, geo->n_cols, geo->orig_cols, geo->col_phys) - bp_touch_axis(prev_x, x_res, geo->n_cols, geo->orig_cols, geo->col_phys)
                                   : bp_touch_axis(g->x, x_res, 0, 0, NULL) - bp_touch_axis(prev_x, x_res, 0, 0, NULL)) * MM_X;
            float jy = (float)(geo ? bp_touch_axis(g->y, y_res, geo->n_rows, geo->orig_rows, geo->row_phys) - bp_touch_axis(prev_y, y_res, geo->n_rows, geo->orig_rows, geo->row_phys)
                                   : bp_touch_axis(g->y, y_res, 0, 0, NULL) - bp_touch_axis(prev_y, y_res, 0, 0, NULL)) * MM_Y;
            if (params->orient & BP_TOUCH_FLIP_X) jx = -jx;
            if (params->orient & BP_TOUCH_FLIP_Y) jy = -jy;
            /* an axis over an electrode gap alternates every frame by design (the 2-frame mean and the zigzag latch
             * handle it): that axis is neither tested nor absorbed */
            if (geo && (in_gap_axis(g->x, x_res, geo->n_cols, geo->col_phys) || in_gap_axis(prev_x, x_res, geo->n_cols, geo->col_phys))) jx = 0.0f;
            if (geo && (in_gap_axis(g->y, y_res, geo->n_rows, geo->row_phys) || in_gap_axis(prev_y, y_res, geo->n_rows, geo->row_phys))) jy = 0.0f;
            if (jx * jx + jy * jy > BP_TOUCH_JUMP_MM * BP_TOUCH_JUMP_MM) { state->jox[k] += jx; state->joy[k] += jy; ++state->jumps; }
        }
        if (fresh) {
            memset(&state->intent[k], 0, sizeof(state->intent[k]));
            state->resume_hint[k] = eligible == 1u && state->recent_motion_ms &&
                                    time_ms >= state->recent_motion_ms && time_ms - state->recent_motion_ms <= 500u;
            state->pair_seen[k] = false;
            /* the frame deferred over an electrode gap was the landing: the lifetime clocks (intent 20/80 ms, tap
             * window, repay start) start there, not one frame late (independent validation: 4 board-1 regressions
             * vs v0.19 in Codex's adversarial suite, e.g. missing_frame_after_confirmation 10 ms -> false click) */
            state->start_ms[k] = state->gap_defer_ms[k] && state->gap_defer_ms[k] < time_ms ? state->gap_defer_ms[k] : time_ms;
            state->gap_defer_ms[k] = 0; state->frames[k] = 0; state->max_dev[k] = 0;
            state->peak_density[k] = 0; state->start_density[k] = density; state->peak_strength[k] = 0;
        }
        if (g->strength > state->peak_strength[k]) state->peak_strength[k] = g->strength;
        state->last_strength[k] = g->strength;
        bool trusted = !state->weak_frames[k];
        /* An admitted single finger moves down to the hold floor. Requiring
         * the entry floor here erased its evidence while its contact was
         * still valid. Paired lifetimes retain their existing scroll policy. */
        unsigned motion_strength = state->pair_seen[k] ? params->enter_strength : params->exit_strength;
        unsigned motion_density = state->pair_seen[k] ? params->enter_density : params->exit_density;
        bool raw_sliding = lift_motion_observe(state, k, raw_mx, raw_my, g->strength, time_ms, fresh,
            trusted && eligible == 1u && g->strength >= motion_strength && density >= motion_density);
        raw_sliding |= landing_proven[k];
        /* lift hold: within the tap window a falling strength means the finger is leaving and the centroid drifts;
         * hold the position. v0.20: only for a contact that has NOT moved yet (01:43 free slide: a finger that
         * re-landed straight into a 30 mm/s stroke dipped to 79 % of its peak, was frozen for 50 ms, and the
         * release jump then tripped the core's spike guard for another 170 ms: 932 ms without output). */
        {
            float mx = (state->held_x[k] - state->start_x[k]) * MM_X, my = (state->held_y[k] - state->start_y[k]) * MM_Y;
            bool moved = raw_sliding || state->intent[k].confirmed ||
                         (state->frames[k] && mx * mx + my * my >= 0.6f * 0.6f);
            /* v0.20.4: not for a member of a pair (another present finger 10..40 mm away): the lighter finger of a
             * scroll often stays under 85 % of its landing peak, and this froze it for the whole 250 ms window */
            bool paired = false;
            for (unsigned j = 0; j < BP_IQS9151_MAX_FINGERS && !paired; j++) {
                if (j == k || !state->admitted[j] || !state->present[j]) continue;
                int32_t dx = (int32_t)g->x - state->raw_x[j], dy = (int32_t)g->y - state->raw_y[j], d2 = dx * dx + dy * dy;
                paired = d2 >= BP_TOUCH_SECOND_MIN_COUNTS * BP_TOUCH_SECOND_MIN_COUNTS && d2 <= BP_TOUCH_SECOND_MAX_COUNTS * BP_TOUCH_SECOND_MAX_COUNTS;
            }
            /* v0.20.4: only while the strength keeps falling (a lift falls frame after frame; a light moving finger
             * dips and rises). Below 85 % of the landing peak alone froze circles for the whole window: "moved" is
             * measured on the frozen position, so it never became true, and the release jumped ~15 mm. */
            bool falling = !prev || (uint32_t)g->strength * 16u <= (uint32_t)prev * 17u;   /* unknown (slot changed): hold as before */
            if (trusted && !moved && !paired && falling && (fp->flags & BP_TF_LIFT) && time_ms - state->start_ms[k] < fp->tap_window_ms &&
                (uint32_t)g->strength * 100u < (uint32_t)state->peak_strength[k] * fp->lift_pct)
                trusted = false;   /* leaving: the centroid drifts while the strength falls */
        }
        {   /* lift tail of a stopped finger, see BP_TOUCH_LIFT_PCT */
            if (fresh) { state->recent_strength[k] = g->strength; state->lift_hold_ms[k] = 0; }
            uint16_t recent = state->recent_strength[k];
            if (state->lift_hold_ms[k]) {
                /* v0.20.4: a lift only falls; a strength that rises again (by 1/8 over the last frame) was a dip */
                bool recovered = raw_sliding || (uint32_t)g->strength * 100u >= (uint32_t)recent * BP_TOUCH_LIFT_RECOVER_PCT ||
                                 (prev && (uint32_t)g->strength * 8u > (uint32_t)prev * 9u);
                bool expired = time_ms - state->lift_hold_ms[k] > BP_TOUCH_LIFT_HOLD_MS;
                if (recovered || expired) { state->lift_hold_ms[k] = 0; state->recent_strength[k] = g->strength; }
                else trusted = false;
            } else if (trusted && (fp->flags & BP_TF_LIFT) && !raw_sliding && !state->moving_now[k] && state->frames[k] > 12u &&
                       (uint32_t)g->strength * 100u < (uint32_t)recent * BP_TOUCH_LIFT_PCT) {
                state->lift_hold_ms[k] = time_ms ? time_ms : 1u; ++state->lift_holds; trusted = false;
            }
            if (trusted) state->recent_strength[k] = (uint16_t)(recent + ((int32_t)g->strength - (int32_t)recent) / 8);
        }
        bool intent_seen = false, thawed_by_intent = false;
        if (!trusted && !state->weak_frames[k] && eligible == 1u && !state->pair_seen[k] &&
            (time_ms - state->start_ms[k] <= fp->tap_window_ms || state->intent[k].confirmed) &&
            (fp->flags & (BP_TF_EURO | BP_TF_STRING)) == (BP_TF_EURO | BP_TF_STRING)) {
            /* Freeze the OUTPUT, not the evidence. A light, still-valid
             * contact must be able to prove a short stroke while frozen.
             * Reuse the landing/strength/fit/deceleration safeguards. */
            int32_t ix = (params->orient & BP_TOUCH_FLIP_X) ? BP_COORD_MAX - raw_mx : raw_mx;
            int32_t iy = (params->orient & BP_TOUCH_FLIP_Y) ? BP_COORD_MAX - raw_my : raw_my;
            ix -= (int32_t)lroundf(state->jox[k] / MM_X);
            iy -= (int32_t)lroundf(state->joy[k] / MM_Y);
            if (ix < 0) ix = 0;
            if (ix > BP_COORD_MAX) ix = BP_COORD_MAX;
            if (iy < 0) iy = 0;
            if (iy > BP_COORD_MAX) iy = BP_COORD_MAX;
            bool was_confirmed = state->intent[k].confirmed;
            bool was_active = state->intent[k].active;
            intent_observe(state, k, ix, iy, g->strength, g->area, time_ms, false);
            intent_seen = true;
            /* A one-time light-pressure step is allowed after it stabilises;
             * a finger still losing strength is a lift, not a new stroke. */
            uint16_t recent_peak = g->strength;
            bp_touch_intent *evidence = &state->intent[k];
            for (unsigned i = 0; i < evidence->n; i++)
                if (time_ms - evidence->p[i].ms <= 30u && evidence->p[i].strength > recent_peak)
                    recent_peak = evidence->p[i].strength;
            /* Lifetime confirmation is not perpetual permission: a stopped
             * finger's final centroid drift must not resurrect movement.
             * Continue an active slide, or require the fresh strict fit. */
            bool valid_slide = evidence->active && ((was_confirmed && was_active) || evidence->pressure_safe);
            if (!valid_slide && (uint32_t)g->strength * 100u < (uint32_t)recent_peak * 90u) {
                evidence->active = false; evidence->confirmed = was_confirmed; evidence->good_ms = 0;
            }
            if (state->intent[k].active) {
                trusted = thawed_by_intent = true;
                state->lift_hold_ms[k] = 0;
                state->recent_strength[k] = g->strength;
            }
        }
        if (trusted && (raw_sliding || (eligible == 1u && !state->pair_seen[k])) &&
            !fresh && state->trusted_ms[k] < state->last_ms[k]) {
            /* Any trusted recovery after frozen output must stay continuous.
             * Strength recovery can thaw a contact before raw motion qualifies.
             * Resume at the last emitted position, carrying an input offset;
             * never present accumulated held travel as a one-frame jump. */
            int32_t tx = (params->orient & BP_TOUCH_FLIP_X) ? BP_COORD_MAX - raw_mx : raw_mx;
            int32_t ty = (params->orient & BP_TOUCH_FLIP_Y) ? BP_COORD_MAX - raw_my : raw_my;
            float dx = tx * MM_X - state->jox[k] - state->held_x[k] * MM_X;
            float dy = ty * MM_Y - state->joy[k] - state->held_y[k] * MM_Y;
            /* A short confidence dip is not a discontinuity when the chip's
             * successive positions stay continuous. Rebasing that case eats
             * the start of a real swipe (7eab:18ms dip,2mm accumulated travel).
             * Long freezes and abrupt recovery steps still discard backlog. */
            int32_t pmx = geo ? bp_touch_axis(prev_x, x_res, geo->n_cols, geo->orig_cols, geo->col_phys)
                              : bp_touch_axis(prev_x, x_res, 0, 0, NULL);
            int32_t pmy = geo ? bp_touch_axis(prev_y, y_res, geo->n_rows, geo->orig_rows, geo->row_phys)
                              : bp_touch_axis(prev_y, y_res, 0, 0, NULL);
            float sx = (raw_mx - pmx) * MM_X, sy = (raw_my - pmy) * MM_Y;
            bool short_continuous = have_prev && time_ms - state->trusted_ms[k] <= 30u &&
                                    sx * sx + sy * sy <= 1.5f * 1.5f;
            if (!short_continuous && dx * dx + dy * dy > 1.5f * 1.5f) {
                state->jox[k] += dx; state->joy[k] += dy;
                state->euro_init[k] = false;
                /* Translate the evidence into the new input origin. Clearing
                 * it here makes a proven short stroke wait another window. */
                bp_touch_intent *m = &state->intent[k];
                for (unsigned i = 0; i < m->n; i++) { m->p[i].x -= dx; m->p[i].y -= dy; }
                m->good_x -= dx; m->good_y -= dy;
            }
        }
        if (trusted || fresh) {
            uint16_t rx = g->x, ry = g->y;
            bool gap = geo && (in_gap_axis(g->x, x_res, geo->n_cols, geo->col_phys) || in_gap_axis(g->y, y_res, geo->n_rows, geo->row_phys));
            if (gap && have_prev) { ++state->gap_frames; rx = (uint16_t)(((uint32_t)g->x + prev_x) / 2u); ry = (uint16_t)(((uint32_t)g->y + prev_y) / 2u); }
            int32_t x = geo ? bp_touch_axis(rx, x_res, geo->n_cols, geo->orig_cols, geo->col_phys)
                            : bp_touch_axis(rx, x_res, 0, 0, NULL);
            int32_t y = geo ? bp_touch_axis(ry, y_res, geo->n_rows, geo->orig_rows, geo->row_phys)
                            : bp_touch_axis(ry, y_res, 0, 0, NULL);
            if (params->orient & BP_TOUCH_FLIP_X) x = BP_COORD_MAX - x;
            if (params->orient & BP_TOUCH_FLIP_Y) y = BP_COORD_MAX - y;
            if (state->jox[k] != 0.0f || state->joy[k] != 0.0f) {   /* the absorbed jumps */
                x -= (int32_t)lroundf(state->jox[k] / MM_X); y -= (int32_t)lroundf(state->joy[k] / MM_Y);
                if (x < 0) x = 0; if (x > BP_COORD_MAX) x = BP_COORD_MAX; if (y < 0) y = 0; if (y > BP_COORD_MAX) y = BP_COORD_MAX;
            }
            {   /* alternation latch, see BP_TOUCH_ZIGZAG_MM */
                float px = (float)x * MM_X, py = (float)y * MM_Y;
                if (fresh) { state->zn[k] = 0; state->zz_ms[k] = 0; }
                if (state->zn[k] >= 3u) {
                    float d2x = px - state->zx[k][0], d2y = py - state->zy[k][0];
                    float d1x = state->zx[k][0] - state->zx[k][1], d1y = state->zy[k][0] - state->zy[k][1];
                    float d0x = state->zx[k][1] - state->zx[k][2], d0y = state->zy[k][1] - state->zy[k][2];
                    float m2 = BP_TOUCH_ZIGZAG_MM * BP_TOUCH_ZIGZAG_MM;
                    if (d2x * d2x + d2y * d2y >= m2 && d1x * d1x + d1y * d1y >= m2 && d0x * d0x + d0y * d0y >= m2 &&
                        d2x * d1x + d2y * d1y < 0.0f && d1x * d0x + d1y * d0y < 0.0f) { state->zz_ms[k] = time_ms ? time_ms : 1u; ++state->zigzags; }
                }
                float ox = px, oy = py;
                if (state->zn[k] >= 1u && state->zz_ms[k] && time_ms - state->zz_ms[k] <= BP_TOUCH_ZIGZAG_HOLD_MS) { ox = 0.5f * (px + state->zx[k][0]); oy = 0.5f * (py + state->zy[k][0]); }
                state->zx[k][2] = state->zx[k][1]; state->zy[k][2] = state->zy[k][1]; state->zx[k][1] = state->zx[k][0]; state->zy[k][1] = state->zy[k][0]; state->zx[k][0] = px; state->zy[k][0] = py;
                if (state->zn[k] < 3u) state->zn[k]++;
                x = (int32_t)lroundf(ox / MM_X); y = (int32_t)lroundf(oy / MM_Y);
            }
            state->gap_now = gap;
            if (eligible == 1u && (fp->flags & (BP_TF_EURO | BP_TF_STRING)) == (BP_TF_EURO | BP_TF_STRING)) {
                if (!intent_seen) intent_observe(state, k, x, y, g->strength, g->area, time_ms, fresh);
            }
            else
                intent_clear(&state->intent[k], true);
            /* The independent raw observer has already proved a coherent
             * stroke. Do not make a thawed short stroke wait for a second
             * trajectory window built from its formerly frozen output. */
            raw_proven[k] = trusted && (raw_sliding || thawed_by_intent) && eligible == 1u && !state->pair_seen[k];
            if (raw_proven[k]) state->intent[k].active = true;
            condition(state, k, x, y, fresh, time_ms, fp);
        } else {
            if (!intent_seen) intent_clear(&state->intent[k], false); /* actual weak/lift frames cannot prove motion */
        }
        if (fresh) { state->start_x[k] = state->held_x[k]; state->start_y[k] = state->held_y[k]; }
        ++state->frames[k]; state->last_ms[k] = time_ms;
        if (density > state->peak_density[k]) state->peak_density[k] = density;
        {
            int32_t ddx = state->held_x[k] - state->start_x[k], ddy = state->held_y[k] - state->start_y[k];
            uint32_t dev = (uint32_t)(ddx < 0 ? -ddx : ddx);
            uint32_t devy = (uint32_t)(ddy < 0 ? -ddy : ddy);
            if (devy > dev) dev = devy;
            if (dev > state->max_dev[k]) state->max_dev[k] = (uint16_t)(dev > 0xFFFFu ? 0xFFFFu : dev);
        }
        emit(state, params, k, g->strength, out);
    }
    /* Never promote a survivor selected from a multi-contact observation. */
    if (out->count != 1u) {
        for (unsigned k = 0; k < BP_IQS9151_MAX_FINGERS; k++) intent_clear(&state->intent[k], true);
        for (unsigned i = 0; i < out->count; i++)
            state->pair_seen[(out->contacts[i].id & 7u) - 1u] = true;
    } else {
        unsigned k = (unsigned)((out->contacts[0].id & 7u) - 1u);
        /* the core consumes the evidence only on the 2nd consecutive evidenced frame (one frame never
         * cancels a tap on its own) */
        if (eligible == 1u && state->intent[k].active && state->trusted_ms[k] == time_ms &&
            !state->weak_frames[k] && f->finger[k].valid) {
            if (state->intent_streak[k] < 255u) state->intent_streak[k]++;
        } else state->intent_streak[k] = 0;
        if (state->intent_streak[k] >= 2u && raw_proven[k] && !state->intent[k].confirmed) {
            state->intent[k].confirmed = true;
            out->motion_intent_id = out->contacts[0].id;
            /* Start here: frozen/landing displacement must not be repaid as
             * a sudden cursor jump. Subsequent deltas retain the same gain. */
            out->motion_origin_x = out->contacts[0].x;
            out->motion_origin_y = out->contacts[0].y;
            if (landing_proven[k]) {
                float scale = BP_TOUCH_REPAY_CAP_MM / sqrtf(landing_dx * landing_dx + landing_dy * landing_dy);
                float dx = (params->orient & BP_TOUCH_FLIP_X) ? -landing_dx : landing_dx;
                float dy = (params->orient & BP_TOUCH_FLIP_Y) ? -landing_dy : landing_dy;
                int32_t ox = out->motion_origin_x - (int32_t)lroundf(dx * scale / MM_X);
                int32_t oy = out->motion_origin_y - (int32_t)lroundf(dy * scale / MM_Y);
                out->motion_origin_x = ox < 0 ? 0 : ox > BP_COORD_MAX ? BP_COORD_MAX : ox;
                out->motion_origin_y = oy < 0 ? 0 : oy > BP_COORD_MAX ? BP_COORD_MAX : oy;
            }
        } else if (state->intent_streak[k] >= 2u && state->intent[k].confirmed && state->intent[k].n) {
            const bp_touch_intent *m = &state->intent[k];
            out->motion_intent_id = out->contacts[0].id;
            /* repay the window only if the finger still moves the way the window went (Codex: a reversal
             * just before confirmation replayed 0.7 mm of the OLD direction for ~85 ms) */
            /* v0.18: repay the trajectory since its settled start (>= 60 ms after admission, so no landing
             * centroid tail), relative to the OUTPUT position, capped. The core emits it in one report at
             * confirmation, so nothing of it trails into a reversal; and it is only paid while the last
             * ~30 ms still move the window's way at >= 30 % of the window's mean speed. */
            unsigned s0 = 0;
            while (s0 + 1u < m->n && m->p[s0].ms < state->start_ms[k] + 60u) s0++;
            float tx = m->p[m->n - 1u].x - m->p[s0].x, ty = m->p[m->n - 1u].y - m->p[s0].y;
            float tspan = (float)(m->p[m->n - 1u].ms - m->p[s0].ms);
            unsigned r0 = m->n - 1u;
            while (r0 && m->p[m->n - 1u].ms - m->p[r0 - 1u].ms <= 30u) r0--;   /* the last ~30 ms */
            float rx = m->p[m->n - 1u].x - m->p[r0].x, ry = m->p[m->n - 1u].y - m->p[r0].y;
            float rspan = (float)(m->p[m->n - 1u].ms - m->p[r0].ms);
            float len = sqrtf(tx * tx + ty * ty), cap = BP_TOUCH_REPAY_CAP_MM;
            /* (r . t / |t|) / rspan >= 0.3 * |t| / tspan */
            bool same_way = len > 0.0f && rspan > 0.0f && tspan > 0.0f &&
                            (rx * tx + ry * ty) * tspan >= 0.3f * len * len * rspan;
            /* repay what the OUTPUT moved since the settled start (the core emitted nothing of it during the
             * candidate); the raw travel would also repay the filter lag and overshoot (v0.18: 52 for 1 mm) */
            float ox = state->ax[k], oy = state->ay[k];
            if (same_way) {
                float px = ox - m->p[s0].ox, py = oy - m->p[s0].oy, pl = sqrtf(px * px + py * py);
                float k_ = pl > cap ? cap / pl : 1.0f; ox -= px * k_; oy -= py * k_;
            }
            out->motion_origin_x = (int32_t)lroundf(ox / MM_X);
            out->motion_origin_y = (int32_t)lroundf(oy / MM_Y);
        }
    }
    if (out->count == 1u && out->motion_intent_id) state->recent_motion_ms = time_ms;
    else if (out->count > 1u) state->recent_motion_ms = 0;
    /* Oldest lifetime first, as with the A000 policy: the core treats contact 0 as primary. */
    for (unsigned i = 0; i < out->count; i++)
        for (unsigned j = i + 1; j < out->count; j++)
            if (out->contacts[j].id < out->contacts[i].id) {
                bp_contact c = out->contacts[i]; out->contacts[i] = out->contacts[j]; out->contacts[j] = c;
                uint16_t s = out->strengths[i]; out->strengths[i] = out->strengths[j]; out->strengths[j] = s;
                unsigned wi = (out->weak_contacts >> i) & 1u, wj = (out->weak_contacts >> j) & 1u;
                out->weak_contacts = (uint8_t)((out->weak_contacts & ~((1u << i) | (1u << j))) | (wj << i) | (wi << j));
            }
    if (!out->count && reported) {
        ++state->suppressed_frames; /* hover: counted for diagnostics, delivered as "no touch" */
    } else if (out->count) {
        ++state->admitted_frames;
    }
}




