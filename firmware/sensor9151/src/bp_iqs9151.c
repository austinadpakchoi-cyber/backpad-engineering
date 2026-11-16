#include "bp_iqs9151.h"
#include <errno.h>
#include <string.h>

static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (uint16_t)p[1] << 8); }

static bool all_ee(const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; i++) if (p[i] != 0xEEu) return false;
    return n > 0;
}

bp_iqs9151_result bp_iqs9151_decode(const uint8_t *raw, size_t n, uint16_t x_res, uint16_t y_res,
                                    bp_iqs9151_frame *out) {
    if (!out) return BP_IQS9151_ARGUMENT;
    memset(out, 0, sizeof(*out));
    if (!raw) return BP_IQS9151_ARGUMENT;
    if (n != BP_IQS9151_BLOCK_BYTES) return BP_IQS9151_LENGTH;
    /* A read outside a RDY window returns 0xEE in every byte: never data. Info+TP flags alone
     * being 0xEEEE is already impossible for a real frame (reserved bits set). */
    if (all_ee(raw, n) || (le16(raw + 12) == 0xEEEEu && le16(raw + 14) == 0xEEEEu)) return BP_IQS9151_COMMS;
    bp_iqs9151_frame f;
    memset(&f, 0, sizeof(f));
    f.rel_x = (int16_t)le16(raw + 0);
    f.rel_y = (int16_t)le16(raw + 2);
    f.gesture_x = le16(raw + 4);
    f.gesture_y = le16(raw + 6);
    f.gestures1 = le16(raw + 8);
    f.gestures2 = le16(raw + 10);
    f.info = le16(raw + 12);
    f.tp = le16(raw + 14);
    f.fingers_reported = (uint8_t)(f.tp & BP_IQS9151_TP_FINGERS_MASK);
    if (f.fingers_reported > BP_IQS9151_MAX_FINGERS) return BP_IQS9151_COUNT;
    for (unsigned k = 0; k < BP_IQS9151_MAX_FINGERS; k++) {
        const uint8_t *p = raw + 16u + 8u * k;
        bp_iqs9151_finger *g = &f.finger[k];
        g->x = le16(p);
        g->y = le16(p + 2);
        g->strength = le16(p + 4);
        g->area = le16(p + 6);
        /* 0xFFFF marks an empty slot (Azoteq example: "65535 means there is no touch"). */
        /* Slots beyond Max Multi-Touches read all zero on the real chip (seen on the V2 board). */
        g->valid = g->x != 0xFFFFu && g->y != 0xFFFFu && (g->strength || g->area);
        if (g->valid && (g->x > x_res || g->y > y_res)) return BP_IQS9151_COORDINATE;
    }
    *out = f;
    return BP_IQS9151_OK;
}

/* ------------------------------------------------------------------ transactions */
static int fail(bp_iqs9151_bus *b, int error) {
    b->initialized = false;
    b->last_error = error;
    return error;
}

/* One transaction in the next fresh window; waits for a window up to the timeout. A read that
 * still returns 0xEE throughout (window lost between RDY and START) is retried in a later window. */
static int transact(bp_iqs9151_bus *b, bool write, uint16_t reg, uint8_t *data, size_t n) {
    uint64_t start = b->hal.now_ms(b->hal.context);
    unsigned ee = 0;
    for (;;) {
        bp_iqs9151_window w = {0};
        int r = write ? b->hal.write(b->hal.context, reg, data, n, &w)
                      : b->hal.read(b->hal.context, reg, data, n, &w);
        if (r == 0 && !write && all_ee(data, n)) {
            if (++ee > 3u) return -EPROTO;
            r = -EAGAIN;
        }
        if (r != -EAGAIN) return r;
        if (b->hal.now_ms(b->hal.context) - start > BP_IQS9151_WINDOW_TIMEOUT_MS) return -ETIMEDOUT;
        b->hal.wait_ms(b->hal.context, 1u);
    }
}

int bp_iqs9151_bus_read_reg(bp_iqs9151_bus *b, uint16_t reg, uint8_t *data, size_t n) {
    if (!b || !data || !n || n > BP_IQS9151_MAX_READ) return -EINVAL;
    return transact(b, false, reg, data, n);
}

static int write_reg(bp_iqs9151_bus *b, uint16_t reg, const uint8_t *data, size_t n) {
    if (!n || n > BP_IQS9151_MAX_SECTION) return -E2BIG;
    uint8_t copy[BP_IQS9151_MAX_SECTION];
    memcpy(copy, data, n);
    return transact(b, true, reg, copy, n);
}

/* Read-modify-write of System Control: command bits are self-clearing on the chip. */
static int sysctrl_set(bp_iqs9151_bus *b, uint16_t bits) {
    uint8_t sc[2];
    int r = transact(b, false, BP_IQS9151_REG_SYSCTRL, sc, 2);
    if (r) return r;
    uint16_t v = (uint16_t)(le16(sc) | bits);
    sc[0] = (uint8_t)v;
    sc[1] = (uint8_t)(v >> 8);
    return transact(b, true, BP_IQS9151_REG_SYSCTRL, sc, 2);
}

static bool expected_byte(uint16_t addr, uint8_t *v) {
    for (unsigned i = 0; i < bp_iqs9151_settings_count; i++) {
        const bp_iqs9151_section *s = &bp_iqs9151_settings[i];
        if (addr >= s->reg && addr < s->reg + s->len) { *v = s->data[addr - s->reg]; return true; }
    }
    return false;
}

static int verify(bp_iqs9151_bus *b, uint16_t reg, size_t n, uint8_t *got) {
    int r = transact(b, false, reg, got, n);
    if (r) return r;
    for (size_t i = 0; i < n; i++) {
        uint8_t want;
        if (expected_byte((uint16_t)(reg + i), &want) && got[i] != want) return -EIO;
    }
    return 0;
}

int bp_iqs9151_bus_reati(bp_iqs9151_bus *b) {
    if (!b) return -EINVAL;
    b->ati_error = false;
    uint64_t start = b->hal.now_ms(b->hal.context);
    int r = sysctrl_set(b, BP_IQS9151_SC_TP_REATI);
    if (r) return r;
    for (;;) {
        uint8_t info[2];
        r = transact(b, false, BP_IQS9151_REG_INFO, info, 2);
        /* A single transaction gives up after BP_IQS9151_WINDOW_TIMEOUT_MS without a window; the chip may
         * be silent that long while it runs ATI, so only the overall ATI timeout ends the wait. */
        if (r == -ETIMEDOUT || r == -EPROTO) {
            if (b->hal.now_ms(b->hal.context) - start > BP_IQS9151_ATI_TIMEOUT_MS) return -ETIMEDOUT;
            continue;
        }
        if (r) return r;
        b->last_info = le16(info);
        if (b->last_info & BP_IQS9151_INFO_REATI_DONE) break;
        if (b->hal.now_ms(b->hal.context) - start > BP_IQS9151_ATI_TIMEOUT_MS) return -ETIMEDOUT;
    }
    b->ati_ms = (uint32_t)(b->hal.now_ms(b->hal.context) - start);
    b->ati_error = (b->last_info & BP_IQS9151_INFO_ATI_ERROR) != 0;
    return b->ati_error ? BP_IQS9151_EATI : 0;
}

static int write_checked(bp_iqs9151_bus *b, uint16_t reg, const uint8_t *data, size_t n) {
    int r = write_reg(b, reg, data, n);
    if (r) return r;
    uint8_t got[BP_IQS9151_MAX_SECTION];
    r = transact(b, false, reg, got, n);
    if (r) return r;
    return memcmp(got, data, n) ? -EIO : 0;
}

/* A channel still at full scale after ATI cannot sense: seen as whole Tx rows / Rx columns on the
 * hand-soldered V2 boards (open or bridged pins). Left enabled, it keeps ATI Error set, and the chip
 * then retries re-ATI every few seconds, possibly with a finger on the pad.
 * A line dead across the whole pad is taken out of the RxTx mapping rather than disabled: a disabled
 * line leaves a hole that splits one finger into two (board 1: a finger at y~2000 plus a weak ghost at
 * the same x, y~1220, either side of the dead centre row TX36). The chip then treats the neighbours
 * as adjacent; col_phys/row_phys keep each remaining line's settings-table index so the touch policy
 * restores the physical spacing. Remaining isolated dead channels are disabled.
 * Returns 1 when the configuration changed (caller repeats ATI), 0 when nothing was changed, <0 on a
 * bus error. */
static int handle_dead_channels(bp_iqs9151_bus *b) {
    unsigned rx = b->total_rx, tx = b->total_tx, n = rx * tx;
    if (!n || rx > 32u || tx > BP_IQS9151_MAX_TX || rx + tx > BP_IQS9151_MAP_ENTRIES ||
        2u * n > BP_IQS9151_MAX_READ) return 0;
    /* Two count snapshots ~50 ms apart must both read full scale: one snapshot can catch a transient
     * (e.g. a finger leaving during start-up), and the decision changes the electrode mapping. */
    uint8_t counts[BP_IQS9151_MAX_READ], again[BP_IQS9151_MAX_READ];
    int r = transact(b, false, BP_IQS9151_REG_COUNTS, counts, 2u * n);
    if (r) return r;
    b->hal.wait_ms(b->hal.context, 50u);
    r = transact(b, false, BP_IQS9151_REG_COUNTS, again, 2u * n);
    if (r) return r;
    unsigned dead = 0;
    for (unsigned t = 0; t < tx; t++)
        for (unsigned c = 0; c < rx; c++)
            if (le16(counts + 2u * (t * rx + c)) >= BP_IQS9151_COUNT_FULL_SCALE &&
                le16(again + 2u * (t * rx + c)) >= BP_IQS9151_COUNT_FULL_SCALE) {
                b->dead_rows[t] |= (uint32_t)1u << c;
                ++dead;
            }
    b->dead_channels = (uint8_t)(dead > 255u ? 255u : dead);
    if (!dead || dead > BP_IQS9151_MAX_DEAD) return 0;

    uint32_t all = rx == 32u ? 0xFFFFFFFFu : ((uint32_t)1u << rx) - 1u, dead_cols = all;
    for (unsigned t = 0; t < tx; t++) dead_cols &= b->dead_rows[t];
    uint8_t cur[BP_IQS9151_MAP_ENTRIES], map[BP_IQS9151_MAP_ENTRIES] = {0}, cols[32], rows[BP_IQS9151_MAX_TX];
    r = transact(b, false, BP_IQS9151_REG_MAPPING, cur, rx + tx);
    if (r) return r;
    unsigned keep_rx = 0, keep_tx = 0, k = 0;
    for (unsigned c = 0; c < rx; c++)
        if (!(dead_cols >> c & 1u)) { cols[keep_rx++] = (uint8_t)c; map[k++] = cur[c]; }
    for (unsigned t = 0; t < tx; t++)
        if (b->dead_rows[t] != all) { rows[keep_tx++] = (uint8_t)t; map[k++] = cur[rx + t]; }
    bool remove = (keep_rx < rx || keep_tx < tx) && keep_rx >= 3u && keep_tx >= 3u;
    if (remove) {
        uint8_t totals[2] = {(uint8_t)keep_rx, (uint8_t)keep_tx};
        uint16_t xr = (uint16_t)((keep_rx - 1u) * 256u), yr = (uint16_t)((keep_tx - 1u) * 256u);
        uint8_t res[4] = {(uint8_t)xr, (uint8_t)(xr >> 8), (uint8_t)yr, (uint8_t)(yr >> 8)};
        r = write_checked(b, BP_IQS9151_REG_TOTALS, totals, sizeof(totals));
        if (!r) r = write_checked(b, BP_IQS9151_REG_RESOLUTION, res, sizeof(res));
        if (!r) r = write_checked(b, BP_IQS9151_REG_MAPPING, map, rx + tx); /* zeroes the freed entries */
        if (r) return r;
        b->removed_rx = (uint8_t)(rx - keep_rx);
        b->removed_tx = (uint8_t)(tx - keep_tx);
        b->total_rx = (uint8_t)keep_rx;
        b->total_tx = (uint8_t)keep_tx;
        b->x_res = xr;
        b->y_res = yr;
        memcpy(b->col_phys, cols, keep_rx);
        memcpy(b->row_phys, rows, keep_tx);
    }
    /* isolated dead channels in the grid now in use; rows beyond the new Total Tx are cleared too */
    uint8_t mask[4u * BP_IQS9151_MAX_TX] = {0};
    unsigned isolated = 0;
    for (unsigned t = 0; t < b->total_tx; t++)
        for (unsigned c = 0; c < b->total_rx; c++)
            if (b->dead_rows[b->row_phys[t]] >> b->col_phys[c] & 1u) {
                mask[4u * t + c / 8u] |= (uint8_t)(1u << (c % 8u));
                ++isolated;
            }
    r = write_checked(b, BP_IQS9151_REG_CH_DISABLE, mask, 4u * tx);
    if (r) return r;
    b->dead_handled = (uint8_t)((isolated ? 1u : 0u) | (remove ? 2u : 0u));
    return 1;
}

int bp_iqs9151_bus_init(bp_iqs9151_bus *b, const bp_iqs9151_hal *hal) {
    if (!b || !hal || !hal->read || !hal->write || !hal->now_ms || !hal->wait_ms) return -EINVAL;
    uint32_t next = b->next_session;
    memset(b, 0, sizeof(*b));
    b->hal = *hal;
    b->next_session = next;

    b->step = BP_IQS9151_STEP_PRODUCT;
    uint8_t id[6];
    int r = transact(b, false, BP_IQS9151_REG_PRODUCT, id, sizeof(id));
    if (r) return fail(b, r);
    b->product = le16(id);
    b->major = le16(id + 2);
    b->minor = le16(id + 4);
    if (b->product != BP_IQS9151_PRODUCT) return fail(b, -ENODEV);

    /* Settings are only trusted after a reset the chip itself reports: otherwise soft-reset it,
     * exactly as the Azoteq example does when Show Reset is already cleared. */
    b->step = BP_IQS9151_STEP_RESET;
    for (unsigned attempt = 0;; attempt++) {
        uint8_t info[2];
        r = transact(b, false, BP_IQS9151_REG_INFO, info, 2);
        if (r) return fail(b, r);
        b->last_info = le16(info);
        if (b->last_info & BP_IQS9151_INFO_SHOW_RESET) break;
        if (attempt >= 2u) return fail(b, -EIO);
        r = sysctrl_set(b, BP_IQS9151_SC_SW_RESET);
        if (r) return fail(b, r);
        ++b->sw_resets;
        b->hal.wait_ms(b->hal.context, BP_IQS9151_RESET_WAIT_MS);
    }

    b->step = BP_IQS9151_STEP_ACK;
    r = sysctrl_set(b, BP_IQS9151_SC_ACK_RESET);
    if (r) return fail(b, r);

    b->step = BP_IQS9151_STEP_SETTINGS;
    for (unsigned i = 0; i < bp_iqs9151_settings_count; i++) {
        const bp_iqs9151_section *s = &bp_iqs9151_settings[i];
        r = write_reg(b, s->reg, s->data, s->len);
        if (r) return fail(b, r);
    }

    /* Geometry and electrode mapping must read back exactly, or coordinates are meaningless. */
    b->step = BP_IQS9151_STEP_VERIFY;
    uint8_t tps[8], map[25];
    r = verify(b, BP_IQS9151_REG_TP_SETTINGS, sizeof(tps), tps);
    if (!r) r = verify(b, BP_IQS9151_REG_MAPPING, sizeof(map), map);
    if (r) return fail(b, r);
    b->tp_settings = tps[0];
    b->total_rx = b->orig_rx = tps[1];
    b->total_tx = b->orig_tx = tps[2];
    for (unsigned i = 0; i < sizeof(b->col_phys); i++) b->col_phys[i] = (uint8_t)i;
    for (unsigned i = 0; i < sizeof(b->row_phys); i++) b->row_phys[i] = (uint8_t)i;
    b->max_touches = tps[3];
    b->x_res = le16(tps + 4);
    b->y_res = le16(tps + 6);
    if (!b->x_res || !b->y_res) return fail(b, -EIO);

    /* An ATI error (e.g. targets not reached through a thick overlay) is published, not fatal:
     * streaming data is still the best evidence for tuning ATI target and thresholds. */
    b->step = BP_IQS9151_STEP_ATI;
    r = bp_iqs9151_bus_reati(b);
    if (r == BP_IQS9151_EATI) {
        int m = handle_dead_channels(b);
        if (m < 0) return fail(b, m);
        if (m > 0) r = bp_iqs9151_bus_reati(b);
    }
    b->ati_error = r == BP_IQS9151_EATI;
    if (r && r != BP_IQS9151_EATI) return fail(b, r);

    b->step = BP_IQS9151_STEP_RESEED;
    r = sysctrl_set(b, BP_IQS9151_SC_TP_RESEED);
    if (r) return fail(b, r);

    b->step = BP_IQS9151_STEP_STREAM;
    if (++b->next_session == 0) b->next_session = 1;
    b->session = b->next_session;
    b->sequence = 0;
    b->initialized = true;
    b->last_error = 0;
    return 0;
}

int bp_iqs9151_bus_read(bp_iqs9151_bus *b, bp_iqs9151_frame *out, bp_iqs9151_window *w) {
    if (!b || !out || !w) return -EINVAL;
    if (!b->initialized) return -EACCES;
    uint8_t raw[BP_IQS9151_BLOCK_BYTES];
    int r = b->hal.read(b->hal.context, BP_IQS9151_REG_BLOCK, raw, sizeof(raw), w);
    if (r) return r;
    bp_iqs9151_result d = bp_iqs9151_decode(raw, sizeof(raw), b->x_res, b->y_res, out);
    if (d == BP_IQS9151_COMMS) return -EAGAIN; /* window was missed; not a fault by itself */
    memcpy(b->last_block, raw, sizeof(raw));
    b->last_block_ms = w->ready_ms;
    if (d != BP_IQS9151_OK) return -EBADMSG;
    for (unsigned k = b->max_touches; k < BP_IQS9151_MAX_FINGERS; k++) out->finger[k].valid = false;
    b->last_info = out->info;
    b->last_tp = out->tp;
    b->ati_error = (out->info & BP_IQS9151_INFO_ATI_ERROR) != 0; /* also after an automatic re-ATI */
    if (out->info & BP_IQS9151_INFO_SHOW_RESET) {
        b->initialized = false;
        return -ECONNRESET;
    }
    ++b->sequence;
    return 0;
}

int bp_iqs9151_bus_write_verify(bp_iqs9151_bus *b, uint16_t reg, const uint8_t *data, size_t n) {
    if (!b || !data || !n || n > 32u) return -EINVAL;
    if (!b->initialized) return -EACCES;
    int r = write_reg(b, reg, data, n);
    if (r) return r;
    uint8_t got[32];
    r = transact(b, false, reg, got, n);
    if (r) return r;
    if (memcmp(got, data, n)) return -EIO;
    if (reg <= BP_IQS9151_REG_TP_SETTINGS && reg + n > BP_IQS9151_REG_TP_SETTINGS)
        b->tp_settings = data[BP_IQS9151_REG_TP_SETTINGS - reg];
    if (reg <= BP_IQS9151_REG_MAX_TOUCHES && reg + n > BP_IQS9151_REG_MAX_TOUCHES) {
        uint8_t m = data[BP_IQS9151_REG_MAX_TOUCHES - reg];
        b->max_touches = m > BP_IQS9151_MAX_FINGERS ? BP_IQS9151_MAX_FINGERS : m;
    }
    return 0;
}

void bp_iqs9151_bus_cancel(bp_iqs9151_bus *b) {
    if (b) b->initialized = false;
}

