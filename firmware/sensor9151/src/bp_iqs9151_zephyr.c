#include "bp_iqs9151_runtime.h"
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
BUILD_ASSERT(CONFIG_I2C_NRFX_TRANSFER_TIMEOUT > 0 && CONFIG_I2C_NRFX_TRANSFER_TIMEOUT <= 100,
             "IQS9151 requires a bounded nrfx I2C timeout <=100ms");
#if !DT_NODE_HAS_STATUS(DT_ALIAS(backpad_touch), okay)
#error "BackPad requires alias backpad-touch to its I2C controller"
#endif
static const struct device *const i2c = DEVICE_DT_GET(DT_ALIAS(backpad_touch));
static const struct gpio_dt_spec rdy = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), rdy_gpios);
static struct gpio_callback ready_callback;
static struct k_spinlock ready_lock;
K_SEM_DEFINE(ready_sem, 0, 1);
static uint32_t ready_epoch, consumed_epoch, force_requests;
static uint64_t ready_ms, last_force_ms;
static bool irq_installed, physical_low, edge_fault;
static bp_iqs9151_bus bus;
static uint8_t tx[2u + BP_IQS9151_MAX_SECTION];

#define BP_IQS9151_FORCE_AFTER_MS 50u
#define BP_IQS9151_WINDOW_MAX_AGE_MS 90u /* chip I2C timeout is 100 ms (evaluation settings) */

/* RDY is open-drain ACTIVE LOW. The raw pin level is read deliberately so a wrong DT polarity
 * cannot invert the protocol. A falling edge opens a new communication window. */
static void ready_changed(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(port); ARG_UNUSED(cb); ARG_UNUSED(pins);
    int level = gpio_pin_get_raw(rdy.port, rdy.pin);
    bool opened = false;
    k_spinlock_key_t key = k_spin_lock(&ready_lock);
    if (level < 0) edge_fault = true;
    else {
        if (!level && !physical_low) {
            if (ready_epoch == UINT32_MAX) edge_fault = true;
            else { ++ready_epoch; ready_ms = (uint64_t)k_uptime_get(); opened = true; }
        }
        physical_low = level == 0;
    }
    k_spin_unlock(&ready_lock, key);
    if (opened) k_sem_give(&ready_sem);
}

/* Outside a window the chip only accepts a comms request (Force Comms Method = 1): writing
 * 0xFF 0x00 then STOP, as the Azoteq example does. Rate-limited, only while RDY is high. */
static void maybe_force_request(uint64_t now) {
    if (now - ready_ms < BP_IQS9151_FORCE_AFTER_MS || now - last_force_ms < BP_IQS9151_FORCE_AFTER_MS) return;
    last_force_ms = now;
    static const uint8_t request[2] = {0xFFu, 0x00u};
    if (!i2c_write(i2c, request, sizeof(request), BP_IQS9151_ADDR)) ++force_requests;
}

static int take_window(bp_iqs9151_window *w) {
    int level = gpio_pin_get_raw(rdy.port, rdy.pin);
    if (level < 0) return level;
    uint64_t now = (uint64_t)k_uptime_get();
    k_spinlock_key_t key = k_spin_lock(&ready_lock);
    int ret = 0;
    if (edge_fault) ret = -EOVERFLOW;
    else if (level || !physical_low || ready_epoch == consumed_epoch) ret = -EAGAIN;
    else if (now < ready_ms || now - ready_ms > BP_IQS9151_WINDOW_MAX_AGE_MS) { consumed_epoch = ready_epoch; ret = -EAGAIN; }
    else { w->epoch = ready_epoch; w->ready_ms = ready_ms; consumed_epoch = ready_epoch; }
    k_spin_unlock(&ready_lock, key);
    if (ret == -EAGAIN && level) maybe_force_request(now);
    return ret;
}

static int hal_read(void *ctx, uint16_t reg, uint8_t *data, size_t n, bp_iqs9151_window *w) {
    ARG_UNUSED(ctx);
    int ret = take_window(w);
    if (ret) return ret;
    uint8_t addr[2] = {(uint8_t)reg, (uint8_t)(reg >> 8)}; /* low byte first */
    ret = i2c_write_read(i2c, BP_IQS9151_ADDR, addr, sizeof(addr), data, n);
    w->done_ms = (uint64_t)k_uptime_get();
    return ret;
}

static int hal_write(void *ctx, uint16_t reg, const uint8_t *data, size_t n, bp_iqs9151_window *w) {
    ARG_UNUSED(ctx);
    if (n > BP_IQS9151_MAX_SECTION) return -E2BIG;
    int ret = take_window(w);
    if (ret) return ret;
    tx[0] = (uint8_t)reg;
    tx[1] = (uint8_t)(reg >> 8);
    memcpy(tx + 2, data, n);
    ret = i2c_write(i2c, tx, n + 2u, BP_IQS9151_ADDR);
    w->done_ms = (uint64_t)k_uptime_get();
    return ret;
}

static uint64_t now_ms(void *ctx) { ARG_UNUSED(ctx); return (uint64_t)k_uptime_get(); }
static void wait_ms(void *ctx, unsigned ms) {
    ARG_UNUSED(ctx);
    /* Wake early on a new window instead of sleeping the full interval. */
    (void)k_sem_take(&ready_sem, K_MSEC(ms));
}

int bp_iqs9151_init(void) {
    bp_iqs9151_bus_cancel(&bus);
    if (!device_is_ready(i2c) || !gpio_is_ready_dt(&rdy)) return -ENODEV;
    if (!irq_installed) {
        int ret = gpio_pin_configure_dt(&rdy, GPIO_INPUT);
        if (ret) return ret;
        gpio_init_callback(&ready_callback, ready_changed, BIT(rdy.pin));
        ret = gpio_add_callback(rdy.port, &ready_callback);
        if (ret) return ret;
        int level = gpio_pin_get_raw(rdy.port, rdy.pin);
        if (level < 0) { gpio_remove_callback(rdy.port, &ready_callback); return level; }
        physical_low = level == 0;
        if (physical_low) { ready_epoch = 1u; ready_ms = (uint64_t)k_uptime_get(); }
        ret = gpio_pin_interrupt_configure_dt(&rdy, GPIO_INT_EDGE_BOTH);
        if (ret) { gpio_remove_callback(rdy.port, &ready_callback); return ret; }
        irq_installed = true;
    }
    k_spinlock_key_t key = k_spin_lock(&ready_lock);
    edge_fault = false;
    k_spin_unlock(&ready_lock, key);
    bp_iqs9151_hal hal = {NULL, hal_read, hal_write, now_ms, wait_ms};
    return bp_iqs9151_bus_init(&bus, &hal);
}

int bp_iqs9151_wait_ready(unsigned timeout_ms) {
    return k_sem_take(&ready_sem, K_MSEC(timeout_ms)) ? -EAGAIN : 0;
}
int bp_iqs9151_read(bp_iqs9151_frame *out, bp_iqs9151_window *w) { return bp_iqs9151_bus_read(&bus, out, w); }
int bp_iqs9151_write_verify(uint16_t reg, const uint8_t *data, size_t n) { return bp_iqs9151_bus_write_verify(&bus, reg, data, n); }
int bp_iqs9151_read_reg(uint16_t reg, uint8_t *data, size_t n) { return bp_iqs9151_bus_read_reg(&bus, reg, data, n); }
int bp_iqs9151_reati(void) { return bp_iqs9151_bus_reati(&bus); }
void bp_iqs9151_cancel(void) { bp_iqs9151_bus_cancel(&bus); }
const bp_iqs9151_bus *bp_iqs9151_bus_state(void) { return &bus; }
uint32_t bp_iqs9151_force_requests(void) { return force_requests; }
int bp_iqs9151_recover_bus(void) {
    bp_iqs9151_bus_cancel(&bus);
    if (!device_is_ready(i2c)) return -ENODEV;
    return i2c_recover_bus(i2c);
}

