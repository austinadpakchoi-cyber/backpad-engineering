#include "bp_experiment_flags.h"
/* BackPad board-specific offline-built BLE HID test application. */
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/settings/settings.h>
#include "bp_bond_button.h"
#include <bluetooth/services/hids.h>
#include <errno.h>
#include <stdint.h>
#include "bp_channel.h"
#include "bp_sensor_service.h"
#include "bp_status.h"
#include "bp_tx_timeout.h"
#include <zephyr/drivers/gpio.h>
#include <hal/nrf_power.h>
#include <nrfx.h>
#include "build_id.h"
#include "bp_hardware_class.h"
BUILD_ASSERT(CONFIG_BT_ATT_TX_COUNT >= BP_CHANNEL_CREDITS + 2u, "HID credits need ATT headroom");
volatile bp_test_status backpad_status = {.magic=0x42505431, .version=1, .phase=1};
/* Main-thread working state is never the debugger's in-progress snapshot. */
static bp_test_status status = {.magic=0x42505431, .version=1, .phase=1};
static uint64_t status_due;
static void publish_status(bool force) {
    uint64_t now=(uint64_t)k_uptime_get();
    status.hfclkstat=NRF_CLOCK->HFCLKSTAT; status.lfclkstat=NRF_CLOCK->LFCLKSTAT;
    status.dcdcen=NRF_POWER->DCDCEN; status.dcdcen0=NRF_POWER->DCDCEN0;
    status.uptime_ms=(uint32_t)now;
    (void)bp_status_publish_due(&backpad_status,&status,now,&status_due,force);
}
const char backpad_build_sha[] = BP_BUILD_SHA;
static const struct gpio_dt_spec led=GPIO_DT_SPEC_GET(DT_ALIAS(led0),gpios);
static const struct gpio_dt_spec bond_button=GPIO_DT_SPEC_GET(DT_ALIAS(sw0),gpios);
/* Independent ABI, main thread publishes below; settings errors are visible. */
volatile uint32_t backpad_bond_diag[12];
static bp_bond_button bond_button_state;
static bool bond_button_ok, bond_clear_pending, bond_link_recycled=true;
static uint32_t bonds_loaded, bond_count, pairing_complete_count, pairing_failure_count;
static uint32_t bond_clear_requests, bond_clear_completed;
static int bond_error;

BT_HIDS_DEF(hids, BP_HID_PAYLOAD_SIZE);
enum event_type { EV_CONNECTED,EV_DISCONNECTED,EV_SECURITY,EV_CCCD,EV_MODE,EV_SUSPEND,EV_COMPLETE,EV_RECYCLED,EV_PARAMS,EV_BONDED,EV_PAIR_FAILED };
struct event {
    enum event_type type;
    struct bt_conn *conn;
    uint32_t epoch;
    uint32_t token;
    uint64_t time_ms;
    uint16_t interval,latency,timeout;
    bool flag;
};
K_MSGQ_DEFINE(events,sizeof(struct event),16,4);
static atomic_t fatal_queue,callback_epoch;
static bp_pointer pointer;
/* Seqlocked live diagnostic counters. No changes to touch capture ABI. */
volatile uint32_t backpad_gesture_diag[23];
static bp_channel channel;
static struct bt_conn *peer;
static bool started,closing;
static uint32_t params_requested_epoch;
static uint64_t params_requested_at;
static unsigned params_attempts;
static bool params_applied,params_unconfirmed_reported;
static void post(enum event_type type,struct bt_conn *conn,bool flag,uint32_t epoch) {
    struct event e={.type=type,.conn=conn?bt_conn_ref(conn):NULL,.flag=flag,.epoch=epoch};
    if(k_msgq_put(&events,&e,K_NO_WAIT)!=0) {
        if(e.conn) bt_conn_unref(e.conn);
        atomic_set(&fatal_queue,1);
    }
}
static uint32_t current_epoch(void) { return (uint32_t)atomic_get(&callback_epoch); }
static void connected(struct bt_conn *conn,uint8_t err) {
    if(err) { printk("BackPad Test connection failure %u\n",err); return; }
    /* Stop before signed atomic counter exhaustion. */
    if(current_epoch()>=INT32_MAX) { atomic_set(&fatal_queue,1); (void)bt_conn_disconnect(conn,BT_HCI_ERR_REMOTE_USER_TERM_CONN); return; }
    atomic_inc(&callback_epoch);
    post(EV_CONNECTED,conn,true,current_epoch());
}
static void disconnected(struct bt_conn *conn,uint8_t reason) {
    printk("BackPad Test disconnected epoch=%u ms=%u reason=%u\n",
        current_epoch(),(unsigned)k_uptime_get(),reason);
    post(EV_DISCONNECTED,conn,false,current_epoch());
}
static void security_changed(struct bt_conn *conn,bt_security_t level,enum bt_security_err err) {
    printk("BackPad Test security level=%u error=%u\n",(unsigned)level,(unsigned)err);
    post(EV_SECURITY,conn,err==BT_SECURITY_ERR_SUCCESS && level>=BT_SECURITY_L2,current_epoch());
}
static void conn_params_updated(struct bt_conn *conn,uint16_t interval,uint16_t latency,uint16_t timeout) {
    /* Units: interval 1.25ms, timeout 10ms. Log negotiated values, not preferences. */
    printk("BackPad Test conn_params epoch=%u ms=%u interval_units=%u latency=%u timeout_units=%u\n",
        current_epoch(),(unsigned)k_uptime_get(),interval,latency,timeout);
    struct event e={.type=EV_PARAMS,.conn=bt_conn_ref(conn),.epoch=current_epoch(),
        .time_ms=(uint64_t)k_uptime_get(),.interval=interval,.latency=latency,.timeout=timeout};
    if(k_msgq_put(&events,&e,K_NO_WAIT)!=0) {
        bt_conn_unref(e.conn); atomic_set(&fatal_queue,1);
    }
}
static void recycled(void) { post(EV_RECYCLED,NULL,false,0); }
BT_CONN_CB_DEFINE(connection_callbacks)={.connected=connected,.disconnected=disconnected,.security_changed=security_changed,.recycled=recycled,.le_param_updated=conn_params_updated};
static void pairing_complete(struct bt_conn *conn, bool bonded) {
    printk("BackPad pairing complete bonded=%u\n",bonded);
    post(EV_BONDED,conn,bonded,current_epoch());
}
static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason) {
    printk("BackPad pairing failed reason=%u\n",(unsigned)reason);
    post(EV_PAIR_FAILED,conn,false,current_epoch());
}
static struct bt_conn_auth_info_cb auth_info={.pairing_complete=pairing_complete,.pairing_failed=pairing_failed};
static void cccd(enum bt_hids_notify_evt event) {
    post(EV_CCCD,NULL,event==BT_HIDS_CCCD_EVT_NOTIFY_ENABLED,current_epoch());
}
static void protocol(enum bt_hids_pm_evt event,struct bt_conn *conn) {
    post(EV_MODE,conn,event==BT_HIDS_PM_EVT_REPORT_MODE_ENTERED,current_epoch());
}
static void control(enum bt_hids_cp_evt event,struct bt_conn *conn) {
    if(event==BT_HIDS_CP_EVT_HOST_SUSP || event==BT_HIDS_CP_EVT_HOST_EXIT_SUSP)
        post(EV_SUSPEND,conn,event==BT_HIDS_CP_EVT_HOST_SUSP,current_epoch());
}
static void complete(struct bt_conn *conn,void *userdata) {
    /* Capture only immutable scalar identity. Never dereference reused slots or
     * infer the epoch from callback_epoch: this callback may belong to old BLE. */
    (void)conn;
    struct event e={.type=EV_COMPLETE,.token=(uint32_t)(uintptr_t)userdata};
    if(k_msgq_put(&events,&e,K_NO_WAIT)!=0) atomic_set(&fatal_queue,1);
}
static int submit(void *context,const uint8_t bytes[4],uint32_t epoch,uint32_t token) {
    (void)context;
    if(epoch!=channel.epoch) return -ENOTCONN;
    if(!peer || closing) return -ENOTCONN;
    bp_channel_stamp(&channel,token,(uint64_t)k_uptime_get());
    /* SDK stores readable buttons before notify_cb. Thus ANY returned error
     * is conservative UNKNOWN; only our pre-submit credit gate returns BUSY. */
    return bt_hids_inp_rep_send_userdata(&hids,peer,0,bytes,4,complete,(void *)(uintptr_t)token);
}
static void send_conn_params(bool fallback) {
    struct bt_le_conn_param preferred={
        .interval_min=fallback?12u:CONFIG_BT_PERIPHERAL_PREF_MIN_INT,
        .interval_max=CONFIG_BT_PERIPHERAL_PREF_MAX_INT,
        .latency=CONFIG_BT_PERIPHERAL_PREF_LATENCY,
        .timeout=CONFIG_BT_PERIPHERAL_PREF_TIMEOUT,
    };
    ++params_attempts; params_requested_at=(uint64_t)k_uptime_get();
    params_unconfirmed_reported=false;
    int err=bt_conn_le_param_update(peer,&preferred);
    /* Zero is local scheduling acceptance, possibly deferred until GAP's
     * timer. The public callback has actual values but no rejection status. */
    printk("BackPad Test conn_param_request epoch=%u ms=%u attempt=%u return=%d min_units=%u max_units=%u latency=%u timeout_units=%u remote_result=unknown\n",
        channel.epoch,(unsigned)params_requested_at,params_attempts,err,preferred.interval_min,
        preferred.interval_max,preferred.latency,preferred.timeout);
    if(!fallback && (err==-EINVAL || err==-ENOTSUP)) {
        printk("BackPad Test conn_param_fallback epoch=%u trigger=local_refusal return=%d\n",channel.epoch,err);
        send_conn_params(true);
    }
}
static void request_conn_params(void) {
    if(!peer || closing || params_requested_epoch==channel.epoch) return;
    params_requested_epoch=channel.epoch;
    params_attempts=0u; params_applied=false; params_unconfirmed_reported=false;
    send_conn_params(false);
}
static void observe_conn_params(const struct event *e) {
    if(params_requested_epoch!=channel.epoch || !params_attempts || params_applied ||
            e->time_ms<params_requested_at) return;
    unsigned minimum=params_attempts>1u?12u:CONFIG_BT_PERIPHERAL_PREF_MIN_INT;
    if(e->interval>=minimum && e->interval<=CONFIG_BT_PERIPHERAL_PREF_MAX_INT &&
            e->latency==CONFIG_BT_PERIPHERAL_PREF_LATENCY &&
            e->timeout==CONFIG_BT_PERIPHERAL_PREF_TIMEOUT) {
        params_applied=true;
        printk("BackPad Test conn_param_observed epoch=%u attempt=%u target_applied=1 interval_units=%u latency=%u timeout_units=%u\n",
            channel.epoch,params_attempts,e->interval,e->latency,e->timeout);
    } else if(params_attempts==1u && e->time_ms-params_requested_at>=CONFIG_BT_CONN_PARAM_UPDATE_TIMEOUT) {
        /* A real late callback outside target permits one conservative range
         * fallback. It does not prove the Mac rejected our original request.
         * Early/queued negotiation callbacks cannot trigger this fallback. */
        printk("BackPad Test conn_param_fallback epoch=%u trigger=actual_outside_target remote_rejection=unknown\n",channel.epoch);
        send_conn_params(true);
    }
}
static void poll_conn_params(uint64_t now) {
    if(params_requested_epoch!=channel.epoch || !params_attempts || params_applied ||
            params_unconfirmed_reported || now<params_requested_at ||
            now-params_requested_at<12000u) return;
    params_unconfirmed_reported=true;
    printk("BackPad Test conn_param_unconfirmed epoch=%u attempt=%u remote_rejection=unknown no_retry=1\n",
        channel.epoch,params_attempts);
}
static void stop_link(uint64_t now) {
    channel.state.connected=false;
    status.phase=4;
    (void)bp_disconnect(&pointer,now);
    if(peer && !closing) {
        closing=true;
        int err=bt_conn_disconnect(peer,BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        printk("BackPad Test stop_link disconnect=%d\n",err);
        /* The stack can already be DISCONNECTED while EV_DISCONNECTED is
         * queued behind SECURITY/MODE events. ENOTCONN is an idempotent close:
         * retain peer until that callback cleans up HIDS and its reference.
         * EV_RECYCLED will then restart advertising. Never kill the main loop
         * for this normal asynchronous ordering. */
        if(err && err!=-ENOTCONN) {
            status.last_error=err; atomic_set(&fatal_queue,1);
        }
    }
}
static const struct bt_data ad[]={
        BT_DATA_BYTES(BT_DATA_FLAGS,BT_LE_AD_GENERAL|BT_LE_AD_NO_BREDR),
        BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,CONFIG_BT_DEVICE_APPEARANCE & 0xff,CONFIG_BT_DEVICE_APPEARANCE >> 8),
        BT_DATA_BYTES(BT_DATA_UUID16_ALL,BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL))
};
static const struct bt_data sd[]={BT_DATA(BT_DATA_NAME_COMPLETE,CONFIG_BT_DEVICE_NAME,sizeof(CONFIG_BT_DEVICE_NAME)-1)};
static int advertise(void) {
    return bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,ad,ARRAY_SIZE(ad),sd,ARRAY_SIZE(sd));
}
static void count_bond(const struct bt_bond_info *info, void *user) {
    (void)info; ++*(uint32_t *)user;
}
static void refresh_bonds(void) {
    uint32_t n=0; bt_foreach_bond(BT_ID_DEFAULT,count_bond,&n); bond_count=n;
}
static int start_bluetooth(void) {
    int err=bt_conn_auth_info_cb_register(&auth_info);
    if(!err) err=bt_enable(NULL);
    if(!err) err=settings_load();
    if(err) { bond_error=err; return err; }
    bonds_loaded=1u; refresh_bonds();
    printk("BackPad settings loaded bonds=%u capacity=%u\n",bond_count,CONFIG_BT_MAX_PAIRED);
    return advertise();
}
static void refresh_subscription(void) {
    if(!peer || closing) return;
    const struct bt_gatt_attr *report=&hids.gp.svc.attrs[hids.inp_rep_group.reports[0].att_ind];
    /* Bond restore may not produce a fresh CCC write callback. Query this peer. */
    channel.state.subscribed=bt_gatt_is_subscribed(peer,report,BT_GATT_CCC_NOTIFY);
}
static void poll_bond_button(uint64_t now) {
    if(bond_button_ok && bp_bond_button_poll(&bond_button_state,gpio_pin_get_dt(&bond_button),now)) {
        ++bond_clear_requests; bond_clear_pending=true;
        (void)bt_le_adv_stop();
        if(peer) stop_link(now);
    }
    if(bond_clear_pending && peer && !closing) stop_link(now);
    if(bond_clear_pending && !peer && bond_link_recycled) {
        /* No live encryption/CCC writes race the explicit physical deletion. */
        bond_error=bt_unpair(BT_ID_DEFAULT,BT_ADDR_LE_ANY);
        if(!bond_error) ++bond_clear_completed;
        refresh_bonds(); bond_clear_pending=false;
        printk("BackPad physical bond clear result=%d remaining=%u\n",bond_error,bond_count);
        int result=advertise();
        if(result && result!=-EALREADY) { status.last_error=result; status.phase=255; }
        else status.phase=3;
    }
    ++backpad_bond_diag[0]; backpad_bond_diag[1]=1u;
    backpad_bond_diag[2]=bonds_loaded; backpad_bond_diag[3]=bond_count;
    backpad_bond_diag[4]=pairing_complete_count; backpad_bond_diag[5]=pairing_failure_count;
    backpad_bond_diag[6]=bond_clear_requests; backpad_bond_diag[7]=bond_clear_completed;
    backpad_bond_diag[8]=(uint32_t)bond_error; backpad_bond_diag[9]=bond_clear_pending;
    backpad_bond_diag[10]=bond_button_state.held; backpad_bond_diag[11]=bond_button_ok;
    ++backpad_bond_diag[0];
}
int main(void) {
    status.device_id[0]=NRF_FICR->DEVICEID[0];
    status.device_id[1]=NRF_FICR->DEVICEID[1];
    status.reset_reason=NRF_POWER->RESETREAS; /* read only */
    publish_status(true); /* actual identity remains readable during startup */
    (void)gpio_pin_configure_dt(&led,GPIO_OUTPUT_INACTIVE);
    bond_button_ok=gpio_is_ready_dt(&bond_button) && gpio_pin_configure_dt(&bond_button,GPIO_INPUT)==0;
    printk("BackPad Test build=%s dev=%08x/%08x\n",backpad_build_sha,
        status.device_id[0],status.device_id[1]);
    if(!bp_hardware_class_supported(NRF_FICR->INFO.PART, NRF_FICR->INFO.PACKAGE,
            NRF_FICR->INFO.RAM, NRF_FICR->INFO.FLASH)) {
        status.last_error=-ENODEV; status.phase=255; publish_status(true);
        printk("BackPad Test unsupported hardware class; radio not started\n");
        return 1;
    }
    bp_sensor_start(); /* acquisition independent of radio startup/readiness */
    status.phase=2; publish_status(true); /* visible even if bt_enable blocks on HFXO */
    bp_channel_init(&channel,submit,NULL);
    bp_config config=bp_default_config(); config.scroll_enabled=true;
    /* IQS9151 V2 touchpad, measured on the tuning page (board flat, FPC at the bottom-left): with
     * rotation 0 a stroke to the right moved the cursor down and a stroke up moved it right, so the
     * desk mounting is rotation 270. On the phone back, seen from the screen, the FPC is top-left:
     * the same plus a Y mirror, selected at run time by backpad_iqs_params.orient bit0 (sensor X
     * mirror, which rotation 270 turns into a screen Y mirror); since v0.20.8 it is the boot default
     * (bp_sensor_service.c, BP_TOUCH_PARAMS_PRODUCTION). Chip flip/switch bits stay off. */
    config.rotation=270u;
    config.invert_y=false;
    config.two_finger_tap=true;
    config.hold_drag=BP_EXP_HOLD_DRAG;config.guarded_hold=BP_EXP_HOLD_DRAG;config.hold_ms=700u;config.release_debounce=BP_EXP_RELEASE_DEBOUNCE;config.scroll_acceleration=BP_EXP_SCROLL_ACCELERATION;
    config.motion_smoothing=true;config.responsive_startup=true;config.high_resolution=true;config.tap_guard=BP_EXP_TAP_GUARD;config.tap_ms=400u;config.tap_slop=1500u; /* v0.15: a real v0.13 tap lasted 359 ms + lift confirm (Codex) */ /* IQS9151 raw-capture replay: 2 of 9 real taps lasted ~300 ms (+30..40 ms of landing settle and lift confirm); slop stays 1.5 mm: 3 mm swallows the first 3 mm of every quick stroke */
    config.scroll_join_ms=120u;config.pair_release_ms=150u;config.spike_guard=BP_EXP_SPIKE_GUARD;config.stationary_guard=BP_EXP_STATIONARY_GUARD;
    config.soft_low_speed=false;
    config.motion_median=BP_EXP_MOTION_MEDIAN;config.travel_still=BP_EXP_TRAVEL_STILL;config.micro_resume=BP_EXP_MICRO_RESUME;
    config.purge_motion_on_zero=BP_EXP_PURGE_MOTION_ON_ZERO;
    config.primary_confidence_gate=BP_EXP_PRIMARY_CONFIDENCE;
    config.motion_confidence_gate=BP_EXP_MOTION_CONFIDENCE;
    config.scroll_repair=BP_EXP_SCROLL_REPAIR;config.pair_pause_30=BP_EXP_PAIR_PAUSE_30;
    config.tap_drag=BP_EXP_TAP_DRAG;config.drag_lock=BP_EXP_DRAG_LOCK;
    config.scroll_latch=BP_EXP_SCROLL_LATCH;config.continuous_gain=BP_EXP_CONTINUOUS_GAIN;
    config.zero_reverse_deadband=BP_EXP_ZERO_REVERSE_DEADBAND;config.fixed_motion_gain=BP_EXP_FIXED_MOTION_GAIN;
    config.adaptive_motion_gain=true;
    config.absolute_spike=BP_EXP_ABSOLUTE_SPIKE;config.precise_stationary=BP_EXP_PRECISE_STATIONARY;
    /* IQS9151: position noise is handled in bp_touch_policy (One Euro + string). With the core's stationary
     * lock on top (150 um radius, 400 um unlock, no catch-up) a deliberate 1 mm move was swallowed in the
     * raw-capture replay (-24 HID, ideal ~104); without it 104 HID, and the held finger's remaining travel
     * is ~90 % its own slow drift (back-and-forth 24 HID over 3 x 1.4 s). */
    config.stationary_guard=false;config.precise_stationary=false;
    /* A fresh touch that moves ~1 mm inside tap_ms was a tap candidate whose motion was then discarded, plus a
     * 0.5 mm start deadzone: a re-landing 1 mm correction produced 0 HID (Codex replay; clean test 15b). */
    /* v0.15: repay only an evidenced trajectory from its own settled start. v0.13 repaid from the landing
     * point, which replayed the landing wander as a cursor jump on every press. */
    config.tap_catch_up=true;
    /* User design: quick tap = left click, two taps = two clicks, press and hold still = left button down,
     * then moving drags / selects text, lifting releases. 600 ms, within 0.4 mm of the landing point. */
    config.hold_drag=true;config.guarded_hold=true;config.hold_without_lock=true;config.hold_ms=600u;
    config.scroll_acceleration=false; /* v0.20.7: off again (user 08:50, Magic Trackpad comparison: a push scrolled 5489 px vs 243 px and jumped up to 464 px per event). v0.20.6: up to 3x detents per mm above 20 mm/s (BP_SCROLL_ACCEL_*); macOS only accelerates by event rate, so a quick push started with 4 px detents (user 08:30: 'not sensitive') */
    config.scroll_incremental=true; /* v0.20.4: the ATI-250 second finger is often weak or dropped; see bp_config */
    config.coalesce_motion=true; /* v0.16: ~150 sensor frames/s over a 7.5 ms link filled the 16-deep queue and cancelled mid-stroke */
    /* v0.18: the IQS9151 policy already filters positions (One Euro + string). The core's second position filter
     * only added lag: after a reversal it kept emitting the old direction for ~80 ms (Codex v0.17 review). */
    config.motion_smoothing=true;
    config.wait_zero_recover_ms=250u; /* v0.17: a cancel no longer freezes the cursor until the finger lifts */
    config.confirmed_secondary=BP_EXP_CONFIRMED_SECONDARY;config.median_tap=BP_EXP_MEDIAN_TAP;config.ignore_weak_secondary=BP_EXP_IGNORE_WEAK_SECONDARY;
    config.short_tap=BP_EXP_SHORT_TAP;config.extended_drag_window=BP_EXP_EXTENDED_DRAG_WINDOW;
    config.scroll_preconfirm_pause=BP_EXP_SCROLL_PRECONFIRM_PAUSE;config.scroll_report_cap=BP_EXP_SCROLL_REPORT_CAP;
    config.scroll_step=2000u; /* v0.20.7: 2 mm, tuned live on the board against the Magic Trackpad (141/-176 px per push vs ~200). v0.20.4 (1 mm): macOS turns one detent into ~4 px and accelerates by event rate, so 2 mm per detent scrolled a 12 mm push only a few lines; 1 mm adds no false ticks in the two-finger rest simulation or the real ATI-250 holds */
    config.gain_q8=5120u;config.pointer_acceleration=false; /* v0.20.7: 20 counts/mm (was BP_HIGH_RES_GAIN_Q8 = 40), tuned live: the same back-and-forth gave 5285 px vs the Magic Trackpad's 3979 (user: 'cursor nearly perfect') */ /* first product requirement: tap/click/scroll */
    if(bp_init(&pointer,&config,bp_channel_send,&channel)!=BP_OK) { status.last_error=-EINVAL; status.phase=255; publish_status(true); return 1; }
    static const uint8_t mask[]={1}; /* retain buttons, never replay relative deltas on reads */
    struct bt_hids_init_param param={0};
    param.rep_map.data=bp_hid_report_map; param.rep_map.size=(uint16_t)bp_hid_report_map_size;
    param.info.bcd_hid=0x0111; param.info.flags=BT_HIDS_NORMALLY_CONNECTABLE;
    param.inp_rep_group_init.cnt=1;
    param.inp_rep_group_init.reports[0].id=BP_HID_REPORT_ID;
    param.inp_rep_group_init.reports[0].size=BP_HID_PAYLOAD_SIZE;
    param.inp_rep_group_init.reports[0].rep_mask=mask;
    param.inp_rep_group_init.reports[0].handler=cccd;
    param.pm_evt_handler=protocol; param.conn_cp_evt_handler=control;
    /* No Boot Mouse characteristic: this application supports report protocol. */
    int err=bt_hids_init(&hids,&param);
    if(err || (err=start_bluetooth())) {
        status.last_error=err; status.phase=255; publish_status(true);
        printk("BackPad Test init error=%d\n",err); return 1; }
    status.phase=3; publish_status(true);
    for(;;) {
        struct event e;
        bool received=k_msgq_get(&events,&e,K_MSEC(10))==0;
        uint64_t now=(uint64_t)k_uptime_get();
        if(atomic_get(&fatal_queue)) {
            printk("BackPad Test event overflow/exhaustion; halted until restart\n"); stop_link(now);
            if(!status.last_error) status.last_error=-ENOBUFS;
            status.phase=255; publish_status(true);
            if(received && e.conn) bt_conn_unref(e.conn);
            status.bt_disable_error=bt_disable();
            while(k_msgq_get(&events,&e,K_NO_WAIT)==0) if(e.conn) bt_conn_unref(e.conn);
            if(peer) { bt_conn_unref(peer); peer=NULL; }
            publish_status(true);
            return 1;
        }
        if(received) {
            if(e.type==EV_CONNECTED && !peer) {
                peer=bt_conn_ref(e.conn); bond_link_recycled=false; closing=false; started=false; status.phase=4;
                if(!bp_channel_open(&channel,e.epoch)) {
                    printk("BackPad Test channel open refused epoch=%u\n",e.epoch);
                    stop_link(now);
                } else {
                    int setup_error=bt_hids_connected(&hids,peer);
                    printk("BackPad Test HIDS connected=%d\n",setup_error);
                    if(setup_error) stop_link(now);
                    else {
                        setup_error=bt_conn_set_security(peer,BT_SECURITY_L2);
                        printk("BackPad Test security request=%d\n",setup_error);
                        if(setup_error) stop_link(now);
                    }
                }
                printk("BackPad Test connection epoch=%u ms=%u\n",e.epoch,(unsigned)now);
                struct bt_conn_info info;
                if(!bt_conn_get_info(peer,&info) && info.type==BT_CONN_TYPE_LE)
                    conn_params_updated(peer,info.le.interval,info.le.latency,info.le.timeout);
            } else if(e.type==EV_DISCONNECTED && e.conn==peer) {
                (void)bt_hids_disconnected(&hids,peer); (void)bp_disconnect(&pointer,now);
                channel.state.connected=false; bt_conn_unref(peer); peer=NULL; started=false; closing=false;
                /* Wait for recycled: advertising needs a free connection object. */
            } else if(e.type==EV_COMPLETE) {
                bp_channel_complete(&channel,e.token);
            } else if(e.epoch==channel.epoch && (!e.conn || e.conn==peer) && !closing) {
                switch(e.type) {
                case EV_BONDED: if(e.flag) { ++pairing_complete_count; refresh_bonds(); } break;
                case EV_PAIR_FAILED: ++pairing_failure_count; break;
                case EV_SECURITY:
                    channel.state.encrypted=e.flag;
                    if(!e.flag) stop_link(now); else request_conn_params();
                    break;
                case EV_PARAMS: observe_conn_params(&e); break;
                case EV_CCCD: channel.state.subscribed=e.flag; break;
                case EV_MODE: channel.state.report_mode=e.flag; break;
                case EV_SUSPEND: channel.state.suspended=e.flag; break;
                default: break;
                }
            }
            if(e.conn) bt_conn_unref(e.conn);
            if(e.type==EV_RECYCLED) bond_link_recycled=true;
            if(e.type==EV_RECYCLED && !peer && !bond_clear_pending) { err=advertise();
                status.last_error=err; status.phase=err?255:3; publish_status(true);
                printk("BackPad Test advertise=%d\n",err); }
        }
        poll_bond_button(now);
        refresh_subscription();
        bp_sample sample;
        for(unsigned drain=0;drain<16u && bp_sensor_take(&sample);++drain) {
            now=(uint64_t)k_uptime_get();
            if(sample.kind==BP_DATA) {
                status.sensor_session=sample.session;status.sensor_sequence=sample.sequence;
                status.sensor_count=sample.count;
                status.sensor_x=sample.count?sample.contacts[0].x:0;
                status.sensor_y=sample.count?sample.contacts[0].y:0;
            }
            if(started && peer && !closing) (void)bp_process(&pointer,&sample,now);
        }
        bp_sensor_health sensor_health;bp_sensor_health_get(&sensor_health);
        status.sensor_frames=sensor_health.frames;status.sensor_no_data=sensor_health.no_data;
        status.sensor_faults=sensor_health.faults;status.sensor_init_calls=sensor_health.init_calls;
        if(peer && !closing) {
            poll_conn_params(now);
            if(started && (!bp_channel_ready(&channel) || !pointer.online)) {
                printk("BackPad Test stop: readiness encrypted=%u subscribed=%u report=%u suspended=%u failed=%u pointer=%u\n",
                       channel.state.encrypted,channel.state.subscribed,channel.state.report_mode,
                       channel.state.suspended,channel.failed,pointer.online);
                stop_link(now);
            }
            else if(!started && bp_channel_ready(&channel)) {
                (void)bp_connect(&pointer,channel.epoch,now); started=true; status.phase=5;
                if(started) printk("BackPad Test ready; release then fresh zero required\n");
            }
            if(started && !closing) {
                (void)bp_tick(&pointer,now);
                if(!closing) (void)bp_flush(&pointer,now);
                /* Each submission keeps its own timestamp; new traffic cannot
                 * postpone a stuck older report. Refresh after bp_flush. */
                uint64_t inflight_since=0u;
                bool waiting=bp_channel_oldest(&channel,&inflight_since);
                now=(uint64_t)k_uptime_get();
                if(!closing && !pointer.online) {
                    printk("BackPad Test stop: pointer offline tx_errors=%u\n",channel.submission_errors);
                    stop_link(now);
                } else if(!closing && bp_tx_expired(waiting,now,inflight_since)) {
                    printk("BackPad Test stop: HID completion timeout age=%llu outstanding=%u accepted=%u completed=%u epoch=%u\n",
                           (unsigned long long)(now-inflight_since),channel.outstanding,channel.accepted,channel.completed,channel.epoch);
                    stop_link(now);
                }
            }
        }
        status.epoch=channel.epoch;
        status.connected=channel.state.connected;
        status.encrypted=channel.state.encrypted;
        status.subscribed=channel.state.subscribed;
        status.report_mode=channel.state.report_mode;
        status.suspended=channel.state.suspended;
        status.pointer_mode=pointer.mode;
        status.tx_accepted=channel.accepted; status.tx_completed=channel.completed;
        status.tx_errors=channel.submission_errors; status.cancellations=pointer.cancellations;
        ++backpad_gesture_diag[0];
        backpad_gesture_diag[1]=3;backpad_gesture_diag[2]=pointer.tap_left;backpad_gesture_diag[3]=pointer.tap_right;
        for(unsigned i=0;i<6;i++)backpad_gesture_diag[4+i]=pointer.tap_reject[i];
        backpad_gesture_diag[10]=pointer.spike_rejected;backpad_gesture_diag[11]=pointer.spike_reanchors;
        backpad_gesture_diag[12]=pointer.motion_clipped;backpad_gesture_diag[13]=pointer.stationary_entries;
        backpad_gesture_diag[14]=pointer.tap_spikes;backpad_gesture_diag[15]=pointer.still.locked;
        backpad_gesture_diag[16]=pointer.last_cancel_line;backpad_gesture_diag[17]=pointer.primary_missing;
        backpad_gesture_diag[18]=pointer.tap_reject[6];
        backpad_gesture_diag[19]=pointer.release_bounces;backpad_gesture_diag[20]=pointer.pair_weak_frames;
        backpad_gesture_diag[21]=pointer.pair_late_frames;backpad_gesture_diag[22]=pointer.pair_unconfirmed;
        ++backpad_gesture_diag[0];
        publish_status(false);
        (void)gpio_pin_set_dt(&led,status.phase==5 ? 1 : ((now/500u)&1u));
    }
}

