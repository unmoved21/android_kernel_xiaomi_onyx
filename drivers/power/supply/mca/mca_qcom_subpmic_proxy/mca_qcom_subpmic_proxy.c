#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/ktime.h>
#include <linux/pm_wakeup.h>

#include <mca/common/mca_log.h>
#include <mca/common/mca_event.h>
#include <mca/common/mca_hwid.h>
#include <mca/common/mca_parse_dts.h>
#include <mca/common/mca_adsp_glink.h>
#include <mca/platform/platform_bc12_class.h>
#include <mca/platform/platform_buckchg_class.h>
#include <mca/platform/platform_cp_class.h>
#include <mca/protocol/protocol_class.h>
#include <mca/protocol/protocol_qc_class.h>
#include <mca/protocol/protocol_pd_class.h>
#include <mca/strategy/strategy_class.h>

#ifndef MCA_LOG_TAG
#define MCA_LOG_TAG "qcom_subpmic"
#endif

enum mca_subpmic_prop {
	SUBPMIC_PROP_SHIP_MODE = 5,
	SUBPMIC_PROP_SHUTDOWN = 6,
	SUBPMIC_PROP_TOO_HOT_LIMIT = 7,
	SUBPMIC_PROP_WLS_INPUT_CURR_LIMIT = 0x1003,
	SUBPMIC_PROP_WLS_INPUT_SUSPEND = 0x1004,
	SUBPMIC_PROP_WLS_CURR = 0x1007,
	SUBPMIC_PROP_WLS_VDD_FLAG = 0x100c,
	SUBPMIC_PROP_CHG_STATUS = 0x2001,
	SUBPMIC_PROP_CHG_TYPE = 0x2002,
	SUBPMIC_PROP_ENABLE_CHARGING = 0x2003,
	SUBPMIC_PROP_BUCK_FSW = 0x2004,
	SUBPMIC_PROP_CHARGE_CURRENT = 0x2005,
	SUBPMIC_PROP_TERM_CURRENT = 0x2006,
	SUBPMIC_PROP_TERM_VOLT = 0x2007,
	SUBPMIC_PROP_INPUT_CURR_LIMIT = 0x2008,
	SUBPMIC_PROP_INPUT_VOLT_LIMIT = 0x2009,
	SUBPMIC_PROP_PRECHG_CURRENT = 0x200a,
	SUBPMIC_PROP_PRECHG_VOLT = 0x200b,
	SUBPMIC_PROP_VSYS_VOLT = 0x200c,
	SUBPMIC_PROP_OTG_BOOST_EN_STATUS = 0x200d,
	SUBPMIC_PROP_OTG_GATE_EN_STATUS = 0x200e,
	SUBPMIC_PROP_BOOST_ENABLE = 0x200f,
	SUBPMIC_PROP_BOOST_VOLTAGE = 0x2010,
	SUBPMIC_PROP_QC_VOLT = 0x2011,
	SUBPMIC_PROP_AICL_ENABLE = 0x2012,
	SUBPMIC_PROP_RERUN_AICL = 0x2013,
	SUBPMIC_PROP_RESTART_AICL = 0x2014,
	SUBPMIC_PROP_USB_AICL_CONT_THD = 0x2015,
	SUBPMIC_PROP_OPT_FWS = 0x2016,
	SUBPMIC_PROP_USB_ADAP_OVERRIDE = 0x2017,
	SUBPMIC_PROP_QC3_VOLT = 0x2018,
	SUBPMIC_PROP_INPUT_SUSPEND = 0x2019,
	SUBPMIC_PROP_OTG_CFG = 0x201a,
	SUBPMIC_PROP_CID_CFG = 0x201b,
	SUBPMIC_PROP_CP_STATE = 0x201c,
	SUBPMIC_PROP_QC_VOLT_CMD = 0x201d,
	SUBPMIC_PROP_LPD_SBU1 = 0x201e,
	SUBPMIC_PROP_LPD_SBU2 = 0x201f,
	SUBPMIC_PROP_LPD_CC1 = 0x2020,
	SUBPMIC_PROP_LPD_CC2 = 0x2021,
	SUBPMIC_PROP_LPD_DP = 0x2022,
	SUBPMIC_PROP_LPD_DM = 0x2023,
	SUBPMIC_PROP_LPD_CONTROL = 0x2024,
	SUBPMIC_PROP_LPD_UART_CONTROL = 0x2025,
	SUBPMIC_PROP_PACK_VBAT = 0x2026,
	SUBPMIC_PROP_PACK_IBAT = 0x2027,
	SUBPMIC_PROP_EU_MODEL = 0x2028,
	SUBPMIC_PROP_AICL_STATUS = 0x2029,
	SUBPMIC_PROP_PACK_TBAT = 0x202a,
	SUBPMIC_PROP_CP_VENDOR_ID = 0x202c,
	SUBPMIC_PROP_USB_ONLINE = 0x20001,
	SUBPMIC_PROP_BUS_VOLT = 0x20002,
	SUBPMIC_PROP_USB_SNS_VOLT = 0x20003,
	SUBPMIC_PROP_BUS_CURR = 0x20004,
	SUBPMIC_PROP_LPD_ENABLE = 0x20009,
	SUBPMIC_PROP_LPD_STATUS = 0x2000a,
	SUBPMIC_PROP_USB_REAL_TYPE = 0x2000c,
};

enum mca_subpmic_notify {
	SUBPMIC_NOTIFY_USB_TYPE = 0,
	SUBPMIC_NOTIFY_CHARGE_TYPE = 2,
	SUBPMIC_NOTIFY_ENABLE_BOOST = 3,
	SUBPMIC_NOTIFY_LPD_STATUS = 4,
	SUBPMIC_NOTIFY_PLATE_SHOCK = 5,
	SUBPMIC_NOTIFY_CC_SHORT_VBUS = 6,
	SUBPMIC_NOTIFY_PPS_PTF = 7,
	SUBPMIC_NOTIFY_SINK_PWR_SUSPEND = 8,
	SUBPMIC_NOTIFY_CP_REVERT = 9,
	SUBPMIC_NOTIFY_OTG_CP_CONFIG = 10,
	SUBPMIC_NOTIFY_SBU_LPD = 11,
};

#define SUBPMIC_RAW_CODE_PLATE_SHOCK 0x3d
#define SUBPMIC_RAW_CODE_CHARGE_TYPE_2 0x06
#define SUBPMIC_RAW_CODE_GLINK_DOWN 0x2e
#define SUBPMIC_RAW_CODE_GLINK_UP 0x2f

#define SUBPMIC_QC_VOLT_MIN 5000
#define SUBPMIC_QC_VOLT_MAX 12000

struct qcom_subpmic_otg_cfg {
	u32 otg_boost_src;
	u32 gpio_chip_type;
	u32 vdd_boost_en;
	u32 otg_ovp_en;
};

struct qcom_subpmic {
	struct device *dev;

	struct delayed_work update_usb_type_work;
	struct work_struct notify_change_work;
	struct delayed_work sync_cfg_work;

	struct notifier_block ship_mode_nb;
	struct notifier_block shutdown_nb;

	struct list_head notify_list;
	spinlock_t notify_lock;

	bool support_2s_charging;
	bool support_dual_panel;
	bool support_multi_bc12;
	bool support_cid;
	u32 cid_gpio_int;
	struct qcom_subpmic_otg_cfg
		otg_cfg;
	bool is_enable_shipmode;
	u8 eu_model;

	int online;
	int real_type;
	int glink_down;
	int init_ok;
	int lpd_status;
	int cc_short_vbus;
	int pps_ptf;
	u8 sink_pwr_suspend;
	int cp_vbus_revert;
	int sbu_lpd;
};

struct qcom_subpmic_notify_entry {
	struct list_head node;
	int type;
	int val;
};

static RAW_NOTIFIER_HEAD(hboost_notifier);

int register_hboost_event_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&hboost_notifier, nb);
}
EXPORT_SYMBOL(register_hboost_event_notifier);

int unregister_hboost_event_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_unregister(&hboost_notifier, nb);
}
EXPORT_SYMBOL(unregister_hboost_event_notifier);

int qti_battery_charger_get_prop(const char *name, int prop_id, int *val)
{
	if (!val)
		return -EINVAL;

	*val = (prop_id != 0) ? 0 : 100000;
	return 0;
}
EXPORT_SYMBOL(qti_battery_charger_get_prop);

static int qcom_subpmic_get_usb_real_type(int *type, void *data)
{
	struct qcom_subpmic *sc = data;
	int rc;

	rc = mca_adsp_glink_read_prop(SUBPMIC_PROP_USB_REAL_TYPE, type,
				      sizeof(*type));
	if (rc != 0)
		*type = sc->real_type;

	return 0;
}

static int qcom_subpmic_bc12_det_en(int en, void *data)
{
	return 0;
}

static int qcom_subpmic_set_qc_volt(void *data, int volt)
{
	struct qcom_subpmic *sc = data;
	ktime_t start;
	int volt_cmd = volt;
	int vbus = 0;
	int real_type;
	int step, target, delta;
	unsigned int settle_thd, step_count;
	int per_step_ms;
	int rc;

	start = ktime_get_boottime();
	real_type = sc->real_type;
	settle_thd = (real_type != XM_CHARGER_TYPE_HVDCP3P5) ? 200 : 20;
	per_step_ms = (real_type != XM_CHARGER_TYPE_HVDCP3P5) ? 50 : 5;

	if (volt < SUBPMIC_QC_VOLT_MIN || volt > SUBPMIC_QC_VOLT_MAX) {
		mca_log_err("invalid qc voltage: %d\n", volt);
		return -1;
	}

	step = (real_type != XM_CHARGER_TYPE_HVDCP3P5) ? 800 : 500;
	mca_adsp_glink_read_prop(SUBPMIC_PROP_BUS_VOLT, &vbus, sizeof(vbus));
	vbus /= 1000;
	delta = volt - vbus;
	target = volt + ((delta == 0 || volt < vbus) ? step : -step);
	mca_log_info("real_type: %d, volt: %d, vbus: %d\n", sc->real_type, volt,
		     vbus);

	rc = mca_adsp_glink_write_prop(SUBPMIC_PROP_QC_VOLT, &volt_cmd,
				       sizeof(volt_cmd));
	if (rc != 0 || sc->real_type == XM_CHARGER_TYPE_HVDCP2 ||
	    volt_cmd == 5000)
		return 0;

	if (abs(delta) < (int)settle_thd)
		return 0;

	step_count = settle_thd ? abs(delta) / settle_thd : 0;
	do {
		msleep(50);
		mca_adsp_glink_read_prop(SUBPMIC_PROP_BUS_VOLT, &vbus,
					 sizeof(vbus));
		vbus /= 1000;
		mca_log_info("vbus: %d\n", vbus);
		if (vbus < 4000 || sc->online == 0) {
			mca_log_err("usb removed, break\n");
			break;
		}
		if (((target < vbus && delta != 0) &&
		     (vbus <= target || delta >= 0)) ||
		    (vbus < target && delta < 0))
			break;
	} while (ktime_ms_delta(ktime_get_boottime(), start) <=
		 (s64)(step_count * per_step_ms));

	return 0;
}

static int qcom_subpmic_set_qc_volt_cmd(void *data, int cmd)
{
	int val = cmd;

	mca_log_info("set_qc_volt_cmd: %d\n", cmd);
	mca_adsp_glink_write_prop(SUBPMIC_PROP_QC_VOLT_CMD, &val, sizeof(val));
	return 0;
}

static int qcom_subpmic_get_bus_volt(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_BUS_VOLT, val,
					sizeof(*val));
}

static int qcom_subpmic_get_bus_curr(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_BUS_CURR, val,
					sizeof(*val));
}

static int qcom_subpmic_get_usb_sns_volt(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_USB_SNS_VOLT, val,
					sizeof(*val));
}

static int qcom_subpmic_get_vsys_volt(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_VSYS_VOLT, val,
					sizeof(*val));
}

static int qcom_subpmic_get_chg_status(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_CHG_STATUS, val,
					sizeof(*val));
}

static int qcom_subpmic_get_chg_type(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_CHG_TYPE, val,
					sizeof(*val));
}

static int qcom_subpmic_get_term_current(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_TERM_CURRENT, val,
					sizeof(*val));
}

static int qcom_subpmic_get_term_volt(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_TERM_VOLT, val,
					sizeof(*val));
}

static int qcom_subpmic_get_wls_curr(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_WLS_CURR, val,
					sizeof(*val));
}

static int qcom_subpmic_get_input_current_limit(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_INPUT_CURR_LIMIT, val,
					sizeof(*val));
}

static int qcom_subpmic_get_usb_aicl_cont_thd(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_USB_AICL_CONT_THD, val,
					sizeof(*val));
}

static int qcom_subpmic_get_lpd_enable(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_LPD_ENABLE, val,
					sizeof(*val));
}

static int qcom_subpmic_get_lpd_status(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_LPD_STATUS, val,
					sizeof(*val));
}

static int qcom_subpmic_get_lpd_sbu1(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_LPD_SBU1, val,
					sizeof(*val));
}

static int qcom_subpmic_get_lpd_sbu2(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_LPD_SBU2, val,
					sizeof(*val));
}

static int qcom_subpmic_get_lpd_cc1(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_LPD_CC1, val,
					sizeof(*val));
}

static int qcom_subpmic_get_lpd_cc2(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_LPD_CC2, val,
					sizeof(*val));
}

static int qcom_subpmic_get_lpd_dp(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_LPD_DP, val, sizeof(*val));
}

static int qcom_subpmic_get_lpd_dm(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_LPD_DM, val, sizeof(*val));
}

static int qcom_subpmic_get_lpd_control(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_LPD_CONTROL, val,
					sizeof(*val));
}

static int qcom_subpmic_get_lpd_uart_control(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_LPD_UART_CONTROL, val,
					sizeof(*val));
}

static int qcom_subpmic_get_pack_vbat(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_PACK_VBAT, val,
					sizeof(*val));
}

static int qcom_subpmic_get_pack_ibat(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_PACK_IBAT, val,
					sizeof(*val));
}

static int qcom_subpmic_get_pack_tbat(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_PACK_TBAT, val,
					sizeof(*val));
}

static int qcom_subpmic_get_aicl_status(void *data, int *val)
{
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_AICL_STATUS, val,
					sizeof(*val));
}

static int qcom_subpmic_get_otg_gate_enable_status(void *data, int *val)
{
	mca_log_info("get otg plugin status\n");
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_OTG_GATE_EN_STATUS, val,
					sizeof(*val));
}

static int qcom_subpmic_get_otg_boost_enable_status(void *data, int *val)
{
	mca_log_info("get otg enable status\n");
	return mca_adsp_glink_read_prop(SUBPMIC_PROP_OTG_BOOST_EN_STATUS, val,
					sizeof(*val));
}

static int qcom_subpmic_get_online(void *data, int *online)
{
	struct qcom_subpmic *sc = data;

	*online = sc->online;
	return 0;
}

static int qcom_subpmic_get_otg_boost_src(void *data, int *src)
{
	struct qcom_subpmic *sc = data;

	*src = sc->otg_cfg.otg_boost_src;
	mca_log_info("get otg_boost_src: %d\n", *src);
	return sc->otg_cfg.otg_boost_src;
}

static int qcom_subpmic_is_charge_done(void *data, bool *done)
{
	int status = 0;
	int rc;

	rc = mca_adsp_glink_read_prop(SUBPMIC_PROP_CHG_STATUS, &status,
				      sizeof(status));
	if (rc >= 0)
		*done = (status == 5);

	return 0;
}

static int qcom_subpmic_is_init_ok(void *data)
{
	struct qcom_subpmic *sc = data;

	if (!sc || sc->init_ok == 0)
		return -1;

	return sc->init_ok;
}

static int qcom_subpmic_is_support_cid(void *data, bool *en)
{
	struct qcom_subpmic *sc = data;

	*en = sc->support_cid;
	return 0;
}

static int qcom_subpmic_get_ship_mode(void *data, bool *en)
{
	struct qcom_subpmic *sc = data;

	*en = sc->is_enable_shipmode;
	return 0;
}

static int qcom_subpmic_set_input_suspend(void *data, bool en)
{
	u8 val = en;

	mca_log_info("set input suspend %d\n", en & 1);
	mca_adsp_glink_write_prop(SUBPMIC_PROP_INPUT_SUSPEND, &val,
				  sizeof(val));
	return 0;
}

static int qcom_subpmic_set_wireless_input_suspend(void *data, bool en)
{
	u8 val = en;

	mca_log_info("set wls input suspend %d\n", en & 1);
	mca_adsp_glink_write_prop(SUBPMIC_PROP_WLS_INPUT_SUSPEND, &val,
				  sizeof(val));
	return 0;
}

static int qcom_subpmic_set_input_current_limit(void *data, int ma)
{
	int val = ma;

	mca_adsp_glink_write_prop(SUBPMIC_PROP_INPUT_CURR_LIMIT, &val,
				  sizeof(val));
	return 0;
}

static int qcom_subpmic_set_wls_input_current_limit(void *data, int ma)
{
	int val = ma;

	mca_adsp_glink_write_prop(SUBPMIC_PROP_WLS_INPUT_CURR_LIMIT, &val,
				  sizeof(val));
	return 0;
}

static int qcom_subpmic_set_input_volt_limit(void *data, int mv)
{
	int val = mv;

	mca_adsp_glink_write_prop(SUBPMIC_PROP_INPUT_VOLT_LIMIT, &val,
				  sizeof(val));
	return 0;
}

static int qcom_subpmic_set_charge_current(void *data, int ma)
{
	int val = ma;

	mca_adsp_glink_write_prop(SUBPMIC_PROP_CHARGE_CURRENT, &val,
				  sizeof(val));
	return 0;
}

static int qcom_subpmic_enable_charging(void *data, bool en)
{
	u8 val = en;

	mca_log_info("enable charging: %d\n", en & 1);
	mca_adsp_glink_write_prop(SUBPMIC_PROP_ENABLE_CHARGING, &val,
				  sizeof(val));
	return 0;
}

static int qcom_subpmic_set_buck_fsw(void *data, int khz)
{
	int val = khz;

	mca_log_info("set buck fsw: %d\n", khz);
	mca_adsp_glink_write_prop(SUBPMIC_PROP_BUCK_FSW, &val, sizeof(val));
	return 0;
}

static int qcom_subpmic_set_term_current(void *data, int ma)
{
	int val = ma;

	mca_log_info("set iterm: %d\n", ma);
	mca_adsp_glink_write_prop(SUBPMIC_PROP_TERM_CURRENT, &val, sizeof(val));
	return 0;
}

static int qcom_subpmic_set_term_volt(void *data, int mv)
{
	int val = mv;

	mca_log_info("set vterm: %d\n", mv);
	mca_adsp_glink_write_prop(SUBPMIC_PROP_TERM_VOLT, &val, sizeof(val));
	return 0;
}

static int qcom_subpmic_set_prechg_volt(void *data, int mv)
{
	int val = mv;

	mca_adsp_glink_write_prop(SUBPMIC_PROP_PRECHG_VOLT, &val, sizeof(val));
	return 0;
}

static int qcom_subpmic_set_prechg_current(void *data, int ma)
{
	int val = ma;

	mca_adsp_glink_write_prop(SUBPMIC_PROP_PRECHG_CURRENT, &val,
				  sizeof(val));
	return 0;
}

static int qcom_subpmic_set_usb_aicl_cont_thd(void *data, int mv)
{
	int val = mv;

	mca_adsp_glink_write_prop(SUBPMIC_PROP_USB_AICL_CONT_THD, &val,
				  sizeof(val));
	return 0;
}

static int qcom_subpmic_set_opt_fws(void *data, int val_in)
{
	int val = val_in;

	mca_adsp_glink_write_prop(SUBPMIC_PROP_OPT_FWS, &val, sizeof(val));
	return 0;
}

static int qcom_subpmic_usb_adapter_allow_override(void *data, bool en)
{
	u8 val = en;

	mca_adsp_glink_write_prop(SUBPMIC_PROP_USB_ADAP_OVERRIDE, &val,
				  sizeof(val));
	return 0;
}

static int qcom_subpmic_set_qc3_volt(void *data, int mv)
{
	int val = mv;

	mca_adsp_glink_write_prop(SUBPMIC_PROP_QC3_VOLT, &val, sizeof(val));
	return 0;
}

static int qcom_subpmic_set_boost_enable(void *data, int src_enable)
{
	int val = src_enable;

	mca_log_info("set otg enable: 0x%2x\n", src_enable);
	mca_adsp_glink_write_prop(SUBPMIC_PROP_BOOST_ENABLE, &val, sizeof(val));
	return 0;
}

static int qcom_subpmic_set_boost_voltage(void *data, int src_value)
{
	int val = src_value;

	mca_log_info("set boost voltage: 0x%2x\n", src_value);
	mca_adsp_glink_write_prop(SUBPMIC_PROP_BOOST_VOLTAGE, &val,
				  sizeof(val));
	return 0;
}

static int qcom_subpmic_set_aicl_enable(void *data, bool en)
{
	u8 val = en;

	mca_log_info("set aicl enable: %d\n", en & 1);
	mca_adsp_glink_write_prop(SUBPMIC_PROP_AICL_ENABLE, &val, sizeof(val));
	return 0;
}

static int qcom_subpmic_set_rerun_aicl(void *data, bool en)
{
	u8 val = en;

	mca_log_info("set rerun aicl: %d\n", en & 1);
	mca_adsp_glink_write_prop(SUBPMIC_PROP_RERUN_AICL, &val, sizeof(val));
	return 0;
}

static int qcom_subpmic_set_restart_aicl(void *data, bool en)
{
	u8 val = en;

	mca_log_info("set restart aicl: %d\n", en & 1);
	mca_adsp_glink_write_prop(SUBPMIC_PROP_RESTART_AICL, &val, sizeof(val));
	return 0;
}

static int qcom_subpmic_set_wls_vdd_flag(void *data, bool en)
{
	u8 val = en;

	mca_log_info("set wls vdd flag: %d\n", en & 1);
	mca_adsp_glink_write_prop(SUBPMIC_PROP_WLS_VDD_FLAG, &val, sizeof(val));
	return 0;
}

static int qcom_subpmic_set_lpd_sbu1(void *data, int val_in)
{
	int val = val_in;

	mca_adsp_glink_write_prop(SUBPMIC_PROP_LPD_SBU1, &val, sizeof(val));
	return 0;
}

static int qcom_subpmic_set_lpd_control(void *data, int val_in)
{
	int val = val_in;

	mca_adsp_glink_write_prop(SUBPMIC_PROP_LPD_CONTROL, &val, sizeof(val));
	return 0;
}

static int qcom_subpmic_set_lpd_uart_control(void *data, int val_in)
{
	int val = val_in;

	mca_adsp_glink_write_prop(SUBPMIC_PROP_LPD_UART_CONTROL, &val,
				  sizeof(val));
	return 0;
}

static int qcom_subpmic_set_too_hot_limit(void *data, int val_in)
{
	int val = val_in;

	mca_adsp_glink_write_prop(SUBPMIC_PROP_TOO_HOT_LIMIT, &val,
				  sizeof(val));
	return 0;
}

static int qcom_subpmic_set_ship_mode(void *data, bool en)
{
	struct qcom_subpmic *sc = data;

	sc->is_enable_shipmode = en;
	mca_log_err("is_enable_shipmode = %d\n", en & 1);
	return 0;
}

static int qcom_subpmic_set_eu_model(void *data, bool en)
{
	struct qcom_subpmic *sc = data;
	u8 val = en;
	int rc;

	rc = mca_adsp_glink_write_prop(SUBPMIC_PROP_EU_MODEL, &val,
				       sizeof(val));
	if (rc < 0)
		mca_log_err("Failed to write eu model to adsp: %d\n", rc);

	sc->eu_model = val;
	mca_log_info("set_eu_model = %d\n", sc->eu_model);
	return 0;
}

static struct protocol_class_qc_ops g_adsp_qc_ops = {
	.protocol_qc_get_qc_type = qcom_subpmic_get_usb_real_type,
	.protocol_qc_set_volt = qcom_subpmic_set_qc_volt,
	.protocol_qc_set_volt_cmd = qcom_subpmic_set_qc_volt_cmd,
};

static struct platform_bc12_class_ops g_qcom_pmic_bc12_ops = {
	.bc12_det_en = qcom_subpmic_bc12_det_en,
	.get_charge_type = qcom_subpmic_get_usb_real_type,
};

static struct platform_class_buckchg_ops g_qcom_buckchg_ops = {
	.is_init_ok = qcom_subpmic_is_init_ok,
	.get_online = qcom_subpmic_get_online,
	.is_charge_done = qcom_subpmic_is_charge_done,
	.get_input_curr_lmt = qcom_subpmic_get_input_current_limit,
	.get_bus_curr = qcom_subpmic_get_bus_curr,
	.get_bus_volt = qcom_subpmic_get_bus_volt,
	.get_usb_sns_volt = qcom_subpmic_get_usb_sns_volt,
	.get_sys_volt = qcom_subpmic_get_vsys_volt,
	.get_chg_status = qcom_subpmic_get_chg_status,
	.get_chg_type = qcom_subpmic_get_chg_type,
	.get_term_curr = qcom_subpmic_get_term_current,
	.get_term_volt = qcom_subpmic_get_term_volt,
	.get_wls_curr = qcom_subpmic_get_wls_curr,
	.set_hiz = qcom_subpmic_set_input_suspend,
	.set_wls_hiz = qcom_subpmic_set_wireless_input_suspend,
	.set_input_curr_lmt = qcom_subpmic_set_input_current_limit,
	.set_wls_input_curr_lmt = qcom_subpmic_set_wls_input_current_limit,
	.set_input_volt_lmt = qcom_subpmic_set_input_volt_limit,
	.set_ichg = qcom_subpmic_set_charge_current,
	.set_chg = qcom_subpmic_enable_charging,
	.set_buck_fsw = qcom_subpmic_set_buck_fsw,
	.set_term_curr = qcom_subpmic_set_term_current,
	.set_term_volt = qcom_subpmic_set_term_volt,
	.set_prechg_volt = qcom_subpmic_set_prechg_volt,
	.set_prechg_curr = qcom_subpmic_set_prechg_current,
	.set_qc_volt = qcom_subpmic_set_qc_volt,
	.set_usb_aicl_cont_thd = qcom_subpmic_set_usb_aicl_cont_thd,
	.get_usb_aicl_cont_thd = qcom_subpmic_get_usb_aicl_cont_thd,
	.set_opt_fws = qcom_subpmic_set_opt_fws,
	.usb_adapter_allow_override = qcom_subpmic_usb_adapter_allow_override,
	.set_qc3_volt = qcom_subpmic_set_qc3_volt,
	.get_otg_boost_src = qcom_subpmic_get_otg_boost_src,
	.get_otg_boost_enable_status = qcom_subpmic_get_otg_boost_enable_status,
	.get_otg_gate_enable_status = qcom_subpmic_get_otg_gate_enable_status,
	.set_boost_enable = qcom_subpmic_set_boost_enable,
	.set_boost_voltage = qcom_subpmic_set_boost_voltage,
	.set_aicl_enable = qcom_subpmic_set_aicl_enable,
	.set_rerun_aicl = qcom_subpmic_set_rerun_aicl,
	.set_restart_aicl = qcom_subpmic_set_restart_aicl,
	.is_support_cid = qcom_subpmic_is_support_cid,
	.set_ship_mode = qcom_subpmic_set_ship_mode,
	.get_ship_mode = qcom_subpmic_get_ship_mode,
	.set_wls_vdd_flag = qcom_subpmic_set_wls_vdd_flag,
	.get_lpd_enable = qcom_subpmic_get_lpd_enable,
	.get_lpd_status = qcom_subpmic_get_lpd_status,
	.get_lpd_sbu1 = qcom_subpmic_get_lpd_sbu1,
	.get_lpd_sbu2 = qcom_subpmic_get_lpd_sbu2,
	.get_lpd_cc1 = qcom_subpmic_get_lpd_cc1,
	.get_lpd_cc2 = qcom_subpmic_get_lpd_cc2,
	.get_lpd_dp = qcom_subpmic_get_lpd_dp,
	.get_lpd_dm = qcom_subpmic_get_lpd_dm,
	.set_lpd_sbu1 = qcom_subpmic_set_lpd_sbu1,
	.set_lpd_control = qcom_subpmic_set_lpd_control,
	.get_lpd_control = qcom_subpmic_get_lpd_control,
	.set_lpd_uart_control = qcom_subpmic_set_lpd_uart_control,
	.get_lpd_uart_control = qcom_subpmic_get_lpd_uart_control,
	.get_pack_vbat = qcom_subpmic_get_pack_vbat,
	.get_pack_ibat = qcom_subpmic_get_pack_ibat,
	.set_eu_model = qcom_subpmic_set_eu_model,
	.get_aicl_status = qcom_subpmic_get_aicl_status,
	.set_too_hot_limit = qcom_subpmic_set_too_hot_limit,
	.get_pack_tbat = qcom_subpmic_get_pack_tbat,
};

static void qcom_subpmic_update_usb_type_work(struct work_struct *work)
{
	struct qcom_subpmic *sc = container_of(to_delayed_work(work),
					       struct qcom_subpmic,
					       update_usb_type_work);
	int usb_online = 0;
	int real_type = 0;
	int pd_active = 0;
	int rc;

	if (sc->support_multi_bc12) {
		protocol_class_pd_get_pd_active(1, &pd_active);
		if (pd_active != 0)
			return;
	}

	rc = mca_adsp_glink_read_prop(SUBPMIC_PROP_USB_ONLINE, &usb_online,
				      sizeof(usb_online));
	if (rc < 0) {
		mca_log_err("Failed to read usb_online rc=%d\n", rc);
		goto retry;
	}

	if (sc->init_ok == 0) {
		mca_event_block_notify(MCA_EVENT_TYPE_HW_INFO,
				       MCA_EVENT_PMIC_INIT_DONE, NULL);
		sc->init_ok = 1;
	}

	if (sc->online != usb_online) {
		mca_log_err("usb online: %d\n", usb_online);
		sc->online = usb_online;
		if (usb_online == 0) {
			protocol_class_pd_set_pd_active(0, 0);
			mca_event_block_notify(MCA_EVENT_TYPE_CHARGER_CONNECT,
					       MCA_EVENT_USB_DISCONNECT, NULL);
			pm_relax(sc->dev);
		} else {
			protocol_class_pd_set_pd_active(0, 1);
			mca_event_block_notify(MCA_EVENT_TYPE_CHARGER_CONNECT,
					       MCA_EVENT_USB_CONNECT, NULL);
		}
	}

	rc = mca_adsp_glink_read_prop(SUBPMIC_PROP_USB_REAL_TYPE, &real_type,
				      sizeof(real_type));
	if (rc < 0) {
		mca_log_err("Failed to read USB_ADAP_TYPE rc=%d\n", rc);
		goto retry;
	}

	if (sc->real_type == real_type) {
		if (sc->glink_down == 0)
			return;
	} else {
		mca_log_err("usb real_type: %d\n", real_type);
		sc->real_type = real_type;
	}

	sc->glink_down = 0;
	mca_event_block_notify(MCA_EVENT_TYPE_CHARGE_TYPE,
			       MCA_EVENT_CHARGE_TYPE_CHANGE, &sc->real_type);
	return;

retry:
	queue_delayed_work(system_wq, &sc->update_usb_type_work,
			   msecs_to_jiffies(500));
}

static int qcom_subpmic_sync_cfg_work_count;

static void qcom_subpmic_sync_cfg_work(struct work_struct *work)
{
	struct qcom_subpmic *sc = container_of(
		to_delayed_work(work), struct qcom_subpmic, sync_cfg_work);
	bool cp_present = false;
	int rc;

	rc = platform_class_cp_get_present(0, &cp_present);
	if (rc != 0) {
		mca_log_err("Failed to get cp state, rc=%d\n", rc);
		goto retry;
	}

	rc = mca_adsp_glink_write_prop(SUBPMIC_PROP_CP_STATE, &cp_present,
				       sizeof(cp_present));
	if (rc != 0) {
		mca_log_err("Failed to send cp state, rc=%d\n", rc);
		goto retry;
	}
	mca_log_err("Success to send cp state, rc=%d\n", 0);

	rc = mca_adsp_glink_write_prop(SUBPMIC_PROP_OTG_CFG, &sc->otg_cfg,
				       sizeof(sc->otg_cfg));
	if (rc != 0) {
		mca_log_err("Failed to send otg cfg, rc=%d\n", rc);
		goto retry;
	}
	mca_log_err("Success to send otg cfg, rc=%d\n", 0);

	if (!sc->support_cid)
		return;

	rc = mca_adsp_glink_write_prop(SUBPMIC_PROP_CID_CFG, &sc->cid_gpio_int,
				       sizeof(sc->cid_gpio_int));
	if (rc != 0) {
		mca_log_err("Failed to send cid cfg, rc=%d\n", rc);
		goto retry;
	}
	mca_log_err("Success to send cid cfg, rc=%d\n", 0);

	rc = mca_adsp_glink_write_prop(SUBPMIC_PROP_EU_MODEL, &sc->eu_model,
				       sizeof(sc->eu_model));
	if (rc != 0) {
		mca_log_err("Failed to send eu model, rc=%d\n", rc);
		goto retry;
	}
	mca_log_err("Success to send eu model, rc=%d\n", 0);
	return;

retry:
	if (qcom_subpmic_sync_cfg_work_count++ < 0x3c)
		queue_delayed_work(system_wq, &sc->sync_cfg_work,
				   msecs_to_jiffies(250));
}

static void qcom_subpmic_notify_change_work(struct work_struct *work)
{
	struct qcom_subpmic *sc =
		container_of(work, struct qcom_subpmic, notify_change_work);
	struct qcom_subpmic_notify_entry *entry, *tmp;

	spin_lock(&sc->notify_lock);
	list_for_each_entry_safe(entry, tmp, &sc->notify_list, node) {
		list_del(&entry->node);
		spin_unlock(&sc->notify_lock);

		switch (entry->type) {
		case SUBPMIC_NOTIFY_PLATE_SHOCK:
			mca_event_block_notify(MCA_EVENT_TYPE_TYPEC_PORT_STATUS,
					       SUBPMIC_RAW_CODE_PLATE_SHOCK,
					       NULL);
			break;
		case SUBPMIC_NOTIFY_ENABLE_BOOST:
			if (sc->otg_cfg.otg_boost_src == 0) {
				if (entry->val == 0)
					sc->cp_vbus_revert = 0;
				mca_event_block_notify(
					MCA_EVENT_TYPE_HW_INFO,
					MCA_EVENT_CP_REVERT_CHANGE,
					&sc->cp_vbus_revert);
			} else {
				mca_log_err("enable_ovpgate: %d\n",
					    entry->val == 0);
				platform_class_cp_enable_ovpgate_with_check(
					0, 0, entry->val == 0);
				if (entry->val == 0) {
					sc->cp_vbus_revert = 0x100 << 16;
					mca_event_block_notify(
						MCA_EVENT_TYPE_HW_INFO,
						MCA_EVENT_CP_REVERT_CHANGE,
						&sc->cp_vbus_revert);
					sc->cp_vbus_revert = 0;
				}
			}
			break;
		case SUBPMIC_NOTIFY_CHARGE_TYPE:
			mca_event_block_notify(MCA_EVENT_TYPE_CHARGE_TYPE,
					       SUBPMIC_RAW_CODE_CHARGE_TYPE_2,
					       NULL);
			break;
		}

		kfree(entry);
		spin_lock(&sc->notify_lock);
	}
	spin_unlock(&sc->notify_lock);
}

static void qcom_subpmic_glink_down_cb(void *priv)
{
	struct qcom_subpmic *sc = priv;

	mca_log_info("glink down\n");
	sc->glink_down = 1;
	mca_event_block_notify(MCA_EVENT_CHARGE_STATUS,
			       SUBPMIC_RAW_CODE_GLINK_DOWN, NULL);
}

static void qcom_subpmic_sync_cb(void *priv)
{
	struct qcom_subpmic *sc = priv;

	mca_log_info("glink up\n");
	mca_event_block_notify(MCA_EVENT_CHARGE_STATUS,
			       SUBPMIC_RAW_CODE_GLINK_UP, NULL);
	queue_delayed_work(system_wq, &sc->update_usb_type_work, 0);
	queue_delayed_work(system_wq, &sc->sync_cfg_work, 0);
}

static void qcom_subpmic_queue_notify(struct qcom_subpmic *sc, int type,
				      int val)
{
	struct qcom_subpmic_notify_entry *entry;

	entry = kmalloc(sizeof(*entry), GFP_ATOMIC);
	if (!entry)
		return;

	entry->type = type;
	entry->val = val;

	if (type == SUBPMIC_NOTIFY_ENABLE_BOOST)
		mca_log_err("recv enable boost notify: %d\n", val);

	spin_lock(&sc->notify_lock);
	list_add_tail(&entry->node, &sc->notify_list);
	spin_unlock(&sc->notify_lock);

	queue_work(system_wq, &sc->notify_change_work);
}

static void qcom_subpmic_report_uevent(const char *fmt, int val)
{
	struct mca_event_notify_data n_data;
	char buf[128] = { 0 };

	n_data.event_len = snprintf(buf, sizeof(buf), fmt, val);
	n_data.event = buf;
	mca_event_report_uevent(&n_data);
}

static void qcom_subpmic_notify_cb(u32 notify_type, void *data, u32 len,
				   void *priv)
{
	struct qcom_subpmic *sc = priv;
	int *val = data;

	if (!sc)
		return;

	pm_wakeup_dev_event(sc->dev, 1000, true);

	switch (notify_type) {
	case SUBPMIC_NOTIFY_USB_TYPE:
		queue_delayed_work(system_wq, &sc->update_usb_type_work, 0);
		break;
	case SUBPMIC_NOTIFY_CHARGE_TYPE:
	case SUBPMIC_NOTIFY_ENABLE_BOOST:
	case SUBPMIC_NOTIFY_PLATE_SHOCK:
		qcom_subpmic_queue_notify(sc, notify_type, val ? *val : 0);
		break;
	case SUBPMIC_NOTIFY_LPD_STATUS:
		sc->lpd_status = *val;
		mca_log_err("recv adsp report lpd status: %d\n",
			    sc->lpd_status);
		qcom_subpmic_report_uevent("POWER_SUPPLY_MOISTURE_DET_STS=%d",
					   sc->lpd_status);
		mca_event_block_notify(MCA_EVENT_TYPE_HW_INFO,
				       MCA_EVENT_LPD_STATUS_CHANGE,
				       &sc->lpd_status);
		break;
	case SUBPMIC_NOTIFY_CC_SHORT_VBUS:
		sc->cc_short_vbus = *val;
		mca_log_err("recv cc short vbus notify: %d\n",
			    sc->cc_short_vbus);
		qcom_subpmic_report_uevent("POWER_SUPPLY_CC_SHORT_VBUS=%d",
					   sc->cc_short_vbus);
		mca_event_block_notify(MCA_EVENT_TYPE_HW_INFO,
				       MCA_EVENT_CC_SHORT_VBUS,
				       &sc->cc_short_vbus);
		break;
	case SUBPMIC_NOTIFY_PPS_PTF:
		sc->pps_ptf = *val;
		mca_log_err("recv pps ptf notify: %d\n", sc->pps_ptf);
		mca_strategy_func_process(STRATEGY_FUNC_TYPE_QUICK_CHARGE,
					  MCA_EVENT_PPS_PTF, sc->pps_ptf);
		mca_strategy_func_process(STRATEGY_FUNC_TYPE_BUCK_CHARGE,
					  MCA_EVENT_PPS_PTF, sc->pps_ptf);
		break;
	case SUBPMIC_NOTIFY_SINK_PWR_SUSPEND:
		if (sc->eu_model == 0)
			break;
		mca_log_err("recv bsinkpowersuspend change\n");
		sc->sink_pwr_suspend = (u8)*val;
		mca_event_block_notify(MCA_EVENT_TYPE_CHARGE_TYPE,
				       MCA_EVENT_SINK_PWR_SUSPEND_CHANGE,
				       &sc->sink_pwr_suspend);
		break;
	case SUBPMIC_NOTIFY_CP_REVERT:
		sc->cp_vbus_revert = *val;
		mca_log_info("recv cp revert notify\n");
		mca_event_block_notify(MCA_EVENT_TYPE_HW_INFO,
				       MCA_EVENT_CP_REVERT_CHANGE,
				       &sc->cp_vbus_revert);
		break;
	case SUBPMIC_NOTIFY_OTG_CP_CONFIG:
		mca_log_info("recv otg cp config notify: %d\n", *val);
		platform_class_cp_set_revchg(0, *val != 0);
		break;
	case SUBPMIC_NOTIFY_SBU_LPD:
		sc->sbu_lpd = *val;
		mca_log_info("recv sbu_lpd: %d\n", sc->sbu_lpd);
		mca_event_block_notify(MCA_EVENT_TYPE_HW_INFO,
				       MCA_EVENT_SBU_LPD_CHANGE, &sc->sbu_lpd);
		break;
	}
}

static struct mca_adsp_glink_ops g_qcom_subpmic_glink_cb = {
	.glink_state_down = qcom_subpmic_glink_down_cb,
	.glink_state_up = qcom_subpmic_sync_cb,
	.notification = qcom_subpmic_notify_cb,
};

static int qcom_subpmic_ship_mode(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	struct qcom_subpmic *sc =
		container_of(nb, struct qcom_subpmic, ship_mode_nb);
	u64 reason = action;
	int chip_vendor = -1;
	int rc;

	if ((action & ~2UL) == 1 && sc->is_enable_shipmode) {
		/* shutdown/ship notifier path: stock logs via raw printk here */
		pr_info("subpmic: set adsp shipmode\n");
		platform_class_cp_get_chip_vendor(0, &chip_vendor);
		pr_info("subpmic: cp type: %d\n", chip_vendor);
		if (chip_vendor != 0)
			platform_class_cp_enable_ovpgate(0, 0);

		rc = mca_adsp_glink_write_prop(SUBPMIC_PROP_SHIP_MODE, &reason,
					       sizeof(reason));
		if (rc < 0)
			pr_err("subpmic: Failed to send ship mode, rc=%d\n", rc);
	}

	return NOTIFY_DONE;
}

static int qcom_subpmic_shutdown_cb(struct notifier_block *nb,
				    unsigned long action, void *data)
{
	u64 reason = action;
	int rc;

	if ((action & ~2UL) == 1) {
		pr_info("subpmic: start adsp shutdown\n");
		rc = mca_adsp_glink_write_prop(SUBPMIC_PROP_SHUTDOWN, &reason,
					       sizeof(reason));
		if (rc < 0)
			pr_err("subpmic: Failed to send shutdown, rc=%d\n", rc);
	}

	return NOTIFY_DONE;
}

static void qcom_subpmic_parse_dt(struct qcom_subpmic *sc,
				  struct device_node *node)
{
	const struct mca_hwid *hwid = mca_get_hwid_info();

	sc->support_2s_charging =
		of_find_property(node, "mi,support-2s-charging", NULL);
	sc->support_dual_panel =
		of_find_property(node, "mi,support-dual-panel", NULL);
	sc->support_multi_bc12 =
		of_find_property(node, "mi,support-multi-bc12", NULL);
	sc->support_cid = of_find_property(node, "mi,support-cid", NULL);

	mca_parse_dts_u32(node, "otg_boost_src", &sc->otg_cfg.otg_boost_src, 2);
	mca_parse_dts_u32(node, "gpio_chip_type", &sc->otg_cfg.gpio_chip_type,
			  1);
	mca_parse_dts_u32(node, "vdd_boost_en", &sc->otg_cfg.vdd_boost_en,
			  0xffffffff);
	mca_parse_dts_u32(node, "otg_ovp_en", &sc->otg_cfg.otg_ovp_en,
			  0xffffffff);

	if (hwid && hwid->platform_version == 1 &&
	    (hwid->build_version == 0 ||
	     (hwid->build_version == 1 && hwid->minor_version == 0))) {
		sc->otg_cfg.otg_boost_src = 2;
		mca_log_err(
			"O2 p1.1 change otg boost from external to haptic\n");
	}

	if (sc->support_cid) {
		mca_parse_dts_u32(node, "cid_gpio_int", &sc->cid_gpio_int, 0);
		mca_log_err("dts sync cid_gpio_int= %#x\n", sc->cid_gpio_int);
	}

	mca_log_err(
		"dts sync otg_cfg_src= %d,gpio_chip_type=%d, vdd_boost_gpio=%d, ovp_en_gpio=%d\n",
		sc->otg_cfg.otg_boost_src, sc->otg_cfg.gpio_chip_type,
		sc->otg_cfg.vdd_boost_en, sc->otg_cfg.otg_ovp_en);
}

static int qcom_subpmic_probe(struct platform_device *pdev)
{
	struct qcom_subpmic *sc;
	int rc;

	mca_log_info("probe start\n");

	sc = devm_kzalloc(&pdev->dev, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;

	sc->dev = &pdev->dev;

	if (!mca_get_hwid_info()) {
		mca_log_err("%s parse dt fail\n", __func__);
		return -ENOMEM;
	}

	qcom_subpmic_parse_dt(sc, pdev->dev.of_node);

	INIT_LIST_HEAD(&sc->notify_list);
	spin_lock_init(&sc->notify_lock);
	INIT_DELAYED_WORK(&sc->update_usb_type_work,
			  qcom_subpmic_update_usb_type_work);
	INIT_DELAYED_WORK(&sc->sync_cfg_work, qcom_subpmic_sync_cfg_work);
	INIT_WORK(&sc->notify_change_work, qcom_subpmic_notify_change_work);

	sc->ship_mode_nb.notifier_call = qcom_subpmic_ship_mode;
	sc->ship_mode_nb.priority = 255;
	register_reboot_notifier(&sc->ship_mode_nb);

	sc->shutdown_nb.notifier_call = qcom_subpmic_shutdown_cb;
	sc->shutdown_nb.priority = 255;
	register_reboot_notifier(&sc->shutdown_nb);

	device_set_wakeup_capable(sc->dev, true);
	device_wakeup_enable(sc->dev);
	platform_set_drvdata(pdev, sc);

	rc = platform_bc12_class_ops_register(0, &g_qcom_pmic_bc12_ops, sc);
	if (rc) {
		mca_log_err("%s register protocol ops fail\n", __func__);
		goto err;
	}

	rc = protocol_class_qc_register_ops(0, &g_adsp_qc_ops, sc);
	if (rc) {
		mca_log_err("%s register qc ops fail\n", __func__);
		goto err;
	}

	rc = platform_class_buckchg_ops_register(0, sc, &g_qcom_buckchg_ops);
	if (rc) {
		mca_log_err("%s register buckchg ops fail\n", __func__);
		goto err;
	}

	rc = mca_adsp_glink_resister_ops(&g_qcom_subpmic_glink_cb, sc);
	if (rc) {
		mca_log_err("%s register glink ops fail\n", __func__);
		goto err;
	}

	queue_delayed_work(system_wq, &sc->update_usb_type_work, 0);
	queue_delayed_work(system_wq, &sc->sync_cfg_work, msecs_to_jiffies(25));
	mca_log_err("probe ok\n");
	return 0;

err:
	unregister_reboot_notifier(&sc->ship_mode_nb);
	unregister_reboot_notifier(&sc->shutdown_nb);
	return rc;
}

static int qcom_subpmic_remove(struct platform_device *pdev)
{
	return 0;
}

static void qcom_subpmic_shutdown(struct platform_device *pdev)
{
}

static const struct of_device_id qcom_subpmic_match_table[] = {
	{ .compatible = "mca,qcom_subpmic" },
	{},
};

static struct platform_driver qcom_subpmic_driver = {
	.driver = {
		.name = "qcom_subpmic",
		.of_match_table = qcom_subpmic_match_table,
	},
	.probe = qcom_subpmic_probe,
	.remove = qcom_subpmic_remove,
	.shutdown = qcom_subpmic_shutdown,
};
module_platform_driver(qcom_subpmic_driver);

MODULE_DESCRIPTION("qcom subpmic  driver");
MODULE_AUTHOR("liyuze1@xiaomi.com");
MODULE_LICENSE("GPL v2");
