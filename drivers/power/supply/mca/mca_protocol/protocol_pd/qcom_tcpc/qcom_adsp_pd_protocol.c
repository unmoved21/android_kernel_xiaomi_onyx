#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/notifier.h>
#include <linux/string.h>
#include <linux/swab.h>

#include <mca/common/mca_log.h>
#include <mca/common/mca_event.h>
#include <mca/common/mca_adsp_glink.h>
#include <mca/protocol/protocol_class.h>
#include <mca/protocol/protocol_pd_class.h>

#ifndef MCA_LOG_TAG
#define MCA_LOG_TAG "qcom_adsp_pd"
#endif

enum adsp_pd_prop {
	PD_PROP_USB_REAL_TYPE = 0x2000c,
	PD_PROP_VERIFY_PROCESS = 0x21002,
	PD_PROP_VDM_VERSION = 0x21003,
	PD_PROP_VDM_VOLTAGE = 0x21004,
	PD_PROP_VDM_TEMP = 0x21005,
	PD_PROP_VDM_SESSION_SEED = 0x21006,
	PD_PROP_VDM_AUTH = 0x21007,
	PD_PROP_VDM_VERIFIED = 0x21008,
	PD_PROP_VDM_REMOVE_COMP = 0x21009,
	PD_PROP_VDM_REVERSE_AUTH = 0x2100a,
	PD_PROP_KEEP_PDO = 0x2100b,
	PD_PROP_VDM_NAK = 0x2100c,
	PD_PROP_DATA_ROLE = 0x2100d,
	PD_PROP_CURRENT_STATE = 0x2100e,
	PD_PROP_ADAPTER_ID = 0x2100f,
	PD_PROP_USB_VENDOR_ID = 0x21010,
	PD_PROP_PRODUCT_VDO = 0x21011,
	PD_PROP_ADAPTER_SVID = 0x21012,
	PD_PROP_PD_VERIFIED = 0x21013,
	PD_PROP_PDOS = 0x21014,
	PD_PROP_VDM_STATE = 0x21015,
	PD_PROP_PPS_MAX_CUR = 0x21016,
	PD_PROP_PPS_APDO_MAX = 0x21017,
	PD_PROP_TYPEC_MODE = 0x21018,
	PD_PROP_TYPEC_CC_ORIENT = 0x21019,
	PD_PROP_SELECT_PPS_PDO = 0x2101a,
	PD_PROP_FIXED_PD_VOLT = 0x2101b,
	PD_PROP_PPS_STATUS = 0x2101c,
	PD_PROP_PPS_MAX_POWER = 0x2101e,
	PD_PROP_HAS_DP = 0x21021,
	PD_PROP_CID_STATUS = 0x21022,
	PD_PROP_OTG_PLUGIN = 0x21023,
	PD_PROP_CC_TOGGLE = 0x21024,
	PD_PROP_SNK_SRC_MODE = 0x21025,
	PD_PROP_CC_STATUS = 0x21026,
	PD_PROP_CC_SHORT_VBUS = 0x21027,
	PD_PROP_PPS_PTF = 0x21028,
	PD_PROP_SUSPEND_SUPPORT = 0x21029,
	PD_PROP_ZIMI_CYPRESS = 0x2102a,
	PD_PROP_GEAR_SHIFT = 0x2102b,
};

#define ADSP_VDM_REQ_INIT 200
#define ADSP_VDM_REQ_KEEP_PDO 0xfe
#define ADSP_VDM_REQ_NAK 0xff
#define ADSP_GLINK_DOWN_CODE 0x33
#define ADSP_REAL_TYPE_PD 0x0b
#define ADSP_PD_TYPE_PD 0x0b
#define ADSP_PD_TYPE_PD_PPS 0x0c

struct adsp_pd_protocol {
	struct device *dev;
	int pd_active;
	int pd_verifed;
	struct notifier_block event_nb;
	struct notifier_block abnormal_nb;
	int verify_process;
};

static int adsp_pd_protocol_get_pps_max_power(unsigned int *max_power,
					      void *data)
{
	return mca_adsp_glink_write_prop(PD_PROP_PPS_MAX_POWER, max_power,
					 sizeof(*max_power));
}

static int adsp_pd_protocol_select_pps_pdo(int volt, int curr, void *data)
{
	u32 val;

	mca_log_info("vbus_mv: %d, ibus_ma: %d\n", volt, curr);
	val = (curr / 50) | ((volt / 20) << 16);
	return mca_adsp_glink_write_prop(PD_PROP_SELECT_PPS_PDO, &val,
					 sizeof(val));
}

static int adsp_pd_protocol_get_pps_ptf(int *pps_ptf, void *data)
{
	return mca_adsp_glink_read_prop(PD_PROP_PPS_PTF, pps_ptf,
					sizeof(*pps_ptf));
}

static int adsp_pd_protocol_set_fixed_pd_volt(int volt, void *data)
{
	struct adsp_pd_protocol *pd = data;
	int val = volt;

	mca_log_info("vbus_mv: %d\n", volt);
	if (pd->verify_process) {
		mca_log_err("pd verify processing, won't set fixed pd volt\n");
		return 0;
	}
	return mca_adsp_glink_write_prop(PD_PROP_FIXED_PD_VOLT, &val,
					 sizeof(val));
}

static int adsp_pd_protocol_set_gear_shift(int gear_shift, void *data)
{
	int val = gear_shift;

	return mca_adsp_glink_write_prop(PD_PROP_GEAR_SHIFT, &val, sizeof(val));
}

static int adsp_pd_protocol_get_pps_max_cur(unsigned int *curr, void *data)
{
	return mca_adsp_glink_read_prop(PD_PROP_PPS_MAX_CUR, curr,
					sizeof(*curr));
}

static int adsp_pd_protocol_get_pps_status(int *volt, int *curr, void *data)
{
	u32 val = 0;
	int ret;

	ret = mca_adsp_glink_read_prop(PD_PROP_PPS_STATUS, &val, sizeof(val));
	if (ret == 0) {
		*curr = (val & 0xffff) * 50;
		*volt = (val >> 16) * 20;
	}
	return ret;
}

static int adsp_pd_protocol_get_pps_apdo_max(unsigned int *apdo_max, void *data)
{
	return mca_adsp_glink_read_prop(PD_PROP_PPS_APDO_MAX, apdo_max,
					sizeof(*apdo_max));
}

static int adsp_pd_protocol_get_pdos(struct pd_pdo *pdos, int count, void *data)
{
	int ret, i;

	ret = mca_adsp_glink_read_prop(PD_PROP_PDOS, pdos,
				       count * sizeof(struct pd_pdo));
	if (count > 0 && ret == 0) {
		for (i = 0; i < count; i++)
			mca_log_info(
				"pdo[%d] min_volt: %d, max_volt: %d, max_current: %d\n",
				i, pdos[i].min_volt, pdos[i].max_volt,
				pdos[i].max_current);
	}
	return ret;
}

static int adsp_pd_protocol_set_pd_active(int pd_active, void *data)
{
	struct adsp_pd_protocol *pd = data;

	if (!pd)
		return -1;
	pd->pd_active = pd_active;
	return 0;
}

static int adsp_pd_protocol_get_pd_active(int *pd_active, void *data)
{
	struct adsp_pd_protocol *pd = data;

	if (!pd)
		return -1;
	*pd_active = pd->pd_active;
	return 0;
}

static int adsp_pd_protocol_get_pd_type(int *type, void *data)
{
	struct adsp_pd_protocol *pd = data;
	int real_type = 0;
	int ret;

	if (!pd)
		return -1;

	ret = mca_adsp_glink_read_prop(PD_PROP_USB_REAL_TYPE, &real_type,
				       sizeof(real_type));
	if (ret == 0) {
		if (real_type == ADSP_REAL_TYPE_PD)
			*type = pd->pd_verifed ? ADSP_PD_TYPE_PD_PPS :
						 ADSP_PD_TYPE_PD;
		else
			*type = 0;
	}
	return ret;
}

static int adsp_pd_protocol_set_verify_process(int verify_process, void *data)
{
	struct adsp_pd_protocol *pd = data;
	int val = verify_process;

	pd->verify_process = verify_process;
	mca_log_err("verify_process: %d\n", verify_process);
	return mca_adsp_glink_write_prop(PD_PROP_VERIFY_PROCESS, &val,
					 sizeof(val));
}

static int adsp_pd_protocol_get_verify_process(int *verify_process, void *data)
{
	return mca_adsp_glink_read_prop(PD_PROP_VERIFY_PROCESS, verify_process,
					sizeof(*verify_process));
}

static int adsp_pd_protocol_set_pd_verifed(int pd_verifed, void *data)
{
	struct adsp_pd_protocol *pd = data;
	int val = pd_verifed;

	mca_log_err("set pd_verifed: %d\n", pd_verifed);
	pd->pd_verifed = pd_verifed;
	return mca_adsp_glink_write_prop(PD_PROP_PD_VERIFIED, &val,
					 sizeof(val));
}

static int adsp_pd_protocol_get_pd_verifed(int *pd_verifed, void *data)
{
	struct adsp_pd_protocol *pd = data;

	mca_log_info("get verifed %d\n", pd->pd_verifed);
	*pd_verifed = pd->pd_verifed;
	return 0;
}

static int adsp_pd_protocol_get_typec_mode(int *typec_mode, void *data)
{
	return mca_adsp_glink_read_prop(PD_PROP_TYPEC_MODE, typec_mode,
					sizeof(*typec_mode));
}

static int adsp_pd_protocol_get_typec_cc_orientation(int *cc_orientation,
						     void *data)
{
	return mca_adsp_glink_read_prop(PD_PROP_TYPEC_CC_ORIENT, cc_orientation,
					sizeof(*cc_orientation));
}

static int adsp_pd_protocol_get_adapter_id(unsigned int *adapter_id, void *data)
{
	return mca_adsp_glink_read_prop(PD_PROP_ADAPTER_ID, adapter_id,
					sizeof(*adapter_id));
}

static int adsp_pd_protocol_get_adapter_svid(unsigned int *adapter_svid,
					     void *data)
{
	return mca_adsp_glink_read_prop(PD_PROP_ADAPTER_SVID, adapter_svid,
					sizeof(*adapter_svid));
}

static int adsp_pd_protocol_get_has_dp(bool *has_dp, void *data)
{
	return mca_adsp_glink_read_prop(PD_PROP_HAS_DP, has_dp,
					sizeof(*has_dp));
}

static int adsp_pd_protocol_get_data_role(unsigned char *pr, void *data)
{
	u32 val = 1;
	int ret;

	ret = mca_adsp_glink_read_prop(PD_PROP_DATA_ROLE, &val, sizeof(val));
	*pr = (unsigned char)val;
	mca_log_info("get data_role: %d\n", val & 0xff);
	return ret;
}

static int adsp_pd_protocol_get_current_state(char *current_state, int len,
					      void *data)
{
	int state = 0;
	int ret;

	if (len < 0x40)
		return -1;

	ret = mca_adsp_glink_read_prop(PD_PROP_CURRENT_STATE, &state,
				       sizeof(state));
	if (ret != 0)
		return ret;

	switch (state) {
	case 0:
		strscpy(current_state, "SRC_Ready", sizeof("SRC_Ready"));
		break;
	case 1:
		strscpy(current_state, "SNK_STARTUP", sizeof("SNK_STARTUP"));
		break;
	case 2:
		strscpy(current_state, "SNK_Ready", sizeof("SNK_Ready"));
		break;
	default:
		strscpy(current_state, "UNKNOWN", sizeof("UNKNOWN"));
		break;
	}
	mca_log_info("get current_state: %u => %s\n", state, current_state);
	return 0;
}

static void adsp_pd_vdm_bswap4(u32 *d)
{
	d[0] = swab32(d[0]);
	d[1] = swab32(d[1]);
	d[2] = swab32(d[2]);
	d[3] = swab32(d[3]);
}

static int adsp_pd_protocol_request_vdm_cmd(enum uvdm_state cmd,
					    unsigned int *vdm_data,
					    unsigned int data_len, void *data)
{
	int prop;

	if (!vdm_data)
		return -1;

	mca_log_info("data_len=%d, cmd=%d, data=%d\n", data_len, cmd,
		     *vdm_data);

	switch (cmd) {
	case USBPD_UVDM_CHARGER_VERSION:
		prop = PD_PROP_VDM_VERSION;
		break;
	case USBPD_UVDM_CHARGER_VOLTAGE:
		prop = PD_PROP_VDM_VOLTAGE;
		break;
	case USBPD_UVDM_CHARGER_TEMP:
		prop = PD_PROP_VDM_TEMP;
		break;
	case USBPD_UVDM_SESSION_SEED:
		if (data_len < 4) {
			prop = 0;
		} else {
			adsp_pd_vdm_bswap4(vdm_data);
			prop = PD_PROP_VDM_SESSION_SEED;
		}
		break;
	case USBPD_UVDM_AUTHENTICATION:
		if (data_len < 4)
			return -1;
		adsp_pd_vdm_bswap4(vdm_data);
		prop = PD_PROP_VDM_AUTH;
		break;
	case USBPD_UVDM_VERIFIED:
		prop = PD_PROP_VDM_VERIFIED;
		break;
	case USBPD_UVDM_REMOVE_COMPENSATION:
		prop = PD_PROP_VDM_REMOVE_COMP;
		break;
	case USBPD_UVDM_REVERSE_AUTHEN:
		if (data_len < 4)
			return -1;
		adsp_pd_vdm_bswap4(vdm_data);
		prop = PD_PROP_VDM_REVERSE_AUTH;
		break;
	default:
		if (cmd == ADSP_VDM_REQ_INIT)
			prop = PD_PROP_DATA_ROLE;
		else if (cmd == ADSP_VDM_REQ_NAK)
			prop = PD_PROP_VDM_NAK;
		else if (cmd == ADSP_VDM_REQ_KEEP_PDO)
			prop = PD_PROP_KEEP_PDO;
		else {
			pr_info("cmd:%d is not support\n", cmd);
			return -EINVAL;
		}
		break;
	}

	return mca_adsp_glink_write_prop(prop, vdm_data, data_len);
}

static int adsp_pd_protocol_get_vdm_cmd(int *cmd,
					struct usbpd_vdm_data *vdm_data,
					void *data)
{
	int ret;

	if (!vdm_data)
		return -1;

	ret = mca_adsp_glink_read_prop(PD_PROP_VDM_STATE, cmd, sizeof(*cmd));
	if (ret != 0)
		return ret;

	mca_log_info("current uvdm_state: %d\n", *cmd);

	switch (*cmd) {
	case USBPD_UVDM_CHARGER_VERSION:
		return mca_adsp_glink_read_prop(PD_PROP_VDM_VERSION,
						&vdm_data->ta_version,
						sizeof(vdm_data->ta_version));
	case USBPD_UVDM_CHARGER_VOLTAGE:
		return mca_adsp_glink_read_prop(PD_PROP_VDM_VOLTAGE,
						&vdm_data->ta_voltage,
						sizeof(vdm_data->ta_voltage));
	case USBPD_UVDM_CHARGER_TEMP:
		return mca_adsp_glink_read_prop(PD_PROP_VDM_TEMP,
						&vdm_data->ta_temp,
						sizeof(vdm_data->ta_temp));
	case USBPD_UVDM_AUTHENTICATION: {
		u32 buf[4] = { 0 };

		ret = mca_adsp_glink_read_prop(PD_PROP_VDM_AUTH, buf,
					       sizeof(buf));
		vdm_data->s_secert[4] = buf[0];
		vdm_data->s_secert[5] = buf[1];
		vdm_data->s_secert[6] = buf[2];
		vdm_data->s_secert[7] = buf[3];
		mca_log_info(
			"get auth data[0]=%u, data[1]=%u, data[2]=%u, data[3]=%u\n",
			buf[0], buf[1], buf[2], buf[3]);
		return ret;
	}
	default:
		return ret;
	}
}

static int adsp_pd_protocol_get_cid_status(bool *status, void *data)
{
	return mca_adsp_glink_read_prop(PD_PROP_CID_STATUS, status,
					sizeof(*status));
}

static int adsp_pd_protocol_get_otg_plugin_status(bool *status, void *data)
{
	return mca_adsp_glink_read_prop(PD_PROP_OTG_PLUGIN, status,
					sizeof(*status));
}

static int adsp_protocol_set_cc_toggle(bool en, void *data)
{
	u8 val = en;

	mca_log_info("set cc toggle: 0x%2x\n", en & 1);
	return mca_adsp_glink_write_prop(PD_PROP_CC_TOGGLE, &val, sizeof(val));
}

static int adsp_protocol_get_cc_toggle(bool *en, void *data)
{
	return mca_adsp_glink_read_prop(PD_PROP_CC_TOGGLE, en, sizeof(*en));
}

static int adsp_pd_protocol_get_snk_src_mode(int *snk_src_mode, void *data)
{
	return mca_adsp_glink_read_prop(PD_PROP_SNK_SRC_MODE, snk_src_mode,
					sizeof(*snk_src_mode));
}

static int adsp_protocol_get_cc_status(bool *status, void *data)
{
	return mca_adsp_glink_read_prop(PD_PROP_CC_STATUS, status,
					sizeof(*status));
}

static int adsp_protocol_get_cc_short_vbus(int *cc_short_vbus, void *data)
{
	return mca_adsp_glink_read_prop(PD_PROP_CC_SHORT_VBUS, cc_short_vbus,
					sizeof(*cc_short_vbus));
}

static int adsp_pd_protocol_get_suspend_support_status(bool *pdsuspendsupported,
						       void *data)
{
	return mca_adsp_glink_read_prop(PD_PROP_SUSPEND_SUPPORT,
					pdsuspendsupported,
					sizeof(*pdsuspendsupported));
}

static int adsp_pd_protocol_get_zimi_cypress_flag(int *zimi_cypress_flag,
						  void *data)
{
	return mca_adsp_glink_read_prop(PD_PROP_ZIMI_CYPRESS, zimi_cypress_flag,
					sizeof(*zimi_cypress_flag));
}

static struct protocol_class_pd_ops g_adsp_pd_ops = {
	.protocol_pd_pps_get_max_power = adsp_pd_protocol_get_pps_max_power,
	.protocol_pd_pps_pdo_select = adsp_pd_protocol_select_pps_pdo,
	.protocol_pd_get_pps_ptf = adsp_pd_protocol_get_pps_ptf,
	.protocol_pd_fixed_pdo_set_vol = adsp_pd_protocol_set_fixed_pd_volt,
	.protocol_pd_set_gear_shift = adsp_pd_protocol_set_gear_shift,
	.protocol_pd_get_pps_max_cur = adsp_pd_protocol_get_pps_max_cur,
	.protocol_pd_get_pps_status = adsp_pd_protocol_get_pps_status,
	.protocol_pd_set_pd_active = adsp_pd_protocol_set_pd_active,
	.protocol_pd_get_pd_active = adsp_pd_protocol_get_pd_active,
	.protocol_pd_get_pps_apdo_max = adsp_pd_protocol_get_pps_apdo_max,
	.protocol_pd_get_pd_type = adsp_pd_protocol_get_pd_type,
	.protocol_pd_get_typec_mode = adsp_pd_protocol_get_typec_mode,
	.protocol_pd_get_typec_cc_orientation =
		adsp_pd_protocol_get_typec_cc_orientation,
	.protocol_pd_get_adapter_id = adsp_pd_protocol_get_adapter_id,
	.protocol_pd_get_adapter_svid = adsp_pd_protocol_get_adapter_svid,
	.protocol_pd_request_vdm_cmd = adsp_pd_protocol_request_vdm_cmd,
	.protocol_pd_get_vdm_cmd = adsp_pd_protocol_get_vdm_cmd,
	.protocol_pd_get_data_role = adsp_pd_protocol_get_data_role,
	.protocol_pd_get_current_state = adsp_pd_protocol_get_current_state,
	.protocol_pd_get_pdos = adsp_pd_protocol_get_pdos,
	.protocol_pd_set_verify_process = adsp_pd_protocol_set_verify_process,
	.protocol_pd_get_verify_process = adsp_pd_protocol_get_verify_process,
	.protocol_pd_set_pd_verifed = adsp_pd_protocol_set_pd_verifed,
	.protocol_pd_get_pd_verifed = adsp_pd_protocol_get_pd_verifed,
	.protocol_pd_get_has_dp = adsp_pd_protocol_get_has_dp,
	.protocol_pd_get_cid_status = adsp_pd_protocol_get_cid_status,
	.protocol_pd_get_otg_plugin_status =
		adsp_pd_protocol_get_otg_plugin_status,
	.protocol_pd_set_cc_toggle = adsp_protocol_set_cc_toggle,
	.protocol_pd_get_cc_toggle = adsp_protocol_get_cc_toggle,
	.protocol_pd_get_snk_src_mode = adsp_pd_protocol_get_snk_src_mode,
	.protocol_pd_get_cc_status = adsp_protocol_get_cc_status,
	.protocol_pd_get_cc_short_vbus = adsp_protocol_get_cc_short_vbus,
	.protocol_pd_get_suspend_support_status =
		adsp_pd_protocol_get_suspend_support_status,
	.protocol_pd_get_zimi_cypress_flag =
		adsp_pd_protocol_get_zimi_cypress_flag,
};

static int adsp_pd_protocol_event_cb(struct notifier_block *nb,
				     unsigned long action, void *data)
{
	struct adsp_pd_protocol *pd =
		container_of(nb, struct adsp_pd_protocol, event_nb);

	if (action != MCA_EVENT_USB_DISCONNECT)
		return 0;

	pd->pd_verifed = 0;
	pd->verify_process = 0;
	return 0;
}

static int adsp_pd_protocol_abnormal_cb(struct notifier_block *nb,
					unsigned long action, void *data)
{
	struct adsp_pd_protocol *pd =
		container_of(nb, struct adsp_pd_protocol, abnormal_nb);

	if (action == ADSP_GLINK_DOWN_CODE)
		pd->pd_verifed = 0;
	return 0;
}

static int adsp_pd_protocol_probe(struct platform_device *pdev)
{
	struct adsp_pd_protocol *pd;
	int rc;

	pd = devm_kzalloc(&pdev->dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->dev = &pdev->dev;
	platform_set_drvdata(pdev, pd);

	rc = protocol_class_pd_register_ops(0, &g_adsp_pd_ops, pd);
	if (rc) {
		mca_log_err("register ops fail\n");
		return rc;
	}

	pd->event_nb.notifier_call = adsp_pd_protocol_event_cb;
	mca_event_block_notify_register(MCA_EVENT_TYPE_CHARGER_CONNECT,
					&pd->event_nb);
	pd->abnormal_nb.notifier_call = adsp_pd_protocol_abnormal_cb;
	mca_event_block_notify_register(MCA_EVENT_CHARGE_STATUS,
					&pd->abnormal_nb);

	mca_log_err("probe ok\n");
	return 0;
}

static int adsp_pd_protocol_remove(struct platform_device *pdev)
{
	return 0;
}

static void adsp_pd_protocol_shutdown(struct platform_device *pdev)
{
}

static const struct of_device_id adsp_pd_protocol_match_table[] = {
	{ .compatible = "mca,adsp_pd_protocol" },
	{},
};
MODULE_DEVICE_TABLE(of, adsp_pd_protocol_match_table);

static struct platform_driver adsp_pd_protocol_driver = {
	.driver = {
		.name = "adsp_pd_protocol",
		.of_match_table = adsp_pd_protocol_match_table,
	},
	.probe = adsp_pd_protocol_probe,
	.remove = adsp_pd_protocol_remove,
	.shutdown = adsp_pd_protocol_shutdown,
};
module_platform_driver(adsp_pd_protocol_driver);

MODULE_DESCRIPTION("qcom pd protocol");
MODULE_AUTHOR("lvchen@xiaomi.com");
MODULE_LICENSE("GPL v2");
