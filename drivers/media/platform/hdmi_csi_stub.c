// SPDX-License-Identifier: GPL-2.0
/*
 * hdmi_csi_stub - V4L2 subdev CSI stub for userspace HDMI-to-CSI daemons
 *
 * This driver exposes a V4L2 subdev with an interface modelled after the
 * Toshiba TC35874X HDMI-to-CSI bridge, but all real hardware access is
 * delegated to a userspace daemon via /dev/hdmi-csi-stub (misc device).
 *
 * The V4L2 interface intentionally mirrors the tc35874x driver so that
 * existing Rockchip ISP / CSI pipelines work without modification.
 *
 * Differences from tc35874x (by design):
 *   - No built-in EDID: HPD stays low until userspace provides one via
 *     V4L2 set_edid.
 *   - No 5V-detect control: the daemon chip (e.g. LT6911C) handles HPD
 *     directly.  The V4L2_CID_DV_RX_POWER_PRESENT control is omitted.
 *   - No enum_frame_size / enum_frame_interval: the active mode always
 *     tracks the detected input signal.
 *   - The daemon owns CSI format, timings and colour signalling. V4L2
 *     set_fmt / s_dv_timings only accept the already-published state.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/of_graph.h>
#include <linux/videodev2.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/version.h>
#include <linux/compat.h>
#include <linux/rk-camera-module.h>
#include <linux/rk_hdmirx_config.h>
#include <linux/hdmi_csi_stub.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");

MODULE_DESCRIPTION("HDMI-to-CSI stub V4L2 subdev for userspace daemons");
MODULE_LICENSE("GPL");

#define DRIVER_VERSION		KERNEL_VERSION(0, 0x01, 0x0)
#define DRIVER_NAME		"hdmi-csi-stub"

#define EDID_BLOCK_SIZE		128
#define EDID_NUM_BLOCKS_MAX	8

/* Maximum number of queued events for the daemon */
#define EVENT_QUEUE_SIZE	16

static const u32 hdmi_csi_mbus_codes[] = {
	MEDIA_BUS_FMT_UYVY8_2X8,
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_UV8_1X8, /* legacy YUV420, CSI DT 0x1a */
};

static bool supported_mbus_code(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(hdmi_csi_mbus_codes); i++)
		if (code == hdmi_csi_mbus_codes[i])
			return true;
	return false;
}

static const struct v4l2_dv_timings_cap hdmi_csi_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	.reserved = { 0 },
	V4L2_INIT_BT_TIMINGS(1, 10000, 1, 10000, 0, 310000000,
			V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			V4L2_DV_BT_STD_GTF | V4L2_DV_BT_STD_CVT,
			V4L2_DV_BT_CAP_PROGRESSIVE |
			V4L2_DV_BT_CAP_INTERLACED |
			V4L2_DV_BT_CAP_REDUCED_BLANKING |
			V4L2_DV_BT_CAP_CUSTOM)
};

/* Custom V4L2 controls -- same CID numbering style as tc35874x */
#define HDMI_CSI_CID_AUDIO_SAMPLING_RATE \
	(V4L2_CID_USER_BASE + 0x1170)
#define HDMI_CSI_CID_AUDIO_PRESENT \
	(V4L2_CID_USER_BASE + 0x1171)

/* ------------------------------------------------------------------ */
/* Event ring (kernel -> daemon via misc device read)                  */
/* ------------------------------------------------------------------ */

struct event_ring {
	struct hdmi_csi_event	buf[EVENT_QUEUE_SIZE];
	unsigned int		head;
	unsigned int		tail;
	unsigned int		count;
};

static void event_ring_init(struct event_ring *r)
{
	r->head = r->tail = r->count = 0;
}

static bool event_ring_empty(const struct event_ring *r)
{
	return r->count == 0;
}

static int event_ring_push(struct event_ring *r, u32 type, u32 value)
{
	if (r->count >= EVENT_QUEUE_SIZE)
		return -ENOSPC;
	memset(&r->buf[r->head], 0, sizeof(r->buf[0]));
	r->buf[r->head].type = type;
	r->buf[r->head].value = value;
	r->head = (r->head + 1) % EVENT_QUEUE_SIZE;
	r->count++;
	return 0;
}

static int event_ring_pop(struct event_ring *r, struct hdmi_csi_event *out)
{
	if (r->count == 0)
		return -EAGAIN;
	*out = r->buf[r->tail];
	r->tail = (r->tail + 1) % EVENT_QUEUE_SIZE;
	r->count--;
	return 0;
}

/* ------------------------------------------------------------------ */
/* Driver state                                                        */
/* ------------------------------------------------------------------ */

struct hdmi_csi_stub_state {
	struct v4l2_subdev		sd;
	struct media_pad		pad;
	struct v4l2_ctrl_handler	hdl;
	struct device			*dev;

	/* Protects all mutable state below */
	struct mutex			lock;

	/* Configuration from DT */
	u8				csi_lanes;
	struct v4l2_mbus_config_mipi_csi2 bus;

	struct hdmi_csi_video		video; /* complete carrier/payload state */

	/* Current V4L2 state */
	struct v4l2_dv_timings		timings;	/* active timings */
	u32				mbus_fmt_code;
	u32				quantization;	/* enum v4l2_quantization */
	u32				colorspace;	/* enum v4l2_colorspace */
	bool				signal_present;
	bool				stream_on;
	u32				stream_sequence;

	/* Audio */
	u32				audio_sample_rate;
	bool				audio_present;

	/* Receiver progress: daemon fields only, sequence stays zero */
	struct hdmi_csi_status		status;
	u32				status_sequence;

	/* EDID storage */
	u8				edid[EDID_NUM_BLOCKS_MAX * EDID_BLOCK_SIZE];
	u8				edid_blocks;

	/* V4L2 controls */
	struct v4l2_ctrl		*audio_sampling_rate_ctrl;
	struct v4l2_ctrl		*audio_present_ctrl;
	struct v4l2_ctrl		*link_freq;
	struct v4l2_ctrl		*pixel_rate;

	/* Mutable link-freq menu (1 item, updated by daemon) */
	s64				link_freq_items[1];

	/* Misc device for daemon communication */
	struct miscdevice		misc;
	struct event_ring		events;
	wait_queue_head_t		event_wq;
	bool				daemon_open;
};

static inline struct hdmi_csi_stub_state *
sd_to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct hdmi_csi_stub_state, sd);
}

static inline struct hdmi_csi_stub_state *
misc_to_state(struct miscdevice *m)
{
	return container_of(m, struct hdmi_csi_stub_state, misc);
}

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static inline unsigned int
calc_fps(const struct v4l2_bt_timings *bt)
{
	u32 fh = V4L2_DV_BT_FRAME_HEIGHT(bt);
	u32 fw = V4L2_DV_BT_FRAME_WIDTH(bt);

	if (!fh || !fw)
		return 0;
	return DIV_ROUND_CLOSEST((unsigned int)bt->pixelclock, fh * fw);
}

/* Push an event to the daemon and wake it. Caller must hold state->lock. */
static void push_daemon_event(struct hdmi_csi_stub_state *st, u32 type,
			      u32 value)
{
	event_ring_push(&st->events, type, value);
	wake_up_interruptible(&st->event_wq);
}

/*
 * Convert daemon-reported hdmi_csi_timings to v4l2_dv_timings.
 * Returns true if the timings represent a valid signal.
 */
static bool timings_to_v4l2(const struct hdmi_csi_timings *in,
			     struct v4l2_dv_timings *out)
{
	struct v4l2_bt_timings *bt;

	memset(out, 0, sizeof(*out));

	if (in->h_active == 0 || in->v_active == 0)
		return false;

	out->type = V4L2_DV_BT_656_1120;
	bt = &out->bt;

	bt->width = in->h_active;
	bt->height = in->v_active;
	bt->pixelclock = (u64)in->pixel_clock * 1000; /* kHz -> Hz */
	bt->hfrontporch = in->h_fp;
	bt->hsync = in->h_sync;
	bt->hbackporch = in->h_bp;
	bt->vfrontporch = in->v_fp;
	bt->vsync = in->v_sync;
	bt->vbackporch = in->v_bp;
	bt->interlaced = in->interlaced ? V4L2_DV_INTERLACED
					: V4L2_DV_PROGRESSIVE;

	bt->polarities = 0;
	if (in->h_sync_pol)
		bt->polarities |= V4L2_DV_HSYNC_POS_POL;
	if (in->v_sync_pol)
		bt->polarities |= V4L2_DV_VSYNC_POS_POL;

	if (in->interlaced) {
		bt->il_vfrontporch = bt->vfrontporch;
		bt->il_vsync = bt->vsync + 1;
		bt->il_vbackporch = bt->vbackporch;
	}

	return true;
}

/* Fire V4L2_EVENT_SOURCE_CHANGE on the subdev. Caller must hold lock. */
static void fire_source_change(struct hdmi_csi_stub_state *st)
{
	static const struct v4l2_event ev = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	};

	if (st->sd.devnode)
		v4l2_subdev_notify_event(&st->sd, &ev);
}

static void get_status(struct hdmi_csi_stub_state *st,
		       struct hdmi_csi_status *status)
{
	mutex_lock(&st->lock);
	*status = st->status;
	status->sequence = st->status_sequence;
	mutex_unlock(&st->lock);
}

/*
 * Replace the daemon's status. Without published video, a stage change is
 * also a source change: it is the only sign that the reason for the missing
 * signal moved on. Caller must hold st->lock.
 */
static void set_status_locked(struct hdmi_csi_stub_state *st,
			      const struct hdmi_csi_status *status)
{
	bool restaged = st->status.rx_stage != status->rx_stage;

	st->status = *status;
	st->status_sequence++;
	if (restaged && !st->signal_present)
		fire_source_change(st);
}

/* V4L2 input status bits for the current state. Caller must hold st->lock. */
static u32 input_status_locked(const struct hdmi_csi_stub_state *st)
{
	if (st->signal_present)
		return 0;

	switch (st->status.rx_stage) {
	case HDMI_CSI_RX_NO_SOURCE:
		return V4L2_IN_ST_NO_POWER | V4L2_IN_ST_NO_SIGNAL |
		       V4L2_IN_ST_NO_SYNC;
	case HDMI_CSI_RX_UNKNOWN:
	case HDMI_CSI_RX_NO_EDID:
	case HDMI_CSI_RX_CONNECTING:
	case HDMI_CSI_RX_WAIT_CLOCK:
		return V4L2_IN_ST_NO_SIGNAL | V4L2_IN_ST_NO_SYNC;
	default:
		/* Signal present but not (or not usably) locked. */
		return V4L2_IN_ST_NO_SYNC;
	}
}

/* ------------------------------------------------------------------ */
/* V4L2 subdev - core ops                                              */
/* ------------------------------------------------------------------ */

static int stub_log_status(struct v4l2_subdev *sd)
{
	struct hdmi_csi_stub_state *st = sd_to_state(sd);

	v4l2_info(sd, "-----HDMI CSI Stub status-----\n");
	v4l2_info(sd, "Signal present: %s\n",
		  st->signal_present ? "yes" : "no");
	v4l2_info(sd, "CSI lanes: %u\n", st->csi_lanes);
	v4l2_info(sd, "Mbus format: 0x%04x\n", st->mbus_fmt_code);
	v4l2_info(sd, "Stream on: %s\n", st->stream_on ? "yes" : "no");
	v4l2_info(sd, "EDID blocks: %u\n", st->edid_blocks);

	if (st->signal_present)
		v4l2_print_dv_timings(sd->name, "Active timings: ",
				      &st->timings, true);
	else
		v4l2_info(sd, "No video detected\n");

	v4l2_info(sd, "Audio present: %s\n",
		  st->audio_present ? "yes" : "no");
	if (st->audio_present)
		v4l2_info(sd, "Audio sample rate: %u Hz\n",
			  st->audio_sample_rate);
	v4l2_info(sd, "Daemon connected: %s\n",
		  st->daemon_open ? "yes" : "no");
	v4l2_info(sd, "Receiver stage: %u, last error: %u (x%u)\n",
		  st->status.rx_stage, st->status.last_error,
		  st->status.error_count);

	return 0;
}

static int stub_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subdev_subscribe(sd, fh, sub);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subdev_subscribe_event(sd, fh, sub);
	default:
		return -EINVAL;
	}
}

static void get_video(struct hdmi_csi_stub_state *st, struct hdmi_csi_video *video)
{
	mutex_lock(&st->lock);
	*video = st->video;
	mutex_unlock(&st->lock);
}

static long stub_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct hdmi_csi_stub_state *st = sd_to_state(sd);

	switch (cmd) {
	case HDMI_CSI_IOC_GET_VIDEO:
		get_video(st, arg);
		return 0;
	case HDMI_CSI_IOC_GET_STATUS:
		get_status(st, arg);
		return 0;
	case RKMODULE_GET_HDMI_MODE:
		*(int *)arg = RKMODULE_HDMIIN_MODE;
		return 0;
	case RK_HDMIRX_CMD_GET_SIGNAL_STABLE_STATUS:
		mutex_lock(&st->lock);
		*(int *)arg = st->signal_present ? 1 : 0;
		mutex_unlock(&st->lock);
		return 0;
	default:
		return -ENOIOCTLCMD;
	}
}

#ifdef CONFIG_COMPAT
static long stub_compat_ioctl32(struct v4l2_subdev *sd,
				unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct hdmi_csi_video video;
	struct hdmi_csi_status status;
	int *seq;
	long ret;

	switch (cmd) {
	case HDMI_CSI_IOC_GET_VIDEO:
		get_video(sd_to_state(sd), &video);
		return copy_to_user(up, &video, sizeof(video)) ? -EFAULT : 0;
	case HDMI_CSI_IOC_GET_STATUS:
		get_status(sd_to_state(sd), &status);
		return copy_to_user(up, &status, sizeof(status)) ? -EFAULT : 0;
	case RKMODULE_GET_HDMI_MODE:
	case RK_HDMIRX_CMD_GET_SIGNAL_STABLE_STATUS:
		seq = kzalloc(sizeof(*seq), GFP_KERNEL);
		if (!seq)
			return -ENOMEM;
		ret = stub_ioctl(sd, cmd, seq);
		if (!ret && copy_to_user(up, seq, sizeof(*seq)))
			ret = -EFAULT;
		kfree(seq);
		return ret;

	default:
		return -ENOIOCTLCMD;
	}
}
#endif

static const struct v4l2_subdev_core_ops stub_core_ops = {
	.log_status		= stub_log_status,
	.subscribe_event	= stub_subscribe_event,
	.unsubscribe_event	= v4l2_event_subdev_unsubscribe,
	.ioctl			= stub_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32		= stub_compat_ioctl32,
#endif
};

/* ------------------------------------------------------------------ */
/* V4L2 subdev - video ops                                             */
/* ------------------------------------------------------------------ */

static int stub_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct hdmi_csi_stub_state *st = sd_to_state(sd);

	mutex_lock(&st->lock);
	*status = input_status_locked(st);
	mutex_unlock(&st->lock);

	return 0;
}

/* Capture clients may confirm the source timings but cannot change them. */
static int stub_s_dv_timings(struct v4l2_subdev *sd,
			     struct v4l2_dv_timings *timings)
{
	struct hdmi_csi_stub_state *st = sd_to_state(sd);
	int ret;

	if (!timings)
		return -EINVAL;

	mutex_lock(&st->lock);
	if (!st->signal_present)
		ret = -ENOLINK;
	else
		ret = v4l2_match_dv_timings(&st->timings, timings, 0, false)
			? 0 : -EINVAL;
	mutex_unlock(&st->lock);
	return ret;
}

static int stub_g_dv_timings(struct v4l2_subdev *sd,
			     struct v4l2_dv_timings *timings)
{
	struct hdmi_csi_stub_state *st = sd_to_state(sd);

	mutex_lock(&st->lock);
	*timings = st->timings;
	mutex_unlock(&st->lock);

	return 0;
}

static int stub_query_dv_timings(struct v4l2_subdev *sd,
				 struct v4l2_dv_timings *timings)
{
	struct hdmi_csi_stub_state *st = sd_to_state(sd);

	mutex_lock(&st->lock);
	if (!st->signal_present) {
		mutex_unlock(&st->lock);
		return -ENOLINK;
	}
	*timings = st->timings;
	mutex_unlock(&st->lock);

	if (!v4l2_valid_dv_timings(timings, &hdmi_csi_timings_cap, NULL, NULL))
		return -ERANGE;

	return 0;
}

static int stub_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct hdmi_csi_stub_state *st = sd_to_state(sd);
	bool stream_on = !!enable;

	mutex_lock(&st->lock);
	if (st->stream_on != stream_on) {
		st->stream_on = stream_on;
		st->stream_sequence++;
		push_daemon_event(st, HDMI_CSI_EVT_STREAM_CHANGE,
				  stream_on ? 1 : 0);
	}
	mutex_unlock(&st->lock);

	v4l2_dbg(1, debug, sd, "%s: %s\n", __func__,
		 enable ? "on" : "off");
	return 0;
}

static int stub_g_frame_interval(struct v4l2_subdev *sd,
				 struct v4l2_subdev_frame_interval *fi)
{
	struct hdmi_csi_stub_state *st = sd_to_state(sd);
	unsigned int fps_val;

	mutex_lock(&st->lock);
	fps_val = calc_fps(&st->timings.bt);
	mutex_unlock(&st->lock);

	if (fps_val == 0) {
		fi->interval.numerator = 1;
		fi->interval.denominator = 60;
	} else {
		fi->interval.numerator = 1;
		fi->interval.denominator = fps_val;
	}
	return 0;
}

static const struct v4l2_subdev_video_ops stub_video_ops = {
	.g_input_status		= stub_g_input_status,
	.s_dv_timings		= stub_s_dv_timings,
	.g_dv_timings		= stub_g_dv_timings,
	.query_dv_timings	= stub_query_dv_timings,
	.s_stream		= stub_s_stream,
	.g_frame_interval	= stub_g_frame_interval,
};

/* ------------------------------------------------------------------ */
/* V4L2 subdev - pad ops                                               */
/* ------------------------------------------------------------------ */

static int stub_enum_mbus_code(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *sd_state,
			       struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(hdmi_csi_mbus_codes))
		return -EINVAL;
	code->code = hdmi_csi_mbus_codes[code->index];
	return 0;
}

/*
 * tc35874x uses enum_frame_size and enum_frame_interval with a fixed table
 * of supported_modes.  We deliberately omit these: the active resolution
 * always matches the input signal, so enumerating a fixed mode list would
 * be misleading.
 */

static int stub_get_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *sd_state,
			struct v4l2_subdev_format *format)
{
	struct hdmi_csi_stub_state *st = sd_to_state(sd);

	if (format->pad || (format->which != V4L2_SUBDEV_FORMAT_ACTIVE &&
			    format->which != V4L2_SUBDEV_FORMAT_TRY))
		return -EINVAL;

	mutex_lock(&st->lock);
	format->format.code = st->mbus_fmt_code;
	format->format.width = st->timings.bt.width;
	format->format.height = st->timings.bt.height;
	format->format.field = (st->timings.bt.interlaced)
		? V4L2_FIELD_INTERLACED : V4L2_FIELD_NONE;
	/* Re-export daemon signalling verbatim, including DEFAULT/unknown. */
	format->format.colorspace = st->colorspace;
	format->format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->format.xfer_func = V4L2_XFER_FUNC_DEFAULT;
	format->format.quantization = st->quantization;
	mutex_unlock(&st->lock);

	return 0;
}

/*
 * The source pad describes what the daemon actually transmits. TRY returns
 * that format; ACTIVE accepts a matching code without mutating source state.
 * Capture-memory layout is negotiated separately at the capture device.
 */
static int stub_set_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *sd_state,
			struct v4l2_subdev_format *format)
{
	u32 code = format->format.code;
	int ret;

	if (!supported_mbus_code(code))
		return -EINVAL;

	ret = stub_get_fmt(sd, sd_state, format);
	if (ret)
		return ret;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    code != format->format.code)
		return -EINVAL;

	return 0;
}

static int stub_g_edid(struct v4l2_subdev *sd,
		       struct v4l2_subdev_edid *edid)
{
	struct hdmi_csi_stub_state *st = sd_to_state(sd);

	memset(edid->reserved, 0, sizeof(edid->reserved));

	if (edid->pad != 0)
		return -EINVAL;

	mutex_lock(&st->lock);

	if (edid->start_block == 0 && edid->blocks == 0) {
		edid->blocks = st->edid_blocks;
		mutex_unlock(&st->lock);
		return 0;
	}

	if (st->edid_blocks == 0) {
		mutex_unlock(&st->lock);
		return -ENODATA;
	}

	if (edid->start_block >= st->edid_blocks || edid->blocks == 0) {
		mutex_unlock(&st->lock);
		return -EINVAL;
	}

	if (edid->start_block + edid->blocks > st->edid_blocks)
		edid->blocks = st->edid_blocks - edid->start_block;

	memcpy(edid->edid,
	       st->edid + edid->start_block * EDID_BLOCK_SIZE,
	       edid->blocks * EDID_BLOCK_SIZE);

	mutex_unlock(&st->lock);
	return 0;
}

static int stub_s_edid(struct v4l2_subdev *sd,
		       struct v4l2_subdev_edid *edid)
{
	struct hdmi_csi_stub_state *st = sd_to_state(sd);
	u16 edid_len;

	memset(edid->reserved, 0, sizeof(edid->reserved));

	if (edid->pad != 0)
		return -EINVAL;
	if (edid->start_block != 0)
		return -EINVAL;
	if (edid->blocks > EDID_NUM_BLOCKS_MAX) {
		edid->blocks = EDID_NUM_BLOCKS_MAX;
		return -E2BIG;
	}

	edid_len = edid->blocks * EDID_BLOCK_SIZE;

	mutex_lock(&st->lock);

	if (edid->blocks == 0) {
		st->edid_blocks = 0;
		memset(st->edid, 0, sizeof(st->edid));
	} else {
		memcpy(st->edid, edid->edid, edid_len);
		st->edid_blocks = edid->blocks;
	}

	push_daemon_event(st, HDMI_CSI_EVT_EDID_UPDATE, 0);
	mutex_unlock(&st->lock);

	v4l2_dbg(1, debug, sd, "s_edid: %u blocks\n", edid->blocks);
	return 0;
}

static int stub_enum_dv_timings(struct v4l2_subdev *sd,
				struct v4l2_enum_dv_timings *timings)
{
	if (timings->pad != 0)
		return -EINVAL;
	return v4l2_enum_dv_timings_cap(timings, &hdmi_csi_timings_cap,
					NULL, NULL);
}

static int stub_dv_timings_cap(struct v4l2_subdev *sd,
			       struct v4l2_dv_timings_cap *cap)
{
	if (cap->pad != 0)
		return -EINVAL;
	*cap = hdmi_csi_timings_cap;
	return 0;
}

static int stub_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
			      struct v4l2_mbus_config *cfg)
{
	struct hdmi_csi_stub_state *st = sd_to_state(sd);

	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->bus.mipi_csi2.num_data_lanes = st->csi_lanes;

	return 0;
}

static const struct v4l2_subdev_pad_ops stub_pad_ops = {
	.enum_mbus_code		= stub_enum_mbus_code,
	/* No enum_frame_size / enum_frame_interval: see comment above */
	.set_fmt		= stub_set_fmt,
	.get_fmt		= stub_get_fmt,
	.get_edid		= stub_g_edid,
	.set_edid		= stub_s_edid,
	.enum_dv_timings	= stub_enum_dv_timings,
	.dv_timings_cap		= stub_dv_timings_cap,
	.get_mbus_config	= stub_g_mbus_config,
};

static const struct v4l2_subdev_ops stub_ops = {
	.core  = &stub_core_ops,
	.video = &stub_video_ops,
	.pad   = &stub_pad_ops,
};

/* ------------------------------------------------------------------ */
/* Custom V4L2 controls                                                */
/* ------------------------------------------------------------------ */

static int stub_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct hdmi_csi_stub_state *st = container_of(ctrl->handler,
			struct hdmi_csi_stub_state, hdl);

	mutex_lock(&st->lock);
	switch (ctrl->id) {
	case V4L2_CID_PIXEL_RATE:
		*ctrl->p_new.p_s64 = (s64)st->timings.bt.pixelclock;
		break;
	}
	mutex_unlock(&st->lock);

	return 0;
}

static const struct v4l2_ctrl_ops stub_custom_ctrl_ops = {
	.g_volatile_ctrl = stub_g_volatile_ctrl,
};

static const struct v4l2_ctrl_config stub_ctrl_audio_sampling_rate = {
	.ops  = &stub_custom_ctrl_ops,
	.id   = HDMI_CSI_CID_AUDIO_SAMPLING_RATE,
	.name = "Audio sampling rate",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 0,
	.max  = 768000,
	.step = 1,
	.def  = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config stub_ctrl_audio_present = {
	.ops  = &stub_custom_ctrl_ops,
	.id   = HDMI_CSI_CID_AUDIO_PRESENT,
	.name = "Audio present",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min  = 0,
	.max  = 1,
	.step = 1,
	.def  = 0,
	.flags = V4L2_CTRL_FLAG_READ_ONLY,
};

/* ------------------------------------------------------------------ */
/* Misc device - daemon communication                                  */
/* ------------------------------------------------------------------ */

/*
 * The audio controls are not volatile so that V4L2_EVENT_CTRL_CH_VALUE is
 * emitted on every change.  They are set outside st->lock, because the
 * control handler lock nests outside st->lock in stub_g_volatile_ctrl().
 * Rate is published before present on the way up and after it on the way
 * down, so a consumer woken by either event never sees present=1 with a
 * stale rate.
 */
static void stub_set_audio(struct hdmi_csi_stub_state *st, bool present,
			   u32 rate)
{
	mutex_lock(&st->lock);
	st->audio_sample_rate = rate;
	st->audio_present = present;
	mutex_unlock(&st->lock);

	if (present) {
		v4l2_ctrl_s_ctrl(st->audio_sampling_rate_ctrl, rate);
		v4l2_ctrl_s_ctrl(st->audio_present_ctrl, 1);
	} else {
		v4l2_ctrl_s_ctrl(st->audio_present_ctrl, 0);
		v4l2_ctrl_s_ctrl(st->audio_sampling_rate_ctrl, rate);
	}
}

static int misc_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *m = filp->private_data;
	struct hdmi_csi_stub_state *st = misc_to_state(m);

	mutex_lock(&st->lock);
	if (st->daemon_open) {
		mutex_unlock(&st->lock);
		return -EBUSY;
	}
	st->daemon_open = true;
	event_ring_init(&st->events);
	mutex_unlock(&st->lock);

	return 0;
}

static int misc_release(struct inode *inode, struct file *filp)
{
	struct miscdevice *m = filp->private_data;
	struct hdmi_csi_stub_state *st = misc_to_state(m);
	static const struct hdmi_csi_status unknown;

	mutex_lock(&st->lock);
	st->daemon_open = false;

	/* If daemon goes away, report signal lost */
	if (st->signal_present) {
		st->signal_present = false;
		memset(&st->video, 0, sizeof(st->video));
		st->mbus_fmt_code = 0;
		st->link_freq_items[0] = 0;
		memset(&st->timings, 0, sizeof(st->timings));
		st->quantization = V4L2_QUANTIZATION_DEFAULT;
		st->colorspace = V4L2_COLORSPACE_DEFAULT;
		fire_source_change(st);
	}
	/* Nothing the daemon described is known any more */
	set_status_locked(st, &unknown);
	mutex_unlock(&st->lock);

	/* Nobody reports audio loss without the daemon, so drop it now */
	stub_set_audio(st, false, 0);

	return 0;
}

static ssize_t misc_read(struct file *filp, char __user *buf,
			 size_t count, loff_t *ppos)
{
	struct miscdevice *m = filp->private_data;
	struct hdmi_csi_stub_state *st = misc_to_state(m);
	struct hdmi_csi_event ev;
	int ret;

	if (count < sizeof(ev))
		return -EINVAL;

	mutex_lock(&st->lock);
	while (event_ring_empty(&st->events)) {
		mutex_unlock(&st->lock);
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(st->event_wq,
				!event_ring_empty(&st->events));
		if (ret)
			return ret;
		mutex_lock(&st->lock);
	}

	ret = event_ring_pop(&st->events, &ev);
	mutex_unlock(&st->lock);

	if (ret)
		return ret;

	if (copy_to_user(buf, &ev, sizeof(ev)))
		return -EFAULT;

	return sizeof(ev);
}

static __poll_t misc_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct miscdevice *m = filp->private_data;
	struct hdmi_csi_stub_state *st = misc_to_state(m);
	__poll_t mask = 0;

	poll_wait(filp, &st->event_wq, wait);

	mutex_lock(&st->lock);
	if (!event_ring_empty(&st->events))
		mask |= EPOLLIN | EPOLLRDNORM;
	mutex_unlock(&st->lock);

	return mask;
}

/* Commit a source snapshot under one lock and notify after every field is set. */
static long publish_video(struct hdmi_csi_stub_state *st,
			 const struct hdmi_csi_video *video)
{
	const struct hdmi_csi_timings *ut = &video->timings;
	u32 mbus_code = video->mbus_fmt_code;
	struct v4l2_dv_timings new_timings;
	bool valid = timings_to_v4l2(ut, &new_timings);
	bool changed;

	mutex_lock(&st->lock);
	changed = st->signal_present != valid ||
		  (valid && !v4l2_match_dv_timings(&st->timings, &new_timings,
						 0, false));
	if (st->mbus_fmt_code != mbus_code) {
		st->mbus_fmt_code = mbus_code;
		changed = true;
	}

	if (st->video.payload_encoding != video->payload_encoding)
		changed = true;
	st->video = *video;

	st->signal_present = valid;
	if (valid) {
		st->timings = new_timings;
		if (st->quantization != ut->quantization ||
		    st->colorspace != ut->colorspace)
			changed = true;
		st->quantization = ut->quantization;
		st->colorspace = ut->colorspace;
	} else {
		memset(&st->timings, 0, sizeof(st->timings));
		st->quantization = V4L2_QUANTIZATION_DEFAULT;
		st->colorspace = V4L2_COLORSPACE_DEFAULT;
	}

	/* byte_clock_kHz * 4 * 1000 -> link frequency in Hz */
	if (st->link_freq_items[0] != (s64)ut->mipi_byte_clock * 4000)
		changed = true;
	st->link_freq_items[0] = (s64)ut->mipi_byte_clock * 4000;

	if (changed)
		fire_source_change(st);
	mutex_unlock(&st->lock);
	return 0;
}

static bool valid_payload(const struct hdmi_csi_video *video)
{
	switch (video->payload_encoding) {
	case HDMI_CSI_PAYLOAD_RGB:
	case HDMI_CSI_PAYLOAD_YUV444:
		return video->mbus_fmt_code == MEDIA_BUS_FMT_RGB888_1X24;
	case HDMI_CSI_PAYLOAD_YUV422:
		return video->mbus_fmt_code == MEDIA_BUS_FMT_UYVY8_2X8;
	case HDMI_CSI_PAYLOAD_YUV420:
		return video->mbus_fmt_code == MEDIA_BUS_FMT_UV8_1X8;
	default:
		return false;
	}
}

static long misc_ioctl_report_video(struct hdmi_csi_stub_state *st,
				    void __user *arg)
{
	struct hdmi_csi_video video;
	struct v4l2_dv_timings timings;
	const struct hdmi_csi_timings *t = &video.timings;

	if (copy_from_user(&video, arg, sizeof(video)))
		return -EFAULT;

	if (!t->h_active || !t->v_active) {
		if (memchr_inv(&video, 0, sizeof(video)))
			return -EINVAL;
	} else {
		if (!supported_mbus_code(video.mbus_fmt_code) ||
		    !valid_payload(&video) ||
		    memchr_inv(video.reserved, 0, sizeof(video.reserved)) ||
		    memchr_inv(t->reserved, 0, sizeof(t->reserved)) ||
		    t->quantization > V4L2_QUANTIZATION_LIM_RANGE ||
		    t->colorspace >= V4L2_COLORSPACE_LAST ||
		    t->h_sync_pol > 1 || t->v_sync_pol > 1 || t->interlaced > 1)
			return -EINVAL;
		timings_to_v4l2(t, &timings);
		if (!t->pixel_clock ||
		    !v4l2_valid_dv_timings(&timings, &hdmi_csi_timings_cap,
					 NULL, NULL))
			return -ERANGE;
	}

	return publish_video(st, &video);
}

static long misc_ioctl_get_caps(void __user *arg)
{
	struct hdmi_csi_caps caps = {
		.abi_version = HDMI_CSI_ABI_VERSION,
		.num_mbus_codes = ARRAY_SIZE(hdmi_csi_mbus_codes),
		.features = HDMI_CSI_FEAT_STATUS,
	};

	BUILD_BUG_ON(ARRAY_SIZE(hdmi_csi_mbus_codes) > HDMI_CSI_MAX_MBUS_CODES);
	memcpy(caps.mbus_codes, hdmi_csi_mbus_codes, sizeof(hdmi_csi_mbus_codes));
	return copy_to_user(arg, &caps, sizeof(caps)) ? -EFAULT : 0;
}

static long misc_ioctl_report_audio(struct hdmi_csi_stub_state *st,
				    void __user *arg)
{
	struct hdmi_csi_audio_info ai;

	if (copy_from_user(&ai, arg, sizeof(ai)))
		return -EFAULT;

	stub_set_audio(st, !!ai.present, ai.sample_rate);

	return 0;
}

static long misc_ioctl_report_status(struct hdmi_csi_stub_state *st,
				     void __user *arg)
{
	struct hdmi_csi_status s;

	if (copy_from_user(&s, arg, sizeof(s)))
		return -EFAULT;

	if (s.rx_stage == HDMI_CSI_RX_UNKNOWN ||
	    s.rx_stage > HDMI_CSI_RX_LOCKED ||
	    s.last_error > HDMI_CSI_ERR_MAX ||
	    s.reserved1 || memchr_inv(s.reserved, 0, sizeof(s.reserved)) ||
	    s.sequence)
		return -EINVAL;

	mutex_lock(&st->lock);
	set_status_locked(st, &s);
	mutex_unlock(&st->lock);
	return 0;
}

static long misc_ioctl_get_config(struct hdmi_csi_stub_state *st,
				  void __user *arg)
{
	struct hdmi_csi_config cfg;

	memset(&cfg, 0, sizeof(cfg));

	mutex_lock(&st->lock);
	cfg.csi_lanes = st->csi_lanes;
	cfg.mbus_fmt_code = st->mbus_fmt_code;
	mutex_unlock(&st->lock);

	if (copy_to_user(arg, &cfg, sizeof(cfg)))
		return -EFAULT;

	return 0;
}

static long misc_ioctl_get_edid(struct hdmi_csi_stub_state *st,
				void __user *arg)
{
	struct hdmi_csi_edid ke;

	memset(&ke, 0, sizeof(ke));

	mutex_lock(&st->lock);
	ke.blocks = st->edid_blocks;
	if (st->edid_blocks)
		memcpy(ke.data, st->edid,
		       st->edid_blocks * EDID_BLOCK_SIZE);
	mutex_unlock(&st->lock);

	if (copy_to_user(arg, &ke, sizeof(ke)))
		return -EFAULT;

	return 0;
}

static long misc_ioctl_get_stream_status(struct hdmi_csi_stub_state *st,
					 void __user *arg)
{
	struct hdmi_csi_stream_status status;

	memset(&status, 0, sizeof(status));

	mutex_lock(&st->lock);
	status.stream_on = st->stream_on ? 1 : 0;
	status.sequence = st->stream_sequence;
	mutex_unlock(&st->lock);

	if (copy_to_user(arg, &status, sizeof(status)))
		return -EFAULT;

	return 0;
}

static long misc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *m = filp->private_data;
	struct hdmi_csi_stub_state *st = misc_to_state(m);
	void __user *uarg = (void __user *)arg;

	switch (cmd) {
	case HDMI_CSI_IOC_GET_VIDEO: {
		struct hdmi_csi_video video;

		get_video(st, &video);
		return copy_to_user(uarg, &video, sizeof(video)) ? -EFAULT : 0;
	}
	case HDMI_CSI_IOC_GET_STATUS: {
		struct hdmi_csi_status status;

		get_status(st, &status);
		return copy_to_user(uarg, &status, sizeof(status)) ? -EFAULT : 0;
	}
	case HDMI_CSI_IOC_REPORT_STATUS:
		return misc_ioctl_report_status(st, uarg);
	case HDMI_CSI_IOC_GET_CAPS:
		return misc_ioctl_get_caps(uarg);
	case HDMI_CSI_IOC_REPORT_VIDEO:
		return misc_ioctl_report_video(st, uarg);
	case HDMI_CSI_IOC_REPORT_AUDIO:
		return misc_ioctl_report_audio(st, uarg);
	case HDMI_CSI_IOC_GET_CONFIG:
		return misc_ioctl_get_config(st, uarg);
	case HDMI_CSI_IOC_GET_EDID:
		return misc_ioctl_get_edid(st, uarg);
	case HDMI_CSI_IOC_GET_STREAM_STATUS:
		return misc_ioctl_get_stream_status(st, uarg);
	default:
		return -ENOTTY;
	}
}

#ifdef CONFIG_COMPAT
static long misc_compat_ioctl(struct file *filp, unsigned int cmd,
			      unsigned long arg)
{
	return misc_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct file_operations misc_fops = {
	.owner		= THIS_MODULE,
	.open		= misc_open,
	.release	= misc_release,
	.read		= misc_read,
	.poll		= misc_poll,
	.unlocked_ioctl	= misc_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= misc_compat_ioctl,
#endif
};

/* ------------------------------------------------------------------ */
/* Probe / remove                                                      */
/* ------------------------------------------------------------------ */

static int stub_parse_dt(struct hdmi_csi_stub_state *st)
{
	struct device *dev = st->dev;
	struct device_node *node = dev->of_node;
	struct v4l2_fwnode_endpoint endpoint = { .bus_type = 0 };
	struct device_node *ep;
	int ret;

	ep = of_graph_get_next_endpoint(node, NULL);
	if (!ep) {
		dev_err(dev, "missing endpoint node\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_alloc_parse(of_fwnode_handle(ep), &endpoint);
	if (ret) {
		dev_err(dev, "failed to parse endpoint\n");
		of_node_put(ep);
		return ret;
	}

	if (endpoint.bus_type != V4L2_MBUS_CSI2_DPHY ||
	    endpoint.bus.mipi_csi2.num_data_lanes == 0) {
		dev_err(dev, "missing or invalid CSI-2 properties\n");
		ret = -EINVAL;
	} else {
		st->csi_lanes = endpoint.bus.mipi_csi2.num_data_lanes;
		st->bus = endpoint.bus.mipi_csi2;
	}

	v4l2_fwnode_endpoint_free(&endpoint);
	of_node_put(ep);
	return ret;
}

static int stub_probe(struct platform_device *pdev)
{
	struct hdmi_csi_stub_state *st;
	struct v4l2_subdev *sd;
	struct device *dev = &pdev->dev;
	int err;

	dev_info(dev, "hdmi-csi-stub driver version %02x.%02x.%02x\n",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0xff);

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->dev = dev;
	mutex_init(&st->lock);
	init_waitqueue_head(&st->event_wq);
	event_ring_init(&st->events);

	err = stub_parse_dt(st);
	if (err)
		goto err_mutex;

	/* V4L2 subdev init */
	sd = &st->sd;
	v4l2_subdev_init(sd, &stub_ops);
	sd->owner = THIS_MODULE;
	sd->dev = dev;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	/* Control handler */
	v4l2_ctrl_handler_init(&st->hdl, 4);

	st->link_freq_items[0] = 0;
	st->link_freq = v4l2_ctrl_new_int_menu(&st->hdl, NULL,
		V4L2_CID_LINK_FREQ, 0, 0, st->link_freq_items);

	st->pixel_rate = v4l2_ctrl_new_std(&st->hdl, &stub_custom_ctrl_ops,
		V4L2_CID_PIXEL_RATE, 0, S64_MAX, 1, 0);
	if (st->pixel_rate)
		st->pixel_rate->flags |= V4L2_CTRL_FLAG_VOLATILE |
					  V4L2_CTRL_FLAG_READ_ONLY;

	st->audio_sampling_rate_ctrl = v4l2_ctrl_new_custom(&st->hdl,
			&stub_ctrl_audio_sampling_rate, NULL);

	st->audio_present_ctrl = v4l2_ctrl_new_custom(&st->hdl,
			&stub_ctrl_audio_present, NULL);

	sd->ctrl_handler = &st->hdl;
	if (st->hdl.error) {
		err = st->hdl.error;
		goto err_hdl;
	}

	/* Media entity */
	st->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	err = media_entity_pads_init(&sd->entity, 1, &st->pad);
	if (err < 0)
		goto err_hdl;

	/* No format is active until the daemon publishes a complete snapshot. */
	st->mbus_fmt_code = 0;
	st->quantization = V4L2_QUANTIZATION_DEFAULT;
	st->colorspace = V4L2_COLORSPACE_DEFAULT;
	st->signal_present = false;
	st->stream_on = false;
	st->stream_sequence = 0;
	memset(&st->timings, 0, sizeof(st->timings));

	sd->dev = dev;
	snprintf(sd->name, sizeof(sd->name), "%s %s", DRIVER_NAME, dev_name(dev));

	platform_set_drvdata(pdev, st);

	err = v4l2_async_register_subdev(sd);
	if (err < 0)
		goto err_entity;

	/* Misc device for daemon */
	st->misc.minor = MISC_DYNAMIC_MINOR;
	st->misc.name = "hdmi-csi-stub";
	st->misc.fops = &misc_fops;
	err = misc_register(&st->misc);
	if (err < 0) {
		dev_err(dev, "failed to register misc device\n");
		goto err_subdev;
	}

	err = v4l2_ctrl_handler_setup(sd->ctrl_handler);
	if (err)
		goto err_misc;

	dev_info(dev, "hdmi-csi-stub registered, %u CSI lanes\n",
		 st->csi_lanes);
	return 0;

err_misc:
	misc_deregister(&st->misc);
err_subdev:
	v4l2_async_unregister_subdev(sd);
err_entity:
	media_entity_cleanup(&sd->entity);
err_hdl:
	v4l2_ctrl_handler_free(&st->hdl);
err_mutex:
	mutex_destroy(&st->lock);
	return err;
}

static int stub_remove(struct platform_device *pdev)
{
	struct hdmi_csi_stub_state *st = platform_get_drvdata(pdev);
	struct v4l2_subdev *sd = &st->sd;

	misc_deregister(&st->misc);
	v4l2_async_unregister_subdev(sd);
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&st->hdl);
	mutex_destroy(&st->lock);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id stub_of_match[] = {
	{ .compatible = "hdmi-csi-stub" },
	{ }
};
MODULE_DEVICE_TABLE(of, stub_of_match);
#endif

static struct platform_driver stub_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(stub_of_match),
	},
	.probe	= stub_probe,
	.remove	= stub_remove,
};

module_platform_driver(stub_driver);
