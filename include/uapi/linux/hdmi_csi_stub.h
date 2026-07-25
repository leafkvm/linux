/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * HDMI-to-CSI stub driver UAPI
 *
 * This header defines the ioctl interface between a userspace HDMI receiver
 * daemon and the kernel-side CSI stub V4L2 subdev driver.  The design is
 * chip-agnostic: any HDMI-to-MIPI-CSI bridge chip whose control plane lives
 * in userspace can use this interface.
 *
 * Communication runs over /dev/hdmi-csi-stub (a misc device):
 *
 *   daemon -> kernel   ioctls to report detected timings & audio status
 *   kernel -> daemon   events delivered via read()/poll()
 *   daemon <- kernel   ioctls to retrieve EDID, configuration and status
 */

#ifndef _UAPI_HDMI_CSI_STUB_H
#define _UAPI_HDMI_CSI_STUB_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define HDMI_CSI_STUB_IOC_MAGIC		'H'

#define HDMI_CSI_EDID_BLOCK_SIZE	128
#define HDMI_CSI_EDID_MAX_BLOCKS	8
#define HDMI_CSI_EDID_MAX_SIZE		(HDMI_CSI_EDID_MAX_BLOCKS * \
					 HDMI_CSI_EDID_BLOCK_SIZE)

/* ------------------------------------------------------------------ */
/* Structures                                                         */
/* ------------------------------------------------------------------ */

/**
 * struct hdmi_csi_timings - Detected video timings reported by the daemon.
 *
 * Set h_active = 0 to indicate "no signal".
 */
struct hdmi_csi_timings {
	__u32 mipi_byte_clock;	/* MIPI byte clock in kHz, 0 = unknown */
	__u32 pixel_clock;	/* pixel clock in kHz */
	__u16 h_active;
	__u16 h_total;
	__u16 h_fp;		/* horizontal front porch */
	__u16 h_sync;		/* horizontal sync width */
	__u16 h_bp;		/* horizontal back porch */
	__u16 v_active;
	__u16 v_total;
	__u8  v_fp;		/* vertical front porch */
	__u8  v_sync;		/* vertical sync width */
	__u8  v_bp;		/* vertical back porch */
	__u8  h_sync_pol;	/* 0 = negative, 1 = positive */
	__u8  v_sync_pol;	/* 0 = negative, 1 = positive */
	__u8  interlaced;	/* 0 = progressive, 1 = interlaced */

	/*
	 * Colour signalling, reported together with the timings since both
	 * describe the same source.  Both use the standard V4L2 enum values
	 * verbatim so the kernel re-exports them through the V4L2 format API
	 * (v4l2_mbus_framefmt) without translation.  Zero (DEFAULT) means
	 * "unknown / let the sink decide", which is also the no-signal value.
	 */
	__u8  quantization;	/* enum v4l2_quantization (sample-value range):
				 *   0 = DEFAULT, 1 = FULL_RANGE, 2 = LIM_RANGE
				 */
	__u8  colorspace;	/* enum v4l2_colorspace (colorimetry):
				 *   0  = DEFAULT, 1  = SMPTE170M (BT.601/525),
				 *   3  = REC709 (BT.709),
				 *   6  = 470_SYSTEM_BG (BT.601/625),
				 *   10 = BT2020. Distinguishes BT.601 vs BT.709
				 *   vs BT.2020; ycbcr_enc/xfer_func are derived
				 *   from it by the standard V4L2 mapping.
				 */
	__u8  reserved[10];	/* zero-filled; reserved for future expansion */
};

/**
 * struct hdmi_csi_audio_info - Audio status reported by the daemon.
 */
struct hdmi_csi_audio_info {
	__u32 sample_rate;	/* Hz, 0 = no audio */
	__u8  present;		/* 0 = absent, 1 = present */
	__u8  reserved[3];
};

/**
 * struct hdmi_csi_config - Current configuration readable by the daemon.
 */
struct hdmi_csi_config {
	__u8  csi_lanes;	/* from device-tree */
	__u8  reserved1[3];
	__u32 mbus_fmt_code;	/* current media-bus format code */
};

/**
 * struct hdmi_csi_edid - EDID data readable by the daemon.
 *
 * blocks == 0 means "EDID cleared, deassert HPD".
 */
struct hdmi_csi_edid {
	__u16 blocks;		/* number of 128-byte blocks */
	__u16 reserved;
	__u8  data[HDMI_CSI_EDID_MAX_SIZE];
};

/**
 * struct hdmi_csi_stream_status - Current stream state readable by daemon.
 */
struct hdmi_csi_stream_status {
	__u8  stream_on;	/* 0 = stopped, 1 = streaming */
	__u8  reserved[3];
	__u32 sequence;		/* increments whenever stream_on changes */
};

/* ------------------------------------------------------------------ */
/* IOCTLs                                                             */
/* ------------------------------------------------------------------ */

/* Daemon -> kernel: push detected timings (or no-signal) */
#define HDMI_CSI_IOC_REPORT_TIMINGS \
	_IOW(HDMI_CSI_STUB_IOC_MAGIC, 0, struct hdmi_csi_timings)

/* Daemon -> kernel: push audio status */
#define HDMI_CSI_IOC_REPORT_AUDIO \
	_IOW(HDMI_CSI_STUB_IOC_MAGIC, 1, struct hdmi_csi_audio_info)

/* Daemon <- kernel: read current configuration */
#define HDMI_CSI_IOC_GET_CONFIG \
	_IOR(HDMI_CSI_STUB_IOC_MAGIC, 2, struct hdmi_csi_config)

/* Daemon <- kernel: read current EDID */
#define HDMI_CSI_IOC_GET_EDID \
	_IOR(HDMI_CSI_STUB_IOC_MAGIC, 3, struct hdmi_csi_edid)

/* Daemon <- kernel: read current stream state */
#define HDMI_CSI_IOC_GET_STREAM_STATUS \
	_IOR(HDMI_CSI_STUB_IOC_MAGIC, 4, struct hdmi_csi_stream_status)

/* ------------------------------------------------------------------ */
/* Events (delivered to daemon via read() on the misc device fd)      */
/* ------------------------------------------------------------------ */

#define HDMI_CSI_EVT_EDID_UPDATE	1  /* V4L2 set_edid was called */
#define HDMI_CSI_EVT_FMT_CHANGE	2  /* V4L2 set_fmt was called  */
#define HDMI_CSI_EVT_STREAM_CHANGE	3  /* value: 0 = off, 1 = on   */

/**
 * struct hdmi_csi_event - single event read by the daemon.
 *
 * The daemon should poll() / select() for POLLIN, then read() one or more
 * events.  Events are queued and never coalesced.  For level-triggered
 * state, such as stream_on, the daemon can use the matching GET_* ioctl to
 * recover the latest state if it missed an event.
 */
struct hdmi_csi_event {
	__u32 type;		/* HDMI_CSI_EVT_* */
	__u32 value;		/* event-specific payload, 0 if unused */
};

#endif /* _UAPI_HDMI_CSI_STUB_H */
