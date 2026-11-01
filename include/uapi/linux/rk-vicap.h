/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * RV1126B MIPI CSI-2 receiver (D-PHY, CSI-2 host, VICAP) for the LeafKVM
 * HDMI-to-CSI bridge.
 *
 * One character device, /dev/vicap, exclusive: while it is open a second
 * open() fails with -EBUSY.  It receives one CSI-2 stream into dma-bufs
 * owned by userspace and reports, per buffer, either only the completed
 * frame (whole-frame mode) or also how many leading rows of the frame are
 * already in memory while it is still being captured (progressive mode).
 *
 * This device is the receiver only.  It does not know or control the
 * transmitter: what the bridge sends (size, data type, lane rate) is agreed
 * in userspace, and the receiver is told with VICAP_IOC_SET_FORMAT.  The
 * receiver should be started before the transmitter.  If the transmitter
 * sends something else than the format says, frames end ABORTED (size error,
 * timeout) or are not received at all; the capture never writes outside the
 * configured frame of a buffer.
 *
 * Life cycle
 *  1. VICAP_IOC_GET_CAPS    lanes, maximum lane rate, data types / formats.
 *  2. VICAP_IOC_SET_FORMAT  link (lanes, lane rate, data type, virtual
 *                           channel), memory layout and mode (stopped only).
 *                           Buffers that were queued go back to userspace.
 *  3. VICAP_IOC_BUF_ADD     register dma-bufs, at any time.
 *  4. VICAP_IOC_QUEUE       hand buffers to the capture.
 *  5. VICAP_IOC_START       powers the D-PHY and the CSI-2 host and streams.
 *                           Needs no buffer: frames without one are
 *                           captured into a kernel discard area and only
 *                           counted (VICAP_REC_* sequence gap).
 *  6. poll() for EPOLLIN, read() records, use and requeue buffers.
 *  7. VICAP_IOC_STOP        every buffer goes back to userspace, pending
 *                           records are dropped.  close() implies it.
 *
 * Device removal stops capture and releases its DMA mappings. Existing file
 * descriptors remain safe to close; read/ioctl return -ENODEV and poll reports
 * EPOLLHUP | EPOLLERR.
 *
 * Buffers.  A buffer is with userspace (registered, not queued), queued, or
 * with the capture.  It goes back to userspace when the read() of its
 * VICAP_REC_DONE or VICAP_REC_ABORTED record returns.  The capture writes a
 * buffer only during the one frame assigned to it, and never after that
 * frame's DONE/ABORTED record: after a fault the hardware is stopped and idle
 * before the record is produced.  Frames are written as
 *   plane 0 row r at  r * stride
 *   plane 1 row r at  uv_offset + r * stride     (NV12/NV16 only)
 * and never beyond the configured width, height and @size, whatever arrives.
 * CPU access is bracketed with DMA_BUF_IOCTL_SYNC as for every dma-buf.
 *
 * Records.  read() returns whole struct vicap_record entries.  The kernel
 * keeps one record slot per buffer and reports the latest state of each, so
 * records never overflow: updates of a frame that was not read yet are
 * merged into one record (a CAPTURING record followed by DONE before
 * userspace read it becomes one DONE record).  A frame whose CAPTURING
 * record was never read and which then fails is requeued silently (it was
 * never seen); one that was seen ends with VICAP_REC_ABORTED.
 *
 * Progressive contract: after a CAPTURING record with @rows = n, rows [0, n)
 * of plane 0, and the plane 1 rows that cover them, are in memory and stay
 * unchanged until the buffer is queued again.  @rows only grows.
 */
#ifndef _UAPI_LINUX_RK_VICAP_H
#define _UAPI_LINUX_RK_VICAP_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define VICAP_MAX_BUFFERS	16
#define VICAP_MAX_FORMATS	16

/* CSI-2 data types the receiver takes */
#define VICAP_DT_YUV420_8_LEGACY	0x1a
#define VICAP_DT_YUV422_8		0x1e
#define VICAP_DT_RGB888			0x24

/**
 * struct vicap_format_desc - a memory format and the data type it takes
 * @fourcc:    V4L2_PIX_FMT_*.
 * @data_type: CSI-2 data type (VICAP_DT_*).
 */
struct vicap_format_desc {
	__u32 fourcc;
	__u32 data_type;
};

/**
 * struct vicap_caps - what the receiver supports (VICAP_IOC_GET_CAPS)
 * @lanes:         data lanes wired to the transmitter (device tree).
 * @num_formats:   entries in @formats.
 * @max_lane_rate: highest D-PHY lane rate, bit/s per lane.
 * @min_lane_rate: lowest D-PHY lane rate, bit/s per lane.
 * @formats:       every (fourcc, data type) pair VICAP_IOC_SET_FORMAT takes.
 */
struct vicap_caps {
	__u32 lanes;
	__u32 num_formats;
	__u64 max_lane_rate;
	__u64 min_lane_rate;
	struct vicap_format_desc formats[VICAP_MAX_FORMATS];
	__u32 reserved[6];
};

/**
 * struct vicap_format - link, memory layout and mode (VICAP_IOC_SET_FORMAT)
 * @width, @height: frame size.  Width is a multiple of 4.
 * @fourcc:     V4L2_PIX_FMT_* memory format, with the data types it takes:
 *                BGR24 (bytes B,G,R) / RGB24 / YUV24 (bytes Y,U,V)  RGB888
 *                NV16 / NV61                                        YUV422_8
 *                NV12 / NV21  (from YUV422 with chroma averaged     YUV422_8
 *                              over row pairs, or from YUV420)      YUV420_8_LEGACY
 * @stride:     in: bytes per row of both planes, 0 for the default
 *              (width rounded up to 128 pixels times the bytes per pixel of
 *              plane 0); otherwise a multiple of 8 and at least the row.
 *              out: the stride used.
 * @uv_offset:  in: offset of plane 1, 0 for stride * height; otherwise a
 *              multiple of 8 and at least that.  out: the offset used, 0 for
 *              single-plane formats.
 * @size:       out: minimum buffer size.
 * @planes:     out: 1 or 2.
 * @uv_height:  out: rows of plane 1 (height or height / 2), 0 if none.
 * @data_type:  in: CSI-2 data type of the stream (VICAP_DT_*).
 * @progress_step: 0: whole-frame mode, only DONE/ABORTED records.
 *              n > 0: progressive mode, CAPTURING records at frame start
 *              (rows 0) and every n received rows.  Even for NV12/NV21.
 * @virtual_channel: in: CSI-2 virtual channel, 0..3.
 * @lanes:      in: data lanes the transmitter uses, 0 for all wired lanes.
 *              out: the lanes used.
 * @lane_rate:  in: D-PHY lane rate in bit/s per lane (twice the link
 *              frequency), within the caps.
 * @fps_num/@fps_den: in: frame rate, 0/0 if unknown (24 Hz assumed).  A
 *              frame that does not end within two frame times plus 20 ms is
 *              aborted (VICAP_F_TIMEOUT).
 */
struct vicap_format {
	__u32 width;
	__u32 height;
	__u32 fourcc;
	__u32 stride;
	__u32 uv_offset;
	__u32 size;
	__u32 planes;
	__u32 uv_height;
	__u32 data_type;
	__u32 progress_step;
	__u32 virtual_channel;
	__u32 lanes;
	__u64 lane_rate;
	__u32 fps_num;
	__u32 fps_den;
	__u32 reserved[4];
};

/**
 * struct vicap_buffer - register or remove a buffer (VICAP_IOC_BUF_ADD/DEL)
 * @fd:    dma-buf (BUF_ADD only).  The kernel keeps its own reference.  The
 *         VICAP sits behind an IOMMU, so any dma-buf heap works.
 * @index: slot chosen by userspace, < VICAP_MAX_BUFFERS.  BUF_ADD: -EBUSY if
 *         taken.  BUF_DEL: also takes a queued buffer back; -EBUSY while the
 *         capture holds it or its DONE/ABORTED record is unread.
 */
struct vicap_buffer {
	__s32 fd;
	__u32 index;
	__u32 reserved[2];
};

enum vicap_record_type {
	VICAP_REC_CAPTURING = 1,	/* frame in progress, @rows ready */
	VICAP_REC_DONE = 2,		/* frame complete, buffer back */
	VICAP_REC_ABORTED = 3,		/* frame failed, buffer back */
};

/**
 * struct vicap_record - one entry of read()
 * @type:       enum vicap_record_type.
 * @index:      buffer.
 * @generation: counts VICAP_IOC_START calls since open(); a frame belongs
 *              to the stream that was started with this value.
 * @sequence:   frame number since START, counting frames that went to the
 *              discard area too (a gap means dropped frames).
 * @rows:       CAPTURING: rows ready.  DONE: height.  ABORTED: the rows that
 *              had been reported ready.
 * @flags:      VICAP_F_*.
 * @frame_num:  16-bit CSI-2 frame number of the frame start packet.
 * @ts_start:   CLOCK_MONOTONIC ns of the frame start interrupt.
 * @ts:         CLOCK_MONOTONIC ns of the interrupt (or event) behind this
 *              state; for DONE the DMA end of the frame.
 */
struct vicap_record {
	__u32 type;
	__u32 index;
	__u32 generation;
	__u32 sequence;
	__u32 rows;
	__u32 flags;
	__u32 frame_num;
	__u32 reserved0;
	__u64 ts_start;
	__u64 ts;
	__u32 reserved[4];
};

/* ABORTED: why.  DONE: warnings seen during the frame. */
#define VICAP_F_SIZE_ERR	(1U << 0)  /* frame size differed from the format */
#define VICAP_F_OVERFLOW	(1U << 1)  /* receiver or DMA FIFO overflow */
#define VICAP_F_BUS_ERR		(1U << 2)  /* AXI bus error */
#define VICAP_F_TIMEOUT		(1U << 3)  /* no frame end in time (signal lost?) */
#define VICAP_F_SEQ_ERR		(1U << 4)  /* interrupts out of order, resynchronised */
#define VICAP_F_BW_LACK		(1U << 6)  /* DONE: DDR bandwidth warning */
#define VICAP_F_LINK_ERR	(1U << 7)  /* DONE: CSI-2 packet error (CRC, ECC, sync) */

/**
 * struct vicap_stats - counters since open() (VICAP_IOC_GET_STATS)
 * @frames:    frame starts.
 * @done:      DONE records produced.
 * @dropped:   frames captured into the discard area (no buffer queued).
 * @aborted:   ABORTED records produced.
 * @recycled:  failed frames requeued silently (never reported).
 * @recoveries: hardware resynchronisations after a fault.
 * @size_errs, @overflows, @bus_errs, @timeouts, @seq_errs, @bw_lacks:
 *             fault events by kind.
 * @link_errs: frames with a CSI-2 host packet error (VICAP_F_LINK_ERR).
 */
struct vicap_stats {
	__u64 frames;
	__u64 done;
	__u64 dropped;
	__u64 aborted;
	__u64 recycled;
	__u64 recoveries;
	__u64 reserved0;
	__u64 size_errs;
	__u64 overflows;
	__u64 bus_errs;
	__u64 timeouts;
	__u64 seq_errs;
	__u64 bw_lacks;
	__u64 link_errs;
	__u64 reserved[2];
};

#define VICAP_IOC_MAGIC		0xA9
#define VICAP_IOC_SET_FORMAT	_IOWR(VICAP_IOC_MAGIC, 0x01, struct vicap_format)
#define VICAP_IOC_BUF_ADD	_IOW(VICAP_IOC_MAGIC, 0x02, struct vicap_buffer)
#define VICAP_IOC_BUF_DEL	_IOW(VICAP_IOC_MAGIC, 0x03, struct vicap_buffer)
#define VICAP_IOC_QUEUE		_IOW(VICAP_IOC_MAGIC, 0x04, __u32)
#define VICAP_IOC_START		_IO(VICAP_IOC_MAGIC, 0x05)
#define VICAP_IOC_STOP		_IO(VICAP_IOC_MAGIC, 0x06)
#define VICAP_IOC_GET_STATS	_IOR(VICAP_IOC_MAGIC, 0x07, struct vicap_stats)
#define VICAP_IOC_GET_CAPS	_IOR(VICAP_IOC_MAGIC, 0x08, struct vicap_caps)

#endif /* _UAPI_LINUX_RK_VICAP_H */
