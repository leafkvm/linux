// SPDX-License-Identifier: GPL-2.0
/*
 * RV1126B MIPI CSI-2 receiver for the LeafKVM HDMI-to-CSI bridge: Innosilicon
 * D-PHY (generic PHY API), Synopsys DesignWare CSI-2 host and one VICAP MIPI
 * path, driven directly.  The userspace contract is in
 * include/uapi/linux/rk-vicap.h.
 *
 * The device tree gives this driver the VICAP block (registers, interrupt,
 * clocks, resets, IOMMU), the CSI-2 host registers, clocks and reset, and the
 * D-PHY instead of the vendor rkcif / mipi-csi2 / csi2-dphy drivers.  The
 * transmitter is not controlled from here: userspace agrees on the stream
 * with the bridge and tells this driver the link and the frame format.
 *
 * Data path: D-PHY -> CSI2HOSTn -> VICAP MIPI path n (n from the host's
 * register base), VICAP ID0.
 *
 * Hardware model (TRM chapter 6, MIPI path n at 0x200 + n * 0x200, ID0):
 *  - two frame address slots (frame0/frame1) used alternately; the address
 *    of a slot is taken at frame start, so a slot is re-pointed at the DMA
 *    end of its previous frame, one frame ahead of its next use;
 *  - frame start, periodic line and per-slot DMA end interrupts; at a line
 *    interrupt LINE_CNT rows are in memory (measured, rolling-encoding.md);
 *  - size error and FIFO overflows lose the frame; the DMA restarts on its
 *    own at the next frame, but the slot sequence is then unknown, so every
 *    fault ends the frames in flight and resynchronises: path soft reset,
 *    wait for AXI idle, re-arm from slot 0;
 *  - DMA self-adapt off and crop on: the DMA writes at most the configured
 *    rows of the configured width, whatever the source sends.  A source that
 *    changes without telling userspace therefore only produces aborted
 *    frames (size error, timeout), never writes outside a buffer's frame.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/poll.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/rk-vicap.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/time64.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <soc/rockchip/rockchip_iommu.h>

static unsigned int debug_dt;
module_param(debug_dt, uint, 0644);
MODULE_PARM_DESC(debug_dt, "test only: capture this CSI-2 data type instead of the format's");
static unsigned int inject_fault;
module_param(inject_fault, uint, 0644);
MODULE_PARM_DESC(inject_fault, "test only: fake a sequence fault at the next n line interrupts (frame starts in whole-frame mode)");

/* Synopsys DW MIPI CSI-2 host (vendor mipi-csi2.c, RK3562 TRM chapter 25) */
#define CSIHOST_N_LANES		0x04
#define CSIHOST_RESETN		0x10
#define CSIHOST_PHY_STATE	0x14
#define CSIHOST_ERR1		0x20	/* clear on read */
#define CSIHOST_ERR2		0x24	/* clear on read */
#define CSIHOST_MSK1		0x28
#define CSIHOST_MSK2		0x2c
#define CSIHOST_CONTROL		0x40
#define CSIHOST_CONTROL_DT_FS(x)	((x) << 8)
#define CSIHOST_CONTROL_DT_FE(x)	((x) << 14)
#define CSIHOST_CONTROL_DT_LS(x)	((x) << 20)
#define CSIHOST_CONTROL_DT_LE(x)	((x) << 26)
#define CSIHOST_MSK2_ERR_ID	0xf000	/* "unknown data type" per VC, as vendor */

/* RV1126B CSI2HOSTn at 0x21c00000 + n * 0x10000 feeds VICAP MIPI path n */
#define CSIHOST_BASE		0x21c00000
#define CSIHOST_STRIDE		0x10000
#define CSIHOST_NUM		4

/* D-PHY lane rates of the Innosilicon hsfreq table, bit/s per lane */
#define VICAP_MIN_LANE_RATE	80000000ULL
#define VICAP_MAX_LANE_RATE	2500000000ULL
#define VICAP_MAX_LANES		4

/* global block */
#define GLB_CTRL		0x000
#define GLB_CTRL_AXI_IDLE	BIT(31)
#define GLB_CTRL_TOISP_PROTECT	BIT(28)	/* vendor leaves it clear */
#define GLB_CTRL_MIPI_WORK(n)	BIT(20 + (n))
#define GLB_INTEN		0x004
#define GLB_INTST		0x008
#define GLB_INT_AXI_ERR		BIT(0)

/* MIPI path block, relative to 0x200 + host * 0x200, ID0 registers */
#define M_CTRL0			0x00
#define M_CTRL1			0x04
#define M_CTRL			0x20
#define M_FRM_Y(s)		(0x24 + (s) * 4)
#define M_FRM_UV(s)		(0x2c + (s) * 4)
#define M_VLW			0x34
#define M_INTEN			0x74
#define M_INTSTAT		0x78
#define M_CROP_START		0x90
#define M_SET_SIZE		0xa0
#define M_LINE_INT_NUM_ID0_1	0xb0
#define M_LINE_INT_NUM_ID2_3	0xb4
#define M_LINE_CNT_ID0_1	0xb8
#define M_SIZE_NUM		0xc0
#define M_FRAME_NUM		0xd0

#define CTRL0_CAP_EN		BIT(0)
#define CTRL0_CROP_EN		BIT(1)
#define CTRL0_DMA_EN		BIT(3)
#define CTRL0_PARSE(x)		((x) << 4)
#define CTRL0_WRDDR(x)		((x) << 8)
#define CTRL0_OUT_ORDER(x)	((x) << 14)
#define CTRL0_UVDS_EN		BIT(16)
/* bit 22 dma_adapt_en stays 0: the DMA writes the configured height */

#define CTRL_CAP_EN		BIT(0)
#define CTRL_HOST_SEL(n)	((n) << 1)
#define CTRL_WATER_LINE_25	(2 << 20)
#define CTRL_SOFT_RST_BITS	0x000a0000	/* mode "directly" + soft_rst */
#define CTRL_SOFT_RST_MASK	0x000f0000

#define VLW_FORCE_UPDATE	BIT(31)
#define LINE_INT_CIRCULAR_ID0	BIT(15)
#define LINE_CNT_MASK		0x3fff

#define INT_FS			BIT(0)
#define INT_DMA_END0		BIT(8)
#define INT_DMA_END1		BIT(9)
#define INT_DMA_ENDS		(INT_DMA_END0 | INT_DMA_END1)
#define INT_DMA_Y_OVF		BIT(16)
#define INT_DMA_UV_OVF		BIT(17)
#define INT_BW_LACK		BIT(18)
#define INT_CSI2_OVF		BIT(19)
#define INT_LINE		BIT(20)
#define INT_SIZE_ERR		BIT(24)
#define INT_OVERFLOWS		(INT_DMA_Y_OVF | INT_DMA_UV_OVF | INT_CSI2_OVF)

#define SLOT_DISCARD		(-1)

struct vicap_fmt {
	u32 fourcc;
	u8 dt;		/* CSI-2 data type */
	u8 parse;	/* CTRL0 sw_parse_type */
	u8 wrddr;	/* CTRL0 sw_wrddr_type */
	u8 order;	/* CTRL0 sw_out_order */
	u8 bpp;		/* bytes per pixel of plane 0 */
	u8 planes;
	u8 uv_vsub;	/* plane 1 rows = height / uv_vsub */
	bool uvds;	/* average chroma of row pairs (YUV422 -> 420) */
};

/*
 * RGB888 arrives as B,G,R per pixel (TRM fig. 6-11) and is stored as is;
 * the RGB888 out order swaps bytes 1/3 (RGB24) or 1/2 (YUV444 carried as
 * U,Y,V -> Y,U,V).  YUV422 arrives as UYVY (CSI-2 standard, in order 0).
 */
static const struct vicap_fmt vicap_fmts[] = {
	{ V4L2_PIX_FMT_BGR24, VICAP_DT_RGB888, 0x7, 0, 0, 3, 1, 1, false },
	{ V4L2_PIX_FMT_RGB24, VICAP_DT_RGB888, 0x7, 0, 3, 3, 1, 1, false },
	{ V4L2_PIX_FMT_YUV24, VICAP_DT_RGB888, 0x7, 0, 1, 3, 1, 1, false },
	{ V4L2_PIX_FMT_NV16, VICAP_DT_YUV422_8, 0x8, 4, 0, 1, 2, 1, false },
	{ V4L2_PIX_FMT_NV61, VICAP_DT_YUV422_8, 0x8, 4, 1, 1, 2, 1, false },
	{ V4L2_PIX_FMT_NV12, VICAP_DT_YUV422_8, 0x8, 5, 0, 1, 2, 2, true },
	{ V4L2_PIX_FMT_NV21, VICAP_DT_YUV422_8, 0x8, 5, 1, 1, 2, 2, true },
	{ V4L2_PIX_FMT_NV12, VICAP_DT_YUV420_8_LEGACY, 0xb, 5, 0, 1, 2, 2, false },
	{ V4L2_PIX_FMT_NV21, VICAP_DT_YUV420_8_LEGACY, 0xb, 5, 1, 1, 2, 2, false },
};
static_assert(ARRAY_SIZE(vicap_fmts) <= VICAP_MAX_FORMATS);

enum vbuf_state {
	B_FREE,		/* slot unused */
	B_IDLE,		/* with userspace */
	B_QUEUED,	/* waiting for a hardware slot */
	B_ACTIVE,	/* in a hardware slot */
	B_DONE,		/* terminal record not read yet */
};

struct vbuf {
	struct dma_buf *dbuf;
	struct dma_buf_attachment *att;
	struct sg_table *sgt;
	dma_addr_t dma;
	size_t size;
	enum vbuf_state state;
	struct list_head qnode;		/* v->queue */
	struct list_head rnode;		/* v->recs while pending */
	bool pending;
	bool seen;			/* a CAPTURING record of this frame was read */
	struct vicap_record rec;
};

struct slot {
	int buf;			/* buffer index or SLOT_DISCARD */
	bool busy;			/* frame started, DMA not ended */
	u32 seq;
	u32 flags;
	u32 frame_num;
	u64 ts_start;
};

struct vicap {
	/* The binding and each open file hold a reference. */
	struct kref ref;
	struct device *dev;
	void __iomem *base;
	void __iomem *mipi;
	void __iomem *csihost;
	unsigned int host;		/* CSI2HOSTn = VICAP MIPI path n */
	unsigned int wired_lanes;
	int irq;
	struct clk_bulk_data *clks;
	int nclks;
	struct reset_control_bulk_data rsts[4];	/* VICAP a, h, d, isp0 */
	struct reset_control *host_rst;
	struct phy *phy;

	struct miscdevice misc;
	struct workqueue_struct *wq;
	struct work_struct recover_work;
	struct timer_list wd;

	/* control path: open, format, buffers, start/stop, works */
	struct mutex lock;
	bool open;
	bool removed;			/* protected by lock and slock */
	bool streaming;
	bool fmt_set;
	struct vicap_format fmt;
	const struct vicap_fmt *hw;
	u64 wd_ns;			/* frame start to DMA end limit */

	/* discard area: one page mapped over the whole frame size */
	struct page *dpage;
	struct sg_table dsgt;
	bool dmapped;
	dma_addr_t discard;

	struct vbuf bufs[VICAP_MAX_BUFFERS];

	/* below: taken in hard IRQ */
	spinlock_t slock;
	bool running;			/* ISR may handle frames */
	u32 fault;			/* VICAP_F_* of a pending recovery */
	struct slot slots[2];
	unsigned int fs_slot;		/* slot of the next frame start */
	unsigned int cur;		/* slot of the latest frame start */
	u32 gen;
	u32 seq;
	struct list_head queue;
	struct list_head recs;
	struct vicap_stats stats;
	wait_queue_head_t wqh;
};

static inline u32 m_read(struct vicap *v, u32 off)
{
	return readl(v->mipi + off);
}

static inline void m_write(struct vicap *v, u32 off, u32 val)
{
	writel(val, v->mipi + off);
}

static inline void host_write(struct vicap *v, u32 off, u32 val)
{
	writel(val, v->csihost + off);
}

static const struct vicap_fmt *find_fmt(u32 fourcc, u32 dt)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vicap_fmts); i++)
		if (vicap_fmts[i].fourcc == fourcc && vicap_fmts[i].dt == dt)
			return &vicap_fmts[i];
	return NULL;
}

/* ---- records ---------------------------------------------------------- */

/* slock held */
static void mark_pending(struct vicap *v, struct vbuf *b)
{
	if (!b->pending) {
		b->pending = true;
		list_add_tail(&b->rnode, &v->recs);
	}
}

/* slock held */
static void unmark_pending(struct vbuf *b)
{
	if (b->pending) {
		b->pending = false;
		list_del(&b->rnode);
	}
}

/* slock held: frame in @sl ends; @type DONE or ABORTED */
static void finish_frame(struct vicap *v, struct slot *sl, u32 type, u32 flags,
			 u64 ts)
{
	struct vbuf *b = &v->bufs[sl->buf];

	b->rec.type = type;
	b->rec.flags = sl->flags | flags;
	b->rec.ts = ts;
	if (type == VICAP_REC_DONE) {
		b->rec.rows = v->fmt.height;
		v->stats.done++;
	} else {
		v->stats.aborted++;
	}
	b->state = B_DONE;
	mark_pending(v, b);
}

/* slock held: point @s at the next queued buffer or the discard area */
static void arm_slot(struct vicap *v, unsigned int s)
{
	struct slot *sl = &v->slots[s];
	dma_addr_t a = v->discard;
	struct vbuf *b;

	sl->busy = false;
	sl->buf = SLOT_DISCARD;
	b = list_first_entry_or_null(&v->queue, struct vbuf, qnode);
	if (b) {
		list_del(&b->qnode);
		b->state = B_ACTIVE;
		sl->buf = b - v->bufs;
		a = b->dma;
	}
	m_write(v, M_FRM_Y(s), lower_32_bits(a));
	if (v->hw->planes == 2)
		m_write(v, M_FRM_UV(s), lower_32_bits(a + v->fmt.uv_offset));
}

/*
 * slock held, hardware stopped: end the frames of both slots.  A frame a
 * reader has seen gets an ABORTED record; anything else goes back to the
 * head of the queue (@requeue) or to userspace.
 */
static void drain_slots(struct vicap *v, u32 flags, bool requeue, u64 ts)
{
	int s;

	for (s = 1; s >= 0; s--) {
		struct slot *sl = &v->slots[s];
		struct vbuf *b;

		if (sl->buf == SLOT_DISCARD)
			continue;
		b = &v->bufs[sl->buf];
		if (sl->busy && b->seen) {
			finish_frame(v, sl, VICAP_REC_ABORTED, flags, ts);
		} else {
			unmark_pending(b);
			if (sl->busy)
				v->stats.recycled++;
			if (requeue) {
				b->state = B_QUEUED;
				list_add(&b->qnode, &v->queue);
			} else {
				b->state = B_IDLE;
			}
		}
		sl->buf = SLOT_DISCARD;
		sl->busy = false;
	}
}

/* ---- interrupt -------------------------------------------------------- */

/* slock held: stop handling frames, let the recovery work resynchronise */
static void raise_fault(struct vicap *v, u32 flags)
{
	m_write(v, M_INTEN, 0);
	writel(0, v->base + GLB_INTEN);
	v->fault |= flags;
	if (flags & VICAP_F_SIZE_ERR)
		v->stats.size_errs++;
	if (flags & VICAP_F_OVERFLOW)
		v->stats.overflows++;
	if (flags & VICAP_F_BUS_ERR)
		v->stats.bus_errs++;
	if (flags & VICAP_F_TIMEOUT)
		v->stats.timeouts++;
	if (flags & VICAP_F_SEQ_ERR)
		v->stats.seq_errs++;
	queue_work(v->wq, &v->recover_work);
}

static void on_line(struct vicap *v, u32 line, u64 ts)
{
	struct slot *sl = &v->slots[v->cur];
	u32 step = v->fmt.progress_step;
	struct vbuf *b;
	u32 rows;

	/* LINE_CNT reads 0 once the frame has ended: nothing to report */
	if (!step || !sl->busy || sl->buf == SLOT_DISCARD)
		return;
	rows = min(line - line % step, v->fmt.height);
	b = &v->bufs[sl->buf];
	if (rows > b->rec.rows) {
		b->rec.rows = rows;
		b->rec.ts = ts;
		mark_pending(v, b);
	}
}

/* returns fault flags; @link: CSI-2 host ERR1 since the last DMA end */
static u32 on_dma_end(struct vicap *v, u32 st, u32 link, u64 ts)
{
	unsigned int order[2], n = 0, i;

	if ((st & INT_DMA_ENDS) == INT_DMA_ENDS) {
		/* both ended in one interrupt: the older frame first */
		order[n++] = v->cur ^ 1;
		order[n++] = v->cur;
	} else {
		order[n++] = (st & INT_DMA_END1) ? 1 : 0;
	}
	for (i = 0; i < n; i++) {
		struct slot *sl = &v->slots[order[i]];

		if (!sl->busy)
			return VICAP_F_SEQ_ERR;
		if (link) {
			sl->flags |= VICAP_F_LINK_ERR;
			v->stats.link_errs++;
		}
		if (sl->buf == SLOT_DISCARD)
			v->stats.dropped++;
		else
			finish_frame(v, sl, VICAP_REC_DONE, 0, ts);
		arm_slot(v, order[i]);
	}
	return 0;
}

static u32 on_fs(struct vicap *v, u64 ts)
{
	struct slot *sl = &v->slots[v->fs_slot];

	/* at most one frame may still be ending while the next one starts */
	if (sl->busy)
		return VICAP_F_SEQ_ERR;
	sl->busy = true;
	sl->seq = v->seq++;
	sl->flags = 0;
	sl->ts_start = ts;
	sl->frame_num = m_read(v, M_FRAME_NUM) & 0xffff;
	v->cur = v->fs_slot;
	v->fs_slot ^= 1;
	v->stats.frames++;
	if (sl->buf != SLOT_DISCARD) {
		struct vbuf *b = &v->bufs[sl->buf];

		b->seen = false;
		b->rec = (struct vicap_record) {
			.type = VICAP_REC_CAPTURING,
			.index = sl->buf,
			.generation = v->gen,
			.sequence = sl->seq,
			.frame_num = sl->frame_num,
			.ts_start = ts,
			.ts = ts,
		};
		if (v->fmt.progress_step)
			mark_pending(v, b);
	}
	mod_timer(&v->wd, jiffies + nsecs_to_jiffies(v->wd_ns) + 1);
	return 0;
}

static irqreturn_t vicap_isr(int irq, void *data)
{
	struct vicap *v = data;
	u32 st, glb, line, link = 0, fault = 0;
	bool wake;
	u64 ts;

	st = m_read(v, M_INTSTAT);
	glb = readl(v->base + GLB_INTST) & GLB_INT_AXI_ERR;
	if (!st && !glb)
		return IRQ_NONE;
	ts = ktime_get_ns();
	m_write(v, M_INTSTAT, st);
	if (glb)
		writel(glb, v->base + GLB_INTST);
	line = m_read(v, M_LINE_CNT_ID0_1) & LINE_CNT_MASK;
	if (st & INT_DMA_ENDS)
		link = readl(v->csihost + CSIHOST_ERR1);

	spin_lock(&v->slock);
	if (!v->running || v->fault) {
		spin_unlock(&v->slock);
		return IRQ_HANDLED;
	}
	wake = !list_empty(&v->recs);
	if (glb)
		fault |= VICAP_F_BUS_ERR;
	if (st & INT_SIZE_ERR)
		fault |= VICAP_F_SIZE_ERR;
	if (st & INT_OVERFLOWS)
		fault |= VICAP_F_OVERFLOW;
	if (st & INT_BW_LACK) {
		v->stats.bw_lacks++;
		if (v->slots[v->cur].busy)
			v->slots[v->cur].flags |= VICAP_F_BW_LACK;
	}
	/* test hook: fake a fault in the middle of a frame (or at its start) */
	if (unlikely(READ_ONCE(inject_fault)) &&
	    (st & (v->fmt.progress_step ? INT_LINE : INT_FS))) {
		WRITE_ONCE(inject_fault, inject_fault - 1);
		fault |= VICAP_F_SEQ_ERR;
	}
	/* time order within a frame: lines, DMA end, next frame start */
	if (!fault && (st & INT_LINE))
		on_line(v, line, ts);
	if (!fault && (st & INT_DMA_ENDS))
		fault |= on_dma_end(v, st, link, ts);
	if (!fault && (st & INT_FS))
		fault |= on_fs(v, ts);
	if (fault)
		raise_fault(v, fault);
	wake = !wake && !list_empty(&v->recs);
	spin_unlock(&v->slock);

	if (wake)
		wake_up_interruptible(&v->wqh);
	if (link)
		dev_warn_ratelimited(v->dev, "CSI-2 host ERR1 %#x\n", link);
	return IRQ_HANDLED;
}

/* a frame did not end in time: the source stopped in the middle of it */
static void vicap_wd(struct timer_list *t)
{
	struct vicap *v = from_timer(v, t, wd);
	u64 now = ktime_get_ns();
	unsigned long flags;
	int s;

	spin_lock_irqsave(&v->slock, flags);
	if (v->running && !v->fault)
		for (s = 0; s < 2; s++)
			if (v->slots[s].busy &&
			    now - v->slots[s].ts_start >= v->wd_ns) {
				raise_fault(v, VICAP_F_TIMEOUT);
				break;
			}
	spin_unlock_irqrestore(&v->slock, flags);
}

/* ---- hardware --------------------------------------------------------- */

static void hw_reset(struct vicap *v)
{
	rockchip_iommu_disable(v->dev);
	reset_control_bulk_assert(ARRAY_SIZE(v->rsts), v->rsts);
	udelay(5);
	reset_control_bulk_deassert(ARRAY_SIZE(v->rsts), v->rsts);
	rockchip_iommu_enable(v->dev);
}

/*
 * Stop the path and make sure no DMA write is outstanding: after this no
 * buffer the hardware held can be written any more.  Falls back to the CRU
 * reset if the AXI master does not go idle.
 */
static void hw_quiesce(struct vicap *v)
{
	u32 val;
	int ret;

	m_write(v, M_INTEN, 0);
	writel(0, v->base + GLB_INTEN);
	spin_lock_irq(&v->slock);
	v->running = false;
	spin_unlock_irq(&v->slock);
	synchronize_irq(v->irq);

	m_write(v, M_CTRL0, 0);
	m_write(v, M_CTRL, m_read(v, M_CTRL) & ~CTRL_CAP_EN);
	m_write(v, M_CTRL, m_read(v, M_CTRL) | CTRL_SOFT_RST_BITS);
	ret = readl_poll_timeout(v->base + GLB_CTRL, val,
				 val & GLB_CTRL_AXI_IDLE, 50, 50000);
	m_write(v, M_CTRL, m_read(v, M_CTRL) & ~CTRL_SOFT_RST_MASK);
	if (ret) {
		dev_err(v->dev, "AXI not idle after path reset, resetting VICAP\n");
		hw_reset(v);
	}
	m_write(v, M_INTSTAT, ~0u);
	writel(~0u, v->base + GLB_INTST);
}

/* program the path for v->fmt and start it; slots already armed */
static void hw_program(struct vicap *v)
{
	const struct vicap_fmt *f = v->hw;
	struct vicap_format *c = &v->fmt;
	u32 inten, val;

	val = readl(v->base + GLB_CTRL);
	val |= GLB_CTRL_MIPI_WORK(v->host);
	val &= ~GLB_CTRL_TOISP_PROTECT;
	writel(val, v->base + GLB_CTRL);

	m_write(v, M_INTSTAT, ~0u);
	writel(~0u, v->base + GLB_INTST);
	m_write(v, M_LINE_INT_NUM_ID0_1, c->progress_step ?
		c->progress_step | LINE_INT_CIRCULAR_ID0 : LINE_CNT_MASK);
	m_write(v, M_LINE_INT_NUM_ID2_3, 0x3fff3fff);

	m_write(v, M_CTRL, CTRL_WATER_LINE_25 | CTRL_HOST_SEL(v->host) |
			   CTRL_CAP_EN);
	m_write(v, M_CTRL, m_read(v, M_CTRL) | CTRL_SOFT_RST_BITS);
	m_write(v, M_CTRL, m_read(v, M_CTRL) & ~CTRL_SOFT_RST_MASK);

	m_write(v, M_SET_SIZE, c->width | c->height << 16);
	m_write(v, M_CROP_START, 0);
	m_write(v, M_VLW, c->stride | VLW_FORCE_UPDATE);
	m_write(v, M_CTRL1, (debug_dt ? debug_dt : f->dt) << 4 |
		c->virtual_channel);

	inten = INT_FS | INT_DMA_ENDS | INT_SIZE_ERR | INT_OVERFLOWS |
		INT_BW_LACK;
	if (c->progress_step)
		inten |= INT_LINE;

	spin_lock_irq(&v->slock);
	v->running = true;
	spin_unlock_irq(&v->slock);
	m_write(v, M_INTEN, inten);
	writel(GLB_INT_AXI_ERR, v->base + GLB_INTEN);
	m_write(v, M_CTRL0, CTRL0_CAP_EN | CTRL0_CROP_EN | CTRL0_DMA_EN |
			    CTRL0_PARSE(f->parse) | CTRL0_WRDDR(f->wrddr) |
			    CTRL0_OUT_ORDER(f->order) |
			    (f->uvds ? CTRL0_UVDS_EN : 0));
}

/* slock held: fresh slot state, both slots armed */
static void reset_slots(struct vicap *v)
{
	v->fault = 0;
	v->fs_slot = 0;
	v->cur = 0;
	arm_slot(v, 0);
	arm_slot(v, 1);
}

/* ---- CSI-2 host and D-PHY ------------------------------------------- */

/*
 * Host out of reset only once the D-PHY and the VICAP are ready; the
 * transmitter may already be sending.  As the vendor driver: every ERR1
 * error latched (polled at DMA end, the interrupt lines are not used), the
 * per-packet "unknown data type" errors of ERR2 masked.
 */
static void host_setup(struct vicap *v)
{
	reset_control_assert(v->host_rst);
	udelay(5);
	reset_control_deassert(v->host_rst);
	host_write(v, CSIHOST_RESETN, 0);
	host_write(v, CSIHOST_N_LANES, v->fmt.lanes - 1);
	host_write(v, CSIHOST_CONTROL,
		   CSIHOST_CONTROL_DT_FS(0x00) | CSIHOST_CONTROL_DT_FE(0x01) |
		   CSIHOST_CONTROL_DT_LS(0x02) | CSIHOST_CONTROL_DT_LE(0x03));
	host_write(v, CSIHOST_MSK1, 0);
	host_write(v, CSIHOST_MSK2, CSIHOST_MSK2_ERR_ID);
	readl(v->csihost + CSIHOST_ERR1);
	readl(v->csihost + CSIHOST_ERR2);
}

static void host_off(struct vicap *v)
{
	host_write(v, CSIHOST_RESETN, 0);
	host_write(v, CSIHOST_MSK1, ~0u);
	host_write(v, CSIHOST_MSK2, ~0u);
}

/*
 * The D-PHY default timings for a lane rate, as
 * phy_mipi_dphy_get_default_config_for_hsclk() of newer kernels computes
 * them (the receiver only uses the rate; the PHY driver validates the rest).
 */
static void dphy_default_opts(u64 rate, unsigned int lanes,
			      struct phy_configure_opts_mipi_dphy *cfg)
{
	unsigned long long ui;

	ui = ALIGN(PSEC_PER_SEC, rate);
	do_div(ui, rate);

	memset(cfg, 0, sizeof(*cfg));
	cfg->clk_post = 60000 + 52 * ui;
	cfg->clk_pre = 8;
	cfg->clk_prepare = 38000;
	cfg->clk_settle = 95000;
	cfg->clk_trail = 60000;
	cfg->clk_zero = 262000;
	cfg->hs_exit = 100000;
	cfg->hs_prepare = 40000 + 4 * ui;
	cfg->hs_zero = 105000 + 6 * ui;
	cfg->hs_settle = 85000 + 6 * ui;
	cfg->hs_skip = 40000;
	cfg->hs_trail = max(4 * 8 * ui, 60000 + 4 * 4 * ui);
	cfg->init = 100;
	cfg->lpx = 50000;
	cfg->ta_get = 5 * cfg->lpx;
	cfg->ta_go = 4 * cfg->lpx;
	cfg->ta_sure = cfg->lpx;
	cfg->wakeup = 1000;
	cfg->hs_clk_rate = rate;
	cfg->lanes = lanes;
}

static int phy_start(struct vicap *v)
{
	union phy_configure_opts opts;
	int ret;

	ret = phy_init(v->phy);
	if (ret)
		return ret;
	dphy_default_opts(v->fmt.lane_rate, v->fmt.lanes, &opts.mipi_dphy);
	ret = phy_set_mode(v->phy, PHY_MODE_MIPI_DPHY);
	if (!ret)
		ret = phy_configure(v->phy, &opts);
	if (!ret)
		ret = phy_power_on(v->phy);
	if (ret)
		phy_exit(v->phy);
	return ret;
}

static void phy_stop(struct vicap *v)
{
	phy_power_off(v->phy);
	phy_exit(v->phy);
}

/* ---- buffers ---------------------------------------------------------- */

/* the buffer's IOVA range must be one block: one address register */
static int contiguous_dma(struct sg_table *sgt, size_t size, dma_addr_t *dma)
{
	struct scatterlist *sg;
	dma_addr_t next = 0;
	size_t len = 0;
	int i;

	for_each_sgtable_dma_sg(sgt, sg, i) {
		if (i && sg_dma_address(sg) != next)
			return -EINVAL;
		if (!i)
			*dma = sg_dma_address(sg);
		next = sg_dma_address(sg) + sg_dma_len(sg);
		len += sg_dma_len(sg);
	}
	if (len < size || upper_32_bits(*dma + size - 1))
		return -EINVAL;
	return 0;
}

static int buf_add(struct vicap *v, const struct vicap_buffer *arg)
{
	struct vbuf *b;
	int ret;

	if (arg->index >= VICAP_MAX_BUFFERS)
		return -EINVAL;
	b = &v->bufs[arg->index];
	if (b->state != B_FREE)
		return -EBUSY;

	b->dbuf = dma_buf_get(arg->fd);
	if (IS_ERR(b->dbuf))
		return PTR_ERR(b->dbuf);
	b->att = dma_buf_attach(b->dbuf, v->dev);
	if (IS_ERR(b->att)) {
		ret = PTR_ERR(b->att);
		goto err_put;
	}
	b->sgt = dma_buf_map_attachment_unlocked(b->att, DMA_FROM_DEVICE);
	if (IS_ERR(b->sgt)) {
		ret = PTR_ERR(b->sgt);
		goto err_detach;
	}
	ret = contiguous_dma(b->sgt, b->dbuf->size, &b->dma);
	if (ret) {
		dev_dbg(v->dev, "buffer %u is not one DMA range\n", arg->index);
		goto err_unmap;
	}
	b->size = b->dbuf->size;
	spin_lock_irq(&v->slock);
	b->state = B_IDLE;
	spin_unlock_irq(&v->slock);
	return 0;

err_unmap:
	dma_buf_unmap_attachment_unlocked(b->att, b->sgt, DMA_FROM_DEVICE);
err_detach:
	dma_buf_detach(b->dbuf, b->att);
err_put:
	dma_buf_put(b->dbuf);
	return ret;
}

static void buf_free(struct vbuf *b)
{
	dma_buf_unmap_attachment_unlocked(b->att, b->sgt, DMA_FROM_DEVICE);
	dma_buf_detach(b->dbuf, b->att);
	dma_buf_put(b->dbuf);
	b->state = B_FREE;
}

static int buf_del(struct vicap *v, u32 index)
{
	struct vbuf *b;
	int ret = 0;

	if (index >= VICAP_MAX_BUFFERS)
		return -EINVAL;
	b = &v->bufs[index];
	spin_lock_irq(&v->slock);
	if (b->state == B_FREE) {
		ret = -ENOENT;
	} else if (b->state == B_QUEUED) {
		list_del(&b->qnode);	/* not in hardware yet */
		b->state = B_IDLE;
	} else if (b->state != B_IDLE) {
		ret = -EBUSY;
	}
	spin_unlock_irq(&v->slock);
	if (!ret)
		buf_free(b);
	return ret;
}

static int buf_queue(struct vicap *v, u32 index)
{
	struct vbuf *b;
	int ret = 0;

	if (index >= VICAP_MAX_BUFFERS)
		return -EINVAL;
	if (!v->fmt_set)
		return -EINVAL;
	b = &v->bufs[index];
	spin_lock_irq(&v->slock);
	if (b->state != B_IDLE) {
		ret = b->state == B_FREE ? -ENOENT : -EBUSY;
	} else if (b->size < v->fmt.size) {
		ret = -ENOSPC;
	} else {
		b->state = B_QUEUED;
		list_add_tail(&b->qnode, &v->queue);
	}
	spin_unlock_irq(&v->slock);
	return ret;
}

static void discard_free(struct vicap *v)
{
	if (v->dmapped)
		dma_unmap_sgtable(v->dev, &v->dsgt, DMA_FROM_DEVICE,
				  DMA_ATTR_SKIP_CPU_SYNC);
	v->dmapped = false;
	if (v->dsgt.sgl)
		sg_free_table(&v->dsgt);
	v->dsgt.sgl = NULL;
	if (v->dpage)
		__free_page(v->dpage);
	v->dpage = NULL;
}

/*
 * Frames without a buffer still need somewhere to go.  The VICAP is behind
 * an IOMMU, so a single page mapped over the whole frame size is enough.
 */
static int discard_alloc(struct vicap *v, size_t size)
{
	unsigned int n = DIV_ROUND_UP(size, PAGE_SIZE), i;
	struct scatterlist *sg;
	int ret;

	v->dpage = alloc_page(GFP_KERNEL | __GFP_ZERO);
	if (!v->dpage)
		return -ENOMEM;
	ret = sg_alloc_table(&v->dsgt, n, GFP_KERNEL);
	if (ret)
		goto err;
	for_each_sgtable_sg(&v->dsgt, sg, i)
		sg_set_page(sg, v->dpage, PAGE_SIZE, 0);
	ret = dma_map_sgtable(v->dev, &v->dsgt, DMA_FROM_DEVICE,
			      DMA_ATTR_SKIP_CPU_SYNC);
	if (ret)
		goto err;
	v->dmapped = true;
	ret = contiguous_dma(&v->dsgt, size, &v->discard);
	if (ret) {
		dev_err(v->dev, "discard area is not one IOVA range\n");
		goto err;
	}
	return 0;
err:
	discard_free(v);
	return ret;
}

/* ---- format ----------------------------------------------------------- */

static int set_format(struct vicap *v, struct vicap_format *c)
{
	const struct vicap_fmt *f;
	u32 row, uv_rows;
	u64 size, min_uv;
	int ret, i;

	if (v->streaming)
		return -EBUSY;
	if (!c->lanes)
		c->lanes = v->wired_lanes;
	if (c->lanes > v->wired_lanes || c->virtual_channel > 3 ||
	    c->lane_rate < VICAP_MIN_LANE_RATE ||
	    c->lane_rate > VICAP_MAX_LANE_RATE ||
	    (c->fps_num && !c->fps_den))
		return -EINVAL;
	f = find_fmt(c->fourcc, c->data_type);
	if (!f || !c->width || c->width % 4 || c->width > 8192 ||
	    !c->height || c->height > 8192)
		return -EINVAL;
	uv_rows = f->planes == 2 ? c->height / f->uv_vsub : 0;
	if (f->uv_vsub == 2 && (c->height % 2 || c->progress_step % 2))
		return -EINVAL;
	if (c->progress_step > min_t(u32, c->height, LINE_CNT_MASK))
		return -EINVAL;

	row = c->width * f->bpp;
	if (!c->stride)
		c->stride = ALIGN(c->width, 128) * f->bpp;
	if (c->stride < row || c->stride % 8 || c->stride >= BIT(20))
		return -EINVAL;
	min_uv = (u64)c->stride * c->height;
	if (min_uv > SZ_256M)
		return -EINVAL;
	if (f->planes == 2) {
		if (!c->uv_offset)
			c->uv_offset = min_uv;
		if (c->uv_offset < min_uv || c->uv_offset % 8)
			return -EINVAL;
		size = (u64)c->uv_offset + (u64)c->stride * uv_rows;
	} else {
		c->uv_offset = 0;
		size = min_uv;
	}
	if (size > SZ_256M)
		return -EINVAL;
	c->size = size;
	c->planes = f->planes;
	c->uv_height = uv_rows;
	memset(c->reserved, 0, sizeof(c->reserved));

	discard_free(v);
	ret = discard_alloc(v, c->size);
	if (ret) {
		v->fmt_set = false;
		return ret;
	}
	spin_lock_irq(&v->slock);
	for (i = 0; i < VICAP_MAX_BUFFERS; i++)
		if (v->bufs[i].state == B_QUEUED) {
			list_del(&v->bufs[i].qnode);
			v->bufs[i].state = B_IDLE;
		}
	spin_unlock_irq(&v->slock);
	v->fmt = *c;
	v->hw = f;
	v->fmt_set = true;
	return 0;
}

/* ---- start / stop ----------------------------------------------------- */

static int vicap_start(struct vicap *v)
{
	struct vicap_format *c = &v->fmt;
	u64 interval;
	int ret;

	if (v->streaming)
		return -EBUSY;
	if (!v->fmt_set)
		return -EINVAL;
	/* a frame must end within two frame times (24 Hz if unknown) */
	interval = c->fps_num ? div_u64((u64)NSEC_PER_SEC * c->fps_den, c->fps_num) :
		   NSEC_PER_SEC / 24;
	v->wd_ns = clamp_t(u64, 2 * interval + 20 * NSEC_PER_MSEC,
			   40 * NSEC_PER_MSEC, 250 * NSEC_PER_MSEC);

	ret = pm_runtime_resume_and_get(v->dev);
	if (ret)
		return ret;
	ret = clk_bulk_prepare_enable(v->nclks, v->clks);
	if (ret)
		goto err_pm;
	hw_reset(v);
	host_setup(v);
	ret = phy_start(v);
	if (ret) {
		dev_err(v->dev, "D-PHY start: %d\n", ret);
		goto err_clk;
	}

	spin_lock_irq(&v->slock);
	v->gen++;
	v->seq = 0;
	reset_slots(v);
	spin_unlock_irq(&v->slock);
	hw_program(v);
	host_write(v, CSIHOST_RESETN, 1);
	v->streaming = true;
	dev_dbg(v->dev, "start %ux%u %.4s dt %#x vc %u, %u lanes at %llu bit/s, stride %u step %u\n",
		c->width, c->height, (char *)&c->fourcc, c->data_type,
		c->virtual_channel, c->lanes, c->lane_rate, c->stride,
		c->progress_step);
	return 0;

err_clk:
	host_off(v);
	spin_lock_irq(&v->slock);
	drain_slots(v, 0, true, 0);
	spin_unlock_irq(&v->slock);
	clk_bulk_disable_unprepare(v->nclks, v->clks);
err_pm:
	pm_runtime_put_sync(v->dev);
	return ret;
}

/* Stop streaming: all buffers go back to userspace, pending records are dropped. */
static void vicap_stop(struct vicap *v)
{
	int i;

	if (!v->streaming)
		return;
	host_off(v);
	hw_quiesce(v);
	del_timer_sync(&v->wd);
	phy_stop(v);
	clk_bulk_disable_unprepare(v->nclks, v->clks);
	pm_runtime_put_sync(v->dev);

	spin_lock_irq(&v->slock);
	for (i = 0; i < VICAP_MAX_BUFFERS; i++) {
		struct vbuf *b = &v->bufs[i];

		if (b->state == B_QUEUED)
			list_del(&b->qnode);
		if (b->state != B_FREE) {
			unmark_pending(b);
			b->state = B_IDLE;
		}
	}
	v->slots[0].buf = v->slots[1].buf = SLOT_DISCARD;
	v->slots[0].busy = v->slots[1].busy = false;
	v->fault = 0;
	spin_unlock_irq(&v->slock);
	v->streaming = false;
	wake_up_interruptible(&v->wqh);
}

static void recover_work(struct work_struct *w)
{
	struct vicap *v = container_of(w, struct vicap, recover_work);
	u32 fault;

	mutex_lock(&v->lock);
	spin_lock_irq(&v->slock);
	fault = v->fault;
	spin_unlock_irq(&v->slock);
	if (!v->streaming || !fault)
		goto out;
	hw_quiesce(v);
	del_timer_sync(&v->wd);
	spin_lock_irq(&v->slock);
	drain_slots(v, v->fault, true, ktime_get_ns());
	reset_slots(v);
	v->stats.recoveries++;
	spin_unlock_irq(&v->slock);
	hw_program(v);
	dev_warn_ratelimited(v->dev, "resynchronised after fault %#x\n", fault);
	wake_up_interruptible(&v->wqh);
out:
	mutex_unlock(&v->lock);
}

/* The device-managed binding reference is released after IRQ/MMIO cleanup. */
static void vicap_free(struct kref *ref)
{
	struct vicap *v = container_of(ref, struct vicap, ref);

	kfree(v);
}

static void vicap_put(void *data)
{
	struct vicap *v = data;

	kref_put(&v->ref, vicap_free);
}

/* lock held, capture stopped: release all mappings while dev is still bound. */
static void buffers_free(struct vicap *v)
{
	int i;

	spin_lock_irq(&v->slock);
	for (i = 0; i < VICAP_MAX_BUFFERS; i++) {
		struct vbuf *b = &v->bufs[i];

		unmark_pending(b);
		if (b->state == B_QUEUED) {
			list_del(&b->qnode);
			b->state = B_IDLE;
		}
	}
	spin_unlock_irq(&v->slock);
	for (i = 0; i < VICAP_MAX_BUFFERS; i++)
		if (v->bufs[i].state != B_FREE)
			buf_free(&v->bufs[i]);
	discard_free(v);
	v->fmt_set = false;
}

/* ---- character device ------------------------------------------------- */

static int vicap_open(struct inode *inode, struct file *f)
{
	struct vicap *v = container_of(f->private_data, struct vicap, misc);
	int ret = 0;

	mutex_lock(&v->lock);
	if (v->removed) {
		ret = -ENODEV;
	} else if (v->open) {
		ret = -EBUSY;
	} else {
		kref_get(&v->ref);
		v->open = true;
		memset(&v->stats, 0, sizeof(v->stats));
		v->gen = 0;
		v->fmt_set = false;
	}
	mutex_unlock(&v->lock);
	f->private_data = v;
	return ret;
}

static int vicap_release(struct inode *inode, struct file *f)
{
	struct vicap *v = f->private_data;

	mutex_lock(&v->lock);
	vicap_stop(v);
	mutex_unlock(&v->lock);
	/* The work is embedded in v; its workqueue may already be removed. */
	cancel_work_sync(&v->recover_work);

	mutex_lock(&v->lock);
	if (!v->removed)
		buffers_free(v);
	/* Keep new openers out until the old session is completely cleaned up. */
	v->open = false;
	mutex_unlock(&v->lock);
	vicap_put(v);
	return 0;
}

static void get_caps(struct vicap *v, struct vicap_caps *c)
{
	unsigned int i;

	memset(c, 0, sizeof(*c));
	c->lanes = v->wired_lanes;
	c->min_lane_rate = VICAP_MIN_LANE_RATE;
	c->max_lane_rate = VICAP_MAX_LANE_RATE;
	c->num_formats = ARRAY_SIZE(vicap_fmts);
	for (i = 0; i < ARRAY_SIZE(vicap_fmts); i++) {
		c->formats[i].fourcc = vicap_fmts[i].fourcc;
		c->formats[i].data_type = vicap_fmts[i].dt;
	}
}

static long vicap_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct vicap *v = f->private_data;
	void __user *up = (void __user *)arg;
	union {
		struct vicap_caps caps;
		struct vicap_format fmt;
		struct vicap_buffer buf;
		struct vicap_stats stats;
		u32 index;
	} u;
	long ret;

	if (_IOC_SIZE(cmd) > sizeof(u))
		return -ENOTTY;
	if ((_IOC_DIR(cmd) & _IOC_WRITE) &&
	    copy_from_user(&u, up, _IOC_SIZE(cmd)))
		return -EFAULT;

	mutex_lock(&v->lock);
	if (v->removed) {
		mutex_unlock(&v->lock);
		return -ENODEV;
	}
	switch (cmd) {
	case VICAP_IOC_GET_CAPS:
		get_caps(v, &u.caps);
		ret = 0;
		break;
	case VICAP_IOC_SET_FORMAT:
		ret = set_format(v, &u.fmt);
		break;
	case VICAP_IOC_BUF_ADD:
		ret = buf_add(v, &u.buf);
		break;
	case VICAP_IOC_BUF_DEL:
		ret = buf_del(v, u.buf.index);
		break;
	case VICAP_IOC_QUEUE:
		ret = buf_queue(v, u.index);
		break;
	case VICAP_IOC_START:
		ret = vicap_start(v);
		break;
	case VICAP_IOC_STOP:
		vicap_stop(v);
		ret = 0;
		break;
	case VICAP_IOC_GET_STATS:
		spin_lock_irq(&v->slock);
		u.stats = v->stats;
		spin_unlock_irq(&v->slock);
		ret = 0;
		break;
	default:
		ret = -ENOTTY;
	}
	mutex_unlock(&v->lock);

	if (!ret && (_IOC_DIR(cmd) & _IOC_READ) &&
	    copy_to_user(up, &u, _IOC_SIZE(cmd)))
		ret = -EFAULT;
	return ret;
}

static bool have_records(struct vicap *v)
{
	return !list_empty_careful(&v->recs);
}

static ssize_t vicap_read(struct file *f, char __user *ubuf, size_t len,
			  loff_t *off)
{
	struct vicap *v = f->private_data;
	struct vicap_record tmp[16];
	unsigned int n = 0;
	int ret;

	if (len < sizeof(tmp[0]))
		return -EINVAL;
	len = min(len / sizeof(tmp[0]), ARRAY_SIZE(tmp));
	for (;;) {
		spin_lock_irq(&v->slock);
		if (v->removed) {
			spin_unlock_irq(&v->slock);
			return -ENODEV;
		}
		while (n < len && !list_empty(&v->recs)) {
			struct vbuf *b = list_first_entry(&v->recs, struct vbuf,
							  rnode);

			unmark_pending(b);
			tmp[n++] = b->rec;
			if (b->rec.type == VICAP_REC_CAPTURING)
				b->seen = true;
			else
				b->state = B_IDLE;
		}
		spin_unlock_irq(&v->slock);
		if (n)
			break;
		if (!READ_ONCE(v->streaming))
			return -EPIPE;
		if (f->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(v->wqh, READ_ONCE(v->removed) ||
					       have_records(v) ||
					       !READ_ONCE(v->streaming));
		if (ret)
			return ret;
	}
	if (copy_to_user(ubuf, tmp, n * sizeof(tmp[0])))
		return -EFAULT;
	return n * sizeof(tmp[0]);
}

static __poll_t vicap_poll(struct file *f, poll_table *wait)
{
	struct vicap *v = f->private_data;

	poll_wait(f, &v->wqh, wait);
	if (READ_ONCE(v->removed))
		return EPOLLHUP | EPOLLERR;
	return have_records(v) ? EPOLLIN | EPOLLRDNORM : 0;
}

static const struct file_operations vicap_fops = {
	.owner = THIS_MODULE,
	.open = vicap_open,
	.release = vicap_release,
	.unlocked_ioctl = vicap_ioctl,
	.read = vicap_read,
	.poll = vicap_poll,
	.llseek = noop_llseek,
};

/* ---- probe ----------------------------------------------------------- */

static const char *const vicap_reset_names[] = { "a", "h", "d", "isp0" };
static_assert(ARRAY_SIZE(vicap_reset_names) == ARRAY_SIZE(((struct vicap *)0)->rsts));

static int vicap_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct vicap *v;
	unsigned int i;
	int ret;

	v = kzalloc(sizeof(*v), GFP_KERNEL);
	if (!v)
		return -ENOMEM;
	kref_init(&v->ref);
	ret = devm_add_action_or_reset(dev, vicap_put, v);
	if (ret)
		return ret;
	v->dev = dev;
	mutex_init(&v->lock);
	spin_lock_init(&v->slock);
	init_waitqueue_head(&v->wqh);
	INIT_LIST_HEAD(&v->queue);
	INIT_LIST_HEAD(&v->recs);
	INIT_WORK(&v->recover_work, recover_work);
	timer_setup(&v->wd, vicap_wd, 0);
	v->slots[0].buf = v->slots[1].buf = SLOT_DISCARD;
	platform_set_drvdata(pdev, v);

	v->base = devm_platform_ioremap_resource_byname(pdev, "vicap");
	if (IS_ERR(v->base))
		return PTR_ERR(v->base);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "csihost");
	if (!res || res->start < CSIHOST_BASE ||
	    (res->start - CSIHOST_BASE) % CSIHOST_STRIDE ||
	    (res->start - CSIHOST_BASE) / CSIHOST_STRIDE >= CSIHOST_NUM)
		return dev_err_probe(dev, -EINVAL, "no CSI-2 host registers\n");
	v->host = (res->start - CSIHOST_BASE) / CSIHOST_STRIDE;
	v->csihost = devm_ioremap_resource(dev, res);
	if (IS_ERR(v->csihost))
		return PTR_ERR(v->csihost);
	v->mipi = v->base + 0x200 + v->host * 0x200;
	if (device_property_read_u32(dev, "leafkvm,data-lanes", &v->wired_lanes) ||
	    !v->wired_lanes || v->wired_lanes > VICAP_MAX_LANES)
		return dev_err_probe(dev, -EINVAL, "leafkvm,data-lanes: 1..4 required\n");

	v->nclks = devm_clk_bulk_get_all(dev, &v->clks);
	if (v->nclks < 0)
		return v->nclks;
	for (i = 0; i < ARRAY_SIZE(v->rsts); i++)
		v->rsts[i].id = vicap_reset_names[i];
	ret = devm_reset_control_bulk_get_exclusive(dev, ARRAY_SIZE(v->rsts),
						    v->rsts);
	if (ret)
		return dev_err_probe(dev, ret, "VICAP resets\n");
	v->host_rst = devm_reset_control_get_exclusive(dev, "csihost");
	if (IS_ERR(v->host_rst))
		return dev_err_probe(dev, PTR_ERR(v->host_rst), "csihost reset\n");
	v->phy = devm_phy_get(dev, "dphy");
	if (IS_ERR(v->phy))
		return dev_err_probe(dev, PTR_ERR(v->phy), "D-PHY\n");
	v->irq = platform_get_irq(pdev, 0);
	if (v->irq < 0)
		return v->irq;
	ret = devm_request_irq(dev, v->irq, vicap_isr, 0, dev_name(dev), v);
	if (ret)
		return ret;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	dma_set_max_seg_size(dev, UINT_MAX);
	v->wq = alloc_ordered_workqueue("vicap", WQ_HIGHPRI);
	if (!v->wq)
		return -ENOMEM;
	pm_runtime_enable(dev);

	v->misc.minor = MISC_DYNAMIC_MINOR;
	v->misc.name = "vicap";
	v->misc.fops = &vicap_fops;
	ret = misc_register(&v->misc);
	if (ret)
		goto err;
	dev_info(dev, "CSI2HOST%u -> MIPI path %u, %u data lanes, irq %d\n",
		 v->host, v->host, v->wired_lanes, v->irq);
	return 0;

err:
	pm_runtime_disable(dev);
	destroy_workqueue(v->wq);
	return ret;
}

static int vicap_remove(struct platform_device *pdev)
{
	struct vicap *v = platform_get_drvdata(pdev);

	/* Exclude new opens; existing files keep v alive but lose hardware access. */
	misc_deregister(&v->misc);
	mutex_lock(&v->lock);
	spin_lock_irq(&v->slock);
	v->removed = true;
	spin_unlock_irq(&v->slock);
	vicap_stop(v);
	buffers_free(v);
	mutex_unlock(&v->lock);
	wake_up_interruptible(&v->wqh);

	/* No IRQ/timer may queue recovery after the workqueue is destroyed. */
	devm_free_irq(&pdev->dev, v->irq, v);
	del_timer_sync(&v->wd);
	cancel_work_sync(&v->recover_work);
	destroy_workqueue(v->wq);
	pm_runtime_disable(&pdev->dev);
	return 0;
}

static const struct of_device_id vicap_of_match[] = {
	{ .compatible = "leafkvm,rv1126b-vicap" },
	{ }
};
MODULE_DEVICE_TABLE(of, vicap_of_match);

static struct platform_driver vicap_driver = {
	.probe = vicap_probe,
	.remove = vicap_remove,
	.driver = {
		.name = "rk-vicap",
		.of_match_table = vicap_of_match,
	},
};
module_platform_driver(vicap_driver);

MODULE_DESCRIPTION("RV1126B MIPI CSI-2 receiver (D-PHY, CSI-2 host, VICAP) for the LeafKVM HDMI bridge");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(DMA_BUF);
