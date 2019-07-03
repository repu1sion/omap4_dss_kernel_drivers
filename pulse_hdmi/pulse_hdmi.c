/* this program is hdmi driver for OMAP44XX wrote by pulse */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/fs.h>          /* for fops */
#include <linux/delay.h>       /* for mdelay */
#include <linux/io.h>          /* for ioremap */
#include <linux/cdev.h>        /* for cdev_init */
#include <linux/slab.h>        /* for kmalloc... damn */

#include "dss.h"
#include <plat/display.h>
#include <plat/hdmi_lib.h>
#include <plat/edid.h>

#define DSS_SUBSYS_NAME "HDMI"

#define HDMI_PLLCTRL	0x58006200
#define HDMI_PHY	0x58006300
/* ADPLLLJ internal reference clock */
#define MAX_REFCLK	2400000 /* 2.4MHz */
#define MIN_REFCLK	500000  /* 0.5Mhz */

/* PLL */
#define PLLCTRL_PLL_CONTROL		0x0ul
#define PLLCTRL_PLL_STATUS		0x4ul
#define PLLCTRL_PLL_GO			0x8ul
#define PLLCTRL_CFG1			0xCul
#define PLLCTRL_CFG2			0x10ul
#define PLLCTRL_CFG3			0x14ul
#define PLLCTRL_CFG4			0x20ul

/* HDMI PHY */
#define HDMI_TXPHY_TX_CTRL		0x0ul
#define HDMI_TXPHY_DIGITAL_CTRL		0x4ul
#define HDMI_TXPHY_POWER_CTRL		0x8ul
#define HDMI_TXPHY_PAD_CFG_CTRL		0xCul

static struct hdmi {
	void __iomem *base_phy;
	void __iomem *base_pll;
	struct mutex lock;
	int code;
	int mode;
	int deep_color;
	int lr_fr;
	int force_set;
	struct hdmi_config cfg;
	struct omap_display_platform_data *pdata;
	struct platform_device *pdev;
} hdmi;

static struct cdev hdmi_cdev;
static struct workqueue_struct *irq_wq;
static dev_t hdmi_dev_id;
static u8 edid[HDMI_EDID_MAX_LENGTH] = {0};

struct hdmi_work_struct {
	struct work_struct work;
	int r;
};

enum hdmi_ioctl_cmds {
	HDMI_ENABLE,
	HDMI_DISABLE,
	HDMI_READ_EDID,
};

struct hdmi_cm {
	int code;
	int mode;
};

struct hdmi_pll_info {
	u16 regn;
	u16 regm;
	u32 regmf;
	u16 regm2;
	u16 regsd;
	u16 dcofreq;
};

struct hdmi_hvsync_pol {
	int vsync_pol;
	int hsync_pol;
};


static DEFINE_SPINLOCK(irqstatus_lock);

/* All supported timing values that OMAP4 supports */
static const struct omap_video_timings all_timings_direct[] = {
	{640, 480, 25200, 96, 16, 48, 2, 10, 33},
	{1280, 720, 74250, 40, 440, 220, 5, 5, 20},
	{1280, 720, 74250, 40, 110, 220, 5, 5, 20},
	{720, 480, 27000, 62, 16, 60, 6, 9, 30},
	{2880, 576, 108000, 256, 48, 272, 5, 5, 39},
	{1440, 240, 27000, 124, 38, 114, 3, 4, 15},
	{1440, 288, 27000, 126, 24, 138, 3, 2, 19},
	{1920, 540, 74250, 44, 528, 148, 5, 2, 15},
	{1920, 540, 74250, 44, 88, 148, 5, 2, 15},
	{1920, 1080, 148500, 44, 88, 148, 5, 4, 36}, /* code 9 */
	{720, 576, 27000, 64, 12, 68, 5, 5, 39},
	{1440, 576, 54000, 128, 24, 136, 5, 5, 39},
	{1920, 1080, 148500, 44, 528, 148, 5, 4, 36},
	{2880, 480, 108000, 248, 64, 240, 6, 9, 30},
	{1920, 1080, 74250, 44, 638, 148, 5, 4, 36},
	/* Vesa frome here */
	{640, 480, 25175, 96, 16, 48, 2, 10, 33},
	{800, 600, 40000, 128, 40, 88, 4 , 1, 23},
	{848, 480, 33750, 112, 16, 112, 8 , 6, 23},
	{1280, 768, 79500, 128, 64, 192, 7 , 3, 20},
	{1280, 800, 83500, 128, 72, 200, 6 , 3, 22},
	{1360, 768, 85500, 112, 64, 256, 6 , 3, 18},
	{1280, 960, 108000, 112, 96, 312, 3 , 1, 36},
	{1280, 1024, 108000, 112, 48, 248, 3 , 1, 38},
	{1024, 768, 65000, 136, 24, 160, 6, 3, 29},
	{1400, 1050, 121750, 144, 88, 232, 4, 3, 32},
	{1440, 900, 106500, 152, 80, 232, 6, 3, 25},
	{1680, 1050, 146250, 176 , 104, 280, 6, 3, 30},
	{1366, 768, 85500, 143, 70, 213, 3, 3, 24},
	{1920, 1080, 148500, 44, 88, 148, 5, 4, 36},
	{1280, 768, 68250, 32, 48, 80, 7, 3, 12},
	{1400, 1050, 101000, 32, 48, 80, 4, 3, 23},
	{1680, 1050, 119000, 32, 48, 80, 6, 3, 21},
	{1280, 800, 79500, 32, 48, 80, 6, 3, 14},
	{1280, 720, 74250, 40, 110, 220, 5, 5, 20},
	{1920, 1200, 154000, 32, 48, 80, 6, 3, 26},
	/* supported 3d timings UNDEROVER full frame */
	{1280, 1470, 148350, 40, 110, 220, 5, 5, 20},
	{1280, 1470, 148500, 40, 110, 220, 5, 5, 20},
	{1280, 1470, 148500, 40, 440, 220, 5, 5, 20}
};

/* Array which maps the timing values with corresponding CEA / VESA code */
static int code_index[ARRAY_SIZE(all_timings_direct)] = {
	1,  19,  4,  2, 37,  6, 21, 20,  5, 16, 17,
	29, 31, 35, 32,
	/* <--15 CEA 22--> vesa*/
	4, 9, 0xE, 0x17, 0x1C, 0x27, 0x20, 0x23, 0x10, 0x2A,
	0X2F, 0x3A, 0X51, 0X52, 0x16, 0x29, 0x39, 0x1B, 0x55, 0X2C,
	4, 4, 19,
};

/* Map VESA code to the corresponding timing values */
static int code_vesa[86] = {
	-1, -1, -1, -1, 15, -1, -1, -1, -1, 16,
	-1, -1, -1, -1, 17, -1, 23, -1, -1, -1,
	-1, -1, 29, 18, -1, -1, -1, 32, 19, -1,
	-1, -1, 21, -1, -1, 22, -1, -1, -1, 20,
	-1, 30, 24, -1, 34, -1, -1, 25, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, 31, 26, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, 27, 28, -1, -1, 33
};

/* Map CEA code to the corresponding timing values (10 entries/line) */
static int code_cea[39] = {
	-1,  0,  3,  3,  2,  8,  5,  5, -1, -1,
	-1, -1, -1, -1, -1, -1,  9, 10, 10,  1,
	7,   6,  6, -1, -1, -1, -1, -1, -1, 11,
	11, 12, 14, -1, -1, 13, 13,  4,  4
};

/* Mapping the Timing values with the corresponding Vsync and Hsync polarity */
static const
struct hdmi_hvsync_pol hvpol_mapping[ARRAY_SIZE(all_timings_direct)] = {
	{0, 0}, {1, 1}, {1, 1}, {0, 0},
	{0, 0}, {0, 0}, {0, 0}, {1, 1},
	{1, 1}, {1, 1}, {0, 0}, {0, 0},
	{1, 1}, {0, 0}, {1, 1}, /* VESA */
	{0, 0}, {1, 1}, {1, 1}, {1, 0},
	{1, 0}, {1, 1}, {1, 1}, {1, 1},
	{0, 0}, {1, 0}, {1, 0}, {1, 0},
	{1, 1}, {1, 1}, {0, 1}, {0, 1},
	{0, 1}, {0, 1}, {1, 1}, {1, 0},
	{1, 1}, {1, 1}, {1, 1}
};

/* -----reg routines----- */
static inline void hdmi_write_reg(u32 base, u16 idx, u32 val)
{
	void __iomem *b;

	switch (base) {
	case HDMI_PHY:
		b = hdmi.base_phy;
		break;
	case HDMI_PLLCTRL:
		b = hdmi.base_pll;
		break;
	default:
		BUG();
	}
	__raw_writel(val, b + idx);
	/* DBG("write = 0x%x idx =0x%x\r\n", val, idx); */
}

static inline u32 hdmi_read_reg(u32 base, u16 idx)
{
	void __iomem *b;
	u32 l;

	switch (base) {
	case HDMI_PHY:
		b = hdmi.base_phy;
		break;
	case HDMI_PLLCTRL:
		b = hdmi.base_pll;
		break;
	default:
		BUG();
	}
	l = __raw_readl(b + idx);

	/* DBG("addr = 0x%p rd = 0x%x idx = 0x%x\r\n", (b+idx), l, idx); */
	return l;
}

#define FLD_GET(val, start, end) (((val) & FLD_MASK(start, end)) >> (end))
#define FLD_MOD(orig, val, start, end) \
	(((orig) & ~FLD_MASK(start, end)) | FLD_VAL(val, start, end))

#define REG_FLD_MOD(b, i, v, s, e) \
	hdmi_write_reg(b, i, FLD_MOD(hdmi_read_reg(b, i), v, s, e))

/* ----- end of reg routines----- */

static int hdmi_ioctl(struct inode *inode, struct file *file,
					  unsigned int cmd, unsigned long arg);

static void hdmi_enable_clocks(int enable)
{
	if (enable)
		dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1 | DSS_CLK_54M |
				DSS_CLK_96M);
	else
		dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1 | DSS_CLK_54M |
				DSS_CLK_96M);
}

static struct omap_dss_device *get_hdmi_device(void)
{
	int match(struct omap_dss_device *dssdev, void *arg)
	{
		return sysfs_streq(dssdev->name , "hdmi");
	}
	return omap_dss_find_device(NULL, match);
}

static void hdmi_configure_lr_fr(void)
{
	int ret;
/*
	if (hdmi.mode == 0 || (hdmi.mode == 1 && hdmi.code == 1)) {
		ret = hdmi_configure_lrfr(HDMI_FULL_RANGE, 1);
		if (!ret)
			dispc_setup_color_fr_lr(1);
		return;
	} else {
		ret = hdmi_configure_lrfr(HDMI_LIMITED_RANGE, 1);
		if (!ret)
			dispc_setup_color_fr_lr(0);
		return;
	}
*/
	return;
}

static void update_cfg(struct hdmi_config *cfg,
					struct omap_video_timings *timings)
{
	cfg->ppl = timings->x_res;
	cfg->lpp = timings->y_res;
	cfg->hbp = timings->hbp;
	cfg->hfp = timings->hfp;
	cfg->hsw = timings->hsw;
	cfg->vbp = timings->vbp;
	cfg->vfp = timings->vfp;
	cfg->vsw = timings->vsw;
	cfg->pixel_clock = timings->pixel_clock;
}

static void update_cfg_pol(struct hdmi_config *cfg, int  code)
{
	cfg->v_pol = hvpol_mapping[code].vsync_pol;
	cfg->h_pol = hvpol_mapping[code].hsync_pol;
}

static int hdmi_is_connected(void)
{
	return hdmi_rxdet();
}

/* -----pll routines----- */
static void compute_pll(int clkin, int phy, struct hdmi_pll_info *pi)
{
	/* run ADPLLLJ loop at max rate
	 * decrease value to be more suitable for our computations
	 */
	int refclk = MAX_REFCLK/10000;
	u32 mf;

	clkin /= 10000;

	pi->regn = (clkin / refclk) - 1;
	pi->regm = phy/refclk;
	pi->regm2 = 1;

	mf = (phy - pi->regm * refclk) * (1 << 18);
	pi->regmf = mf/(refclk);

	if (phy > 1000 * 100) {
		pi->dcofreq = 1;
	} else {
		pi->dcofreq = 0;
	}

	/* funny way to implement ceiling function */
	pi->regsd = ((pi->regm * clkin / 10) / ((pi->regn + 1) * 256) + 5) / 10;

	DSSDBG("M = %d Mf = %d\n", pi->regm, pi->regmf);
	DSSDBG("range = %d sd = %d\n", pi->dcofreq, pi->regsd);
}

static int hdmi_pll_init(int refsel, int dcofreq, struct hdmi_pll_info *fmt,
									u16 sd)
{
	u32 r;
	unsigned t = 500000;
	u32 pll = HDMI_PLLCTRL;

	/* PLL start always use manual mode */
	REG_FLD_MOD(pll, PLLCTRL_PLL_CONTROL, 0x0, 0, 0);

	r = hdmi_read_reg(pll, PLLCTRL_CFG1);
	r = FLD_MOD(r, fmt->regm, 20, 9); /* CFG1__PLL_REGM */
	r = FLD_MOD(r, fmt->regn, 8, 1);  /* CFG1__PLL_REGN */

	hdmi_write_reg(pll, PLLCTRL_CFG1, r);

	r = hdmi_read_reg(pll, PLLCTRL_CFG2);

	r = FLD_MOD(r, 0x0, 12, 12); /* PLL_HIGHFREQ divide by 2 */
	r = FLD_MOD(r, 0x1, 13, 13); /* PLL_REFEN */
	r = FLD_MOD(r, 0x0, 14, 14); /* PHY_CLKINEN de-assert during locking */

	if (dcofreq) {
		/* divider programming for 1080p */
		REG_FLD_MOD(pll, PLLCTRL_CFG3, sd, 17, 10);
		r = FLD_MOD(r, 0x4, 3, 1); /* 1000MHz and 2000MHz */
	} else {
		r = FLD_MOD(r, 0x2, 3, 1); /* 500MHz and 1000MHz */
	}

	hdmi_write_reg(pll, PLLCTRL_CFG2, r);

	r = hdmi_read_reg(pll, PLLCTRL_CFG4);
	r = FLD_MOD(r, fmt->regm2, 24, 18);
	r = FLD_MOD(r, fmt->regmf, 17, 0);

	hdmi_write_reg(pll, PLLCTRL_CFG4, r);

	/* go now */
	REG_FLD_MOD(pll, PLLCTRL_PLL_GO, 0x1ul, 0, 0);

	/* wait for bit change */
	while (FLD_GET(hdmi_read_reg(pll, PLLCTRL_PLL_GO), 0, 0))
		;

	/* Wait till the lock bit is set */
	/* read PLL status */
	while (0 == FLD_GET(hdmi_read_reg(pll, PLLCTRL_PLL_STATUS), 1, 1)) {
		udelay(1);
		if (!--t) {
			printk(KERN_WARNING "HDMI: cannot lock PLL\n");
			DSSDBG("CFG1 0x%x\n", hdmi_read_reg(pll, PLLCTRL_CFG1));
			DSSDBG("CFG2 0x%x\n", hdmi_read_reg(pll, PLLCTRL_CFG2));
			DSSDBG("CFG4 0x%x\n", hdmi_read_reg(pll, PLLCTRL_CFG4));
			return -EIO;
		}
	}

	DSSDBG("PLL locked!\n");

	return 0;
}

static int hdmi_pll_reset(void)
{
	int t = 0;

	/* SYSREEST  controled by power FSM*/
	REG_FLD_MOD(HDMI_PLLCTRL, PLLCTRL_PLL_CONTROL, 0x0, 3, 3);

	/* READ 0x0 reset is in progress */
	while (!FLD_GET(hdmi_read_reg(HDMI_PLLCTRL,
			PLLCTRL_PLL_STATUS), 0, 0)) {
		udelay(1);
		if (t++ > 1000) {
			ERR("Failed to sysrest PLL\n");
			return -ENODEV;
		}
	}

	return 0;
}

static int hdmi_pll_program(struct hdmi_pll_info *fmt)
{
	u32 r;
	int refsel;

	HDMI_PllPwr_t PllPwrWaitParam;

	/* wait for wrapper rest */
	HDMI_W1_SetWaitSoftReset();

	/* power off PLL */
	PllPwrWaitParam = HDMI_PLLPWRCMD_ALLOFF;
	r = HDMI_W1_SetWaitPllPwrState(HDMI_WP, PllPwrWaitParam);
	if (r)
		return r;

	/* power on PLL */
	PllPwrWaitParam = HDMI_PLLPWRCMD_BOTHON_ALLCLKS;
	r = HDMI_W1_SetWaitPllPwrState(HDMI_WP, PllPwrWaitParam);
	if (r)
		return r;

	hdmi_pll_reset();

	refsel = 0x3; /* select SYSCLK reference */

	r = hdmi_pll_init(refsel, fmt->dcofreq, fmt, fmt->regsd);

	return r;
}

/* -----end of pll routines----- */

/* double check the order */
static int hdmi_phy_init(u32 w1,
		u32 phy, int tmds)
{
	int r;

	DSSINFO("hdmi_phy_init() \n");

	/* wait till PHY_PWR_STATUS=LDOON */
	/* HDMI_PHYPWRCMD_LDOON = 1 */
	r = HDMI_W1_SetWaitPhyPwrState(w1, 1);
	if (r)
		return r;

	/* wait till PHY_PWR_STATUS=TXON */
	r = HDMI_W1_SetWaitPhyPwrState(w1, 2);
	if (r)
		return r;

	/* read address 0 in order to get the SCPreset done completed */
	/* Dummy access performed to solve resetdone issue */
	hdmi_read_reg(phy, HDMI_TXPHY_TX_CTRL);

	/* write to phy address 0 to configure the clock */
	/* use HFBITCLK write HDMI_TXPHY_TX_CONTROL__FREQOUT field */
	REG_FLD_MOD(phy, HDMI_TXPHY_TX_CTRL, tmds, 31, 30);

	/* write to phy address 1 to start HDMI line (TXVALID and TMDSCLKEN) */
	hdmi_write_reg(phy, HDMI_TXPHY_DIGITAL_CTRL, 0xF0000000);

	/*  write to phy address 3 to change the polarity control  */
	REG_FLD_MOD(phy, HDMI_TXPHY_PAD_CFG_CTRL, 0x1, 27, 27);

	return 0;
}

static int hdmi_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct omap_dss_device *dssdev = get_hdmi_device();
	int r = 0;

	if (dssdev == NULL)
		return -EIO;

	DSSINFO("hdmi_ioctl()\n");

	switch (cmd) {
	case HDMI_ENABLE:
		//hdmi_enable_video(dssdev);
		break;
	case HDMI_DISABLE:
		//hdmi_disable_video(dssdev);
		break;
	case HDMI_READ_EDID:
		//hdmi_get_edid(dssdev);
		break;
	default:
		r = -EINVAL;
		DSSWARN("Unsupported IOCTL\n");
		break;
	}

	return r;
}

/* OK */
static int get_timings_index(void)
{
	int code;

	if (hdmi.mode == 0)
		code = code_vesa[hdmi.code];
	else
		code = code_cea[hdmi.code];
	if (code == -1 || all_timings_direct[code].x_res >= 2048) {
		code = 9;
		hdmi.code = 16;
		hdmi.mode = 1;
	}
	DSSINFO("get_timings_index(). exit. code: %d mode: %d \n",
		hdmi.code, hdmi.mode);
	
	return code;
}

static int hdmi_read_edid(struct omap_video_timings *dp)
{
	int r = 0, ret = 0, code = 0;

	memset(edid, 0, HDMI_EDID_MAX_LENGTH);

	ret = HDMI_CORE_DDC_READEDID(HDMI_CORE_SYS, edid, HDMI_EDID_MAX_LENGTH);
	if (ret != 0) 
		printk(KERN_WARNING "HDMI failed to read E-EDID\n");

//		hdmi.code = 4; /*setting default value of 640 480 VGA*/
//		hdmi.mode = 0;

	code = get_timings_index();

	*dp = all_timings_direct[code];

	DSSDBG(KERN_INFO"hdmi read EDID:\n");

	return r;
}

/*
 * For timings change to take effect, phy needs to be off. Interrupts will be
 * ignored during this sequence to avoid queuing work items that generate
 * hotplug events.
 */
static int hdmi_reset(struct omap_dss_device *dssdev,
					enum omap_dss_reset_phase phase)
{
	int r = 0;

	if (phase & OMAP_DSS_RESET_OFF) {
		mutex_lock(&hdmi.lock);

		/* don't reset PHY unless in full power state */
		if (hdmi_power == HDMI_POWER_FULL) {
			in_reset = HDMI_IN_RESET;
			hdmi_notify_pwrchange(HDMI_EVENT_POWERPHYOFF);
			hdmi_power_off_phy(dssdev);
		}
		hdmi_power = HDMI_POWER_MIN;
	}
	if (phase & OMAP_DSS_RESET_ON) {
		if (hdmi_power == HDMI_POWER_MIN) {
			r = hdmi_reconfigure(dssdev);
			hdmi_notify_pwrchange(HDMI_EVENT_POWERPHYON);
			in_reset = 0;
		} else if (hdmi_power == HDMI_POWER_OFF) {
			r = -EINVAL;
		}
		/* succeed in HDMI_POWER_MIN state */
		mutex_unlock(&hdmi.lock);
	}
	return r;
}

static void hdmi_work_queue(struct work_struct *ws)
{
	struct hdmi_work_struct *work =
			container_of(ws, struct hdmi_work_struct, work);
	struct omap_dss_device *dssdev = get_hdmi_device();
	int r = work->r;

	DSSINFO("hdmi_work_queue() r = 0x%x \n", r);

	hdmi_is_connected();

	mutex_lock(&hdmi.lock);
	if (!dssdev)
		goto done;

	if (r & HDMI_CONNECT)
	{
		//hdmi_notify_status(dssdev, true);
		hdmi_set_irqs(0);
	}

done:
	mutex_unlock(&hdmi.lock);
	kfree(work);
}
	


/*
	unsigned long time;
	static ktime_t last_connect, last_disconnect;
	int action = 0;
*/

#if 0
	mutex_lock(&hdmi.lock);
	if (dssdev == NULL || dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		/* HDMI is disabled, there cannot be any HDMI irqs */
		goto done;
#endif


#if 0

	DSSDBG("irqstatus=%08x\n hdmi/vid_power=%d/%d",
		r, hdmi_power, video_power);

	DSSDBG("irqstatus=%08x\n dssdev->state = %d, "
		"hdmi_power = %d", r, dssdev->state, hdmi_power);
	if (r & (HDMI_CONNECT | HDMI_DISCONNECT))
		action = (hdmi_is_connected()) ? HDMI_CONNECT : HDMI_DISCONNECT;

	DSSDBG("\n\n");
	DSSINFO("<%s> action = %d, r = 0x%x, hdmi_power = %d,"
		" hdmi_connected = %d",
		__func__, action, r, hdmi_power, hdmi_connected);

	if (action & HDMI_DISCONNECT) {
		/* cancel auto-notify */
		DSSINFO("Disable Display - HDMI_DISCONNECT\n");
		mutex_unlock(&hdmi.lock);
		cancel_hot_plug_notify_work();
		mutex_lock(&hdmi.lock);
	}

	if ((action & HDMI_DISCONNECT) && !(r & HDMI_IN_RESET) &&
	    (hdmi_power == HDMI_POWER_FULL)) {
		/*
		 * Wait at least 100ms after HDMI_CONNECT to decide if
		 * cable is really disconnected
		 */
		last_disconnect = ktime_get();
		time = ktime_to_ms(ktime_sub(last_disconnect, last_connect));
		if (time < 100)
			mdelay(100 - time);
		if (hdmi_connected)
			goto done;

		hdmi_notify_status(dssdev, false);

		if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
			/* HDMI is disabled, no need to process */
			goto done;

		HDMI_W1_StopVideoFrame(HDMI_WP);
		if (dssdev->platform_disable)
			dssdev->platform_disable(dssdev);
			dispc_enable_digit_out(0);

		if (hdmi.hdmi_stop_frame_cb)
			(*hdmi.hdmi_stop_frame_cb)();

		HDMI_W1_SetWaitPllPwrState(HDMI_WP, HDMI_PLLPWRCMD_ALLOFF);
		edid_set = custom_set = false;
		set_video_power(dssdev, HDMI_POWER_MIN);

		/* turn OFF clocks on disconnect*/
		if (cpu_is_omap44xx())
			hdmi_set_48Mhz_l3_cstr(dssdev, false);

		DSSINFO("Disable Display Done - HDMI_DISCONNECT\n\n");
	}

	/* read connect timestamp */
	if (action & HDMI_CONNECT)
		last_connect = ktime_get();

	if (action & HDMI_CONNECT &&
	    ) {
		user_hpd_state = false;

		hdmi_notify_status(dssdev, true);
	}

	if ((action & HDMI_CONNECT) && (video_power == HDMI_POWER_MIN) &&
		(hdmi_power != HDMI_POWER_FULL)) {

		DSSINFO("Physical Connect\n");

		/* turn ON clocks, set L3 and core constraints on connect*/
		if (cpu_is_omap44xx())
			if (hdmi_set_48Mhz_l3_cstr(dssdev, true))
				goto done;

		edid_set = false;
		custom_set = true;
		hdmi_reconfigure(dssdev);
		if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
			/* HDMI is disabled, no need to process */
			goto done;
		custom_set = false;
		DSSINFO("Physical Connect Done:\n\n");
	}

#endif

//done:

#if 0

	if ((r & HDMI_FIRST_HPD) && (!edid_set) && (!custom_set)) {
		mutex_unlock(&hdmi.lock);
		/* cancel auto-notify - must be done outside mutex */
		cancel_hot_plug_notify_work();

		mdelay(100);
		mutex_lock(&hdmi.lock);

		/*
		 * HDMI should already be full on. We use this to read EDID
		 * the first time we enable HDMI via HPD.
		 */
		DSSINFO("Enabling display - HDMI_FIRST_HPD\n");

		if (!hdmi_connected || (hdmi_power == HDMI_POWER_MIN)) {
			DSSINFO("irqstatus=0x%08x ignoring FIRST_HPD when "
				"hdmi_connected = %d, hdmi_power = %d\n",
				r, hdmi_connected, hdmi_power);
			/* If HDMI is disconnected before FIRST HPD is processed
			 * return without reconfiguring the HDMI and do not
			 * send any hot plug event to the userspace in this case
			 *  and reset irq's before returning.
			 */
			if (hdmi_opt_clk_state) {
				hdmi_set_irqs(0);
				goto hpd_modify;
			} else
				goto done2;

		}

		hdmi_reconfigure(dssdev);
		hdmi_notify_status(dssdev, true);
		DSSINFO("Enabling display Done- HDMI_FIRST_HPD\n\n");
	}

hpd_modify:
	if (r & HDMI_HPD_MODIFY) {
		struct omap_overlay *ovl;
		int i;
		bool found = false;

		/* check if any overlays are connected to TV and
		 * send disconnect event before reconfiguring the phy
		 */
		for (i = 0; i < dssdev->manager->num_overlays; i++) {
			ovl = dssdev->manager->overlays[i];
			if (!(strcmp(ovl->manager->name, "tv")))
				if (ovl->info.enabled) {
					DSSINFO("Dettach overlays before"
						 "reconfiguring HDMI - "
						 "HDMI_HPD_MODIFY\n");
					hdmi_notify_status(dssdev, false);
					found = true;
				}
			if (found)
				break;
		}

		/*
		 * HDMI HPD state low followed by a HPD state high
		 * with more than 100ms duration is recieved so EDID should
		 * reread and HDMI reconfigued.
		 */
		DSSINFO("Reconfigure HDMI PHY - HDMI_HPD_MODIFY\n\n");
		/* force a new power-up to read EDID */
		edid_set = false;
		custom_set = false;
		hdmi_reconfigure(dssdev);
		hdmi_notify_status(dssdev, true);
		DSSINFO("Reconfigure HDMI PHY Done- HDMI_HPD_MODIFY\n\n");
	}
#endif

#if 0
done2:
	mutex_unlock(&hdmi.lock);
	kfree(work);
#endif
//}


static inline void hdmi_handle_irq_work(int r)
{
	struct hdmi_work_struct *work;
	DSSINFO("hdmi_handle_irq_work()\n");

	work = kmalloc(sizeof(*work), GFP_ATOMIC);

	if (work) {
		INIT_WORK(&work->work, hdmi_work_queue);
		work->r = r;
		queue_work(irq_wq, &work->work);
	} else {
		DSSERR("Cannot allocate memory to create work\n");
	}
}


static irqreturn_t hdmi_irq_handler(int irq, void *arg)
{
	int r = 0;
	unsigned long flags;

	/* don't let to disable clocks while we processing interrupts */
	dss_clk_lock();
	/* don't let work with HDMI regs when clocks are off */
	if (!dss_get_mainclk_state()) {
		dss_clk_unlock();
		return IRQ_HANDLED;
	}

	/* process interrupt in critical section to handle conflicts */
	spin_lock_irqsave(&irqstatus_lock, flags);

	HDMI_W1_HPD_handler(&r);

	DSSINFO("hdmi_irq_handler(). Received IRQ r=%08x\n", r);

	if ((r & HDMI_CONNECT) || (r & HDMI_FIRST_HPD))
		hdmi_enable_clocks(1);

	spin_unlock_irqrestore(&irqstatus_lock, flags);
	dss_clk_unlock();

	if (r)
		hdmi_handle_irq_work(r);

	return IRQ_HANDLED;
}

/* called from sound/soc/omap/omap-hdmi.c */
int hdmi_set_audio_power(bool _audio_on)
{
	struct omap_dss_device *dssdev = get_hdmi_device();
//	bool notify;
	int r;

	if (!dssdev)
		return -EPERM;

#if 0
	mutex_lock(&hdmi.lock_aux);
	/*
	 * For now we don't broadcase HDMI_EVENT_POWEROFF if audio is the last
	 * one turning off HDMI to prevent a possibility for an infinite loop.
	 */
	notify = _audio_on && (hdmi_power != HDMI_POWER_FULL);
	r = hdmi_set_power(dssdev, video_power, v_suspended, _audio_on);
	mutex_unlock(&hdmi.lock_aux);
	if (notify)
		hdmi_notify_pwrchange(HDMI_EVENT_POWERON);
	wake_up_interruptible(&audio_wq);
#endif
	return r;
}


static struct hdmi_cm hdmi_get_code(struct omap_video_timings *timing)
{
	int i = 0, code = -1, temp_vsync = 0, temp_hsync = 0;
	int timing_vsync = 0, timing_hsync = 0;
	struct omap_video_timings temp;
	struct hdmi_cm cm = {-1};
	DSSDBG("hdmi_get_code()");

	for (i = 0; i < ARRAY_SIZE(all_timings_direct); i++) {
		temp = all_timings_direct[i];
		if (temp.pixel_clock != timing->pixel_clock ||
		    temp.x_res != timing->x_res ||
		    temp.y_res != timing->y_res)
			continue;

		temp_hsync = temp.hfp + temp.hsw + temp.hbp;
		timing_hsync = timing->hfp + timing->hsw + timing->hbp;
		temp_vsync = temp.vfp + temp.vsw + temp.vbp;
		timing_vsync = timing->vfp + timing->vsw + timing->vbp;

		DSSDBG("Temp_hsync = %d, temp_vsync = %d, "
			"timing_hsync = %d, timing_vsync = %d",
			temp_hsync, temp_vsync, timing_hsync, timing_vsync);

		if (temp_hsync == timing_hsync && temp_vsync == timing_vsync) {
			code = i;
			cm.code = code_index[i];
			cm.mode = code < OMAP_HDMI_TIMINGS_VESA_START;
			DSSDBG("Video code = %d mode = %s\n",
			cm.code, cm.mode ? "CEA" : "VESA");
			break;
		}
	}
	return cm;
}


/* both funcs called from dsscomp/device.c */
bool is_hdmi_interlaced(void)
{
	return hdmi.cfg.interlace;
}

void get_hdmi_mode_code(struct omap_video_timings *timing, int* mode, int* code)
{
	struct hdmi_cm cm;
	cm = hdmi_get_code(timing);
	*mode = cm.mode;
	*code = cm.code;
}

/* called from edid.c */
const struct omap_video_timings *hdmi_get_omap_timing(int ix)
{
	if (ix < 0 || ix >= ARRAY_SIZE(all_timings_direct))
		return NULL;
	return all_timings_direct + ix;
}

/* called from display.c */
int hdmi_init_display(struct omap_dss_device *dssdev)
{
	DSSDBG("hdmi_init_display()\n");

	/* register HDMI specific sysfs files */
	/* note: custom_edid_timing should perhaps be moved here too,
	 * instead of generic code?  Or edid sysfs file should be moved
	 * to generic code.. either way they should be in same place..
	 */
//TODO: check - do we need sysfs files or not?
#if 0
	if (device_create_file(&dssdev->dev, &dev_attr_edid))
		DSSERR("failed to create sysfs file\n");
	if (device_create_file(&dssdev->dev, &dev_attr_yuv))
		DSSERR("failed to create sysfs file\n");
	if (device_create_file(&dssdev->dev, &dev_attr_deepcolor))
		DSSERR("failed to create sysfs file\n");
	if (device_create_file(&dssdev->dev, &dev_attr_lr_fr))
		DSSERR("failed to create sysfs file\n");
	if (device_create_file(&dssdev->dev, &dev_attr_audio_power))
		DSSERR("failed to create sysfs file\n");
	if (device_create_file(&dssdev->dev, &dev_attr_code))
		DSSERR("failed to create sysfs file\n");
#endif
	return 0;
}

static int hdmi_start_display(struct omap_dss_device *dssdev)
{
	int r;
	DSSINFO("hdmi_start_display()\n");
	/* checking for driver */
	r = omap_dss_start_device(dssdev);
	if (r) {
		DSSERR("failed to start device\n");
		goto err;
	}

	/* PAD0_HDMI_HPD_PAD1_HDMI_CEC */
	omap_writel(0x01100110, 0x4A100098);
	/* PAD0_HDMI_DDC_SCL_PAD1_HDMI_DDC_SDA */
	omap_writel(0x01100110 , 0x4A10009C);
	/* CONTROL_HDMI_TX_PHY */
	omap_writel(0x10000000, 0x4A100610);

	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);

	/* enable clock(s) */
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	dss_mainclk_state_enable();
	/* turn ON HDMI_PHY_48M_FCKL */
	hdmi_opt_clock_enable();

err:
	return r;
}

/* -----hdmi power routines----- */
static int hdmi_power_on(struct omap_dss_device *dssdev)
{
	int r = 0;
	int code = 0;
	int dirty;
	struct omap_video_timings *p;
	struct hdmi_pll_info pll_data;
	struct deep_color *vsdb_format = NULL;
	int clkin, phy = 0, max_tmds = 0, temp = 0, tmds_freq;

	DSSINFO("hdmi_power_on() \n");

	code = get_timings_index();
	dssdev->panel.timings = all_timings_direct[code];

	hdmi_enable_clocks(1);

	p = &dssdev->panel.timings;

	code = get_timings_index();

	DSSDBG("No edid set thus will be calling hdmi_read_edid");
	r = hdmi_read_edid(p);
	if (r) {
		r = -EIO;
		goto err;
	}

	vsdb_format = kzalloc(sizeof(*vsdb_format), GFP_KERNEL);
	hdmi_deep_color_support_info(edid, vsdb_format);
	DSSINFO("deep_color_bit30=%d bit36=%d max_tmds_freq=%d\n",
		vsdb_format->bit_30, vsdb_format->bit_36,
		vsdb_format->max_tmds_freq);
	max_tmds = vsdb_format->max_tmds_freq * 500;

	dirty = get_timings_index() != code;

	update_cfg(&hdmi.cfg, p);

	code = get_timings_index();
	update_cfg_pol(&hdmi.cfg, code);

	dssdev->panel.timings = all_timings_direct[code];

	/*TODO:DSS_FCK2 for now, until SYS_CLK support implemented in DSS core*/
	clkin = dss_clk_get_rate(DSS_CLK_FCK2);

	switch (hdmi.deep_color) {
	case 1:
		temp = (p->pixel_clock * 125) / 100 ;
		if (vsdb_format->bit_30) {
			if (max_tmds != 0 && max_tmds >= temp)
				phy = temp;
		} else {
			phy = temp;
		}
		hdmi.cfg.deep_color = 1;
		break;
	case 2:
		if (p->pixel_clock == 148500) {
			printk(KERN_ERR"36 bit deep color not supported");
			goto err;
		}

		temp = (p->pixel_clock * 150) / 100;

			if (vsdb_format->bit_36) {
				if (max_tmds != 0 && max_tmds >= temp)
					phy = temp;
			} else {
				printk(KERN_ERR "TV does not support Deep color");
				goto err;
			}

		hdmi.cfg.deep_color = 2;
		break;
	case 0:
	default:
		phy = p->pixel_clock;
		hdmi.cfg.deep_color = 0;
		break;
	}

	compute_pll(clkin, phy, &pll_data);

	HDMI_W1_StopVideoFrame(HDMI_WP);

	dispc_enable_digit_out(0);

	if (dirty)
		omap_dss_notify(dssdev, OMAP_DSS_SIZE_CHANGE);

	/* config the PLL and PHY first */
	r = hdmi_pll_program(&pll_data);
	if (r) {
		DSSERR("Failed to lock PLL\n");
		r = -EIO;
		goto err;
	}

	/* TMDS freq_out in the PHY should be set based on the TMDS clock */
	if (phy <= 50000)
		tmds_freq = 0x0;
	else if ((phy > 50000) && (phy <= 100000))
		tmds_freq = 0x1;
	else
		tmds_freq = 0x2;

	r = hdmi_phy_init(HDMI_WP, HDMI_PHY, tmds_freq);
	if (r) {
		DSSERR("Failed to start PHY\n");
		r = -EIO;
		goto err;
	}

	hdmi.cfg.vsi_enabled = false;

	hdmi.cfg.hdmi_dvi = hdmi_has_ieee_id((u8 *)edid) && hdmi.mode;
	hdmi.cfg.video_format = hdmi.code;
	hdmi.cfg.supports_ai = hdmi_ai_supported(edid);

	DSSINFO("%s:%d res=%dx%d ", hdmi.cfg.hdmi_dvi ? "CEA" : "VESA",
		hdmi.code, dssdev->panel.timings.x_res,
		dssdev->panel.timings.y_res);

	if ((hdmi.mode)) {
		switch (hdmi.code) {
		case 20:
		case 5:
		case 6:
		case 21:
			hdmi.cfg.interlace = 1;
			break;
		default:
			hdmi.cfg.interlace = 0;
			break;
		}
	}

	hdmi_configure_lr_fr();

	hdmi_lib_enable(&hdmi.cfg);

	hdmi_configure_lr_fr();

	/* these settings are independent of overlays */
	dss_switch_tv_hdmi(1);

	/* bypass TV gamma table */
	dispc_enable_gamma_table(0);

	/* allow idle mode */
	dispc_set_idle_mode();

	dispc_set_tv_divisor();

	/* tv size */
	dispc_set_digit_size(dssdev->panel.timings.x_res,
			dssdev->panel.timings.y_res);

	HDMI_W1_StartVideoFrame(HDMI_WP);

	dispc_enable_digit_out(1);
	hdmi_set_irqs(0);

	kfree(vsdb_format);

	return 0;
err:
	kfree(vsdb_format);
	return r;
}


static void hdmi_power_off_phy(struct omap_dss_device *dssdev)
{
	HDMI_W1_StopVideoFrame(HDMI_WP);
	hdmi_set_irqs(1);
	dssdev->manager->disable(dssdev->manager);
	HDMI_W1_SetWaitPhyPwrState(HDMI_WP, 0);
	HDMI_W1_SetWaitPllPwrState(HDMI_WP, HDMI_PLLPWRCMD_ALLOFF);
}

static void hdmi_power_off(struct omap_dss_device *dssdev)
{
	hdmi_power_off_phy(dssdev);
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
	hdmi_enable_clocks(0);

	/* TODO: disable mainclock here or not? that is the question */
}

/* -----end hdmi power routines----- */



/* -----hdmi panel driver routines----- */

/*OK*/
static int hdmi_panel_probe(struct omap_dss_device *dssdev)
{
	int code;
	DSSINFO("hdmi_panel_probe()\n");

	/* we need such flags! */
	dssdev->panel.config = OMAP_DSS_LCD_TFT |
			OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS; 
	hdmi.deep_color = 0;
	hdmi.lr_fr = HDMI_LIMITED_RANGE;
	code = get_timings_index();
	dssdev->panel.timings = all_timings_direct[code];
	return 0;
}

/* maybe called when we set hpd to 0 via sysfs */
static void hdmi_panel_disable(struct omap_dss_device *dssdev)
{
	DSSINFO("hdmi_panel_disable()\n");

	dump_stack();

	hdmi_power_off(dssdev);
	hdmi.code = 16;
	hdmi.mode = 1;
}

/* called in display.c only. never seen yet */
static int hdmi_panel_enable(struct omap_dss_device *dssdev)
{
	DSSINFO("hdmi_panel_enable()\n");
	//hdmi_enable_video(dssdev);
	return 0;
}

/*OK. called from display.c return 1 here to have HDMI display enabled*/
static bool hdmi_panel_is_enabled(struct omap_dss_device *dssdev)
{
	DSSINFO("hdmi_panel_is_enabled()\n");
	//return video_power == HDMI_POWER_FULL;
	return 1;
}

static int hdmi_panel_suspend(struct omap_dss_device *dssdev)
{
	DSSINFO("hdmi_panel_suspend()\n");
	//hdmi_suspend = true;
	//hdmi_suspend_video(dssdev);
	return 0;
}

static int hdmi_panel_resume(struct omap_dss_device *dssdev)
{
	DSSINFO("hdmi_panel_resume()\n");
	//hdmi_suspend = false;
	//hdmi_resume_video(dssdev);
	return 0;
}

static void hdmi_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	DSSINFO("hdmi_get_timings()\n");
	*timings = dssdev->panel.timings;
}

static void hdmi_set_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	DSSINFO("hdmi_set_timings()\n");
	dssdev->panel.timings = *timings;

	//TODO: do we need to check display state?
	/* turn the phy off and on to get new timings to use */
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		hdmi_reset(dssdev, OMAP_DSS_RESET_BOTH);
}

static int hdmi_check_timings(struct omap_dss_device *dssdev,
				struct omap_video_timings *timings)
{
	DSSINFO("hdmi_check_timings()\n");

	if (memcmp(&dssdev->panel.timings, timings, sizeof(*timings)) == 0)
		return 0;

	return -EINVAL;
}

/* called during init via sysfs - we need it to start hdmi! */
static int hdmi_enable_hpd(struct omap_dss_device *dssdev)
{
	int r = 0;
	DSSINFO("hdmi_enable_hpd()\n");

	mutex_lock(&hdmi.lock);

	r = hdmi_start_display(dssdev);

	if (!r)
		r = hdmi_power_on(dssdev);

	

//TODO: rewrite enable hpd
#if 0
	/* check if main display in suspend mode */
	if (dss_get_mainclk_state()) {
		r = hdmi_set_power(dssdev, video_power ? : HDMI_POWER_MIN ,
		false , audio_on);
	} else {
		dssdev->activate_after_resume = true;
		video_power = 1;
	}

#endif 
	mutex_unlock(&hdmi.lock);
	DSSINFO("hdmi_enable_hpd() exit. r=%d \n", r);
	return r;
}

/* -----end hdmi panel driver routines----- */

static const struct file_operations hdmi_fops = {
	.owner = THIS_MODULE,
	.ioctl = hdmi_ioctl,
};

static struct omap_dss_driver hdmi_driver = {
	.probe				= hdmi_panel_probe,
	.disable			= hdmi_panel_disable,
	.smart_enable			= hdmi_panel_enable,
	.smart_is_enabled		= hdmi_panel_is_enabled,
	.suspend			= hdmi_panel_suspend,
	.resume				= hdmi_panel_resume,
	.get_timings			= hdmi_get_timings,
	.set_timings			= hdmi_set_timings,
	.check_timings			= hdmi_check_timings,
//	.get_edid			= hdmi_get_edid,
//	.set_custom_edid_timing_code	= hdmi_set_custom_edid_timing_code,
	.hpd_enable			= hdmi_enable_hpd,
//	.reset				= hdmi_reset,

	.driver			= {
		.name   = "hdmi_panel",
		.owner  = THIS_MODULE,
	},
};

/* main init func called from core.c
   here we create char device for hdmi, alloc irq, set wq etc
OK */
int hdmi_init(struct platform_device *pdev)
{
	int r = 0;
	int hdmi_irq;
	struct resource *hdmi_mem;
	DSSINFO("hdmi_init()\n");

	/* init of hdmi struct */
	hdmi.pdata = pdev->dev.platform_data;
	hdmi.pdev = pdev;
	mutex_init(&hdmi.lock);
	hdmi_mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	hdmi.base_pll = ioremap(hdmi_mem->start + 0x200,
				 resource_size(hdmi_mem));
	if (!hdmi.base_pll) {
		DSSERR("can't ioremap pll\n");
		return -ENOMEM;
	}
	hdmi.base_phy = ioremap(hdmi_mem->start + 0x300, 64);	
	if (!hdmi.base_phy) {
		DSSERR("can't ioremap phy\n");
		return -ENOMEM;
	}
	hdmi_lib_init();

	/* Get the major number for this module */
	r = alloc_chrdev_region(&hdmi_dev_id, 0, 1, "hdmi");
	if (r) {
		DSSERR("can not register character device\n");
		return -ENOMEM;
	}

	/* initialize character device */
	cdev_init(&hdmi_cdev, &hdmi_fops);
	hdmi_cdev.owner = THIS_MODULE;
	hdmi_cdev.ops = &hdmi_fops;

	/* add char driver */
	r = cdev_add(&hdmi_cdev, hdmi_dev_id, 1);
	if (r) {
		DSSERR("can not add hdmi char driver\n");
		unregister_chrdev_region(hdmi_dev_id, 1);
		return -ENOMEM;
	}

	/* request irq */
	hdmi_irq = platform_get_irq(pdev, 0);
	DSSINFO("hdmi_init. irq number got: %d \n", hdmi_irq);
	r = request_irq(hdmi_irq, hdmi_irq_handler, 0, "OMAP HDMI", NULL);
	if (r) {
		DSSERR("can not get irq\n");
		return -ENOMEM;
	}

	/* create worker for hdmi workqueue */
	irq_wq = create_singlethread_workqueue("hdmi_worker");

	return omap_dss_register_driver(&hdmi_driver);
}

/* OK */
void hdmi_exit(void)
{
	DSSINFO("hdmi_exit()\n");
	hdmi_lib_exit();
	destroy_workqueue(irq_wq);
	free_irq(OMAP44XX_IRQ_DSS_HDMI, NULL);
	iounmap(hdmi.base_pll);
	iounmap(hdmi.base_phy);
}
