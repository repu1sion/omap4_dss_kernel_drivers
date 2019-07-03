/*
 * Toshiba TC358765 DSI-to-LVDS (D2L) chip driver
*/

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <video/omapdss.h> /* for omap_video_timings */
#include "d2l.h"

#define DEBUG

#ifdef DEBUG
 #define debug(x...) printk(KERN_WARNING x)
#else
 #define debug(x...)
#endif

#define debug_err(x...) printk(KERN_ERR x)

char *version = "1.00";

/* horiz timings in pixelclocks, vert timings in line clocks */
static struct omap_video_timings d2l_timings = {
	.x_res		= D2L_WIDTH,
	.y_res		= D2L_HEIGHT,
	.pixel_clock	= D2L_PCLK,
	.hfp		= D2L_HFP,
	.hsw            = D2L_HSW,
	.hbp            = D2L_HBP,
	.vfp            = D2L_VFP,
	.vsw            = D2L_VSW,
	.vbp            = D2L_VBP,
};

struct d2l_board_data { 
	int x_res;
	int y_res;
	int reset_gpio;
};

static struct d2l_vc_s {
	int chan0;
	int chan1;
} d2l_vc;

bool firstboot = true;

/* common functions */

/* get data from board file. we need it to convert void* dssdev->data
   to our type */
static struct d2l_board_data *d2l_get_board_data(struct omap_dss_device *dssdev)
{
	return (struct d2l_board_data *)dssdev->data;
}

static int d2l_read_register(struct omap_dss_device *dssdev, u16 reg)
{
	int rv;
	u8 buf[4];
	u32 val;

	rv = dsi_vc_gen_read_2(dssdev, d2l_vc.chan1, reg, buf, 4);
	if (rv < 0) {
		debug_err("%s fail\n", __func__);
		goto err;
	}
	val = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
	debug("d2l reg read %x, val=%08x\n", reg, val);

err:
	return rv;
}

static int d2l_write_register(struct omap_dss_device *dssdev, u16 reg,
		u32 value)
{
	int rv;
	u8 buf[6];

	buf[0] = (reg >> 0) & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = (value >> 0) & 0xff;
	buf[3] = (value >> 8) & 0xff;
	buf[4] = (value >> 16) & 0xff;
	buf[5] = (value >> 24) & 0xff;

	rv = dsi_vc_gen_write_nosync(dssdev, d2l_vc.chan1, buf, 6);
	debug("d2l reg write %x, val=%08x\n", reg, value);
	if (rv)
		debug_err("d2l reg write reg(%x) val(%x) failed: %d\n",
			       reg, value, rv);
	return rv;
}

static void d2l_dump_regs(struct omap_dss_device *dssdev)
{
	d2l_read_register(dssdev, PPI_TX_RX_TA);
	d2l_read_register(dssdev, PPI_LPTXTIMECNT);
	d2l_read_register(dssdev, PPI_D0S_CLRSIPOCOUNT);
	d2l_read_register(dssdev, PPI_D1S_CLRSIPOCOUNT);
	d2l_read_register(dssdev, PPI_D2S_CLRSIPOCOUNT);
	d2l_read_register(dssdev, PPI_D3S_CLRSIPOCOUNT);
	d2l_read_register(dssdev, DSI_LANEENABLE);
	d2l_read_register(dssdev, PPI_LANEENABLE);
	d2l_read_register(dssdev, PPI_STARTPPI);
	d2l_read_register(dssdev, DSI_STARTDSI);
	d2l_read_register(dssdev, LVPHY1);
	d2l_read_register(dssdev, LVPHY0);
	d2l_read_register(dssdev, VPCTRL);
	d2l_read_register(dssdev, HTIM1);
	d2l_read_register(dssdev, HTIM2);
	d2l_read_register(dssdev, VTIM1);
	d2l_read_register(dssdev, VTIM2);
	d2l_read_register(dssdev, LVCFG);
	d2l_read_register(dssdev, SYSRST);
	d2l_read_register(dssdev, VFUEN);
}

static void d2l_hw_reset(struct omap_dss_device *dssdev)
{
	struct d2l_board_data *board_data = d2l_get_board_data(dssdev);
	debug("%s\n", __func__);

	if (board_data == NULL || board_data->reset_gpio == -1)
		goto err;

	gpio_set_value(board_data->reset_gpio, 1);
	udelay(100);
	/* reset the panel */
	gpio_set_value(board_data->reset_gpio, 0);
	/* assert reset */
	udelay(100);
	gpio_set_value(board_data->reset_gpio, 1);

	/* wait after releasing reset */ 
	msleep(100);

err:
	return;
}

int pulse_dumpregs_task(struct omap_dss_device *dssdev)
{
	while (1)
	{
		msleep(5000);
		d2l_dump_regs(dssdev);
		
	}
	return 0;

}

/* init all needed D2L registers */
static void d2l_init_registers(struct omap_dss_device *dssdev)
{
//	d2l_write_register(dssdev, PPI_TX_RX_TA, 0x00000004);
	/* SYSLPTX Timing Generation Counter */
	d2l_write_register(dssdev, PPI_LPTXTIMECNT, 0x00000004);
	/* D*S_CLRSIPOCOUNT = (THS-SETTLE + THS-ZERO) / HS_byte_clock_period */
	d2l_write_register(dssdev, PPI_D0S_CLRSIPOCOUNT, 0x00000003);
	d2l_write_register(dssdev, PPI_D1S_CLRSIPOCOUNT, 0x00000003);
	d2l_write_register(dssdev, PPI_D2S_CLRSIPOCOUNT, 0x00000003);
	d2l_write_register(dssdev, PPI_D3S_CLRSIPOCOUNT, 0x00000003);	
	/* program before StartPPI register enabled!!! */
	d2l_write_register(dssdev, PPI_LANEENABLE, 0x0000001F);
	/* program before StartDSI register enabled!!! */
	d2l_write_register(dssdev, DSI_LANEENABLE, 0x0000001F);
	/* start PPI and DSI */
	d2l_write_register(dssdev, PPI_STARTPPI, 0x00000001);
	d2l_write_register(dssdev, DSI_STARTDSI, 0x00000001);

	/* it's already 0 by default */
	//d2l_write_register(dssdev, LVPHY1, 0x00000000);
	/* setup LVDS PHY */
	d2l_write_register(dssdev, LVPHY0, 0x00004006);
	/* setup videopath. on-chip video-timing gen enabled */
	d2l_write_register(dssdev, VPCTRL, 0x00F00110);
	/* setup video timings because VTGEN enabled */
	d2l_write_register(dssdev, HTIM1, (D2L_HBP << 16 | D2L_HSW));
	d2l_write_register(dssdev, HTIM2, (D2L_HFP << 16 | D2L_WIDTH));
	d2l_write_register(dssdev, VTIM1, (D2L_VBP << 16 | D2L_VSW));
	d2l_write_register(dssdev, VTIM2, (D2L_VFP << 16 | D2L_HEIGHT));
	/* enable LVDS transmitter. DSI_CLOCK/3 chosen as default */
	d2l_write_register(dssdev, LVCFG, 0x00000001);

	/* softreset for LCD controller, LVDS-PHY, Line Buffer */
	d2l_write_register(dssdev, SYSRST, 0x00000004);
	/* upload new timings to hw registers on next vsync */
	d2l_write_register(dssdev, VFUEN, 0x00000001);

	kernel_thread((int (*)(void *))pulse_dumpregs_task,dssdev, 0);
}

/* init all needed D2L registers during resume*/
static void d2l_resume_init_registers(struct omap_dss_device *dssdev)
{
//	d2l_write_register(dssdev, PPI_TX_RX_TA, 0x00000004);
	/* SYSLPTX Timing Generation Counter */
	d2l_write_register(dssdev, PPI_LPTXTIMECNT, 0x00000004);
	/* D*S_CLRSIPOCOUNT = (THS-SETTLE + THS-ZERO) / HS_byte_clock_period */
	d2l_write_register(dssdev, PPI_D0S_CLRSIPOCOUNT, 0x00000003);
	d2l_write_register(dssdev, PPI_D1S_CLRSIPOCOUNT, 0x00000003);
	d2l_write_register(dssdev, PPI_D2S_CLRSIPOCOUNT, 0x00000003);
	d2l_write_register(dssdev, PPI_D3S_CLRSIPOCOUNT, 0x00000003);	
	/* program before StartPPI register enabled!!! */
	d2l_write_register(dssdev, PPI_LANEENABLE, 0x0000001F);
	/* program before StartDSI register enabled!!! */
	d2l_write_register(dssdev, DSI_LANEENABLE, 0x0000001F);
	/* start PPI and DSI */
	d2l_write_register(dssdev, PPI_STARTPPI, 0x00000001);
	d2l_write_register(dssdev, DSI_STARTDSI, 0x00000001);

	/* it's already 0 by default */
	//d2l_write_register(dssdev, LVPHY1, 0x00000000);
	/* setup LVDS PHY */
	d2l_write_register(dssdev, LVPHY0, 0x00004006);
	/* setup videopath. on-chip video-timing gen enabled */
	d2l_write_register(dssdev, VPCTRL, 0x00F00110);
	/* setup video timings because VTGEN enabled */
	d2l_write_register(dssdev, HTIM1, (D2L_HBP << 16 | D2L_HSW));
	d2l_write_register(dssdev, HTIM2, (D2L_HFP << 16 | D2L_WIDTH));
	d2l_write_register(dssdev, VTIM1, (D2L_VBP << 16 | D2L_VSW));
	d2l_write_register(dssdev, VTIM2, (D2L_VFP << 16 | D2L_HEIGHT));
	/* enable LVDS transmitter. DSI_CLOCK/3 chosen as default */
	d2l_write_register(dssdev, LVCFG, 0x00000001);

	/* softreset for LCD controller, LVDS-PHY, Line Buffer */
	d2l_write_register(dssdev, SYSRST, 0x00000004);
	/* upload new timings to hw registers on next vsync */
	d2l_write_register(dssdev, VFUEN, 0x00000001);
}

/* routines */
static int d2l_probe(struct omap_dss_device *dssdev)
{
	int rv;
	struct d2l_board_data *board_data = d2l_get_board_data(dssdev);
	debug("%s\n", __func__);

	/* reassign timings to values from board file */
	d2l_timings.x_res = board_data->x_res;
	d2l_timings.y_res = board_data->y_res;
	/* choose according value from omap_panel_config */
	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	/* assign timings to dssdev */
	dssdev->panel.timings = d2l_timings;
	/* set pixel size */
	dssdev->ctrl.pixel_size = 24;
	/* ac-bias pin transitions per interrupt and pin frequency */
	dssdev->panel.acbi = 0;
	dssdev->panel.acb = 40;

	/* request virtual channel. 0 - for video data. 1 for write/read regs */
	rv = omap_dsi_request_vc(dssdev, &d2l_vc.chan0);
	if (rv)
		goto err;
	rv = omap_dsi_set_vc_id(dssdev, d2l_vc.chan0, 0);	
	if (rv)
		goto err;	
	rv = omap_dsi_request_vc(dssdev, &d2l_vc.chan1);
	if (rv)
		goto err;
	rv = omap_dsi_set_vc_id(dssdev, d2l_vc.chan1, 0);	
	if (rv)
		goto err;

err:
	debug("%s end. rv %d \n", __func__, rv);
	return rv;
}

static void d2l_remove(struct omap_dss_device *dssdev)
{
	debug("%s \n", __func__);
	omap_dsi_release_vc(dssdev, d2l_vc.chan0);
	omap_dsi_release_vc(dssdev, d2l_vc.chan1);
}

static int d2l_enable(struct omap_dss_device *dssdev)
{
	int rv = -EINVAL;
	debug("%s \n", __func__);

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		goto err;

	dsi_bus_lock(dssdev);

	/* At power on the first vsync has not been received yet */
	dssdev->first_vsync = false;
	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);
	rv = omapdss_dsi_display_enable(dssdev);
	if (rv)
		goto err;
	omapdss_dsi_vc_enable_hs(dssdev, d2l_vc.chan0, true);

	if (firstboot) {
		d2l_hw_reset(dssdev);
		dsi_videomode_panel_preinit(dssdev);
		d2l_init_registers(dssdev);
		firstboot = false;
	}
	else {
		//msleep(100);
		//d2l_hw_reset(dssdev);
		dsi_videomode_panel_preinit(dssdev); /* must have */
		//d2l_dump_regs(dssdev);
		d2l_resume_init_registers(dssdev);
	}

	dsi_video_mode_enable(dssdev, 0x3e);

	dsi_bus_unlock(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

err:
	debug("%s end. rv %d \n", __func__, rv);
	return rv;
}

static void d2l_disable(struct omap_dss_device *dssdev)
{
	debug("%s \n", __func__);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dsi_bus_lock(dssdev);

		dsi_video_mode_disable(dssdev);
		omapdss_dsi_display_disable(dssdev, false, false);
		if (dssdev->platform_disable)
			dssdev->platform_disable(dssdev);
		
		dsi_bus_unlock(dssdev);               
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	}
}

#if 0
static int d2l_suspend(struct omap_dss_device *dssdev)
{
	debug("%s\n", __func__);
	dump_stack();
	d2l_disable(dssdev);
	return 0;
}
#endif

/* this func called every frame so no debug inside */
static void d2l_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = d2l_timings.x_res;
	*yres = d2l_timings.y_res;
}

static void d2l_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	debug("%s\n", __func__);
	*timings = dssdev->panel.timings;
}

static void d2l_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	debug("%s\n", __func__);
}

static int d2l_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	debug("%s\n", __func__);
	return 0;
}

static struct omap_dss_driver d2l_driver = {
	.probe		= d2l_probe,
	.remove		= d2l_remove,

	.enable		= d2l_enable,
	.disable	= d2l_disable,

// need it only for some hdmi case described in #16412
#if 0
	.suspend	= d2l_suspend,
	.resume		= NULL,        
#endif

	.get_resolution	= d2l_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings	= d2l_get_timings,
	.set_timings	= d2l_set_timings,
	.check_timings	= d2l_check_timings,

	.driver         = {
		.name   = "d2l",
		.owner  = THIS_MODULE,
	},
};

static int __init d2l_init(void)
{
	int rv;
	debug("d2l driver by repu1sion. version %s \n", version);

	rv = omap_dss_register_driver(&d2l_driver);

	debug("%s end, rv: %d\n", __func__, rv);
	return rv;
}

static void __exit d2l_exit(void)
{
	omap_dss_unregister_driver(&d2l_driver);
}

module_init(d2l_init);
module_exit(d2l_exit);

MODULE_AUTHOR("Andrii Guriev <x0160204@ti.com>");
MODULE_DESCRIPTION("TC358765 DSI-2-LVDS Driver");
MODULE_LICENSE("GPL");
