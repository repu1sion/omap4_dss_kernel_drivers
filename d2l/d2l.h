#ifndef __PANEL_D2L_H__
#define __PANEL_D2L_H__

/* DSI D-PHY Layer Registers */
#define	D0W_DPHYCONTTX		0x0004		/* Data Lane 0 DPHY TX */
#define	CLW_DPHYCONTRX		0x0020		/* Clock Lane DPHY RX */
#define	D0W_DPHYCONTRX		0x0024		/* Data Land 0 DPHY Rx */
#define	D1W_DPHYCONTRX		0x0028		/* Data Lane 1 DPHY Rx */
#define	D2W_DPHYCONTRX		0x002c		/* Data Lane 2 DPHY Rx */
#define	D3W_DPHYCONTRX		0x0030		/* Data Lane 3 DPHY Rx */
#define	COM_DPHYCONTRX		0x0038		/* DPHY Rx Common */
#define	CLW_CNTRL		0x0040		/* Clock Lane */
#define	D0W_CNTRL		0x0044		/* Data Lane 0 */
#define	D1W_CNTRL		0x0048		/* Data Lane 1 */
#define	D2W_CNTRL		0x004c		/* Data Lane 2 */
#define	D3W_CNTRL		0x0050		/* Data Lane 3 */
#define	DFTMODE_CNTRL		0x0054		/* DFT Mode */

/* DSI PPI Layer Registers */
#define	PPI_STARTPPI		0x0104		/* Start control bit */
#define	PPI_BUSYPPI		0x0108			/* Busy bit */
#define	PPI_LINEINITCNT		0x0110		/* Line In initialization */
#define	PPI_LPTXTIMECNT		0x0114		/* LPTX timing signal */
#define	PPI_LANEENABLE		0x0134		/* Lane Enable */
#define	PPI_TX_RX_TA		0x013c		/* BTA timing param */
#define	PPI_CLS_ATMR		0x0140		/* Analog timer fcn */
#define	PPI_D0S_ATMR		0x0144		/* Analog timer fcn Lane 0 */
#define	PPI_D1S_ATMR		0x0148		/* Analog timer fcn Lane 1 */
#define	PPI_D2S_ATMR		0x014c		/* Analog timer fcn Lane 2 */
#define	PPI_D3S_ATMR		0x0150		/* Analog timer fcn Lane 3 */
#define	PPI_D0S_CLRSIPOCOUNT	0x0164		/* Assertion timer Lane 0 */
#define	PPI_D1S_CLRSIPOCOUNT	0x0168		/* Assertion timer Lane 1 */
#define	PPI_D2S_CLRSIPOCOUNT	0x016c		/* Assertion timer Lane 1 */
#define	PPI_D3S_CLRSIPOCOUNT	0x0170		/* Assertion timer Lane 1 */
#define CLS_PRE			0x0180		/* PHY IO cntr */
#define D0S_PRE			0x0184		/* PHY IO cntr */
#define D1S_PRE			0x0188		/* PHY IO cntr */
#define D2S_PRE			0x018c		/* PHY IO cntr */
#define D3S_PRE			0x0190		/* PHY IO cntr */
#define CLS_PREP		0x01a0		/* PHY IO cntr */
#define D0S_PREP		0x01a4		/* PHY IO cntr */
#define D1S_PREP		0x01a8		/* PHY IO cntr */
#define D2S_PREP		0x01ac		/* PHY IO cntr */
#define D3S_PREP		0x01b0		/* PHY IO cntr */
#define CLS_ZERO		0x01c0		/* PHY IO cntr */
#define	D0S_ZERO		0x01c4		/* PHY IO cntr */
#define	D1S_ZERO		0x01c8		/* PHY IO cntr */
#define	D2S_ZERO		0x01cc		/* PHY IO cntr */
#define	D3S_ZERO		0x01d0		/* PHY IO cntr */
#define PPI_CLRFLG		0x01e0		/* PRE cntrs */
#define PPI_CLRSIPO		0x01e4		/* Clear SIPO */
#define PPI_HSTimeout		0x01f0		/* HS RX timeout */
#define PPI_HSTimeoutEnable	0x01f4		/* Enable HS Rx Timeout */

/* DSI Protocol Layer Registers */
#define DSI_STARTDSI		0x0204		/* DSI TX start bit */
#define DSI_BUSYDSI		0x0208		/* DSI busy bit */
#define DSI_LANEENABLE		0x0210		/* Lane enable */
#define DSI_LANESTATUS0		0x0214		/* HS Rx mode */
#define DSI_LANESTATUS1		0x0218		/* ULPS or STOP state */
#define DSI_INTSTATUS		0x0220		/* Interrupt status */
#define DSI_INTMASK		0x0224		/* Interrupt mask */
#define DSI_INTCLR		0x0228		/* Interrupt clear */
#define DSI_LPTXTO		0x0230		/* LP Tx Cntr */

/* DSI General Registers */
#define	DSIERRCNT		0x0300		/* DSI Error Count */

/* DSI Application Layer Registers */
#define APLCTRL			0x0400		/* Application Layer Cntrl */
#define RDPKTLN			0x0404		/* Packet length */

/* Video Path Registers */
#define	VPCTRL			0x0450		/* Video Path */
#define HTIM1			0x0454		/* Horizontal Timing */
#define HTIM2			0x0458		/* Horizontal Timing */
#define VTIM1			0x045c		/* Vertical Timing */
#define VTIM2			0x0460		/* Vertical Timing */
#define VFUEN			0x0464		/* Video Frame Timing */


/* LVDS Registers */
#define LVMX0003		0x0480		/* LVDS Mux Input - Bit 0 to 3 */
#define LVMX0407		0x0484		/* LVDS Mux Input - Bit 4 to 7 */
#define LVMX0811		0x0488		/* LVDS Mux Input - Bit 8 to 11 */
#define LVMX1215		0x048c		/* LVDS Mux Input - Bit 12 to 15 */
#define LVMX1619		0x0490		/* LVDS Mux Input - Bit 16 to 19 */
#define LVMX2023		0x0494		/* LVDS Mux Input - Bit 20 to 23 */
#define LVMX2427		0x0498		/* LVDS Mux Input - Bit 24 to 27 */
#define LVCFG			0x049c		/* LVDS Config */
#define	LVPHY0			0x04a0		/* LVDS PHY Reg 0 */
#define	LVPHY1			0x04a1		/* LVDS PHY Reg 1 */

/* System Registers */
#define SYSSTAT			0x0500		/* System Status */
#define SYSRST			0x0504		/* System Reset */

/* GPIO Registers */
#define GPIOC			0x0520		/* GPIO Control */
#define GPIOO			0x0520		/* GPIO Output */
#define GPIOI			0x0520		/* GPIO Input */

/* Chip Revision Registers */
#define IDREG			0x0580		/* Chip and Revision ID */

/* Debug Register */
#define DEBUG00			0x05a0		/* Debug */
#define DEBUG01			0x05a4		/* LVDS Data */


/* default timings for tablet2 */
#define D2L_WIDTH	1280
#define D2L_HEIGHT	800
#define D2L_PCLK	65183
#define D2L_HFP		10
#define D2L_HSW		20
#define D2L_HBP		10
#define D2L_VFP		4
#define D2L_VSW		4
#define D2L_VBP		4

#endif
