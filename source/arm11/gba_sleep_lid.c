/*
 * Automatic lid integration for universal GBA Sleep/Wake support.
 *
 * Lid Close:
 *   GBA L+Select -> GBA Sleep IRQ handler -> real SWI 03h / STOP
 *   then N3DS audio/capture/backlights off + sleep power LED
 *
 * Lid Open:
 *   GBA R+Select + LGY wake/ack -> GBA Sleep IRQ handler resumes
 *   then N3DS presentation is restored
 *
 * This module does not modify or replace the GBA Sleep IRQ handler.
 */
#include "arm.h"
#include "types.h"
#include "drivers/gfx.h"
#include "arm11/config.h"
#include "arm11/open_agb_firm.h"
#include "arm11/gba_sleep_lid.h"
#include "arm11/drivers/codec.h"
#include "arm11/drivers/gpio.h"
#include "arm11/drivers/hid.h"
#include "arm11/drivers/lgycap.h"
#include "arm11/drivers/lgy11.h"
#include "arm11/drivers/mcu.h"

#define GBA_SLEEP_BUTTONS  0x0204u  // L + Select.
#define GBA_WAKE_BUTTONS   0x0104u  // R + Select.

void gbaSleepHandleLid(void)
{
	if(!oafIsGbaSleepAvailable())
		return;

	Lgy11 *const lgy11 = getLgy11Regs();

	const u16 oldPadSel = lgy11->pad_sel;
	const u16 oldPadVal = lgy11->pad_val;
	const u16 injectedButtons =
		GBA_SLEEP_BUTTONS | GBA_WAKE_BUTTONS;

	/*
	 * Lid close: inject the same L+Select combination that works manually.
	 * The GBA Sleep IRQ handler then enters real GBA SWI 03h / STOP.
	 */
	lgy11->pad_sel = oldPadSel | injectedButtons;
	LGY11_setInputState(GBA_SLEEP_BUTTONS);

	/*
	 * Suspend the N3DS presentation only after the GBA STOP trigger is injected.
	 * The GBA STOP request is injected before presentation is suspended.
	 */
	CODEC_setVolumeOverride(-128);
	LGYCAP_stop(LGYCAP_DEV_TOP);
	GFX_powerOffBacklight(GFX_BL_BOTH);
	MCU_setPowerLedPattern(MCU_PWR_LED_SLEEP);

	while(GPIO_read(GPIO_1_SHELL))
		__wfi();

	/*
	 * Lid open: present R+Select first, then wake/ack LGY.
	 */
	LGY11_setInputState(GBA_WAKE_BUTTONS);
	REG_HID_PADCNT = 0;
	lgy11->sleep |= (u16)(BIT(0) | BIT(1));

	/*
	 * Keep the synthetic wake combination asserted for one VBlank.
	 */
	GFX_waitForVBlank0();

	lgy11->pad_val = oldPadVal;
	lgy11->pad_sel = oldPadSel;

	MCU_setPowerLedPattern(MCU_PWR_LED_AUTO);
	GFX_powerOnBacklight(GFX_BL_BOTH);
	LGYCAP_start(LGYCAP_DEV_TOP);
	CODEC_setVolumeOverride(g_oafConfig.volume);
}
