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
#include "fs.h"
#include "arm11/config.h"
#include "arm11/fmt.h"
#include "arm11/open_agb_firm.h"
#include "arm11/oaf_video.h"
#include "arm11/arm11_suspend_probe.h"
#include "arm11/gba_sleep_lid.h"
#include "arm11/drivers/codec.h"
#include "arm11/drivers/cfg11.h"
#include "arm11/drivers/gpio.h"
#include "arm11/drivers/hid.h"
#include "arm11/drivers/lgycap.h"
#include "arm11/drivers/lgy11.h"
#include "arm11/drivers/mcu.h"
#include "arm11/drivers/pdn.h"
#include "arm11/drivers/scu.h"
#include "arm11/drivers/timer.h"

#define GBA_SLEEP_BUTTONS  0x0204u  // L + Select.
#define GBA_WAKE_BUTTONS   0x0104u  // R + Select.
#define POWER_STATE_LOG_PATH  OAF_WORK_DIR "/power_state.log"
#define POWER_STATE_LOG_SIZE  8192u

typedef struct
{
	u8 level;
	u32 millivolts;
	s8 temperature;
} BatterySnapshot;

/*
 * Read-only diagnostic snapshot of ARM11/SoC power-related registers.
 * No register is modified by this instrumentation.
 */
typedef struct
{
	u16 pdnCnt;
	u32 pdnWakeEnable;
	u32 pdnWakeReason;
	u32 pdnGpuCnt;
	u8 pdnFcramCnt;
	u8 pdnI2sCnt;
	u8 pdnCamCnt;
	u8 pdnDspCnt;
	u8 pdnG1Cnt;
	u16 pdnLgrSocmode;
	u16 pdnLgrCnt;
	u8 pdnLgrCpuCnt[4];
	u8 cfgWifiPower;
	u8 cfgCdmaCnt;
	u8 cfgGpuN3dsCnt;
	u16 cfgSpiCnt;
	u32 scuCtrl;
	u32 scuConfig;
	u32 scuCpuStat;
} PowerStateSnapshot;

static BatterySnapshot readBatterySnapshot(void)
{
	BatterySnapshot snapshot;
	snapshot.level = MCU_getBatteryLevel();
	snapshot.millivolts = (u32)(MCU_getBatteryVoltage() * 1000.0f + 0.5f);
	snapshot.temperature = MCU_getBatteryTemperature();
	return snapshot;
}

static PowerStateSnapshot readPowerStateSnapshot(void)
{
	PowerStateSnapshot snapshot;
	Pdn *const pdn = getPdnRegs();
	Cfg11 *const cfg11 = getCfg11Regs();
	Scu *const scu = getScuRegs();

	snapshot.pdnCnt = pdn->cnt;
	snapshot.pdnWakeEnable = pdn->wake_enable;
	snapshot.pdnWakeReason = pdn->wake_reason;
	snapshot.pdnGpuCnt = pdn->gpu_cnt;
	snapshot.pdnFcramCnt = pdn->fcram_cnt;
	snapshot.pdnI2sCnt = pdn->i2s_cnt;
	snapshot.pdnCamCnt = pdn->cam_cnt;
	snapshot.pdnDspCnt = pdn->dsp_cnt;
	snapshot.pdnG1Cnt = pdn->g1_cnt;
	snapshot.pdnLgrSocmode = pdn->lgr_socmode;
	snapshot.pdnLgrCnt = pdn->lgr_cnt;
	for(u32 i = 0; i < 4; i++) snapshot.pdnLgrCpuCnt[i] = pdn->lgr_cpu_cnt[i];

	snapshot.cfgWifiPower = cfg11->wifi_power;
	snapshot.cfgCdmaCnt = cfg11->cdma_cnt;
	snapshot.cfgGpuN3dsCnt = cfg11->gpu_n3ds_cnt;
	snapshot.cfgSpiCnt = cfg11->spi_cnt;

	snapshot.scuCtrl = scu->ctrl;
	snapshot.scuConfig = scu->config;
	snapshot.scuCpuStat = scu->cpu_stat;
	return snapshot;
}

static u32 formatPowerStateSnapshot(char *const buf, const u32 size,
                                    const char *const label,
                                    const PowerStateSnapshot *const s)
{
	u32 off = 0;
	off += ee_snprintf(buf + off, size - off,
	                   "Power state [%s]: PDN cnt=%04X wake_en=%08lX wake_reason=%08lX\n",
	                   label, s->pdnCnt, (unsigned long)s->pdnWakeEnable,
	                   (unsigned long)s->pdnWakeReason);
	off += ee_snprintf(buf + off, size - off,
	                   "  PDN: gpu=%08lX fcram=%02X i2s=%02X cam=%02X dsp=%02X g1=%02X\n",
	                   (unsigned long)s->pdnGpuCnt, s->pdnFcramCnt, s->pdnI2sCnt,
	                   s->pdnCamCnt, s->pdnDspCnt, s->pdnG1Cnt);
	off += ee_snprintf(buf + off, size - off,
	                   "  LGR: socmode=%04X cnt=%04X cpu=%02X/%02X/%02X/%02X\n",
	                   s->pdnLgrSocmode, s->pdnLgrCnt, s->pdnLgrCpuCnt[0],
	                   s->pdnLgrCpuCnt[1], s->pdnLgrCpuCnt[2], s->pdnLgrCpuCnt[3]);
	off += ee_snprintf(buf + off, size - off,
	                   "  CFG11: wifi=%02X cdma=%02X gpu_n3ds=%02X spi=%04X\n",
	                   s->cfgWifiPower, s->cfgCdmaCnt, s->cfgGpuN3dsCnt, s->cfgSpiCnt);
	off += ee_snprintf(buf + off, size - off,
	                   "  SCU: ctrl=%08lX config=%08lX cpu_stat=%08lX\n",
	                   (unsigned long)s->scuCtrl, (unsigned long)s->scuConfig,
	                   (unsigned long)s->scuCpuStat);

	return off;
}

static Result appendPowerStateLog(const PowerStateSnapshot *const awakeBefore,
                                  const PowerStateSnapshot *const sleepReady,
                                  const PowerStateSnapshot *const awakeAfter,
                                  const BatterySnapshot *const batteryBefore,
                                  const BatterySnapshot *const batteryAfter,
                                  const u32 arm11Wakeups,
                                  const Arm11SuspendWakeInfo *const pdnWake)
{
	static char logBuf[POWER_STATE_LOG_SIZE];
	u32 off = 0;

	off += ee_snprintf(logBuf + off, sizeof(logBuf) - off,
	                   "\n=== GBA Sleep Power-State Dump ===\n");
	off += ee_snprintf(logBuf + off, sizeof(logBuf) - off,
	                   "ARM11 wakeups while lid closed: %lu\n",
	                   (unsigned long)arm11Wakeups);
	if(pdnWake != NULL)
	{
		off += ee_snprintf(logBuf + off, sizeof(logBuf) - off,
		                   "ARM11 PDN sleep: entered=%u cnt=%04X->%04X wake_en=%08lX->%08lX wake_reason=%08lX shell_open=%u\n",
		                   pdnWake->entered ? 1u : 0u,
		                   pdnWake->pdnCntBeforeSleep, pdnWake->pdnCntAfterWake,
		                   (unsigned long)pdnWake->wakeEnableBefore,
		                   (unsigned long)pdnWake->wakeEnableDuringSleep,
		                   (unsigned long)pdnWake->wakeReasonAfterWake,
		                   pdnWake->shellOpenAfterWake);
		off += ee_snprintf(logBuf + off, sizeof(logBuf) - off,
		                   "ARM11 SCU sleep: cpu_stat before=%08lX wake=%08lX restored=%08lX\n",
		                   (unsigned long)pdnWake->scuCpuStatBeforeSleep,
		                   (unsigned long)pdnWake->scuCpuStatAtWake,
		                   (unsigned long)pdnWake->scuCpuStatAfterRestore);
	}
	off += formatPowerStateSnapshot(logBuf + off, sizeof(logBuf) - off,
	                                "AWAKE-BEFORE", awakeBefore);
	off += formatPowerStateSnapshot(logBuf + off, sizeof(logBuf) - off,
	                                "SLEEP-READY", sleepReady);
	off += formatPowerStateSnapshot(logBuf + off, sizeof(logBuf) - off,
	                                "AWAKE-AFTER", awakeAfter);
	off += ee_snprintf(logBuf + off, sizeof(logBuf) - off,
	                   "Battery: %u%% / %lu mV / %d C -> %u%% / %lu mV / %d C\n",
	                   batteryBefore->level, (unsigned long)batteryBefore->millivolts,
	                   (int)batteryBefore->temperature,
	                   batteryAfter->level, (unsigned long)batteryAfter->millivolts,
	                   (int)batteryAfter->temperature);

	if(off >= sizeof(logBuf)) off = sizeof(logBuf) - 1;

	FHandle file;
	Result res = fOpen(&file, POWER_STATE_LOG_PATH, FA_OPEN_APPEND | FA_WRITE);
	if(res != RES_OK) return res;

	res = fWrite(file, logBuf, off, NULL);
	const Result closeRes = fClose(file);
	return (res != RES_OK ? res : closeRes);
}

void gbaSleepHandleLid(void)
{
	if(!oafIsGbaSleepAvailable())
		return;

	/* Measure under the normal fully-awake load before entering sleep. */
	const BatterySnapshot batteryBefore = readBatterySnapshot();
	const PowerStateSnapshot powerAwakeBefore = readPowerStateSnapshot();

	/*
	 * Use the deep graphics/PDN path for every video mode. Keep the L+Select
	 * injection before video suspension; color-profile Core-1 teardown happens
	 * later inside OAF_videoSuspendForGfxSleep().
	 */
	const bool deepGfxSleep = true;

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
	/*
	 * Suspend the codec and I2S hardware instead of only muting it.
	 * CODEC_deinit() preserves the state needed by CODEC_wakeup() and gates
	 * the I2S clock once the DAC/touch paths are quiesced.
	 */
	CODEC_deinit();
	GFX_powerOffBacklight(GFX_BL_BOTH);

	if(deepGfxSleep)
	{
		OAF_videoSuspendForGfxSleep();
		GFX_sleep();
	}
	else
	{
		LGYCAP_stop(LGYCAP_DEV_TOP);
		GFX_sleepLcd();
		GFX_sleepPdc();
	}
	MCU_setPowerLedPattern(MCU_PWR_LED_SLEEP);

	/* Snapshot the final low-power configuration immediately before WFI. */
	const PowerStateSnapshot powerSleepReady = readPowerStateSnapshot();

	u32 arm11Wakeups = 0;

	/*
	 * Enter PDN system sleep. If a prerequisite is unavailable, use the lid WFI
	 * fallback. A successful wake restores both active SCU core states and
	 * returns in the same Core-0 context with the raw wake state recorded.
	 */
	Arm11SuspendWakeInfo pdnWake = {0};
	bool pdnSleepReturned = false;
	if(deepGfxSleep)
		pdnSleepReturned = arm11SuspendProbeEnter(&pdnWake);

	if(!pdnSleepReturned)
	{
		while(GPIO_read(GPIO_1_SHELL))
		{
			__wfi();
			arm11Wakeups++;
		}
	}


	/*
	 * Do not touch the framebuffer-backed console while the graphics domain is
	 * asleep. VRAM/GPU access is safe again only after the cold resume.
	 */
	if(!deepGfxSleep)
	{
		ee_printf("GBA Sleep: ARM11 wakeups while lid closed: %lu (lightweight gfx sleep)\n",
		          (unsigned long)arm11Wakeups);
	}

	if(deepGfxSleep)
	{
		GFX_sleepAwakeCold();
		OAF_videoResumeAfterGfxSleep();

		/* The debug console is safe again only after the graphics cold resume. */
		if(pdnSleepReturned)
			ee_printf("GBA Sleep: ARM11 PDN system sleep resumed successfully\n");
		else
			ee_printf("GBA Sleep: ARM11 wakeups while lid closed: %lu (GPU/VRAM off fallback)\n",
			          (unsigned long)arm11Wakeups);
	}
	else
	{
		GFX_sleepPdcAwake();
		GFX_sleepLcdAwake();
	}

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
	CODEC_wakeup();
	CODEC_setVolumeOverride(g_oafConfig.volume);

	/*
	 * Measure again under the restored fully-awake load. The short settling
	 * delay keeps the voltage comparison from being dominated by resume
	 * transients. The same delay is used by the upstream baseline build.
	 */
	TIMER_sleepMs(250);
	const BatterySnapshot batteryAfter = readBatterySnapshot();
	const PowerStateSnapshot powerAwakeAfter = readPowerStateSnapshot();

	const Result logRes = appendPowerStateLog(&powerAwakeBefore, &powerSleepReady,
	                                          &powerAwakeAfter, &batteryBefore,
	                                          &batteryAfter, arm11Wakeups,
	                                          (deepGfxSleep ? &pdnWake : NULL));
	if(logRes == RES_OK)
		ee_printf("Power-state log written: %s\n", POWER_STATE_LOG_PATH);
	else
		ee_printf("Power-state log write failed: %08lX\n", (unsigned long)logRes);

	ee_printf("Battery A/B [LOW-POWER]: %u%% / %lu mV / %d C -> %u%% / %lu mV / %d C\n",
	          batteryBefore.level, (unsigned long)batteryBefore.millivolts,
	          (int)batteryBefore.temperature,
	          batteryAfter.level, (unsigned long)batteryAfter.millivolts,
	          (int)batteryAfter.temperature);

}
