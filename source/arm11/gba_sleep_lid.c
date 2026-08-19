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
#include "drivers/pxi.h"
#include "ipc_handler.h"
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

static Result startArm9SleepDiag(void)
{
	return PXI_sendCmd(IPC_CMD9_ARM9_SLEEP_DIAG_START, NULL, 0);
}

static Result stopArm9SleepDiag(Arm9SleepDiagReport *const report)
{
	const u32 cmdBuf[2] = {(u32)report, sizeof(*report)};
	return PXI_sendCmd(IPC_CMD9_ARM9_SLEEP_DIAG_STOP, cmdBuf, 2);
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

static u32 formatArm9PowerStateSnapshot(char *const buf, const u32 size,
                                       const char *const label,
                                       const Arm9PowerStateSnapshot *const s)
{
	u32 off = 0;
	off += ee_snprintf(buf + off, size - off,
	                   "ARM9 state [%s]: IRQ ie=%08lX if=%08lX PXI cnt=%08lX\n",
	                   label, (unsigned long)s->irqIe, (unsigned long)s->irqIf,
	                   (unsigned long)s->pxiCnt);
	off += ee_snprintf(buf + off, size - off,
	                   "  TIMER: 0=%04X/%04X 1=%04X/%04X 2=%04X/%04X 3=%04X/%04X (val/cnt)\n",
	                   s->timerVal[0], s->timerCnt[0], s->timerVal[1], s->timerCnt[1],
	                   s->timerVal[2], s->timerCnt[2], s->timerVal[3], s->timerCnt[3]);
	off += ee_snprintf(buf + off, size - off,
	                   "  CFG9: xdma_req=%02X card_power=%02X cardctl=%04X sdmmcctl=%04X extmem=%02X\n",
	                   s->cfgXdmaReq, s->cfgCardPower, s->cfgCardCtl,
	                   s->cfgSdmmcCtl, s->cfgExtmemCnt9);
	off += ee_snprintf(buf + off, size - off,
	                   "  NDMA: gcnt=%08lX cnt=%08lX/%08lX/%08lX/%08lX/%08lX/%08lX/%08lX/%08lX\n",
	                   (unsigned long)s->ndmaGcnt,
	                   (unsigned long)s->ndmaCnt[0], (unsigned long)s->ndmaCnt[1],
	                   (unsigned long)s->ndmaCnt[2], (unsigned long)s->ndmaCnt[3],
	                   (unsigned long)s->ndmaCnt[4], (unsigned long)s->ndmaCnt[5],
	                   (unsigned long)s->ndmaCnt[6], (unsigned long)s->ndmaCnt[7]);
	for(u32 i = 0; i < 2; i++)
	{
		off += ee_snprintf(buf + off, size - off,
		                   "  TMIO%lu: status=%08lX mask=%08lX clk=%04X port=%04X rst=%04X cdet_mask=%04X\n",
		                   (unsigned long)(i + 1), (unsigned long)s->tmioStatus[i],
		                   (unsigned long)s->tmioStatusMask[i], s->tmioClkCtrl[i],
		                   s->tmioPortSel[i], s->tmioSoftRst[i], s->tmioExtCdetMask[i]);
	}

	return off;
}

static u32 formatArm9IrqCounts(char *const buf, const u32 size,
                              const Arm9SleepDiagReport *const report)
{
	static const char *const irqNames[30] =
	{
		"NDMA0", "NDMA1", "NDMA2", "NDMA3", "NDMA4", "NDMA5", "NDMA6", "NDMA7",
		"TIMER0", "TIMER1", "TIMER2", "TIMER3", "PXI_SYNC", "PXI_NOT_FULL", "PXI_NOT_EMPTY",
		"AES", "TMIO1", "TMIO1_IRQ", "TMIO3", "TMIO3_IRQ", "DEBUG_RECV", "DEBUG_SEND",
		"RSA", "CTR_CARD1", "CTR_CARD2", "CGC", "CGC_DET", "DS_CARD", "DMAC2", "DMAC2_ABORT"
	};

	u32 off = 0;
	bool any = false;
	off += ee_snprintf(buf + off, size - off, "ARM9 IRQ counts while lid closed:\n");
	for(u32 i = 0; i < 30; i++)
	{
		if(report->irqCounts[i] == 0) continue;
		any = true;
		off += ee_snprintf(buf + off, size - off, "  %02lu %-13s %lu\n",
		                   (unsigned long)i, irqNames[i],
		                   (unsigned long)report->irqCounts[i]);
	}
	if(!any) off += ee_snprintf(buf + off, size - off, "  none\n");

	return off;
}

static Result appendPowerStateLog(const PowerStateSnapshot *const awakeBefore,
                                  const PowerStateSnapshot *const sleepReady,
                                  const PowerStateSnapshot *const awakeAfter,
                                  const Arm9SleepDiagReport *const arm9Diag,
                                  const Result arm9DiagStartRes,
                                  const Result arm9DiagStopRes,
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
	off += ee_snprintf(logBuf + off, sizeof(logBuf) - off,
	                   "ARM9 sleep diag: start=%08lX stop=%08lX wakeups=%lu\n",
	                   (unsigned long)arm9DiagStartRes, (unsigned long)arm9DiagStopRes,
	                   (unsigned long)arm9Diag->wakeups);
	if(arm9DiagStartRes == RES_OK && arm9DiagStopRes == RES_OK)
	{
		off += formatArm9PowerStateSnapshot(logBuf + off, sizeof(logBuf) - off,
		                                    "BEGIN", &arm9Diag->before);
		off += formatArm9PowerStateSnapshot(logBuf + off, sizeof(logBuf) - off,
		                                    "END", &arm9Diag->after);
		off += formatArm9IrqCounts(logBuf + off, sizeof(logBuf) - off, arm9Diag);
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

#define GFX_RESUME_TRACE_MCU_OFF 0u
#define GFX_RESUME_TRACE_MAGIC0  0x47u /* G */
#define GFX_RESUME_TRACE_MAGIC1  0x52u /* R */

typedef struct
{
	u8 magic0;
	u8 magic1;
	u8 stage;
	u8 check;
} GfxResumeTraceMarker;

static const char *getGfxResumeTraceStageName(const u8 stage)
{
	switch(stage)
	{
		case 0x10: return "BEFORE_GFX_SLEEP";
		case 0x11: return "AFTER_GFX_SLEEP";
		case 0x20: return "LID_OPENED";
		case 0x30: return "BEFORE_COLD_RESUME";
		case 0x40: return "COLD_ENTER";
		case 0x41: return "HARDWARE_RESET_DONE";
		case 0x42: return "VRAM_FILL_SUBMITTED";
		case 0x43: return "PPF_WORKAROUND_SUBMITTED";
		case 0x44: return "PDC_INIT_DONE";
		case 0x45: return "LCD_INIT_DONE";
		case 0x46: return "PSC0_WAIT_DONE";
		case 0x47: return "PSC1_WAIT_DONE";
		case 0x48: return "PPF_WAIT_DONE";
		case 0x49: return "FORCE_BLACK_OFF_DONE";
		case 0x50: return "COLD_RESUME_RETURNED";
		case 0x51: return "OAF_VIDEO_RESUME_DONE";
		case 0x60: return "GBA_WAKE_SIGNALLED";
		case 0x61: return "VBLANK_DONE";
		case 0x70: return "PRESENTATION_RESTORED";
		default:   return "UNKNOWN";
	}
}

static void writeGfxResumeTraceStageRaw(const u8 stage)
{
	const GfxResumeTraceMarker marker =
	{
		.magic0 = GFX_RESUME_TRACE_MAGIC0,
		.magic1 = GFX_RESUME_TRACE_MAGIC1,
		.stage  = stage,
		.check  = (u8)(stage ^ 0xFFu)
	};
	(void)MCU_setFreeRamData(GFX_RESUME_TRACE_MCU_OFF,
	                         (const u8*)&marker, sizeof(marker));
}

static void writeGfxResumeTraceStage(const u32 stage)
{
	writeGfxResumeTraceStageRaw((u8)(0x40u + stage));
}

static void clearGfxResumeTrace(void)
{
	const GfxResumeTraceMarker clear = {0};
	(void)MCU_setFreeRamData(GFX_RESUME_TRACE_MCU_OFF,
	                         (const u8*)&clear, sizeof(clear));
}

void gbaSleepReportPreviousResumeTrace(void)
{
	GfxResumeTraceMarker marker = {0};
	if(!MCU_getFreeRamData(GFX_RESUME_TRACE_MCU_OFF,
	                       (u8*)&marker, sizeof(marker)))
		return;

	if(marker.magic0 != GFX_RESUME_TRACE_MAGIC0 ||
	   marker.magic1 != GFX_RESUME_TRACE_MAGIC1 ||
	   marker.check != (u8)(marker.stage ^ 0xFFu))
		return;

	ee_printf("Previous GFX resume stopped at %02X: %s\n",
	          marker.stage, getGfxResumeTraceStageName(marker.stage));

	char line[128];
	const int len = ee_snprintf(line, sizeof(line),
	                            "Previous GFX resume stopped at %02X: %s\n",
	                            marker.stage, getGfxResumeTraceStageName(marker.stage));
	if(len > 0)
	{
		FHandle file;
		if(fOpen(&file, POWER_STATE_LOG_PATH, FA_OPEN_APPEND | FA_WRITE) == RES_OK)
		{
			const u32 writeLen = ((u32)len < sizeof(line)) ? (u32)len : (sizeof(line) - 1u);
			(void)fWrite(file, line, writeLen, NULL);
			(void)fClose(file);
		}
	}

	clearGfxResumeTrace();
}

void gbaSleepHandleLid(void)
{
	if(!oafIsGbaSleepAvailable())
		return;

	/* Measure under the normal fully-awake load before entering sleep. */
	const BatterySnapshot batteryBefore = readBatterySnapshot();
	const PowerStateSnapshot powerAwakeBefore = readPowerStateSnapshot();

	/*
	 * Use the proven deep GFX/PDN path for every video mode.
	 *
	 * IMPORTANT: the L+Select injection sequence below intentionally stays in
	 * the exact same position/order as 13a2. Color-profile Core-1 teardown is
	 * performed later inside OAF_videoSuspendForGfxSleep(), after the trigger
	 * has already been injected.
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
		writeGfxResumeTraceStageRaw(0x10);
		GFX_sleep();
		writeGfxResumeTraceStageRaw(0x11);
	}
	else
	{
		LGYCAP_stop(LGYCAP_DEV_TOP);
		GFX_sleepLcd();
		GFX_sleepPdc();
	}
	MCU_setPowerLedPattern(MCU_PWR_LED_SLEEP);

	/*
	 * Arm the ARM9 diagnostic only after all normal low-power setup is done.
	 * The START command itself is explicitly excluded from the counters.
	 */
	Arm9SleepDiagReport arm9Diag = {0};
	const bool arm9DiagEnabled = !deepGfxSleep;
	const Result arm9DiagStartRes = (arm9DiagEnabled ? startArm9SleepDiag() : RES_OK);

	/* Snapshot the final low-power configuration immediately before WFI. */
	const PowerStateSnapshot powerSleepReady = readPowerStateSnapshot();

	u32 arm11Wakeups = 0;

	/*
	 * Phase 12: enter the real PDN system-sleep path proven by Phase 11.
	 * If a prerequisite is missing, fall back to the known-good 09f WFI loop.
	 * On a real PDN wake the function restores both active SCU core states and
	 * returns here in the same Core-0 context, with the raw wake state saved.
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
	 * Stop ARM9 recording as soon as the lid opens. The STOP PXI interrupt is
	 * removed from ARM9's IRQ-source counters by the ARM9 handler.
	 */
	if(deepGfxSleep)
	{
		writeGfxResumeTraceStageRaw(0x20);
		MCU_setPowerLedPattern(MCU_PWR_LED_FAST_RED);
	}

	const Result arm9DiagStopRes = (arm9DiagEnabled && arm9DiagStartRes == RES_OK ?
	                                stopArm9SleepDiag(&arm9Diag) : arm9DiagStartRes);

	/*
	 * Never touch the framebuffer-backed debug console while the graphics
	 * domain is asleep.  In the deep preflight path VRAM/GPU are still off
	 * here, so even diagnostic ee_printf() calls must wait until cold resume
	 * has completed.
	 */
	if(!deepGfxSleep)
	{
		ee_printf("GBA Sleep: ARM11 wakeups while lid closed: %lu (lightweight gfx sleep)\n",
		          (unsigned long)arm11Wakeups);
		if(arm9DiagStartRes == RES_OK && arm9DiagStopRes == RES_OK)
			ee_printf("GBA Sleep: ARM9 WFI wakeups while lid closed: %lu\n",
			          (unsigned long)arm9Diag.wakeups);
		else
			ee_printf("GBA Sleep: ARM9 diagnostic failed: start=%08lX stop=%08lX\n",
			          (unsigned long)arm9DiagStartRes, (unsigned long)arm9DiagStopRes);
	}

	if(deepGfxSleep)
	{
		writeGfxResumeTraceStageRaw(0x30);
		GFX_sleepAwakeColdDebug(writeGfxResumeTraceStage);
		writeGfxResumeTraceStageRaw(0x50);
		MCU_setPowerLedPattern(MCU_PWR_LED_FAST_BLUE);
		OAF_videoResumeAfterGfxSleep();
		writeGfxResumeTraceStageRaw(0x51);

		/* The debug console is safe again only after the graphics cold resume. */
		if(pdnSleepReturned)
			ee_printf("GBA Sleep: ARM11 PDN system sleep resumed successfully\n");
		else
			ee_printf("GBA Sleep: ARM11 wakeups while lid closed: %lu (GPU/VRAM off fallback)\n",
			          (unsigned long)arm11Wakeups);
		ee_printf("GBA Sleep: ARM9 diagnostic skipped during GPU/VRAM preflight\n");
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
	if(deepGfxSleep) writeGfxResumeTraceStageRaw(0x60);

	/*
	 * Keep the synthetic wake combination asserted for one VBlank.
	 */
	GFX_waitForVBlank0();
	if(deepGfxSleep) writeGfxResumeTraceStageRaw(0x61);

	lgy11->pad_val = oldPadVal;
	lgy11->pad_sel = oldPadSel;

	MCU_setPowerLedPattern(MCU_PWR_LED_AUTO);
	GFX_powerOnBacklight(GFX_BL_BOTH);
	LGYCAP_start(LGYCAP_DEV_TOP);
	CODEC_wakeup();
	CODEC_setVolumeOverride(g_oafConfig.volume);
	if(deepGfxSleep) writeGfxResumeTraceStageRaw(0x70);

	/*
	 * Measure again under the restored fully-awake load. The short settling
	 * delay keeps the voltage comparison from being dominated by resume
	 * transients. The same delay is used by the upstream baseline build.
	 */
	TIMER_sleepMs(250);
	const BatterySnapshot batteryAfter = readBatterySnapshot();
	const PowerStateSnapshot powerAwakeAfter = readPowerStateSnapshot();

	const Result logRes = appendPowerStateLog(&powerAwakeBefore, &powerSleepReady,
	                                          &powerAwakeAfter, &arm9Diag,
	                                          arm9DiagStartRes, arm9DiagStopRes,
	                                          &batteryBefore, &batteryAfter, arm11Wakeups,
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

	if(deepGfxSleep) clearGfxResumeTrace();
}
