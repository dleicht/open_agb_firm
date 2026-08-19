/*
 * ARM11 PDN system-sleep coordination and retained wake-state diagnostics.
 * Both active cores enter SCU DORMANT before the PDN sleep request.
 */
#include "arm.h"
#include "types.h"
#include "system.h"
#include "fs.h"
#include "arm11/fmt.h"
#include "arm11/config.h"
#include "arm11/open_agb_firm.h"
#include "arm11/arm11_suspend_probe.h"
#include "arm11/drivers/gpio.h"
#include "arm11/drivers/i2c.h"
#include "arm11/drivers/interrupt.h"
#include "arm11/drivers/mcu.h"
#include "arm11/drivers/mcu_regmap.h"
#include "arm11/drivers/pdn.h"
#include "arm11/drivers/scu.h"

#define ARM11_SUSPEND_COOKIE_MAGIC       0x53313150u /* Retained suspend cookie. */
#define ARM11_SUSPEND_COOKIE_MAGIC_INV   (~ARM11_SUSPEND_COOKIE_MAGIC)
#define ARM11_SUSPEND_PROBE_MCU_OFF      8u
#define ARM11_SUSPEND_PROBE_LOG_PATH     OAF_WORK_DIR "/power_state.log"

#define PROBE_MARK_ARMED                 0xA0u
#define PROBE_MARK_VRAM_STATUS_OK        0xA1u
#define PROBE_MARK_CORE1_DORMANT_READY   0xA2u
#define PROBE_MARK_CORE0_SLEEP_ENTER     0xA3u
#define PROBE_MARK_RESUME_ENTRY          0xB0u
#define PROBE_MARK_SAME_CONTEXT_RETURN   0xB1u
#define PROBE_MARK_CORE0_NORMAL          0xB2u
#define PROBE_MARK_CORE1_NORMAL          0xB3u
#define PROBE_MARK_PDN_WAKE_ACKED         0xB4u
#define PROBE_MARK_VRAM_STATUS_MISSING   0xE1u
#define PROBE_MARK_MCU_PREPARE_FAILED    0xE2u

typedef struct
{
	u32 magic;
	u32 magicInv;
	vu32 armed;
	vu32 core1Ready;
	vu32 core1ResumeRequested;
	vu32 core1Resumed;
} Arm11SuspendProbeCookie;

/*
 * NOLOAD retention state.  The linker keeps this outside BSS so the early
 * reset-entry hook can inspect it before normal startup clears BSS.
 */
__attribute__((section(".arm11_suspend_noinit"), aligned(32), used))
static volatile Arm11SuspendProbeCookie g_arm11SuspendProbeCookie;

typedef struct
{
	u8 magic0;
	u8 magic1;
	u8 stage;
	u8 check;
} Arm11SuspendProbeMarker;

#define PROBE_MCU_MAGIC0 0x51u
#define PROBE_MCU_MAGIC1 0x11u

static const char* getProbeStageName(const u8 stage)
{
	switch(stage)
	{
		case PROBE_MARK_ARMED:               return "SUSPEND_ARMED";
		case PROBE_MARK_VRAM_STATUS_OK:      return "VRAM_STATUS_OK";
		case PROBE_MARK_CORE1_DORMANT_READY: return "CORE1_DORMANT_READY";
		case PROBE_MARK_CORE0_SLEEP_ENTER:   return "CORE0_SLEEP_ENTER";
		case PROBE_MARK_RESUME_ENTRY:        return "RESUME_ENTRY_REACHED";
		case PROBE_MARK_SAME_CONTEXT_RETURN: return "SAME_CONTEXT_RETURN";
		case PROBE_MARK_CORE0_NORMAL:        return "CORE0_NORMAL";
		case PROBE_MARK_CORE1_NORMAL:        return "CORE1_NORMAL";
		case PROBE_MARK_PDN_WAKE_ACKED:      return "PDN_WAKE_ACKED";
		case PROBE_MARK_VRAM_STATUS_MISSING: return "VRAM_STATUS_MISSING";
		case PROBE_MARK_MCU_PREPARE_FAILED:  return "MCU_PREPARE_FAILED";
		default:                             return "UNKNOWN";
	}
}

static void writeProbeMarker(const u8 stage)
{
	const Arm11SuspendProbeMarker marker =
	{
		.magic0 = PROBE_MCU_MAGIC0,
		.magic1 = PROBE_MCU_MAGIC1,
		.stage = stage,
		.check = (u8)(stage ^ 0xFFu)
	};
	(void)MCU_setFreeRamData(ARM11_SUSPEND_PROBE_MCU_OFF,
	                         (const u8*)&marker, sizeof(marker));
}

/*
 * Record an unexpected reset-entry wake before normal libn3ds startup. This
 * path cannot depend on IRQs, events, mutexes or initialized BSS.
 */
static void writeEarlyResumeMarker(void)
{
	static const u8 marker[4] =
	{
		PROBE_MCU_MAGIC0,
		PROBE_MCU_MAGIC1,
		PROBE_MARK_RESUME_ENTRY,
		(u8)(PROBE_MARK_RESUME_ENTRY ^ 0xFFu)
	};

	I2cBus *const i2c = getI2cBusRegs(I2C_BUS2);
	while(i2c->cnt & I2C_EN);
	i2c->cntex = I2C_CLK_STRETCH_EN;
	i2c->scl = I2C_DELAYS(5u, 0u);

	(void)I2C_writeRegIntSafe(I2C_DEV_CTR_MCU, MCU_REG_FREE_RAM_OFF,
	                          ARM11_SUSPEND_PROBE_MCU_OFF);
	(void)I2C_writeRegArrayIntSafe(I2C_DEV_CTR_MCU, MCU_REG_FREE_RAM_DATA,
	                               marker, sizeof(marker));
}

/*
 * Called on Core 0 after a valid stack exists but before BSS is cleared. A
 * matching PDN wake reason prevents a later hard reset from reusing a stale
 * retained cookie.
 */
void arm11SuspendProbeEarlyResume(void)
{
	volatile Arm11SuspendProbeCookie *const cookie = &g_arm11SuspendProbeCookie;
	if(cookie->magic != ARM11_SUSPEND_COOKIE_MAGIC ||
	   cookie->magicInv != ARM11_SUSPEND_COOKIE_MAGIC_INV ||
	   cookie->armed != 1u)
		return;

	Pdn *const pdn = getPdnRegs();
	if((pdn->wake_reason & (PDN_WAKE_SHELL_OPENED | PDN_WAKE_MCU)) == 0)
		return;

	/* Prevent a later hard reset from reusing the retained wake cookie. */
	cookie->armed = 0;
	__dsb();

	writeEarlyResumeMarker();

	/*
	 * A reset-entry wake cannot safely resume the suspended execution context,
	 * so remain parked and preserve the marker for the next boot.
	 */
	while(1) __wfi();
}

static void core1DormantProbeEntry(void)
{
	Scu *const scu = getScuRegs();

	/*
	 * core1Standby() unregisters IPI1 immediately before calling this entry.
	 * Re-arm it here so Core 0 can wake Core 1 out of the post-PDN WFI.
	 */
	IRQ_registerIsr(IRQ_IPI1, 14, 0, (IrqIsr)NULL);

	/* Publish readiness while Core 1 still participates in coherency. */
	g_arm11SuspendProbeCookie.core1Ready = 1u;
	__dsb();

	u32 cpuStat = scu->cpu_stat;
	cpuStat &= ~SCU_STAT_MASK(1u);
	cpuStat |= SCU_STAT_DORMANT(1u);
	scu->cpu_stat = cpuStat;
	__dsb();

	/*
	 * Enter WFI exactly once while advertising DORMANT. If the global PDN
	 * shell-open wake releases Core 1 as well as Core 0, execution resumes here.
	 * Do not enter a second WFI while still marked DORMANT: that can deadlock
	 * Core 0's later IPI release because a dormant secondary core may no longer
	 * accept the software interrupt used to wake it.
	 */
	__wfi();

	/* We are executing again: immediately stop advertising Core 1 as dormant. */
	cpuStat = scu->cpu_stat;
	cpuStat &= ~SCU_STAT_MASK(1u);
	cpuStat |= SCU_STAT_NORMAL(1u);
	scu->cpu_stat = cpuStat;
	__dsb();

	/* Publish the automatic PDN wake before waiting for Core 0's release. */
	g_arm11SuspendProbeCookie.core1Resumed = 1u;
	__dsb();

	/* Stay awake until Core 0 has observed/accepted the secondary-core resume. */
	while(g_arm11SuspendProbeCookie.core1ResumeRequested == 0u)
		__asm__ volatile("nop");

	/* The temporary IPI registration is no longer needed. */
	IRQ_unregisterIsr(IRQ_IPI1);
	/* Return to libn3ds core1Standby(). */
}

bool arm11SuspendProbeEnter(Arm11SuspendWakeInfo *const info)
{
	Pdn *const pdn = getPdnRegs();
	Scu *const scu = getScuRegs();

	if(info != NULL)
	{
		*info = (Arm11SuspendWakeInfo){0};
		info->pdnCntBeforeSleep = pdn->cnt;
		info->wakeEnableBefore = pdn->wake_enable;
		info->scuCpuStatBeforeSleep = scu->cpu_stat;
	}

	/*
	 * GFX_sleep() ran before this function. Treat bit15 as a hardware status,
	 * not as a software request. Do not enter PDN sleep unless the graphics
	 * power-down prerequisite is observable without fabricating the bit.
	 */
	if((pdn->cnt & PDN_CNT_VRAM_OFF) == 0)
	{
		writeProbeMarker(PROBE_MARK_VRAM_STATUS_MISSING);
		return false;
	}
	writeProbeMarker(PROBE_MARK_VRAM_STATUS_OK);

	/* Clear any deferred shell-close MCU event before arming the wake path. */
	(void)MCU_getIrqs(0xFFFFFFFFu);

	const u32 staleReason = pdn->wake_reason;
	if(staleReason != 0)
		pdn->wake_reason = staleReason;
	pdn->wake_enable = PDN_WAKE_SHELL_OPENED | PDN_WAKE_MCU;
	if(info != NULL) info->wakeEnableDuringSleep = pdn->wake_enable;

	if(!MCU_writeReg(MCU_REG_SYS_PWR, BIT(4)))
	{
		writeProbeMarker(PROBE_MARK_MCU_PREPARE_FAILED);
		pdn->wake_enable = (info != NULL ? info->wakeEnableBefore : 0u);
		return false;
	}

	g_arm11SuspendProbeCookie.magic = ARM11_SUSPEND_COOKIE_MAGIC;
	g_arm11SuspendProbeCookie.magicInv = ARM11_SUSPEND_COOKIE_MAGIC_INV;
	g_arm11SuspendProbeCookie.armed = 1u;
	g_arm11SuspendProbeCookie.core1Ready = 0u;
	g_arm11SuspendProbeCookie.core1ResumeRequested = 0u;
	g_arm11SuspendProbeCookie.core1Resumed = 0u;
	__dsb();
	writeProbeMarker(PROBE_MARK_ARMED);

	/* Put the otherwise-idle second ARM11 core into the documented SCU state. */
	__systemBootCore1(core1DormantProbeEntry);
	while(g_arm11SuspendProbeCookie.core1Ready == 0u)
		__asm__ volatile("nop");
	while((scu->cpu_stat & SCU_STAT_MASK(1u)) != SCU_STAT_DORMANT(1u))
		__asm__ volatile("nop");
	writeProbeMarker(PROBE_MARK_CORE1_DORMANT_READY);

	/* No I/O is performed after Core 0 advertises DORMANT. */
	writeProbeMarker(PROBE_MARK_CORE0_SLEEP_ENTER);
	u32 cpuStat = scu->cpu_stat;
	cpuStat &= ~SCU_STAT_MASK(0u);
	cpuStat |= SCU_STAT_DORMANT(0u);
	scu->cpu_stat = cpuStat;
	__dsb();

	/* Preserve the hardware VRAM-off status while asserting only SLEEP. */
	pdn->cnt = (u16)((pdn->cnt & PDN_CNT_VRAM_OFF) | PDN_CNT_SLEEP);
	__dsb();
	__isb();
	__wfi();

	/* Capture the raw wake state before acknowledging or restoring anything. */
	if(info != NULL)
	{
		info->entered = true;
		info->pdnCntAfterWake = pdn->cnt;
		info->wakeReasonAfterWake = pdn->wake_reason;
		info->scuCpuStatAtWake = scu->cpu_stat;
		info->shellOpenAfterWake = (GPIO_read(GPIO_1_SHELL) ? 0u : 1u);
	}
	writeProbeMarker(PROBE_MARK_SAME_CONTEXT_RETURN);

	/* Core 0 is executing again, so stop advertising it as DORMANT first. */
	cpuStat = scu->cpu_stat;
	cpuStat &= ~SCU_STAT_MASK(0u);
	cpuStat |= SCU_STAT_NORMAL(0u);
	scu->cpu_stat = cpuStat;
	__dsb();
	writeProbeMarker(PROBE_MARK_CORE0_NORMAL);

	/* Release Core 1 and give its WFI loop an explicit wake event. */
	g_arm11SuspendProbeCookie.core1ResumeRequested = 1u;
	__dsb();
	IRQ_softInterrupt(IRQ_IPI1, BIT(1));
	while(g_arm11SuspendProbeCookie.core1Resumed == 0u)
		__asm__ volatile("nop");
	writeProbeMarker(PROBE_MARK_CORE1_NORMAL);

	if(info != NULL) info->scuCpuStatAfterRestore = scu->cpu_stat;

	/* Acknowledge the raw wake reason and restore the caller's wake mask. */
	const u32 wakeReason = pdn->wake_reason;
	if(wakeReason != 0u) pdn->wake_reason = wakeReason;
	pdn->wake_enable = (info != NULL ? info->wakeEnableBefore : 0u);

	g_arm11SuspendProbeCookie.armed = 0u;
	writeProbeMarker(PROBE_MARK_PDN_WAKE_ACKED);
	return true;
}

void arm11SuspendProbeReportPrevious(void)
{
	Arm11SuspendProbeMarker marker = {0};
	if(!MCU_getFreeRamData(ARM11_SUSPEND_PROBE_MCU_OFF,
	                       (u8*)&marker, sizeof(marker)))
		return;

	if(marker.magic0 != PROBE_MCU_MAGIC0 ||
	   marker.magic1 != PROBE_MCU_MAGIC1 ||
	   marker.check != (u8)(marker.stage ^ 0xFFu))
		return;

	const char *const name = getProbeStageName(marker.stage);
	ee_printf("Previous ARM11 suspend state: %02X: %s\n", marker.stage, name);

	char line[128];
	const int len = ee_snprintf(line, sizeof(line),
	                            "Previous ARM11 suspend state: %02X: %s\n",
	                            marker.stage, name);
	if(len > 0)
	{
		FHandle file;
		if(fOpen(&file, ARM11_SUSPEND_PROBE_LOG_PATH,
		         FA_OPEN_APPEND | FA_WRITE) == RES_OK)
		{
			const u32 writeLen = ((u32)len < sizeof(line)) ?
			                     (u32)len : (sizeof(line) - 1u);
			(void)fWrite(file, line, writeLen, NULL);
			(void)fClose(file);
		}
	}

	const Arm11SuspendProbeMarker clear = {0};
	(void)MCU_setFreeRamData(ARM11_SUSPEND_PROBE_MCU_OFF,
	                         (const u8*)&clear, sizeof(clear));
}
