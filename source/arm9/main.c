/*
 *   This file is part of open_agb_firm
 *   Copyright (C) 2021 derrek, profi200
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "types.h"
#include "mem_map.h"
#include "arm.h"
#include "ipc_handler.h"
#include "drivers/pxi.h"
#include "drivers/tmio.h"
#include "arm9/drivers/cfg9.h"
#include "arm9/drivers/irq9.h"
#include "arm9/drivers/ndma.h"
#include "arm9/drivers/timer.h"


/*
 * Temporary read-only ARM9 sleep diagnostics.
 *
 * The generic IRQ entry code increments g_arm9SleepDiagIrqCounts[] while
 * g_arm9SleepDiagActive is set. The main WFI loop separately counts how many
 * times the CPU actually returned from WFI. No power or IRQ configuration is
 * changed by this instrumentation.
 */
vu32 g_arm9SleepDiagActive = 0;
u32 g_arm9SleepDiagIrqCounts[32] = {0};

static vu32 g_arm9SleepDiagStartPending = 0;
static u32 g_arm9SleepDiagWakeups = 0;
static Arm9PowerStateSnapshot g_arm9SleepDiagBefore;

static void readArm9PowerStateSnapshot(Arm9PowerStateSnapshot *const snapshot)
{
	Irq9 *const irq = getIrq9Regs();
	Cfg9 *const cfg9 = getCfg9Regs();
	Pxi *const pxi = getPxiRegs();

	snapshot->irqIe = irq->ie;
	snapshot->irqIf = irq->_if;

	for(u32 i = 0; i < 4; i++)
	{
		Timer *const timer = getTimerRegs(i);
		snapshot->timerVal[i] = timer->val;
		snapshot->timerCnt[i] = timer->cnt;
	}

	snapshot->cfgXdmaReq = cfg9->xdma_req;
	snapshot->cfgCardPower = cfg9->card_power;
	snapshot->cfgCardCtl = cfg9->cardctl;
	snapshot->cfgSdmmcCtl = cfg9->sdmmcctl;
	snapshot->cfgExtmemCnt9 = cfg9->extmemcnt9;
	snapshot->_reserved[0] = 0;
	snapshot->_reserved[1] = 0;
	snapshot->_reserved[2] = 0;

	snapshot->pxiCnt = pxi->cnt;
	snapshot->ndmaGcnt = REG_NDMA_GCNT;
	for(u32 i = 0; i < 8; i++) snapshot->ndmaCnt[i] = getNdmaChRegs(i)->cnt;

	Tmio *const tmio1 = getTmioRegs(0);
	snapshot->tmioStatus[0] = tmio1->sd_status;
	snapshot->tmioStatusMask[0] = tmio1->sd_status_mask;
	snapshot->tmioClkCtrl[0] = tmio1->sd_clk_ctrl;
	snapshot->tmioPortSel[0] = tmio1->sd_portsel;
	snapshot->tmioSoftRst[0] = tmio1->soft_rst;
	snapshot->tmioExtCdetMask[0] = tmio1->ext_cdet_mask;

	/* TMIO3 is remappable. Do not read its ARM9 window when mapped to ARM11. */
	if(!(snapshot->cfgSdmmcCtl & SDMMCCTL_TMIO3_MAP11))
	{
		Tmio *const tmio3 = getTmioRegs(1);
		snapshot->tmioStatus[1] = tmio3->sd_status;
		snapshot->tmioStatusMask[1] = tmio3->sd_status_mask;
		snapshot->tmioClkCtrl[1] = tmio3->sd_clk_ctrl;
		snapshot->tmioPortSel[1] = tmio3->sd_portsel;
		snapshot->tmioSoftRst[1] = tmio3->soft_rst;
		snapshot->tmioExtCdetMask[1] = tmio3->ext_cdet_mask;
	}
	else
	{
		snapshot->tmioStatus[1] = UINT32_MAX;
		snapshot->tmioStatusMask[1] = UINT32_MAX;
		snapshot->tmioClkCtrl[1] = UINT16_MAX;
		snapshot->tmioPortSel[1] = UINT16_MAX;
		snapshot->tmioSoftRst[1] = UINT16_MAX;
		snapshot->tmioExtCdetMask[1] = UINT16_MAX;
	}
}

void OAF_arm9SleepDiagRequestStart(void)
{
	g_arm9SleepDiagActive = 0;
	g_arm9SleepDiagStartPending = 0;
	g_arm9SleepDiagWakeups = 0;
	for(u32 i = 0; i < 32; i++) g_arm9SleepDiagIrqCounts[i] = 0;

	readArm9PowerStateSnapshot(&g_arm9SleepDiagBefore);

	/*
	 * Arm only after the PXI START interrupt returns to main(). This prevents
	 * the diagnostic command itself from being counted as a WFI wakeup.
	 */
	g_arm9SleepDiagStartPending = 1;
}

void OAF_arm9SleepDiagStop(Arm9SleepDiagReport *const report)
{
	/* The IRQ entry code sees the old active state before this handler runs. */
	const bool stopIrqWasCounted = (g_arm9SleepDiagActive != 0);

	/* Stop recording before the STOP command returns to the WFI loop. */
	g_arm9SleepDiagActive = 0;
	g_arm9SleepDiagStartPending = 0;

	/* Remove the diagnostic-generated STOP PXI interrupt from the data. */
	if(stopIrqWasCounted && g_arm9SleepDiagIrqCounts[12] > 0) g_arm9SleepDiagIrqCounts[12]--;

	report->wakeups = g_arm9SleepDiagWakeups;
	for(u32 i = 0; i < 32; i++) report->irqCounts[i] = g_arm9SleepDiagIrqCounts[i];
	report->before = g_arm9SleepDiagBefore;
	readArm9PowerStateSnapshot(&report->after);
}


int main(void)
{
	while(1)
	{
		__wfi();

		if(g_arm9SleepDiagStartPending)
		{
			/* The WFI return caused by the START command is intentionally skipped. */
			g_arm9SleepDiagStartPending = 0;
			g_arm9SleepDiagActive = 1;
			continue;
		}

		if(g_arm9SleepDiagActive) g_arm9SleepDiagWakeups++;
	}

	return 0;
}
