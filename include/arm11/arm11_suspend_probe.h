#pragma once

#include "types.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    bool entered;
    u16 pdnCntBeforeSleep;
    u16 pdnCntAfterWake;
    u32 wakeEnableBefore;
    u32 wakeEnableDuringSleep;
    u32 wakeReasonAfterWake;
    u32 scuCpuStatBeforeSleep;
    u32 scuCpuStatAtWake;
    u32 scuCpuStatAfterRestore;
    u8 shellOpenAfterWake;
} Arm11SuspendWakeInfo;

/*
 * Phase 12 experimental ARM11 system suspend.
 *
 * Phase 11 established on real N3DS hardware that PDN sleep returns behind
 * WFI in the same Core-0 execution context when both active ARM11 cores are
 * advertised as SCU DORMANT.  This function now restores the SCU/core state
 * and returns to the caller so the proven 09f graphics cold-resume can run.
 *
 * false: a prerequisite was missing; caller should use the known-good WFI
 *        fallback.
 * true:  PDN sleep was entered and returned on wake.
 */
bool arm11SuspendProbeEnter(Arm11SuspendWakeInfo *info);
void arm11SuspendProbeEarlyResume(void);
void arm11SuspendProbeReportPrevious(void);

#ifdef __cplusplus
}
#endif
