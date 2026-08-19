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
 * Enter ARM11 PDN system sleep with both active cores advertised as SCU
 * DORMANT. On wake, restore the SCU/core state and return in the same Core-0
 * execution context.
 *
 * false: a prerequisite was unavailable; caller should use the WFI fallback.
 * true:  PDN sleep was entered and returned on wake.
 */
bool arm11SuspendProbeEnter(Arm11SuspendWakeInfo *info);
void arm11SuspendProbeEarlyResume(void);
void arm11SuspendProbeReportPrevious(void);

#ifdef __cplusplus
}
#endif
