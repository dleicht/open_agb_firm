#pragma once
/*
 * Automatic lid integration for universal GBA Sleep/Wake support.
 */

#ifdef __cplusplus
extern "C"
{
#endif

void gbaSleepReportPreviousResumeTrace(void);
void gbaSleepHandleLid(void);

#ifdef __cplusplus
} // extern "C"
#endif
