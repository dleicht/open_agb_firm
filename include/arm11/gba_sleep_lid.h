#pragma once
/*
 * Automatic system sleep-input integration for universal GBA Sleep/Wake.
 * Handles the clamshell lid and the original 2DS hardware sleep switch through
 * the shared KEY_SHELL signal.
 */

#ifdef __cplusplus
extern "C"
{
#endif

void gbaSleepHandleSystemSleepInput(void);

#ifdef __cplusplus
} // extern "C"
#endif
