# open_agb_firm with extras!
This fork adds additional features to [open_agb_firm](https://github.com/profi200/open_agb_firm).

# Clamshell Sleep/Wake:
- real GBA BIOS SWI 03h / STOP
- universal pre-BIOS IRQ handler
- dynamic per-ROM handler placement
- EEPROM-safe handler allocation
- manual L+Select Sleep / R+Select Wake
- automatic lid-close Sleep / lid-open Wake
- N3DS display/audio/power LED sleep integration

# Full screen stretching:
- Adding a full screen "stretch" scaler to the video options in the config.
- It stretches the matrix-scaled 360x240 image horizontally to the full 400x240 top-screen framebuffer.

Based on [profi200/open_agb_firm](https://github.com/profi200/open_agb_firm). Licensed under GPL-3.0-or-later.  
Please find the original documentation there.
