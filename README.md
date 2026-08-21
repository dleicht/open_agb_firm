# open_agb_firm with extras!
This fork adds additional features to [open_agb_firm](https://github.com/profi200/open_agb_firm).
Please find the original documentation there.

## tl;dr
`Native GBA SWI 03h sleep using dynamically injected ARM7 IRQ code, combined with real 3DS PDN system sleep, runtime IRQ-vector lid triggering, Core 1/color-profile support, border restoration, and reliable full-system wake.`

## Battery life
> [!TIP]
> Since this uses real device sleep it gives you way longer battery life!

Here are the results of my latest 10 hours sleep test. This was done running [Goodboy Galaxy](https://goodboygalaxy.itch.io/goodboy-galaxy-gba), but the game shouldn't make much of a difference:

| Fork | Drain in %/h | Estimated runtime |
| ---- | ------------ | ----------------- |
| profi200 | 13 %/h | ~ 8 hours |
| dleicht | 1,5 %/h | ~ 66 hours |


## Full changelog

### Dynamic On-the-Fly GBA IRQ Sleep-Handler Injection
- Implements a custom **ARM7 GBA IRQ sleep handler** without requiring any modification to the original ROM file on disk
- Dynamically searches the loaded ROM image for a safe location at runtime
- Prefers validated unused **`0x00` / `0xFF` ROM padding**
- Falls back to open_agb_firm's controlled **fake open-bus ROM tail** when no suitable padding is available
- Uses guard regions around injected code to avoid overwriting adjacent ROM data
- Respects EEPROM-sensitive ROM address space and avoids the reserved EEPROM area
- Validates the selected injection location before installing the handler
- Flushes the modified ROM area from cache so the Legacy/GBA side sees the injected ARM7 code
- Dynamically redirects the GBA BIOS IRQ path to the injected handler
- Falls back transparently to the original BIOS IRQ handler if no safe injection location can be found
- Allows the same sleep implementation to work across games without maintaining per-ROM patches or fixed handler addresses

### Universal Sleep/Wake
- Runtime IRQ Vector Redirection
- The injected handler contains both the normal GBA IRQ path and the shared `SWI 03h` sleep path
- Manual **L + SELECT** continues to enter the sleep path through the normal handler
- Automatic buttonless lid sleep temporarily redirects the **ARM7 IRQ vector directly to the already injected sleep entry**
- No synthetic GBA key combo is required for automatic lid sleep
- The normal IRQ vector is restored after the GBA has entered sleep
- This avoids input leakage into games while reusing the exact same proven sleep handler

### Real ARM11 PDN System Sleep
- ARM11 enters the 3DS **PDN system-sleep state** instead of merely waiting in WFI
- Wake is driven by the lid/system wake source
- SCU/CPU state is restored after wake

### ARM11 Core 1 Sleep Compatibility
- Handles the Core 1 worker used by open_agb_firm for color conversion
- Cleanly returns Core 1 to standby before PDN sleep
- Restarts the color-conversion worker after wake

### Full `colorProfile != none` Sleep/Wake Support
- Color-profile rendering now survives full PDN sleep
- Color LUT and required video/IRQ state are rebuilt after wake
- Works with the Core 1 color-conversion path

### Cold Graphics Resume
- GPU/display state is rebuilt after PDN sleep
- VRAM-dependent presentation state is restored
- Capture/rendering resumes correctly after wake

### `border.bgr` Restoration
- Unscaled / `scaler=none` mode restores the custom border after graphics power-down
- Works together with PDN sleep and color profiles

### Audio Power Handling
- Codec/I2S are suspended during sleep
- Audio hardware and configured volume are restored after wake

### Power-State Diagnostics
- `power_state.log` records:
  - ARM11 PDN state
  - SCU state
  - Relevant power-domain state
  - Battery measurements

### Fullscreen Stretch Scaler
- Adds the fullscreen/stretch rendering option developed on top of the original open_agb_firm scaler implementation
