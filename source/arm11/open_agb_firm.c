/*
 *   This file is part of open_agb_firm
 *   Copyright (C) 2024 derrek, profi200
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

#include <stdlib.h>
#include <string.h>
#include "types.h"
#include "drivers/cache.h"
#include "util.h"
#include "arm11/fast_rom_padding.h"
#include "oaf_error_codes.h"
#include "fs.h"
#include "arm11/fmt.h"
#include "arm11/drivers/mcu.h"
#include "drivers/gfx.h"
#include "arm11/drivers/hid.h"
#include "fsutil.h"
#include "arm11/filebrowser.h"
#include "arm11/config.h"
#include "arm11/save_type.h"
#include "arm11/patch.h"
#include "arm11/drivers/codec.h"
#include "drivers/lgy_common.h"
#include "arm11/oaf_video.h"
#include "arm11/drivers/lgy11.h"
#include "kernel.h"
#include "kevent.h"


static KHandle g_frameReadyEvent = 0;
static bool g_gbaSleepAvailable = false;


#define GBA_SLEEP_HANDLER_ROM_BASE          0x08000000u
#define GBA_SLEEP_BIOS_IRQ_HANDLER      0x00000128u
#define GBA_SLEEP_EEPROM_RESERVED_OFF   0x01FFFF00u
#define GBA_SLEEP_HANDLER_ALIGN         4u
#define GBA_SLEEP_HANDLER_PAD_GUARD             0x40u
#define GBA_SLEEP_HANDLER_FORCE_ENTRY_OFF       0x20u

static u32 g_gbaSleepHandlerGbaAddr = GBA_SLEEP_BIOS_IRQ_HANDLER;

typedef enum
{
	GBA_SLEEP_PLACE_NONE = 0,
	GBA_SLEEP_PLACE_ROM_PADDING,
	GBA_SLEEP_PLACE_OPEN_BUS_TAIL
} GbaSleepHandlerPlacementKind;

typedef struct
{
	u32 offset;
	u32 spanStart;
	u32 spanSize;
	u8 fillByte;
	GbaSleepHandlerPlacementKind kind;
} GbaSleepHandlerPlacement;

static bool isEepromSaveType(const u16 saveType)
{
	const u16 type = saveType & SAVE_TYPE_MASK;
	return type == SAVE_TYPE_EEPROM_8k ||
	       type == SAVE_TYPE_EEPROM_8k_2 ||
	       type == SAVE_TYPE_EEPROM_64k ||
	       type == SAVE_TYPE_EEPROM_64k_2;
}

static bool rangeFits(const u32 offset, const u32 size, const u32 limit)
{
	return offset <= limit && size <= limit - offset;
}

static bool findGbaSleepHandlerPlacement(const u8 *const rom,
                                       const u32 romSize,
                                       const u32 handlerSize,
                                       const u16 saveType,
                                       GbaSleepHandlerPlacement *const placementOut)
{
	if(rom == NULL || placementOut == NULL || handlerSize == 0 ||
	   romSize == 0 || romSize > LGY_MAX_ROM_SIZE ||
	   handlerSize > LGY_MAX_ROM_SIZE)
		return false;

	placementOut->offset = 0;
	placementOut->spanStart = 0;
	placementOut->spanSize = 0;
	placementOut->fillByte = 0;
	placementOut->kind = GBA_SLEEP_PLACE_NONE;

	if(handlerSize > LGY_MAX_ROM_SIZE - GBA_SLEEP_HANDLER_PAD_GUARD * 2u)
		return false;

	const bool eeprom = isEepromSaveType(saveType);
	u32 searchEnd = romSize;
	if(eeprom && searchEnd > GBA_SLEEP_EEPROM_RESERVED_OFF)
		searchEnd = GBA_SLEEP_EEPROM_RESERVED_OFF;

	/*
	 * First choice: validated padding inside the final loaded ROM.
	 * Find homogeneous 0x00/0xFF runs and keep the highest-address safe
	 * candidate. Guards on both sides remain exactly 0x40 bytes.
	 */
	bool found = false;
	u32 bestOffset = 0;
	u32 bestRunStart = 0;
	u32 bestRunSize = 0;
	u8 bestFill = 0;

	if(searchEnd > 0)
	{
		u32 runStart = 0;
		u8 runFill = rom[0];
		const u32 required = handlerSize + GBA_SLEEP_HANDLER_PAD_GUARD * 2u;

		for(u32 i = 1; i <= searchEnd; i++)
		{
			const bool runEnded = (i == searchEnd || rom[i] != runFill);
			if(!runEnded)
				continue;

			const u32 runLen = i - runStart;
			if((runFill == 0x00u || runFill == 0xFFu) && runLen >= required)
			{
				u32 candidate = i - GBA_SLEEP_HANDLER_PAD_GUARD - handlerSize;
				candidate &= ~(GBA_SLEEP_HANDLER_ALIGN - 1u);

				if(candidate >= runStart + GBA_SLEEP_HANDLER_PAD_GUARD &&
				   rangeFits(candidate, handlerSize, i - GBA_SLEEP_HANDLER_PAD_GUARD))
				{
					bestOffset = candidate;
					bestRunStart = runStart;
					bestRunSize = runLen;
					bestFill = runFill;
					found = true;
				}
			}

			if(i < searchEnd)
			{
				runStart = i;
				runFill = rom[i];
			}
		}
	}

	if(found)
	{
		placementOut->offset = bestOffset;
		placementOut->spanStart = bestRunStart;
		placementOut->spanSize = bestRunSize;
		placementOut->fillByte = bestFill;
		placementOut->kind = GBA_SLEEP_PLACE_ROM_PADDING;
		return true;
	}

	/*
	 * Second choice: OAF's fake-open-bus tail.
	 * OAF owns the area after the virtual ROM/mirror and fills it with its
	 * fake-open-bus pattern. 1 MiB carts occupy 4 MiB because of mirroring.
	 */
	u32 occupiedEnd = romSize;
	if(occupiedEnd == 0x00100000u)
		occupiedEnd = 0x00400000u;

	u32 tailEnd = LGY_MAX_ROM_SIZE;
	if(eeprom && tailEnd > GBA_SLEEP_EEPROM_RESERVED_OFF)
		tailEnd = GBA_SLEEP_EEPROM_RESERVED_OFF;

	if(occupiedEnd > tailEnd)
		return false;

	const u32 candidate = (occupiedEnd + GBA_SLEEP_HANDLER_ALIGN - 1u) &
	                      ~(GBA_SLEEP_HANDLER_ALIGN - 1u);
	if(rangeFits(candidate, handlerSize, tailEnd))
	{
		placementOut->offset = candidate;
		placementOut->spanStart = occupiedEnd;
		placementOut->spanSize = tailEnd - occupiedEnd;
		placementOut->fillByte = 0;
		placementOut->kind = GBA_SLEEP_PLACE_OPEN_BUS_TAIL;
		return true;
	}

	return false;
}

static bool validateGbaSleepHandlerPlacement(const u8 *const rom,
                                    const u32 handlerSize,
                                    const GbaSleepHandlerPlacement *const placement)
{
	if(rom == NULL || placement == NULL || placement->kind == GBA_SLEEP_PLACE_NONE)
		return false;
	if(!rangeFits(placement->offset, handlerSize, LGY_MAX_ROM_SIZE))
		return false;
	if((placement->offset & (GBA_SLEEP_HANDLER_ALIGN - 1u)) != 0)
		return false;

	if(placement->kind == GBA_SLEEP_PLACE_ROM_PADDING)
	{
		if(placement->fillByte != 0x00u && placement->fillByte != 0xFFu)
			return false;
		if(!rangeFits(placement->spanStart, placement->spanSize,
		                    LGY_MAX_ROM_SIZE))
			return false;
		if(placement->offset < placement->spanStart + GBA_SLEEP_HANDLER_PAD_GUARD)
			return false;
		if(!rangeFits(placement->offset, handlerSize,
		                    placement->spanStart + placement->spanSize - GBA_SLEEP_HANDLER_PAD_GUARD))
			return false;

		// Re-check the exact bytes we are about to replace.
		for(u32 i = 0; i < handlerSize; i++)
			if(rom[placement->offset + i] != placement->fillByte)
				return false;
	}
	else if(placement->kind != GBA_SLEEP_PLACE_OPEN_BUS_TAIL)
	{
		return false;
	}

	return true;
}

static u32 fixRomPadding(const u32 romFileSize)
{
	// Pad unused ROM area with 0xFFs (trimmed ROMs).
	// Smallest retail ROM chip is 8 Mbit (1 MiB).
	u32 romSize = nextPow2(romFileSize);
	romSize = (romSize < 0x100000 ? 0x100000 : romSize);
	const uintptr_t romLoc = LGY_ROM_LOC;
	memset((void*)(romLoc + romFileSize), 0xFF, romSize - romFileSize);

	u32 mirroredSize = romSize;
	if(romSize == 0x100000) // 1 MiB.
	{
		// ROM mirroring for Classic NES Series/others with 8 Mbit ROM.
		// The ROM is mirrored exactly 4 times.
		// Thanks to endrift for discovering this.
		mirroredSize = 0x400000; // 4 MiB.
		uintptr_t mirrorLoc = romLoc + romSize;
		do
		{
			memcpy((void*)mirrorLoc, (void*)romLoc, romSize);
			mirrorLoc += romSize;
		} while(mirrorLoc < romLoc + mirroredSize);
	}

	// Fake "open bus" padding.
	if(romSize < LGY_MAX_ROM_SIZE)
		makeOpenBusPaddingFast((u32*)(romLoc + mirroredSize));

	// We don't return the mirrored size because the db hashes are over unmirrored dumps.
	return romSize;
}

static Result loadGbaRom(const char *const path, u32 *const romSizeOut)
{
	FHandle f;
	Result res = fOpen(&f, path, FA_OPEN_EXISTING | FA_READ);
	if(res == RES_OK)
	{
		u32 fileSize = fSize(f);
		if(fileSize > LGY_MAX_ROM_SIZE)
		{
			fileSize = LGY_MAX_ROM_SIZE;
			ee_puts("Warning: ROM file is too big. Expect crashes.");
		}

		u32 read;
		res = fRead(f, (void*)LGY_ROM_LOC, fileSize, &read);
		fClose(f);

		if(read == fileSize) *romSizeOut = fixRomPadding(fileSize);

	}

	return res;
}

void changeBacklight(s16 amount)
{
	u8 min, max;
	if(MCU_getSystemModel() >= 4)
	{
		min = 16;
		max = 142;
	}
	else
	{
		min = 20;
		max = 117;
	}

	s16 newVal = g_oafConfig.backlight + amount;
	newVal = (newVal > max ? max : newVal);
	newVal = (newVal < min ? min : newVal);
	g_oafConfig.backlight = (u8)newVal;

	GFX_setLcdLuminance(newVal);
}

static void updateBacklight(void)
{
	// Check for special button combos.
	const u32 kHeld = hidKeysHeld();
	static bool backlightOn = true;
	if(hidKeysDown() && kHeld)
	{
		// Adjust LCD brightness up.
		const s16 steps = g_oafConfig.backlightSteps;
		if(kHeld == (KEY_X | KEY_DUP))
			changeBacklight(steps);

		// Adjust LCD brightness down.
		if(kHeld == (KEY_X | KEY_DDOWN))
			changeBacklight(-steps);

		// Disable backlight switching in debug builds on 2DS.
		const GfxBl lcd = (MCU_getSystemModel() != SYS_MODEL_2DS ? GFX_BL_TOP : GFX_BL_BOT);
#ifndef NDEBUG
		if(lcd != GFX_BL_BOT)
#endif
		{
			// Turn off backlight.
			if(backlightOn && kHeld == (KEY_X | KEY_DLEFT))
			{
				backlightOn = false;
				GFX_powerOffBacklight(lcd);
			}

			// Turn on backlight.
			if(!backlightOn && kHeld == (KEY_X | KEY_DRIGHT))
			{
				backlightOn = true;
				GFX_powerOnBacklight(lcd);
			}
		}
	}
}

static Result showFileBrowser(char romAndSavePath[512])
{
	Result res;
	char *lastDir = (char*)calloc(512, 1);
	if(lastDir != NULL)
	{
		do
		{
			// Get last ROM launch path.
			res = fsLoadPathFromFile("lastdir.txt", lastDir);
			if(res != RES_OK)
			{
				if(res == RES_FR_NO_FILE) strcpy(lastDir, "sdmc:/");
				else                      break;
			}

			// Show file browser.
			*romAndSavePath = '\0';
			res = browseFiles(lastDir, romAndSavePath);
			if(res == RES_FR_NO_PATH)
			{
				// Second chance in case the last dir has been deleted.
				strcpy(lastDir, "sdmc:/");
				res = browseFiles(lastDir, romAndSavePath);
				if(res != RES_OK) break;
			}
			else if(res != RES_OK) break;

			size_t cmpLen = strrchr(romAndSavePath, '/') - romAndSavePath;
			if((size_t)(strchr(romAndSavePath, '/') - romAndSavePath) == cmpLen) cmpLen++; // Keep the first '/'.
			if(cmpLen < 512)
			{
				if(cmpLen < strlen(lastDir) || strncmp(lastDir, romAndSavePath, cmpLen) != 0)
				{
					strncpy(lastDir, romAndSavePath, cmpLen);
					lastDir[cmpLen] = '\0';
					res = fsQuickWrite("lastdir.txt", lastDir, cmpLen + 1);
				}
			}
		} while(0);

		free(lastDir);
	}
	else res = RES_OUT_OF_MEM;

	return res;
}

static void rom2GameCfgPath(char romPath[512])
{
	if (g_oafConfig.useSavesFolder)
	{
		// Extract the file name and change the extension.
		// For cfg2SavePath() we need to reserve 2 extra bytes/chars.
		char tmpIniFileName[256];
		safeStrcpy(tmpIniFileName, strrchr(romPath, '/') + 1, 256 - 2);
		strcpy(tmpIniFileName + strlen(tmpIniFileName) - 4, ".ini");

		// Construct the new path.
		strcpy(romPath, OAF_SAVE_DIR "/");
		strcat(romPath, tmpIniFileName);
	}
	else
	{
		// Change the extension to .ini.
		strcpy(romPath + strlen(romPath) - 4, ".ini");
	}
}

static void gameCfg2SavePath(char cfgPath[512], const u8 saveSlot)
{
	if(saveSlot > 9)
	{
		*cfgPath = '\0'; // Prevent using the ROM as save file.
		return;
	}

	static char numberedExt[7] = {'.', 'X', '.', 's', 'a', 'v', '\0'};

	// Change the extension.
	// This relies on rom2GameCfgPath() to reserve 2 extra bytes/chars.
	numberedExt[1] = '0' + saveSlot;
	strcpy(cfgPath + strlen(cfgPath) - 4, (saveSlot == 0 ? ".sav" : numberedExt));
}

Result oafParseConfigEarly(void)
{
	Result res;
	do
	{
		// Create the work dir and switch to it.
		res = fsMakePath(OAF_WORK_DIR);
		if(res != RES_OK && res != RES_FR_EXIST) break;

		res = fChdir(OAF_WORK_DIR);
		if(res != RES_OK) break;

		// Create the saves folder.
		res = fMkdir(OAF_SAVE_DIR);
		if(res != RES_OK && res != RES_FR_EXIST) break;

		// Create screenshots folder.
		res = fMkdir(OAF_SCREENSHOT_DIR);
		if(res != RES_OK && res != RES_FR_EXIST) break;

		// Parse the config.
		res = parseOafConfig("config.ini", &g_oafConfig, true);
	} while(0);

	return res;
}

Result oafInitAndRun(void)
{
	Result res;
	char *const filePath = (char*)calloc(512, 1);
	if(filePath != NULL)
	{
		do
		{
			// Try to load the ROM path from autoboot.txt.
			// If this file doesn't exist show the file browser.
			res = fsLoadPathFromFile("autoboot.txt", filePath);
			if(res == RES_FR_NO_FILE)
			{
				res = showFileBrowser(filePath);
				if(res != RES_OK || *filePath == '\0') break;
				ee_puts("Loading...");
			}
			else if(res != RES_OK) break;

			//make copy of rom path
			char *const romFilePath = (char*)calloc(strlen(filePath)+1, 1);
			if(romFilePath == NULL) { res = RES_OUT_OF_MEM; break; }
			strcpy(romFilePath, filePath);

			// Load the ROM file.
			u32 romSize;
			res = loadGbaRom(filePath, &romSize);
			if(res != RES_OK) break;

			// Load the per-game config.
			rom2GameCfgPath(filePath);
			res = parseOafConfig(filePath, &g_oafConfig, false);
			if(res != RES_OK && res != RES_FR_NO_FILE) break;

			// Adjust the path for the save file and get save type.
			gameCfg2SavePath(filePath, g_oafConfig.saveSlot);
			u16 saveType;
			if(g_oafConfig.saveType != 0xFF)
				saveType = g_oafConfig.saveType;
			else if(g_oafConfig.useGbaDb || g_oafConfig.saveOverride)
				saveType = getSaveType(&g_oafConfig, romSize, filePath);
			else
				saveType = detectSaveType(romSize, g_oafConfig.defaultSave);

			patchRom(romFilePath, &romSize);

			/*
			 * Universal GBA Sleep/Wake support.
			 *
			 * Persistent ARM7 vector 0x18 enters the GBA Sleep IRQ handler before
			 * the normal GBA BIOS IRQ routine. The normal IRQ fast path inspects
			 * KEYINPUT only through FIQ-banked registers, touches no shared game
			 * registers, and uses no IRQ stack before chaining to BIOS 0x128.
			 *
			 * Manual controls:
			 *   L + Select -> real GBA STOP
			 *   R + Select -> wake
			 *
			 * Automatic lid sleep temporarily redirects vector slot 0x14 to the
			 * handler's shared sleep entry, so no GBA gameplay key is injected.
			 *
			 * The handler address is selected at runtime from validated ROM padding
			 * or OAF's fake-open-bus tail. EEPROM offset 0x01FFFF00..0x01FFFFFF
			 * is never used.
			 */
			static const u8 gbaSleepIrqHandler[] =
			{
				0xD1, 0xF0, 0x21, 0xE3, 0xDC, 0x80, 0x9F, 0xE5, 0xB0, 0x90, 0xD8, 0xE1,
				0x81, 0xAF, 0xA0, 0xE3, 0x0A, 0x00, 0x19, 0xE1, 0x01, 0x00, 0x00, 0x0A,
				0x92, 0xF0, 0x21, 0xE3, 0x4A, 0xFF, 0xA0, 0xE3, 0x92, 0xF0, 0x21, 0xE3,
				0x0F, 0x50, 0x2D, 0xE9, 0xF0, 0x0F, 0x2D, 0xE9, 0x01, 0x03, 0xA0, 0xE3,
				0x60, 0x10, 0x80, 0xE2, 0xFC, 0x03, 0xB1, 0xE8, 0xFC, 0x03, 0x2D, 0xE9,
				0xFC, 0x03, 0xB1, 0xE8, 0xFC, 0x03, 0x2D, 0xE9, 0x9C, 0x70, 0x9F, 0xE5,
				0x9C, 0x80, 0x9F, 0xE5, 0x9C, 0x90, 0x9F, 0xE5, 0xB0, 0x40, 0xD8, 0xE1,
				0xB0, 0x50, 0xD9, 0xE1, 0xB0, 0x60, 0xD0, 0xE1, 0x90, 0x10, 0x9F, 0xE5,
				0x00, 0x10, 0x88, 0xE5, 0x8C, 0x10, 0x9F, 0xE5, 0xB0, 0x10, 0xC9, 0xE1,
				0x00, 0x10, 0xA0, 0xE3, 0xB4, 0x18, 0xC0, 0xE1, 0x80, 0x10, 0x86, 0xE3,
				0xB0, 0x10, 0xC0, 0xE1, 0x00, 0x00, 0x03, 0xEF, 0x41, 0x2F, 0xA0, 0xE3,
				0xB0, 0x10, 0xD7, 0xE1, 0x02, 0x00, 0x11, 0xE1, 0xFC, 0xFF, 0xFF, 0x1A,
				0xB0, 0x10, 0xD7, 0xE1, 0x02, 0x30, 0x01, 0xE0, 0x02, 0x00, 0x53, 0xE1,
				0xFB, 0xFF, 0xFF, 0x1A, 0xB0, 0x40, 0xC8, 0xE1, 0xB0, 0x50, 0xC9, 0xE1,
				0x01, 0x1A, 0xA0, 0xE3, 0xB2, 0x10, 0xC8, 0xE1, 0xB0, 0x60, 0xC0, 0xE1,
				0xFC, 0x03, 0xBD, 0xE8, 0x84, 0x30, 0x80, 0xE5, 0x80, 0x10, 0x80, 0xE2,
				0xFC, 0x03, 0xA1, 0xE8, 0x60, 0x10, 0x80, 0xE2, 0xFC, 0x03, 0xBD, 0xE8,
				0xFC, 0x03, 0xA1, 0xE8, 0xF0, 0x0F, 0xBD, 0xE8, 0xB6, 0x10, 0xD0, 0xE1,
				0xA0, 0x00, 0x51, 0xE3, 0xFC, 0xFF, 0xFF, 0x1A, 0x0F, 0x50, 0xBD, 0xE8,
				0x04, 0xF0, 0x5E, 0xE2, 0x30, 0x01, 0x00, 0x04, 0x00, 0x02, 0x00, 0x04,
				0x32, 0x01, 0x00, 0x04, 0x00, 0x30, 0xFF, 0xFF, 0x04, 0xC1, 0x00, 0x00,
			};
			GbaSleepHandlerPlacement gbaSleepPlacement;
			u32 gbaSleepHandlerGbaAddr = GBA_SLEEP_BIOS_IRQ_HANDLER;
			if(findGbaSleepHandlerPlacement((const u8*)LGY_ROM_LOC,
			                              romSize,
			                              sizeof(gbaSleepIrqHandler),
			                              saveType,
			                              &gbaSleepPlacement) &&
			   validateGbaSleepHandlerPlacement((const u8*)LGY_ROM_LOC,
			                           sizeof(gbaSleepIrqHandler),
			                           &gbaSleepPlacement))
			{
				memcpy((void*)(LGY_ROM_LOC + gbaSleepPlacement.offset),
				       gbaSleepIrqHandler, sizeof(gbaSleepIrqHandler));

				// Make the injected ARM code visible to the LGY/ARM7 side explicitly.
				flushDCacheRange((void*)(LGY_ROM_LOC + gbaSleepPlacement.offset),
				                 sizeof(gbaSleepIrqHandler));

				gbaSleepHandlerGbaAddr = GBA_SLEEP_HANDLER_ROM_BASE + gbaSleepPlacement.offset;
#ifndef NDEBUG
				if(gbaSleepPlacement.kind == GBA_SLEEP_PLACE_ROM_PADDING)
				{
					ee_printf("GBA Sleep: IRQ handler @ %08lX ROM+%08lX pad=%02X run=%08lX+%08lX\n",
					          (unsigned long)gbaSleepHandlerGbaAddr,
					          (unsigned long)gbaSleepPlacement.offset,
					          (unsigned)gbaSleepPlacement.fillByte,
					          (unsigned long)gbaSleepPlacement.spanStart,
					          (unsigned long)gbaSleepPlacement.spanSize);
				}
				else
				{
					ee_printf("GBA Sleep: IRQ handler @ %08lX ROM+%08lX source=open-bus-tail\n",
					          (unsigned long)gbaSleepHandlerGbaAddr,
					          (unsigned long)gbaSleepPlacement.offset);
				}
#endif
			}
			else
			{
				// Transparent fallback: normal BIOS IRQ path, Sleep/Wake disabled.
				ee_puts("GBA Sleep: no validated IRQ handler location; Sleep/Wake disabled.");
			}
			free(romFilePath);

			// Set audio output and volume.
			CODEC_setAudioOutput(g_oafConfig.audioOut);
			CODEC_setVolumeOverride(g_oafConfig.volume);

			// Select the runtime GBA Sleep IRQ handler before ARM9 builds the BIOS overlay.
			// ARM9 defaults to BIOS 0x128 and also resets to it after each prepare.
			g_gbaSleepAvailable = false;
			res = LGY_setGbaIrqHandlerAddress(gbaSleepHandlerGbaAddr);
			if(res != RES_OK)
			{
				ee_puts("GBA Sleep: IRQ handler target rejected; using BIOS IRQ fallback.");
				gbaSleepHandlerGbaAddr = GBA_SLEEP_BIOS_IRQ_HANDLER;
			}
			else if(gbaSleepHandlerGbaAddr != GBA_SLEEP_BIOS_IRQ_HANDLER)
				g_gbaSleepAvailable = true;

			g_gbaSleepHandlerGbaAddr = gbaSleepHandlerGbaAddr;

			// Prepare ARM9 for GBA mode + save loading.
			res = LGY_prepareGbaMode(g_oafConfig.directBoot, saveType, filePath);
			if(res == RES_OK)
			{
				// Initialize video output (frame capture, post processing ect.).
				g_frameReadyEvent = OAF_videoInit();

				// Setup button overrides.
				const u32 *const maps = g_oafConfig.buttonMaps;
				u16 overrides = 0;
				for(unsigned i = 0; i < 10; i++)
					if(maps[i] != 0) overrides |= 1u<<i;
				LGY11_selectInput(overrides);

				// Sync LgyCap start with LCD VBlank.
				GFX_waitForVBlank0();
				LGY11_switchMode();
			}
		} while(0);
	}
	else res = RES_OUT_OF_MEM;

	free(filePath);

	return res;
}

bool oafIsGbaSleepAvailable(void)
{
	return g_gbaSleepAvailable;
}

Result oafSetGbaForcedSleepVector(const bool forced)
{
	if(!g_gbaSleepAvailable) return RES_INVALID_ARG;

	const u32 target = g_gbaSleepHandlerGbaAddr +
	                   (forced ? GBA_SLEEP_HANDLER_FORCE_ENTRY_OFF : 0u);
	return LGY_setGbaIrqVectorAddress(target);
}

void oafUpdate(void)
{
	const u32 *const maps = g_oafConfig.buttonMaps;
	const u32 kHeld = hidKeysHeld();
	u16 pressed = 0;
	for(unsigned i = 0; i < 10; i++)
	{
		if((kHeld & maps[i]) != 0)
			pressed |= 1u<<i;
	}
	LGY11_setInputState(pressed);

	CODEC_runHeadphoneDetection();
	updateBacklight();
	waitForEvent(g_frameReadyEvent);
	clearEvent(g_frameReadyEvent);
}

void oafFinish(void)
{
	g_gbaSleepAvailable = false;
	g_gbaSleepHandlerGbaAddr = GBA_SLEEP_BIOS_IRQ_HANDLER;
	// frameReadyEvent deleted by this function.
	OAF_videoExit();
	g_frameReadyEvent = 0;
	LGY11_deinit();
}