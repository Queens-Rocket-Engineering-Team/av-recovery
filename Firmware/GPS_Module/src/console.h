#ifndef CONSOLE_H
#define CONSOLE_H

#ifndef FLIGHT_BUILD

#include <Arduino.h>
#include <cstdint>

class AimNetwork;
class Logger;
class FlashTable;

// Actions returned to main.cpp so state mutations stay in the caller.
enum ConsoleAction : uint8_t {
  CONSOLE_ACTION_NONE       = 0U,
  CONSOLE_ACTION_ENTER      = 1U,
  CONSOLE_ACTION_EXIT       = 2U,
  CONSOLE_ACTION_FLASH_DUMP = 3U,
  CONSOLE_ACTION_DUMP_DONE  = 4U
};

// Call once in setup() after Serial and all peripherals are ready.
void consoleInit(Stream& serial,
                 AimNetwork& aim,
                 Logger& log,
                 FlashTable& flash);

// Poll from OPERATIONAL state. Returns CONSOLE_ACTION_ENTER on 'd'.
ConsoleAction consoleCheckEntry(void);

// Service the debug console menus. Called from DEBUG_CONSOLE state.
ConsoleAction consoleService(uint8_t currentState, uint32_t networkNowMs);

// Service a running flash dump. Called from FLASH_DUMP state.
ConsoleAction consoleServiceFlashDump(void);

#endif // FLIGHT_BUILD

#endif  // CONSOLE_H
