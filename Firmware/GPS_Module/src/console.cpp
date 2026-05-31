#include "console.h"

#ifndef FLIGHT_BUILD

#include "node.h"

#include <aim_network.h>
#include <logger.h>

enum ConsoleMenu : uint8_t {
  CONSOLE_MENU_ROOT      = 0U,
  CONSOLE_MENU_LOG_MASK  = 1U,
  CONSOLE_MENU_FLASH     = 2U,
  CONSOLE_MENU_GPS       = 3U,
  CONSOLE_MENU_FLASH_ERASE_CONFIRM = 4U
  // add more menus as needed
};

static Stream* s_serial = nullptr;
static AimNetwork* s_aim = nullptr;
static Logger* s_log = nullptr;
static ConsoleMenu s_menu = CONSOLE_MENU_ROOT;

static int readChar(void) {
  AIM_ASSERT(s_serial != nullptr);

  if (s_serial->available() <= 0) {
    return -1;
  }

  const int rxByte = s_serial->read();
  if (rxByte < 0) {
    return -1;
  }

  char c = static_cast<char>(rxByte);
  if ((c >= 'A') && (c <= 'Z')) {
    c = static_cast<char>(c + ('a' - 'A'));
  }

  if ((c == '\n') || (c == '\r') || (c == ' ') || (c == '\t')) {
    return -1;
  }

  return static_cast<int>(c);
}

static void showMenu(ConsoleMenu menu) {
  AIM_ASSERT(s_serial != nullptr);

  s_menu = menu;
  switch (menu) {
    case CONSOLE_MENU_ROOT:
      s_serial->println("DEBUG: q exit | b back");
      s_serial->println("1 status | 2 log | 3 flash | 4 gps");
      break;
    case CONSOLE_MENU_LOG_MASK:
      s_serial->println("LOG mask: q exit | b back");
      s_serial->println("1 DEBUG only | 2 INFO only | 3 WARN only | 4 ERROR only");
      s_serial->println("5 all | 6 INFO/WARN/ERROR");
      break;
    case CONSOLE_MENU_FLASH:
      s_serial->println("FLASH: q exit | b back");
      s_serial->println("1 info | 2 dump | 3 erase");
      break;
    case CONSOLE_MENU_GPS:
      s_serial->println("GPS: q exit | b back");
      s_serial->println("1 snapshot | 2 parser stats");
      break;
    case CONSOLE_MENU_FLASH_ERASE_CONFIRM:
      s_serial->println("FLASH ERASE: q exit | b back");
      s_serial->println("1 confirm");
      break;
    default:
      s_menu = CONSOLE_MENU_ROOT;
      s_serial->println("DEBUG: q exit | b back");
      s_serial->println("1 status | 2 log | 3 flash");
      break;
  }
}

static void printGpsSnapshot(void) {
  AIM_ASSERT(s_serial != nullptr);

  GpsDebugSnapshot gps = {};
  if (!nodeGetGpsDebugSnapshot(&gps)) {
    s_serial->println("gps snapshot unavailable");
    return;
  }

  s_serial->print("gps timeValid(parser/state)=");
  s_serial->print(static_cast<unsigned>(gps.parserTimeValid ? 1U : 0U));
  s_serial->print("/");
  s_serial->println(static_cast<unsigned>(gps.hasValidTime ? 1U : 0U));

  s_serial->print("gps locValid(parser/state)=");
  s_serial->print(static_cast<unsigned>(gps.parserLocationValid ? 1U : 0U));
  s_serial->print("/");
  s_serial->println(static_cast<unsigned>(gps.hasValidLocation ? 1U : 0U));

  s_serial->print("gps sats(valid/count)=");
  s_serial->print(static_cast<unsigned>(gps.parserSatellitesValid ? 1U : 0U));
  s_serial->print("/");
  s_serial->println(static_cast<unsigned long>(gps.satellites));

  s_serial->print("timeOfDayMs=");
  s_serial->println(static_cast<unsigned long>(gps.timeOfDayMs));

  s_serial->print("readableTime=");
  s_serial->println(gps.readableTimeStr);

  s_serial->print("lonNano=");
  s_serial->println(static_cast<long long>(gps.longitudeNano));
  s_serial->print("latNano=");
  s_serial->println(static_cast<long long>(gps.latitudeNano));
}

static void printGpsParserStats(void) {
  AIM_ASSERT(s_serial != nullptr);

  GpsDebugSnapshot gps = {};
  if (!nodeGetGpsDebugSnapshot(&gps)) {
    s_serial->println("gps parser stats unavailable");
    return;
  }

  s_serial->print("chars=");
  s_serial->println(static_cast<unsigned long>(gps.charsProcessed));
  s_serial->print("sentencesWithFix=");
  s_serial->println(static_cast<unsigned long>(gps.sentencesWithFix));
  s_serial->print("checksum pass/fail=");
  s_serial->print(static_cast<unsigned long>(gps.passedChecksum));
  s_serial->print("/");
  s_serial->println(static_cast<unsigned long>(gps.failedChecksum));
  s_serial->print("satellites=");
  s_serial->println(static_cast<unsigned long>(gps.satellites));
}

static void printStatus(uint8_t currentState, uint32_t networkNowMs) {
  AIM_ASSERT(s_serial != nullptr);
  AIM_ASSERT(s_aim != nullptr);
  AIM_ASSERT(s_log != nullptr);

  s_serial->print("name=");
  s_serial->print(NODE_NAME);
  s_serial->print(" state=");
  s_serial->print(static_cast<unsigned>(currentState));
  s_serial->print(" logMask=0x");
  s_serial->print(static_cast<unsigned>(s_log->filterMask()), HEX);
  s_serial->print(" nowMs=");
  s_serial->print(static_cast<unsigned long>(networkNowMs));
  s_serial->print(" offset=");
  s_serial->println(static_cast<long>(s_aim->getTimeOffset()));
  s_serial->print("version=");
  s_serial->print(AIM_NETWORK_VERSION_STRING);
  s_serial->print(" build=");
  s_serial->print(__DATE__);
  s_serial->print(" ");
  s_serial->println(__TIME__);
}

static void printState(uint8_t state) {
  AIM_ASSERT(s_serial != nullptr);

  s_serial->print("state=");
  s_serial->println(static_cast<unsigned>(state));
}

static void setLogMask(uint8_t mask, const char* label) {
  AIM_ASSERT(s_serial != nullptr);
  AIM_ASSERT(s_log != nullptr);

  s_log->setFilterMask(mask);
  s_serial->print("log mask=0x");
  s_serial->print(static_cast<unsigned>(s_log->filterMask()), HEX);
  s_serial->print(" (");
  s_serial->print(label);
  s_serial->println(")");
}

void consoleInit(Stream& serial,
                 AimNetwork& aim,
                 Logger& log) {
  s_serial = &serial;
  s_aim = &aim;
  s_log = &log;
  s_menu = CONSOLE_MENU_ROOT;
}

ConsoleAction consoleCheckEntry(void) {
  const int c = readChar();
  if (c == 'd') {
    showMenu(CONSOLE_MENU_ROOT);
    return CONSOLE_ACTION_ENTER;
  }
  return CONSOLE_ACTION_NONE;
}

ConsoleAction consoleService(uint8_t currentState, uint32_t networkNowMs) {
  const int c = readChar();
  if (c < 0) {
    return CONSOLE_ACTION_NONE;
  }

  if (c == 'q') {
    s_menu = CONSOLE_MENU_ROOT;
    printState(static_cast<uint8_t>(OPERATIONAL));
    return CONSOLE_ACTION_EXIT;
  }

  switch (s_menu) {
    
    case CONSOLE_MENU_ROOT:
      switch (c) {
        case 'b':
          showMenu(CONSOLE_MENU_ROOT);
          break;
        case '1':
          printStatus(currentState, networkNowMs);
          break;
        case '2':
          showMenu(CONSOLE_MENU_LOG_MASK);
          break;
        case '3':
          showMenu(CONSOLE_MENU_FLASH);
          break;
        case '4':
          showMenu(CONSOLE_MENU_GPS);
          break;
        default:
          showMenu(CONSOLE_MENU_ROOT);
          break;
      }
      break;

    case CONSOLE_MENU_LOG_MASK:
      switch (c) {
        case 'b':
          showMenu(CONSOLE_MENU_ROOT);
          break;
        case '1':
          setLogMask(static_cast<uint8_t>(LogLevel::DEBUG), "DEBUG only");
          break;
        case '2':
          setLogMask(static_cast<uint8_t>(LogLevel::INFO), "INFO only");
          break;
        case '3':
          setLogMask(static_cast<uint8_t>(LogLevel::WARN), "WARN only");
          break;
        case '4':
          setLogMask(static_cast<uint8_t>(LogLevel::ERROR), "ERROR only");
          break;
        case '5':
          setLogMask(static_cast<uint8_t>(LogLevel::DEBUG) |
                     static_cast<uint8_t>(LogLevel::INFO) |
                     static_cast<uint8_t>(LogLevel::WARN) |
                     static_cast<uint8_t>(LogLevel::ERROR),
                     "DEBUG/INFO/WARN/ERROR");
          break;
        case '6':
          setLogMask(static_cast<uint8_t>(LogLevel::INFO) |
                     static_cast<uint8_t>(LogLevel::WARN) |
                     static_cast<uint8_t>(LogLevel::ERROR),
                     "INFO/WARN/ERROR");
          break;
        default:
          showMenu(CONSOLE_MENU_LOG_MASK);
          break;
      }
      break;

    case CONSOLE_MENU_FLASH:
      switch (c) {
        case 'b':
          showMenu(CONSOLE_MENU_ROOT);
          break;
        case '1':
          return CONSOLE_ACTION_FLASH_INFO;
        case '2':
          return CONSOLE_ACTION_FLASH_DUMP;
        case '3':
          showMenu(CONSOLE_MENU_FLASH_ERASE_CONFIRM);
          break;
        default:
          showMenu(CONSOLE_MENU_FLASH);
          break;
      }
      break;

      case CONSOLE_MENU_FLASH_ERASE_CONFIRM: // submenu of MENU_FLASH
      switch (c) {
        case 'b':
          showMenu(CONSOLE_MENU_FLASH);
          break;
        case '1':
          return CONSOLE_ACTION_FLASH_ERASE;
        default:
          showMenu(CONSOLE_MENU_FLASH_ERASE_CONFIRM);
          break;
      }
      break;

      case CONSOLE_MENU_GPS:
      switch (c) {
        case 'b':
          showMenu(CONSOLE_MENU_ROOT);
          break;
        case '1':
          printGpsSnapshot();
          break;
        case '2':
          printGpsParserStats();
          break;
          return CONSOLE_ACTION_FLASH_ERASE;
        default:
          showMenu(CONSOLE_MENU_GPS);
          break;
      }
      break;

    default:
      showMenu(CONSOLE_MENU_ROOT);
      break;
  }

  return CONSOLE_ACTION_NONE;
}

void consoleResume(void) {
  showMenu(s_menu);
}

#endif // FLIGHT_BUILD
