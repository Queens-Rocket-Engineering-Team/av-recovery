#include "node.h"
#include "console.h"

#include <IWatchdog.h>
#include <logger.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SerialFlash.h>
#include <flash_table.h>

static constexpr uint32_t kWatchdogTimeoutUs = 2000000U;
static constexpr uint8_t kMaxRxFramesPerLoop = 8U;

struct NodeSchedulerState {
  NodeState value = INIT;
  uint32_t lastHeartbeatTxMs = 0U;
};

static AimCanDriver g_canHw(NODE_ORIGIN, NODE_CAN_BAUD, NODE_CAN_BUS);
static AimNetwork g_aim(&g_canHw, NODE_ORIGIN);
static SoftwareSerial g_serial(NODE_SERIAL_RX_PIN, NODE_SERIAL_TX_PIN);
static Logger g_log(g_serial, NODE_ORIGIN, LogLevel::INFO);
static uint32_t g_flashLastVals[NODE_FLASH_TABLE_COLS];
static uint8_t g_flashIoBuffer[NODE_MCU_BUFFER_SIZE];
static FlashTable g_flashTable(
  &SerialFlash,
  NODE_FLASH_TABLE_COLS,
  NODE_FLASH_ORIGIN_REFRESH_INT,
  NODE_FLASH_TABLE_SIZE,
  NODE_FLASH_TABLE_NUM,
  NODE_MCU_BUFFER_SIZE,
  g_flashLastVals,
  g_flashIoBuffer);
static NodeSchedulerState g_schedulerState = {};

void serviceCanRx(uint32_t networkNowMs) {
  // Handle incoming bus messages and custom packet branches here.
  (void)networkNowMs;
  for (uint8_t i = 0U; i < kMaxRxFramesPerLoop; i++) {
    aimPkt pkt = {};
    if (!g_aim.readPkt(pkt)) {
      break;
    }
  }
}

void serviceCanTx(uint32_t schedulerNowMs, uint32_t networkNowMs) {
  // Add periodic transmit-side behavior in this service pattern.

  // TX SECTION 1: node heartbeat.
  if ((schedulerNowMs - g_schedulerState.lastHeartbeatTxMs) >= AIM_HEARTBEAT_TX_INTERVAL_DEFAULT_MS) {
    g_schedulerState.lastHeartbeatTxMs = schedulerNowMs;
    const uint32_t payload = static_cast<uint32_t>(g_schedulerState.value);
    const bool heartbeatSent = g_aim.sendTimedPkt(networkNowMs, payload, AIM_DEST_BROADCAST, AIM_TYP_HEARTBEAT);
    if (!heartbeatSent) {
      LOG_ERROR("Heartbeat TX failed");
    } else {
      LOG_DEBUG("Heartbeat TX ok");
    }
  }

  nodeServiceCanTx(schedulerNowMs, networkNowMs, g_aim);
}

void runStateMachine(uint32_t schedulerNowMs, uint32_t networkNowMs) {
  AIM_ASSERT(g_schedulerState.value <= FAULT);  // precondition: corrupted state -> reset

  switch (g_schedulerState.value) {
    case OPERATIONAL:
#ifndef FLIGHT_BUILD
      if (consoleCheckEntry() == CONSOLE_ACTION_ENTER) {
        g_schedulerState.value = DEBUG_CONSOLE;
      }
#endif
      break;

#ifndef FLIGHT_BUILD
    case DEBUG_CONSOLE: {
      const ConsoleAction act = consoleService(
          static_cast<uint8_t>(g_schedulerState.value), networkNowMs);
      if (act == CONSOLE_ACTION_EXIT) {
        g_schedulerState.value = OPERATIONAL;
      } else if (act == CONSOLE_ACTION_FLASH_INFO) {
        g_flashTable.commandInfo(&g_serial);
      } else if (act == CONSOLE_ACTION_FLASH_DUMP) {
        if (g_flashTable.commandDump(&g_serial, 512U, nullptr, nullptr)) {
          g_schedulerState.value = FLASH_DUMP;
          g_serial.print("state=");
          g_serial.println(static_cast<unsigned>(FLASH_DUMP));
        }
      } else if (act == CONSOLE_ACTION_FLASH_ERASE) {
        g_flashTable.commandErase(&g_serial);
        g_schedulerState.value = FLASH_ERASE;
      }
      break;
    }

    case FLASH_DUMP: {
      if (g_serial.available() > 0) {
        const int c = g_serial.read();
        if (c == 'q' || c == 'Q') {
          g_flashTable.cancelDump();
          g_serial.println("flash dump canceled");
          g_schedulerState.value = DEBUG_CONSOLE;
          g_serial.print("state=");
          g_serial.println(static_cast<unsigned>(DEBUG_CONSOLE));
          consoleResume();
          break;
        }
      }

      const FlashTableServiceResult r = g_flashTable.serviceDump(&g_serial, 16U);
      if (r != FLASHTABLE_SERVICE_ACTIVE) {
        static const char* const kDumpMsg[] = {
          "flash dump idle",
          nullptr,
          "flash dump done",
          "flash dump aborted",
          "flash dump error"
        };
        const uint8_t idx = static_cast<uint8_t>(r);
        if (idx < 5U && kDumpMsg[idx] != nullptr) {
          g_serial.println(kDumpMsg[idx]);
        }
        g_schedulerState.value = DEBUG_CONSOLE;
        g_serial.print("state=");
        g_serial.println(static_cast<unsigned>(DEBUG_CONSOLE));
        consoleResume();
      }
      break;
    }

    case FLASH_ERASE: {
      const FlashTableServiceResult r = g_flashTable.serviceErase();
      if (r != FLASHTABLE_SERVICE_ACTIVE) {
        g_serial.println(r == FLASHTABLE_SERVICE_DONE ? "flash erase done" : "flash erase error");
        g_schedulerState.value = DEBUG_CONSOLE;
        consoleResume();
      }
      break;
    }
#endif

    case SAFE_MODE:
    case LOW_POWER:
    case FAULT:
      break;

    default:
      AIM_ASSERT(false);
      break;
  }

  nodeUpdate(schedulerNowMs);
  serviceCanTx(schedulerNowMs, networkNowMs);
}

void setup(void) {
  AIM_ASSERT(NODE_ORIGIN <= AIM_ORG_ADDR_MAX);
  g_serial.begin(NODE_SERIAL_BAUD);
  g_logger = &g_log;
  LOG_INFO("Boot node origin=%u", static_cast<unsigned>(NODE_ORIGIN));
  IWatchdog.begin(kWatchdogTimeoutUs);
  LOG_INFO("Watchdog ready");

  g_aim.begin();

#ifndef FLIGHT_BUILD
  consoleInit(g_serial, g_aim, g_log);
#endif

  // NODE EXTENSION POINT: add one-time node setup here.
  SPI.setSCLK(NODE_FLASH_SCK_PIN);
  SPI.setMISO(NODE_FLASH_MISO_PIN);
  SPI.setMOSI(NODE_FLASH_MOSI_PIN);
  SPI.begin();
  if (SerialFlash.begin(NODE_FLASH_CS_PIN)) {
    g_flashTable.init(&g_serial);
  }
  if (g_flashTable.isReady()) {
    LOG_INFO("Flash ready");
  } else {
    LOG_WARN("Flash init failed");
  }

  nodeInit(millis());
#ifndef FLIGHT_BUILD
  g_serial.println("Console ready. d=enter debug");
#endif
  g_schedulerState.lastHeartbeatTxMs = millis();
  g_schedulerState.value = OPERATIONAL;
}

void loop(void) {
  const uint32_t schedulerNowMs = millis();
  const uint32_t networkNowMs = nodeGetNetworkNowMs(schedulerNowMs);

  // Main scheduler order: RX, state machine, watchdog.
  serviceCanRx(networkNowMs);
  runStateMachine(schedulerNowMs, networkNowMs);

  IWatchdog.reload();
}