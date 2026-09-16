#include "console.h"

#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "logging_flash.h"
#include "sequencer.h"

namespace tvc::flight::console {
namespace {

char g_line[64];
uint8_t g_len = 0;

void toUpperInPlace(char* s) {
    for (; *s; ++s) *s = static_cast<char>(toupper(static_cast<unsigned char>(*s)));
}

void printHelp() {
    Serial.println(F(
        "\n--- STAND_OPEN bench console ---\n"
        " SELFTEST            (re)run diagnostics; IDLE or FAULT only\n"
        " TARE                zero the load cell and encoders; IDLE only\n"
        " PROFILE <n>         0=HOLD 1=STEP_X 2=STEP_Y 3=RAMP_X 4=CHIRP_X; IDLE only\n"
        " DRY_RUN <ON|OFF>    igniter output must be an LED+resistor when ON; IDLE only\n"
        " ARM                 requires physical arm switch closed + continuity; IDLE only\n"
        " COUNTDOWN           starts the 10 s count; prints a fire_token; ARMED only\n"
        " FIRE_CONFIRM <tok>  required within 2 s of T-0 or the run aborts to SAFE\n"
        " ABORT               abort to SAFE from any state\n"
        " RESET               SAFE -> IDLE\n"
        " DUMP_FLASH          hex-dump the LittleFS mirror over serial\n"
        " STATUS / ?          one-line status\n"
        " HELP / H            this text\n"));
}

void handleLine(char* line) {
    while (*line == ' ') line++;
    if (*line == '\0') return;

    char* cmd = line;
    char* arg = nullptr;
    for (char* p = line; *p; ++p) {
        if (*p == ' ') {
            *p = '\0';
            arg = p + 1;
            while (*arg == ' ') arg++;
            break;
        }
    }
    toUpperInPlace(cmd);

    if (strcmp(cmd, "SELFTEST") == 0) {
        sequencer::cmdSelftest();
    } else if (strcmp(cmd, "ARM") == 0) {
        sequencer::cmdArm();
    } else if (strcmp(cmd, "COUNTDOWN") == 0) {
        sequencer::cmdCountdown();
    } else if (strcmp(cmd, "FIRE_CONFIRM") == 0) {
        if (!arg || !*arg) {
            Serial.println(F("usage: FIRE_CONFIRM <token>"));
        } else {
            sequencer::cmdFireConfirm(static_cast<uint16_t>(atol(arg)));
        }
    } else if (strcmp(cmd, "ABORT") == 0 || strcmp(cmd, "X") == 0) {
        sequencer::cmdAbort("operator ABORT");
    } else if (strcmp(cmd, "PROFILE") == 0) {
        if (!arg || !*arg) {
            Serial.println(F("usage: PROFILE <0=HOLD 1=STEP_X 2=STEP_Y 3=RAMP_X 4=CHIRP_X>"));
        } else {
            sequencer::cmdSetProfile(static_cast<uint8_t>(atoi(arg)));
        }
    } else if (strcmp(cmd, "DRY_RUN") == 0) {
        if (!arg || !*arg) {
            Serial.println(F("usage: DRY_RUN <ON|OFF>"));
        } else {
            toUpperInPlace(arg);
            if (strcmp(arg, "ON") == 0) sequencer::cmdSetDryRun(true);
            else if (strcmp(arg, "OFF") == 0) sequencer::cmdSetDryRun(false);
            else Serial.println(F("usage: DRY_RUN <ON|OFF>"));
        }
    } else if (strcmp(cmd, "TARE") == 0) {
        sequencer::cmdTare();
    } else if (strcmp(cmd, "DUMP_FLASH") == 0) {
        logging_flash::dumpHexOverSerial();
    } else if (strcmp(cmd, "RESET") == 0) {
        sequencer::cmdResetToIdle();
    } else if (strcmp(cmd, "STATUS") == 0 || strcmp(cmd, "?") == 0) {
        sequencer::printStatus();
    } else if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "H") == 0) {
        printHelp();
    } else {
        Serial.printf("unknown command: %s (HELP for the list)\n", cmd);
    }
}

}  // namespace

void begin() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {}
    Serial.println(F("TVC STAND_OPEN firmware ready."));
    printHelp();
}

void poll() {
    while (Serial.available()) {
        char c = static_cast<char>(Serial.read());
        if (c == '\n' || c == '\r') {
            if (g_len) {
                g_line[g_len] = '\0';
                handleLine(g_line);
                g_len = 0;
            }
        } else if (g_len < sizeof(g_line) - 1) {
            g_line[g_len++] = c;
        }
    }
}

}  // namespace tvc::flight::console
