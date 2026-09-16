#include "logging_flash.h"

#include <Arduino.h>
#include <LittleFS.h>

#include "params.h"

namespace tvc::flight::logging_flash {
namespace {

constexpr uint32_t kRegionBytes = 1024UL * 1024;
constexpr uint32_t kDecimation = params::control::loop_hz / params::logging::flash_mirror_rate_hz;
constexpr const char* kFilename = "flight.bin";

LittleFS_Program g_fs;
File g_file;
uint32_t g_tick = 0;
uint32_t g_bytes_written = 0;
bool g_open = false;

}  // namespace

bool begin() {
    g_open = false;
    g_tick = 0;
    g_bytes_written = 0;

    if (!g_fs.begin(kRegionBytes)) return false;
    if (g_fs.exists(kFilename)) g_fs.remove(kFilename);
    g_file = g_fs.open(kFilename, FILE_WRITE);
    if (!g_file) return false;

    g_open = true;
    return true;
}

void push(const Record& r) {
    if (!g_open) return;
    g_tick++;
    if (g_tick % kDecimation != 0) return;
    g_file.write(reinterpret_cast<const uint8_t*>(&r), sizeof(Record));
    g_bytes_written += sizeof(Record);
}

void close() {
    if (!g_open) return;
    g_file.close();
    g_open = false;
}

uint32_t bytesWritten() { return g_bytes_written; }

void dumpHexOverSerial() {
    File f = g_fs.open(kFilename, FILE_READ);
    if (!f) {
        Serial.println(F("!! flash mirror: no file to dump"));
        return;
    }
    Serial.printf("--- FLASH DUMP BEGIN %s (%lu bytes) ---\n", kFilename,
                  static_cast<unsigned long>(f.size()));
    uint8_t buf[32];
    int n;
    while ((n = f.read(buf, sizeof(buf))) > 0) {
        for (int i = 0; i < n; i++) Serial.printf("%02X", buf[i]);
        Serial.println();
    }
    f.close();
    Serial.println(F("--- FLASH DUMP END ---"));
}

}  // namespace tvc::flight::logging_flash
