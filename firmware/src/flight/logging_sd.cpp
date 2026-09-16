#include "logging_sd.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <SdFat.h>
#include <string.h>

#include "git_hash.h"
#include "params.h"

namespace tvc::flight::logging_sd {
namespace {

constexpr uint32_t kRingCapacity = params::logging::ring_seconds * params::logging::sd_rate_hz;
// 256 records = 18944 bytes = 37 x 512: lands each flush on a 512-byte boundary (SPEC 4.6).
constexpr uint32_t kFlushChunk = 256;
constexpr uint64_t kPreallocBytes = 64ULL * 1024 * 1024;
constexpr int kEepromBootCountAddr = 0;

DMAMEM Record g_ring[kRingCapacity];
uint32_t g_head = 0;      // next write index, monotonic (mod kRingCapacity to index)
uint32_t g_flushed = 0;   // next not-yet-flushed index, monotonic

SdFs g_sd;
FsFile g_file;
char g_filename[32] = {0};
uint32_t g_bytes_written = 0;
bool g_open = false;

#pragma pack(push, 1)
struct LogHeader {
    uint32_t magic;  // 'TVCL'
    uint32_t schema;
    char params_hash[65];
    char git_hash[20];
    uint8_t mode;
    uint8_t profile;
    uint8_t reserved0[2];
    uint32_t boot_count;
    uint32_t record_size;
};
#pragma pack(pop)

uint32_t nextBootCount() {
    uint32_t n = 0;
    EEPROM.get(kEepromBootCountAddr, n);
    if (n == 0xFFFFFFFFu) n = 0;  // erased flash reads as all-ones
    n++;
    EEPROM.put(kEepromBootCountAddr, n);
    return n;
}

void writeRange(uint32_t start_idx, uint32_t count) {
    if (count == 0) return;
    uint32_t start = start_idx % kRingCapacity;
    uint32_t first = (count <= kRingCapacity - start) ? count : (kRingCapacity - start);
    g_file.write(reinterpret_cast<const uint8_t*>(&g_ring[start]), first * sizeof(Record));
    if (first < count) {
        g_file.write(reinterpret_cast<const uint8_t*>(&g_ring[0]),
                     (count - first) * sizeof(Record));
    }
    g_bytes_written += count * sizeof(Record);
}

}  // namespace

bool begin(Mode mode, Profile profile) {
    g_open = false;
    g_head = 0;
    g_flushed = 0;
    g_bytes_written = 0;

    if (!g_sd.begin(SdioConfig(FIFO_SDIO))) return false;

    for (uint16_t i = 0; i < 999; i++) {
        snprintf(g_filename, sizeof(g_filename), "SF_%03u.BIN", i);
        if (!g_sd.exists(g_filename)) break;
    }
    if (!g_file.open(g_filename, O_RDWR | O_CREAT | O_TRUNC)) return false;
    if (!g_file.preAllocate(kPreallocBytes)) return false;

    LogHeader h{};
    h.magic = 0x4C435654;  // "TVCL"
    h.schema = 1;
    // memcpy, not strncpy: kParamsHash is exactly 64 hex chars with no NUL in that span, so
    // strncpy's "no NUL if source >= n" case would trip -Wstringop-truncation; h{} already
    // zero-initialized the trailing byte, so this copy leaves it correctly NUL-terminated.
    memcpy(h.params_hash, params::kParamsHash, sizeof(h.params_hash) - 1);
    strncpy(h.git_hash, kFirmwareGitHash, sizeof(h.git_hash) - 1);
    h.mode = static_cast<uint8_t>(mode);
    h.profile = static_cast<uint8_t>(profile);
    h.boot_count = nextBootCount();
    h.record_size = sizeof(Record);
    g_file.write(reinterpret_cast<const uint8_t*>(&h), sizeof(h));
    g_bytes_written = sizeof(h);

    g_open = true;
    return true;
}

void push(const Record& r) {
    if (!g_open) return;
    g_ring[g_head % kRingCapacity] = r;
    g_head++;
    if (g_head - g_flushed >= kFlushChunk) {
        writeRange(g_flushed, kFlushChunk);
        g_flushed += kFlushChunk;
    }
}

void flushAll() {
    if (!g_open) return;
    uint32_t n = g_head - g_flushed;
    writeRange(g_flushed, n);
    g_flushed += n;
}

void close() {
    if (!g_open) return;
    flushAll();
    g_file.truncate();
    g_file.close();
    g_open = false;
}

uint32_t bytesWritten() { return g_bytes_written; }
const char* filename() { return g_filename; }

}  // namespace tvc::flight::logging_sd
