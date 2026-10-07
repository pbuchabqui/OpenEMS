// Simulated ECU for the dashboard (tools/openems_dash) without hardware.
//
// Runs the firmware's own protocol, calibration and NVM code on the host and
// serves it on a pseudo-terminal, so `python server.py --port <pty>` talks to
// the real command parser, page apply/serialize and burn path.
//
//   sim_ecu [--nvm FILE]          serve; burned pages persist in FILE
//   sim_ecu --dump-pages FILE     write the factory pages (compile-time
//                                 defaults) as JSON and exit
//
// The engine is stopped (rpm 0): calibration, burn and output test work; the
// virtual engine (test/sim/engine_sim) is not driven in real time here.
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include "app/nvm_boot.h"
#include "app/ui_protocol.h"
#include "app/ui_protocol_internal.h"
#include "engine/calibration.h"
#include "engine/output_test.h"
#include "engine/table3d.h"
#include "hal/flash.h"
#include "hal/system.h"

// Diagnostic counters owned by main_stm32.cpp on the target ('D' dump).
uint32_t g_dbg_rev_limit_trips = 0u;
uint32_t g_dbg_rev_limit_rpm_x10 = 0u;
uint32_t g_dbg_rev_limit_rpm_max = 0u;

namespace {

constexpr uint16_t kPage0Bytes = 512u;

void host_boot() {
    // Same order as main_stm32.cpp: page 0 first (it carries the layout
    // version that gates the table pages), then the tables.
    uint8_t page0[kPage0Bytes] = {};
    ems::hal::nvm_load_calibration(0u, page0, kPage0Bytes);
    ems::app::ui_boot_apply_page0(page0, kPage0Bytes);
    ems::app::nvm_boot_load_tables(
        page0[ems::engine::kCalLayoutVersionOffset] == ems::engine::kCalLayoutVersion);
    ems::app::ui_init();
}

bool load_nvm(const std::string& path) {
    uint32_t len = 0u;
    uint8_t* img = ems::hal::nvm_host_calibration_image(&len);
    FILE* f = std::fopen(path.c_str(), "rb");
    if (f == nullptr) { return false; }
    const size_t n = std::fread(img, 1, len, f);
    std::fclose(f);
    return n == len;
}

void save_nvm(const std::string& path) {
    uint32_t len = 0u;
    const uint8_t* img = ems::hal::nvm_host_calibration_image(&len);
    const std::string tmp = path + ".tmp";
    FILE* f = std::fopen(tmp.c_str(), "wb");
    if (f == nullptr) { return; }
    std::fwrite(img, 1, len, f);
    std::fclose(f);
    std::rename(tmp.c_str(), path.c_str());
}

std::string base64(const uint8_t* p, size_t n) {
    static const char* k = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    for (size_t i = 0; i < n; i += 3) {
        uint32_t v = static_cast<uint32_t>(p[i]) << 16;
        if (i + 1 < n) { v |= static_cast<uint32_t>(p[i + 1]) << 8; }
        if (i + 2 < n) { v |= p[i + 2]; }
        out += k[(v >> 18) & 63];
        out += k[(v >> 12) & 63];
        out += (i + 1 < n) ? k[(v >> 6) & 63] : '=';
        out += (i + 2 < n) ? k[v & 63] : '=';
    }
    return out;
}

// Pages a tune holds: everything editable (3 = realtime, 10/12 = learned).
constexpr uint8_t kTunePages[] = {0x00, 0x01, 0x02, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0B};

int dump_pages(const std::string& path) {
    host_boot();  // empty flash -> compile-time defaults
    FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) { std::perror(path.c_str()); return 1; }
    std::fprintf(f, "{\n  \"format\": \"openems-tune\",\n  \"layout_version\": %u,\n  \"pages\": {",
                 static_cast<unsigned>(ems::engine::kCalLayoutVersion));
    bool first = true;
    for (uint8_t pg : kTunePages) {
        ems::app::ui_detail::sync_page_from_table(pg);
        const uint8_t* p = ems::app::ui_detail::page_ptr(pg);
        const uint16_t n = ems::app::ui_detail::page_size(pg);
        std::vector<uint8_t> buf(p, p + n);
        if (pg == 0x00) {  // a tune always carries the layout version it was made for
            buf[ems::engine::kCalLayoutVersionOffset] = ems::engine::kCalLayoutVersion;
        }
        std::fprintf(f, "%s\n    \"%u\": \"%s\"", first ? "" : ",", pg, base64(buf.data(), n).c_str());
        first = false;
    }
    std::fprintf(f, "\n  }\n}\n");
    std::fclose(f);
    std::printf("wrote %s\n", path.c_str());
    return 0;
}

uint32_t now_ms() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint32_t>(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

}  // namespace

int main(int argc, char** argv) {
    std::string nvm_path;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--dump-pages" && i + 1 < argc) { return dump_pages(argv[++i]); }
        if (a == "--nvm" && i + 1 < argc) { nvm_path = argv[++i]; continue; }
        std::fprintf(stderr, "usage: %s [--nvm FILE] | --dump-pages FILE\n", argv[0]);
        return 2;
    }
    if (!nvm_path.empty() && load_nvm(nvm_path)) {
        std::printf("loaded flash image %s\n", nvm_path.c_str());
    }
    host_boot();

    const int master = posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0 || grantpt(master) != 0 || unlockpt(master) != 0) {
        std::perror("pty");
        return 1;
    }
    const char* slave_name = ptsname(master);
    // Keep the slave open (raw) so the pty never hangs up between clients.
    const int slave = open(slave_name, O_RDWR | O_NOCTTY);
    termios tio{};
    tcgetattr(slave, &tio);
    cfmakeraw(&tio);
    tcsetattr(slave, TCSANOW, &tio);
    fcntl(master, F_SETFL, fcntl(master, F_GETFL) | O_NONBLOCK);
    std::printf("OpenEMS sim ECU on %s\n", slave_name);
    std::fflush(stdout);

    uint32_t last_erase = ems::hal::nvm_test_erase_count();
    const uint32_t t0 = now_ms();
    for (;;) {
        const uint32_t t = now_ms() - t0;
        host_set_millis(t);
        uint8_t in[256];
        const ssize_t n = read(master, in, sizeof(in));
        for (ssize_t i = 0; i < n; ++i) { ems::app::ui_rx_byte(in[i]); }
        ems::app::ui_process();
        ems::engine::output_test_poll(t, 0u);
        // Persist a burn before its reply goes out: a client that kills the
        // sim right after the ACK must still find the data on restart.
        if (!nvm_path.empty() && ems::hal::nvm_test_erase_count() != last_erase) {
            last_erase = ems::hal::nvm_test_erase_count();
            save_nvm(nvm_path);
        }
        uint8_t out[512];
        size_t k = 0;
        uint8_t b = 0u;
        while (k < sizeof(out) && ems::app::ui_tx_pop(b)) { out[k++] = b; }
        size_t done = 0;
        while (done < k) {
            const ssize_t w = write(master, out + done, k - done);
            if (w > 0) { done += static_cast<size_t>(w); }
            else if (errno != EAGAIN) { break; }
        }
        if (n <= 0 && k == 0) { usleep(500); }
    }
    static_cast<void>(slave);
}
