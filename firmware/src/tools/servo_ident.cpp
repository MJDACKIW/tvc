/*
 * servo_ident.cpp -- bench tool for env:teensy_test_servo (SPEC.md Sections 1, 9 Phase 3).
 *
 * Jogs one servo channel at a time in small microsecond steps (default 8 us, the legacy
 * config.h ballpark for "about 1 degree" on this linkage -- a step-sizing convenience only,
 * not a calibration), reads back the AS5600 encoder angle on that axis, and lets the operator
 * mark (us, angle) pairs while sweeping through the servo's range. FIT does a least-squares
 * line through the marked points and reports us_per_deg / neutral_us / a suggested sign_x or
 * sign_y for params.yaml.
 *
 * Reuses firmware/src/flight/encoders.cpp and servo_drive.cpp as-is (see platformio.ini's
 * build_src_filter for this env). Deliberately does not touch params::servo::* itself --
 * those are exactly the values this tool exists to derive, so it drives the servo through
 * servo_drive::writeUs() (raw microseconds), never writeDeg() (which needs them already set).
 * No pyro/SD/IMU/loadcell/core involved: this tool only touches two pins and two I2C buses.
 */
#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "encoders.h"
#include "servo_drive.h"

namespace encoders = tvc::flight::encoders;
namespace servo_drive = tvc::flight::servo_drive;

namespace {

constexpr int kUsNeutralGuess = 1500;
constexpr int kDefaultStepUs = 8;
constexpr int kMaxPoints = 64;

struct Point {
    int us;
    float angle_deg;
};

char g_axis = 'X';  // 'X' or 'Y'
int g_us_x = kUsNeutralGuess, g_us_y = kUsNeutralGuess;
int g_step_us = kDefaultStepUs;

Point g_points_x[kMaxPoints];
int g_points_x_n = 0;
Point g_points_y[kMaxPoints];
int g_points_y_n = 0;

int& selectedUs() { return (g_axis == 'X') ? g_us_x : g_us_y; }
Point* selectedPoints() { return (g_axis == 'X') ? g_points_x : g_points_y; }
int& selectedPointCount() { return (g_axis == 'X') ? g_points_x_n : g_points_y_n; }
float selectedAngleDeg() { return (g_axis == 'X') ? encoders::pitchDeg() : encoders::yawDeg(); }

void applyServos() { servo_drive::writeUs(g_us_x, g_us_y); }

void printHelp() {
    Serial.println(F(
        "\n--- servo_ident ---\n"
        " AXIS <X|Y>   select the channel to jog (the other stays where it is)\n"
        " STEP <us>    set the jog step size in microseconds (default 8)\n"
        " +            jog the selected axis one step toward max us\n"
        " -            jog the selected axis one step toward min us\n"
        " SET <us>     command an absolute microsecond value on the selected axis\n"
        " CENTER       command 1500 us on both axes\n"
        " ZERO         re-capture the encoder neutral (do this with the linkage centered)\n"
        " READ         print the selected axis's commanded us and measured encoder angle\n"
        " MARK         READ, then record the (us, angle) pair for the fit\n"
        " LIST         print all recorded points for the selected axis\n"
        " UNDO         remove the last recorded point\n"
        " CLEAR        clear all recorded points for the selected axis\n"
        " FIT          least-squares fit of the recorded points -> us_per_deg, neutral_us\n"
        " STATUS / ?   one-line status\n"
        " HELP / H     this text\n"));
}

void printStatus() {
    Serial.printf("[axis %c] us=%d step=%d points=%d  (other axis us=%d)\n", g_axis,
                  selectedUs(), g_step_us, selectedPointCount(),
                  (g_axis == 'X') ? g_us_y : g_us_x);
}

void doRead() {
    encoders::read();
    Serial.printf("axis %c: us=%d  angle=%.2f deg\n", g_axis, selectedUs(), selectedAngleDeg());
}

void doMark() {
    int& n = selectedPointCount();
    if (n >= kMaxPoints) {
        Serial.println(F("point buffer full -- FIT or CLEAR first"));
        return;
    }
    encoders::read();
    Point p{selectedUs(), selectedAngleDeg()};
    selectedPoints()[n++] = p;
    Serial.printf("marked #%d: us=%d angle=%.2f deg\n", n, p.us, p.angle_deg);
}

void doList() {
    Point* pts = selectedPoints();
    int n = selectedPointCount();
    Serial.printf("axis %c: %d point(s)\n", g_axis, n);
    for (int i = 0; i < n; i++) {
        Serial.printf("  #%2d  us=%4d  angle=%+7.2f deg\n", i + 1, pts[i].us, pts[i].angle_deg);
    }
}

// Least-squares line us = a + b*angle_deg. Reports us_per_deg = |b|, neutral_us = a, and a
// suggested sign (+1/-1) from the sign of b, matching servo_drive.cpp's degToUs() convention
// (us = neutral_us + sign*deg*us_per_deg).
void doFit() {
    Point* pts = selectedPoints();
    int n = selectedPointCount();
    if (n < 2) {
        Serial.println(F("need at least 2 points -- jog + MARK across the range first"));
        return;
    }

    double sum_x = 0, sum_y = 0;
    for (int i = 0; i < n; i++) { sum_x += pts[i].angle_deg; sum_y += pts[i].us; }
    double mean_x = sum_x / n, mean_y = sum_y / n;

    double sxy = 0, sxx = 0, syy = 0;
    for (int i = 0; i < n; i++) {
        double dx = pts[i].angle_deg - mean_x;
        double dy = pts[i].us - mean_y;
        sxy += dx * dy;
        sxx += dx * dx;
        syy += dy * dy;
    }
    if (sxx < 1e-6) {
        Serial.println(F("all recorded angles are the same -- jog the servo between marks"));
        return;
    }
    double b = sxy / sxx;
    double a = mean_y - b * mean_x;

    double ss_res = 0;
    for (int i = 0; i < n; i++) {
        double resid = pts[i].us - (a + b * pts[i].angle_deg);
        ss_res += resid * resid;
    }
    double r2 = (syy > 1e-9) ? (1.0 - ss_res / syy) : 1.0;

    Serial.printf("axis %c fit (n=%d, R^2=%.4f):\n", g_axis, n, r2);
    Serial.printf("  us_per_deg = %.3f\n", fabs(b));
    Serial.printf("  neutral_us = %.1f\n", a);
    Serial.printf("  suggested sign_%c = %d\n", (g_axis == 'X') ? 'x' : 'y', (b >= 0) ? 1 : -1);
    Serial.println(F("  -> paste into params.yaml under servo:, then run tools/gen_params.py"));
    if (r2 < 0.98) {
        Serial.println(F("  R^2 is low for a linear linkage -- check for binding, backlash, or"));
        Serial.println(F("  a marked point taken before the servo/gimbal finished settling."));
    }
}

void handleLine(char* line) {
    while (*line == ' ') line++;
    if (*line == '\0') return;

    char* cmd = line;
    char* arg = nullptr;
    for (char* p = line; *p; ++p) {
        if (*p == ' ') { *p = '\0'; arg = p + 1; while (*arg == ' ') arg++; break; }
    }
    for (char* p = cmd; *p; ++p) *p = static_cast<char>(toupper(static_cast<unsigned char>(*p)));

    if (strcmp(cmd, "AXIS") == 0) {
        if (arg && (*arg == 'X' || *arg == 'x')) g_axis = 'X';
        else if (arg && (*arg == 'Y' || *arg == 'y')) g_axis = 'Y';
        else { Serial.println(F("usage: AXIS <X|Y>")); return; }
        printStatus();
    } else if (strcmp(cmd, "STEP") == 0) {
        if (!arg || !*arg) { Serial.println(F("usage: STEP <us>")); return; }
        g_step_us = atoi(arg);
        printStatus();
    } else if (strcmp(cmd, "+") == 0) {
        selectedUs() = min(selectedUs() + g_step_us, servo_drive::kUsMax);
        applyServos();
        doRead();
    } else if (strcmp(cmd, "-") == 0) {
        selectedUs() = max(selectedUs() - g_step_us, servo_drive::kUsMin);
        applyServos();
        doRead();
    } else if (strcmp(cmd, "SET") == 0) {
        if (!arg || !*arg) { Serial.println(F("usage: SET <us>")); return; }
        selectedUs() = atoi(arg);
        applyServos();
        doRead();
    } else if (strcmp(cmd, "CENTER") == 0) {
        g_us_x = g_us_y = kUsNeutralGuess;
        applyServos();
        printStatus();
    } else if (strcmp(cmd, "ZERO") == 0) {
        encoders::zero();
        Serial.println(F("encoder neutral captured"));
    } else if (strcmp(cmd, "READ") == 0) {
        doRead();
    } else if (strcmp(cmd, "MARK") == 0) {
        doMark();
    } else if (strcmp(cmd, "LIST") == 0) {
        doList();
    } else if (strcmp(cmd, "UNDO") == 0) {
        int& n = selectedPointCount();
        if (n > 0) { n--; Serial.printf("removed point #%d\n", n + 1); }
    } else if (strcmp(cmd, "CLEAR") == 0) {
        selectedPointCount() = 0;
        Serial.printf("axis %c points cleared\n", g_axis);
    } else if (strcmp(cmd, "FIT") == 0) {
        doFit();
    } else if (strcmp(cmd, "STATUS") == 0 || strcmp(cmd, "?") == 0) {
        printStatus();
    } else if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "H") == 0) {
        printHelp();
    } else {
        Serial.printf("unknown command: %s (HELP for the list)\n", cmd);
    }
}

char g_line[64];
uint8_t g_len = 0;

void pollSerial() {
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

}  // namespace

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {}

    encoders::begin();
    if (!encoders::selftestPresence()) {
        Serial.println(F("!! one or both AS5600 encoders did not ACK -- check Wire1/Wire2 wiring"));
    }
    encoders::zero();

    servo_drive::begin();
    servo_drive::writeUs(kUsNeutralGuess, kUsNeutralGuess);  // known position, not writeDeg()'s
                                                              // NaN-params path (see header)
    g_us_x = g_us_y = kUsNeutralGuess;

    Serial.println(F("servo_ident ready. 'H' for help."));
    printStatus();
}

void loop() {
    pollSerial();
}
