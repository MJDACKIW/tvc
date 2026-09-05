/*
 * static_fire.cpp — clamped static fire controller, 2DOF TVC test stand
 * Teensy 4.1 / PlatformIO
 *
 * Runs an open-loop gimbal profile against a fixed stand while logging thrust,
 * commanded angle, and IMU at 250 Hz. Purpose is plant identification: thrust
 * curve T(t), servo step response, and moment authority. No attitude feedback —
 * the vehicle cannot rotate, so there is nothing to close a loop on.
 *
 * HARDWARE INTERLOCK REQUIRED. The physical arming switch must break the pyro
 * high side. ARM_SENSE_PIN only reports that switch's position; software must
 * never be the sole thing standing between a countdown and an igniter.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <SdFat.h>
#include "config.h"

// ───────────────────────────── state ─────────────────────────────
enum State : uint8_t {
  ST_BOOT = 0, ST_IDLE, ST_ARMED, ST_COUNTDOWN,
  ST_IGNITE, ST_BURN, ST_TAILOFF, ST_SAFE, ST_FAULT
};
static const char *STATE_NAME[] = {
  "BOOT","IDLE","ARMED","COUNTDOWN","IGNITE","BURN","TAILOFF","SAFE","FAULT"
};

static State    g_state    = ST_BOOT;
static Profile  g_profile  = PROF_HOLD;
static uint32_t g_t_state  = 0;   // millis at state entry
static uint32_t g_t_ignite = 0;   // millis at gate-high
static uint32_t g_t_below  = 0;   // millis thrust first dropped below BURNOUT_N
static bool     g_logging  = false;

static bool g_dry = false;

static Servo    g_srv_x, g_srv_y;
static float    g_cmd_x = 0.0f, g_cmd_y = 0.0f;  // applied angle, deg
static float    g_tgt_x = 0.0f, g_tgt_y = 0.0f;  // requested angle, deg

static float    g_thrust_N = 0.0f;
static float    g_thrust_peak = 0.0f;
static float    g_impulse_Ns  = 0.0f;
static int16_t  g_gyro[3] = {0}, g_accel[3] = {0};

static SdFs     g_sd;
static FsFile   g_file;
static char     g_fname[32];

#pragma pack(push, 1)
struct Record {
  uint32_t t_us;
  uint8_t  state;
  uint8_t  profile;
  float    thrust_N;
  float    cmd_x_deg, cmd_y_deg;
  int16_t  gx, gy, gz;
  int16_t  ax, ay, az;
};
#pragma pack(pop)
static_assert(sizeof(Record) == 30, "record packing changed");

static Record  g_buf[128];
static uint8_t g_buf_n = 0;

// ───────────────────────── HX711 (non-blocking) ─────────────────────────
static int32_t g_hx_raw = 0;

static bool hxReady() { return digitalReadFast(HX_DOUT_PIN) == LOW; }

static int32_t hxRead() {
  int32_t v = 0;
  for (uint8_t i = 0; i < 24; i++) {
    digitalWriteFast(HX_SCK_PIN, HIGH);
    delayMicroseconds(1);
    v = (v << 1) | digitalReadFast(HX_DOUT_PIN);
    digitalWriteFast(HX_SCK_PIN, LOW);
    delayMicroseconds(1);
  }
  digitalWriteFast(HX_SCK_PIN, HIGH);   // 25th pulse: gain 128, channel A
  delayMicroseconds(1);
  digitalWriteFast(HX_SCK_PIN, LOW);
  if (v & 0x800000) v |= ~0xFFFFFF;     // sign extend 24 -> 32
  return v;
}

static void serviceLoadCell() {
  if (!hxReady()) return;
  g_hx_raw = hxRead();
  g_thrust_N = (float)(g_hx_raw - HX_OFFSET) / HX_SCALE;
  if (g_thrust_N > g_thrust_peak) g_thrust_peak = g_thrust_N;
}

// ───────────────────────────── IMU ─────────────────────────────
static void imuInit() {
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x18); Wire.endTransmission(); // +/-2000 dps
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x18); Wire.endTransmission(); // +/-16 g
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x02); Wire.endTransmission(); // DLPF 94 Hz
}

static void applyFrame(const int16_t in[3], int16_t out[3]) {
  for (uint8_t i = 0; i < 3; i++) {
    int32_t s = 0;
    for (uint8_t j = 0; j < 3; j++) s += (int32_t)R_BS[i][j] * in[j];
    out[i] = (int16_t)constrain(s, -32768, 32767);
  }
}

static void imuRead() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return;
  int16_t a[3], g[3];
  for (uint8_t i = 0; i < 3; i++) a[i] = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                       // temperature, discarded
  for (uint8_t i = 0; i < 3; i++) g[i] = (Wire.read() << 8) | Wire.read();
  applyFrame(a, g_accel);
  applyFrame(g, g_gyro);
}

// ───────────────────────── actuators ─────────────────────────
static void writeServos() {
  float x = constrain(g_cmd_x, -GIMBAL_MAX_DEG, GIMBAL_MAX_DEG);
  float y = constrain(g_cmd_y, -GIMBAL_MAX_DEG, GIMBAL_MAX_DEG);
  long ux = SERVO_X_CENTER + lroundf(SERVO_X_SIGN * x * SERVO_X_US_PER_DEG);
  long uy = SERVO_Y_CENTER + lroundf(SERVO_Y_SIGN * y * SERVO_Y_US_PER_DEG);
  g_srv_x.writeMicroseconds(constrain(ux, SERVO_US_MIN, SERVO_US_MAX));
  g_srv_y.writeMicroseconds(constrain(uy, SERVO_US_MIN, SERVO_US_MAX));
}

static void slewServos(float dt) {
  const float step = SERVO_SLEW_DEG_S * dt;
  float dx = g_tgt_x - g_cmd_x, dy = g_tgt_y - g_cmd_y;
  g_cmd_x += (fabsf(dx) <= step) ? dx : (dx > 0 ? step : -step);
  g_cmd_y += (fabsf(dy) <= step) ? dy : (dy > 0 ? step : -step);
  writeServos();
}

static void centerServos() { g_tgt_x = g_tgt_y = 0.0f; }

// ───────────────────────── profile generator ─────────────────────────
// t is seconds since ignition confirmation.
static void profileAngles(float t, float &tx, float &ty) {
  tx = ty = 0.0f;
  switch (g_profile) {
    case PROF_HOLD:
      break;

    case PROF_STEP_X:
    case PROF_STEP_Y: {
      float ph = fmodf(t, STEP_PERIOD_S) / STEP_PERIOD_S;
      float v  = (ph < 0.5f) ? PROF_AMP_DEG : -PROF_AMP_DEG;
      if (g_profile == PROF_STEP_X) tx = v; else ty = v;
      break;
    }

    case PROF_RAMP_X: {
      float ph = fmodf(t, RAMP_PERIOD_S) / RAMP_PERIOD_S;      // 0..1
      tx = PROF_AMP_DEG * (4.0f * fabsf(ph - 0.5f) - 1.0f);    // triangle
      break;
    }

    case PROF_CHIRP_X: {
      // Logarithmic sweep CHIRP_F0 -> CHIRP_F1 over CHIRP_DUR.
      // Phase is the integral of instantaneous frequency; getting this wrong
      // smears the frequency axis of the resulting Bode estimate.
      float k = powf(CHIRP_F1_HZ / CHIRP_F0_HZ, 1.0f / CHIRP_DUR_S);
      float u = fminf(t, CHIRP_DUR_S);
      float phase = TWO_PI * CHIRP_F0_HZ * (powf(k, u) - 1.0f) / logf(k);
      tx = PROF_AMP_DEG * sinf(phase);
      break;
    }

    default: break;
  }
}

// ───────────────────────────── logging ─────────────────────────────
static void logOpen() {
  for (uint16_t i = 0; i < 999; i++) {
    snprintf(g_fname, sizeof(g_fname), "SF_%03u.BIN", i);
    if (!g_sd.exists(g_fname)) break;
  }
  if (!g_file.open(g_fname, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println(F("!! log open failed"));
    return;
  }
  g_file.preAllocate(4UL * 1024 * 1024);   // avoid allocation stalls mid-burn
  g_buf_n = 0;
  g_logging = true;
  Serial.printf("logging -> %s\n", g_fname);
}

static void logWrite(uint32_t t_us) {
  if (!g_logging) return;
  Record &r = g_buf[g_buf_n++];
  r.t_us = t_us; r.state = g_state; r.profile = g_profile;
  r.thrust_N = g_thrust_N;
  r.cmd_x_deg = g_cmd_x; r.cmd_y_deg = g_cmd_y;
  r.gx = g_gyro[0]; r.gy = g_gyro[1]; r.gz = g_gyro[2];
  r.ax = g_accel[0]; r.ay = g_accel[1]; r.az = g_accel[2];
  if (g_buf_n >= 128) {
    g_file.write((const uint8_t *)g_buf, sizeof(g_buf));
    g_buf_n = 0;
  }
}

static void logClose() {
  if (!g_logging) return;
  if (g_buf_n) g_file.write((const uint8_t *)g_buf, g_buf_n * sizeof(Record));
  g_file.truncate();
  g_file.close();
  g_logging = false;
  Serial.printf("closed %s\n", g_fname);
}

// ───────────────────────────── pyro ─────────────────────────────
static bool armSwitchClosed() { return digitalReadFast(ARM_SENSE_PIN) == LOW; }
static bool continuityOK()    { return analogRead(PYRO_CONT_PIN) > CONT_ADC_MIN; }
static void pyroOff()         { digitalWriteFast(PYRO_GATE_PIN, LOW); }
static void pyroOn()          { digitalWriteFast(PYRO_GATE_PIN, HIGH); }

// ───────────────────────── state machine ─────────────────────────
static void enter(State s) {
  g_state = s;
  g_t_state = millis();
  Serial.printf(">> %s\n", STATE_NAME[s]);
}

static void abortToSafe(const char *why) {
  
  pyroOff();
  centerServos();
  g_cmd_x = g_cmd_y = 0.0f;
  writeServos();
  logClose();
  noTone(BUZZER_PIN);
  Serial.printf("!! ABORT: %s\n", why);
  enter(ST_SAFE);
}

static void serviceState() {
  const uint32_t now = millis();

  switch (g_state) {
    case ST_IDLE:
      centerServos();
      break;

    case ST_ARMED:
      if (!armSwitchClosed()) { abortToSafe("arm switch opened"); }
      break;

    case ST_COUNTDOWN: {
      if (!armSwitchClosed()) { abortToSafe("arm switch opened"); break; }
      uint32_t el = now - g_t_state;
      if (el >= COUNTDOWN_MS) {
        if (!continuityOK()) { abortToSafe("igniter continuity lost"); break; }
        logOpen();
        pyroOn();
        g_t_ignite = now;
        enter(ST_IGNITE);
        break;
      }
      static uint32_t last_beep = 0;
      uint32_t rem = (COUNTDOWN_MS - el) / 1000;
      if (now - last_beep > (rem < 3 ? 250u : 1000u)) {
        last_beep = now;
        tone(BUZZER_PIN, rem < 3 ? 2200 : 1400, 80);
        Serial.printf("T-%lu\n", (unsigned long)rem + 1);
      }
      break;
    }

    case ST_IGNITE:
      if (now - g_t_ignite >= PYRO_FIRE_MS) pyroOff();
      if (g_thrust_N >= THRUST_DETECT_N) {
        pyroOff();
        enter(ST_BURN);
      } else if (now - g_t_ignite >= NO_IGNITION_MS) {
        abortToSafe("no thrust detected — assume misfire, WAIT 60 s before approach");
      }
      break;

    case ST_BURN: {
  float t = (now - g_t_ignite) * 1e-3f;
  profileAngles(t, g_tgt_x, g_tgt_y);

  if (g_dry) {                                  // no thrust to watch
    if (t >= DRY_RUN_S) { centerServos(); enter(ST_TAILOFF); }
    break;
  }
  if (g_thrust_N < BURNOUT_N) {
    if (g_t_below == 0) g_t_below = now;
    else if (now - g_t_below >= BURNOUT_HOLD_MS) { centerServos(); enter(ST_TAILOFF); }
  } else {
    g_t_below = 0;
  }
  break;
}

    case ST_TAILOFF:
      if (now - g_t_state >= POST_BURN_LOG_MS) {
        logClose();
        Serial.printf("peak %.1f N   impulse %.1f N-s\n", g_thrust_peak, g_impulse_Ns);
        enter(ST_SAFE);
      }
      break;

    case ST_SAFE:
    case ST_FAULT:
      pyroOff();
      centerServos();
      break;

    default: break;
  }
}

// ───────────────────────── serial console ─────────────────────────
static char    g_line[40];
static uint8_t g_line_n = 0;

static void status() {
  Serial.printf("[%s] prof=%u  thrust=%7.2f N  raw=%ld  cmd=(%.2f, %.2f) deg  "
                "arm_sw=%s  cont=%s\n",
                STATE_NAME[g_state], g_profile, g_thrust_N, (long)g_hx_raw,
                g_cmd_x, g_cmd_y,
                armSwitchClosed() ? "CLOSED" : "open",
                continuityOK() ? "OK" : "OPEN");
}

static void handleLine(char *s) {
  while (*s == ' ') s++;
  char c = *s;
  char *arg = s + 1;
  while (*arg == ' ') arg++;

  if (c == 'x' || g_state == ST_FAULT) { abortToSafe("operator abort"); return; }

  switch (c) {
    case '?': status(); break;

    case 'p':
      if (g_state != ST_IDLE) { Serial.println(F("profile locked outside IDLE")); break; }
      { long v = atol(arg);
        if (v >= 0 && v < PROF_COUNT) { g_profile = (Profile)v; status(); } }
      break;

    case 'd':   // dry run: execute the profile with no pyro
      g_dry = true;
    if (g_state != ST_IDLE) { Serial.println(F("IDLE only")); break; }
      logOpen();
      g_t_ignite = millis();
      g_thrust_peak = 0; g_impulse_Ns = 0; g_t_below = 0;
      enter(ST_BURN);
      Serial.println(F("DRY RUN — pyro inhibited, profile running"));
      break;

    case 'z':   // tare
      if (g_state != ST_IDLE) break;
      Serial.printf("tare: set HX_OFFSET = %ld\n", (long)g_hx_raw);
      break;

    case 'a':
      if (g_state != ST_IDLE) { Serial.println(F("IDLE only")); break; }
      if (!armSwitchClosed()) { Serial.println(F("close the physical arm switch first")); break; }
      if (!continuityOK())    { Serial.println(F("no igniter continuity")); break; }
      enter(ST_ARMED);
      break;

    case 'f':
      g_dry = false;
      if (g_state != ST_ARMED) { Serial.println(F("must be ARMED")); break; }
      if (strncmp(arg, "GO", 2) != 0) { Serial.println(F("send: f GO")); break; }
      g_thrust_peak = 0; g_impulse_Ns = 0; g_t_below = 0;
      enter(ST_COUNTDOWN);
      break;

    case 'r':
      if (g_state == ST_SAFE) { enter(ST_IDLE); }
      break;

    case 'h':
      Serial.println(F(
        "\n--- static fire ---\n"
        " ?        status\n"
        " p <n>    profile 0=HOLD 1=STEP_X 2=STEP_Y 3=RAMP_X 4=CHIRP_X\n"
        " z        print tare value for HX_OFFSET\n"
        " d        DRY RUN — profile with pyro inhibited\n"
        " a        arm (requires physical switch + continuity)\n"
        " f GO     start countdown\n"
        " x        ABORT (any state)\n"
        " r        reset SAFE -> IDLE\n"));
      break;

    default: break;
  }
}

// ───────────────────────────── main ─────────────────────────────
void setup() {
  pinMode(PYRO_GATE_PIN, OUTPUT);
  pyroOff();                                  // before anything else
  pinMode(ARM_SENSE_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(HX_SCK_PIN, OUTPUT);
  pinMode(HX_DOUT_PIN, INPUT_PULLUP);
  digitalWriteFast(HX_SCK_PIN, LOW);

  Serial.begin(115200);
  Serial.printf("sizeof(Record) = %u\n", (unsigned)sizeof(Record));
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  g_srv_x.attach(SERVO_X_PIN, SERVO_US_MIN, SERVO_US_MAX);
  g_srv_y.attach(SERVO_Y_PIN, SERVO_US_MIN, SERVO_US_MAX);
  writeServos();

  imuInit();

  if (!g_sd.begin(SdioConfig(FIFO_SDIO))) {
    Serial.println(F("!! SD init failed — logging disabled"));
  }

  Serial.println(F("static fire controller ready. 'h' for help."));
  enter(ST_IDLE);
}

void loop() {
  static uint32_t t_ctrl = 0, t_telem = 0;

  while (Serial.available()) {
    char k = (char)Serial.read();
    if (k == '\n' || k == '\r') {
      if (g_line_n) { g_line[g_line_n] = '\0'; handleLine(g_line); g_line_n = 0; }
    } else if (g_line_n < sizeof(g_line) - 1) {
      g_line[g_line_n++] = k;
    }
  }

  serviceLoadCell();

  uint32_t now = micros();
  if (now - t_ctrl >= CTRL_PERIOD_US) {
    float dt = (now - t_ctrl) * 1e-6f;
    t_ctrl = now;

    imuRead();
    serviceState();
    slewServos(dt);

    if (g_state == ST_BURN && g_thrust_N > 0.0f) g_impulse_Ns += g_thrust_N * dt;
    logWrite(now);

    digitalWriteFast(LED_PIN, (g_state >= ST_ARMED && g_state <= ST_BURN)
                              ? ((millis() >> 7) & 1) : LOW);
  }

  if (now - t_telem >= TELEM_PERIOD_US) {
    t_telem = now;
    // LoRa downlink slots in here. Same fields as Record, 20 Hz.
    if (g_state == ST_BURN || g_state == ST_TAILOFF) status();
  }
}
