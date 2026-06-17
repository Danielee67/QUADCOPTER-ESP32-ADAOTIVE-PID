/*
 ╔══════════════════════════════════════════════════════════════════════════╗
 ║   ESP32 Quadcopter – Advanced Research Firmware                          ║
 ║   • Mahony AHRS  •  Dynamic Notch  •  PT1 Gyro LPF  •  PT1 D-term        ║
 ║   • Air-Density Adaptive PID  •  Anti-windup  •  D-on-measurement        ║
 ║   • CH6 ≤ 1300 → PID-FIX  |  CH6 > 1300 → ADAPTIVE                       ║
 ║                                                                          ║
 ║   VIBRATION TUNING GUIDE:                                                ║
 ║   1. First fly: if still oscillating → halve KpRate (0.08)               ║
 ║   2. Tune KpRate up in +0.02 steps until wobble starts, back off 20%     ║
 ║   3. Then add Kd in +0.002 steps (damps without slowing response)        ║
 ║   4. GYRO_LPF_HZ: lower = smoother but sluggish. 60-100 Hz is good       ║
 ║   5. dCutoff in PIDConfig: 20-40 Hz kills D noise. Lower if buzzing      ║
 ║                                                                          ║
 ║   Hardware: ESP32 | MPU6050 | BME280 | GPS NEO-6M                        ║
 ║   Props: 1245 | Motor: 2212 930KV | Frame: 500mm | W: 1.4 kg             ║
 ╚══════════════════════════════════════════════════════════════════════════╝
*/

#ifndef ESP32'
#error "Select ESP32 board: Tools -> Board -> ESP32 Dev Module"
#endif

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP32Servo.h>
#include <TinyGPS++.h>
#include <BluetoothSerial.h>
#include <math.h>

// ════════════════════════════════════════════════════
//  SECTION 1 – PINS & LIMITS
// ════════════════════════════════════════════════════
static const int SDA_PIN     = 21;
static const int SCL_PIN     = 22;
static const int ESC_M1_PIN  = 13;
static const int ESC_M2_PIN  = 14;
static const int ESC_M3_PIN  = 26;
static const int ESC_M4_PIN  = 33;
static const int IBUS_RX_PIN = 16;
static const int IBUS_TX_PIN = 27;
static const int GPS_RX_PIN  = 17;
static const int GPS_TX_PIN  = 5;

static const int PWM_MIN  = 1000;
static const int PWM_IDLE = 1050;
static const int PWM_MAX  = 2000;

// ════════════════════════════════════════════════════
//  SECTION 2 – PHYSICAL CONSTANTS
// ════════════════════════════════════════════════════
static const float SEALEVEL_HPA = 1013.25f;
static const float R_AIR        = 287.05f;   // J/(kg·K)
static const float RHO_SEA      = 1.225f;    // kg/m³
static const float KT_COEFF     = 1.35e-5f;  // thrust coeff
static const float KQ_COEFF     = 1.85e-7f;  // torque coeff
static const float PROP_DIAM_M  = 0.3048f;   // 12 inch
static const float V_BATT       = 11.1f;     // 3S LiPo
static const float MASS_KG      = 1.4f;
static const float KV_RPM       = 930.0f;    // motor KV

// ════════════════════════════════════════════════════
//  SECTION 3 – STRUCT DEFINITIONS  (must come first)
// ════════════════════════════════════════════════════

// ── 3A-0. PT1 Low-Pass Filter ────────────────────────
// Simple one-pole IIR — cuts high-freq noise from gyro
// cutoff_hz : -3dB frequency
// RC = 1/(2π·fc),  α = dt/(RC+dt)
struct PT1 {
    float state;
    PT1() : state(0.0f) {}
    float process(float x, float cutoff_hz, float dt) {
        float RC = 1.0f / (2.0f * (float)M_PI * cutoff_hz);
        float alpha = dt / (RC + dt);
        state += alpha * (x - state);
        return state;
    }
    void reset(float v=0.0f){ state=v; }
};

// ── 3A. Biquad Notch Filter ──────────────────────────
struct BiquadNotch {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;

    BiquadNotch() : b0(1),b1(0),b2(0),a1(0),a2(0),
                    x1(0),x2(0),y1(0),y2(0) {}

    void setFreq(float f0_hz, float fs_hz, float Q) {
        if (f0_hz < 10.0f || f0_hz >= fs_hz * 0.49f) return;
        float w0    = 2.0f * (float)M_PI * f0_hz / fs_hz;
        float cosw0 = cosf(w0);
        float alpha = sinf(w0) / (2.0f * Q);
        float inv   = 1.0f / (1.0f + alpha);
        b0 =  1.0f        * inv;
        b1 = -2.0f*cosw0  * inv;
        b2 =  1.0f        * inv;
        a1 = -2.0f*cosw0  * inv;
        a2 = (1.0f-alpha) * inv;
    }

    float process(float x) {
        float y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;
        x2=x1; x1=x; y2=y1; y1=y;
        return y;
    }
};

// ── 3B. PID State ────────────────────────────────────
struct PIDState {
    float integral;
    float prevMeas;
    float prevError;
    PT1   dFilter;    // separate D-term low-pass (cutoff lower than gyro)
    PIDState() : integral(0), prevMeas(0), prevError(0) {}
    void reset() {
        integral=0; prevMeas=0; prevError=0;
        dFilter.reset(0.0f);
    }
};

// Global flag: freeze all integrals when throttle is below flight
// threshold. Prevents windup while armed on the ground.
bool pid_integrators_active = false;

// ── 3C. PID Config ───────────────────────────────────
struct PIDConfig {
    float Kp, Ki, Kd, Kff;
    float outMin, outMax;
    float N;          // derivative filter pole (legacy, kept for compat)
    float dCutoff;    // PT1 cutoff for D-term [Hz]. 0 = no extra filter
    PIDConfig(float kp, float ki, float kd, float kff,
              float mn, float mx, float n=20.0f, float dc=30.0f)
        : Kp(kp), Ki(ki), Kd(kd), Kff(kff),
          outMin(mn), outMax(mx), N(n), dCutoff(dc) {}
};

// ════════════════════════════════════════════════════
//  SECTION 4 – GLOBAL OBJECTS
// ════════════════════════════════════════════════════
Adafruit_MPU6050 mpu;
Adafruit_BME280  bme;
Servo            esc1, esc2, esc3, esc4;
HardwareSerial   IBUS_S(2);
HardwareSerial   GPS_Serial(1);
TinyGPSPlus      gps;
BluetoothSerial  BT;

// ════════════════════════════════════════════════════
//  SECTION 5 – GLOBAL VARIABLES
// ════════════════════════════════════════════════════

// ── Mahony AHRS ──────────────────────────────────────
float Kp_mahony = 2.0f;
float Ki_mahony = 0.005f;
float q0=1.0f, q1=0.0f, q2=0.0f, q3=0.0f;
float mex_int=0.0f, mey_int=0.0f, mez_int=0.0f;
float roll_deg=0.0f, pitch_deg=0.0f, yaw_deg=0.0f;
float yawRate_dps=0.0f;
float gx_offset=0.0f, gy_offset=0.0f, gz_offset=0.0f;

// ── In-Flight Gyro Bias Estimator ────────────────────
// MPU6050 bias drifts ~0.1-0.3°/s per °C of temperature change.
// When craft is hovering quietly we slowly track residual bias
// and subtract it — fixes slow tilt-drift and yaw wander.
float gx_bias_if = 0.0f;   // in-flight bias correction X
float gy_bias_if = 0.0f;   // in-flight bias correction Y
float gz_bias_if = 0.0f;   // in-flight bias correction Z
// Only update when all rates are below this threshold (craft still)
static const float BIAS_STILL_TH = 4.0f;    // deg/s
// How fast to track — τ ≈ 1/(2000 × 0.00005) = 10 sec
static const float BIAS_ALPHA    = 0.00005f;

// ── Attitude zero-offset (captured after AHRS converges) ──
// Allows the quad to treat any resting surface as "level zero"
float roll_offset  = 0.0f;
float pitch_offset = 0.0f;
float yaw_offset   = 0.0f;
bool  ahrs_zeroed  = false;  // set true once after boot convergence

// ── Dynamic Notch ────────────────────────────────────
static const int   N_NOTCH  = 4;
static const float NOTCH_Q  = 5.0f;
static const int   PROP_BLADES = 2;   // 12x4.5 غالباً 2 شفرات
static float       fs_hz_est = 1000.0f; // تقدير تردد الحلقة (يتحدث بـ dt)
BiquadNotch notchRoll[N_NOTCH];
BiquadNotch notchPitch[N_NOTCH];
BiquadNotch notchYaw[N_NOTCH];
float motor_rpm[4]  = {0,0,0,0};

// ── PT1 Low-Pass filters on gyro (applied AFTER notch) ───
// Cutoff 80 Hz: passes attitude rates, kills motor vibrations
// Lower = smoother but more phase lag. 80 Hz is a safe start.
static const float GYRO_LPF_HZ = 80.0f;
PT1 pt1Roll, pt1Pitch, pt1Yaw;

// ── Air model ────────────────────────────────────────
float air_rho       = RHO_SEA;
float air_rho_ratio = 1.0f;
float reynoldsNumber[4] = {0,0,0,0};
float motor_thrust[4]   = {0,0,0,0};
float motor_torque[4]   = {0,0,0,0};

// ── BME280 ───────────────────────────────────────────
bool  bme_ok       = false;
float bme_temp_C   = 25.0f;
float bme_pres_hPa = SEALEVEL_HPA;
float bme_alt_m    = 0.0f;
float bme_alt0_m   = 0.0f;
float bme_humidity = 0.0f;

// ══════════════════════════════════════════════════════
//  PID GAINS — نسخة الاستقرار الكامل
//
//  المنطق:
//  • Ki = 0 في كل مكان — حتى يثبت الطيران أولاً
//  • KdAng = 0.005 — يُخمّد الـ overshoot في الحلقة الخارجية
//  • KpRate = 0.80 ثبت أنه يعطي استجابة كافية
//  • KdRate = 0.030 يبقى كما هو
//
//  بعد تحقيق الاستقرار الكامل، أضف Ki بهذا الترتيب:
//  1. KiRate = 0.005 أولاً (صغير جداً)
//  2. انتظر دقيقة طيران — إذا استقر → ارفع إلى 0.008
//  3. لا تلمس KiAng أو KiYaw إلا بعد اكتمال الخطوتين
//
//                    Kp      Ki      Kd      Kff   outMin  outMax
PIDConfig cfgAngR (0.45f, 0.000f, 0.005f,  0.00f, -200.0f, 200.0f);
PIDConfig cfgAngP (0.45f, 0.000f, 0.005f,  0.00f, -200.0f, 200.0f);
PIDConfig cfgRateR(0.1050f, 0.000f, 0.00730f,  0.00f, -100.0f, 100.0f);
PIDConfig cfgRateP(0.1050f, 0.000f, 0.007530f,  0.00f, -100.0f, 100.0f);
PIDConfig cfgYaw  (1.20f, 0.000f, 0.000f,  0.00f, -100.0f, 100.0f);
PIDConfig cfgAlt  (1.50f, 0.080f, 0.800f,  0.00f, -150.0f, 150.0f);
PIDConfig cfgGps  (0.60f, 0.010f, 0.080f,  0.00f,  -10.0f,  10.0f);

PIDState pidAngR, pidAngP;
PIDState pidRateR, pidRateP;
PIDState pidYaw;
PIDState pidAlt;
PIDState pidGpsX, pidGpsY;

// ── Mode & arm ───────────────────────────────────────
bool  adaptiveMode   = false;
bool  armed          = false;
bool  altHoldEnabled = false;
bool  gpsHoldEnabled = false;
float altHoldTarget  = 0.0f;
float yawHoldTarget  = 0.0f;
double homeLat=0.0, homeLon=0.0;

// ── Mixer outputs ────────────────────────────────────
int   baseThrottle = PWM_IDLE;
int   m1=PWM_MIN, m2=PWM_MIN, m3=PWM_MIN, m4=PWM_MIN;
float uRoll=0.0f, uPitch=0.0f, uYaw=0.0f;

// ── iBUS ─────────────────────────────────────────────
static const int CH_ROLL  = 0;
static const int CH_PITCH = 1;
static const int CH_THR   = 2;
static const int CH_YAW   = 3;
static const int CH_ARM   = 4;
static const int CH_MODE  = 5;  // <=1300 FIX, >1300 ADAPTIVE
static const float MAX_ANGLE    = 25.0f;
static const float MAX_RATE_TGT = 250.0f;
static const float MAX_YAW_RATE = 120.0f;
static const int   DEADBAND     = 15;
static const float ARM_LIM      = 25.0f;
uint16_t ibus_ch[14] = {1500,1500,1000,1500,1000,1000};
uint32_t lastIbusMs  = 0;

// ════════════════════════════════════════════════════
//  SECTION 6 – HELPER INLINE FUNCTIONS
// ════════════════════════════════════════════════════
static inline float fmap(float x, float i0, float i1,
                          float o0, float o1) {
    return (x-i0)*(o1-o0)/(i1-i0)+o0;
}
static inline float fclamp(float v, float lo, float hi) {
    return (v<lo)?lo:(v>hi?hi:v);
}
static inline int iDeadband(int v, int c=1500, int db=DEADBAND) {
    return (abs(v-c)<=db) ? c : v;
}

// ════════════════════════════════════════════════════
//  SECTION 7 – MAHONY AHRS
//  Ref: Mahony et al., ECC 2005 – SO(3) complementary filter
// ════════════════════════════════════════════════════
void mahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az, float dt)
{
    // ── Accelerometer Magnitude Gate ──────────────────
    // هذا هو الإصلاح الجذري لمشكلة "عشوائي بعد 3-5 ثواني"
    //
    // أثناء الطيران: norm = sqrt(ax²+ay²+az²) يتغير بسبب قوى الحركة
    // مثال: طائرة تتسارع بـ 2m/s² → norm = 1.2g بدلاً من 1.0g
    // الـ Mahony يُصحح الكواترنيون باتجاه خاطئ → AHRS ينحرف
    //
    // الحل: نقبل المسرّع كمرجع فقط إذا كان norm بين 0.85g و 1.15g
    // خارج هذا النطاق = الطائرة تتحرك → نثق بالجيروسكوب فقط
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return;

    // 9.81 * 0.85 = 8.34,  9.81 * 1.15 = 11.28
    bool accelReliable = (norm > 8.34f && norm < 11.28f);

    float ex = 0.0f, ey = 0.0f, ez = 0.0f;

    if (accelReliable) {
        // المسرّع موثوق → احسب خطأ التصحيح
        float axn = ax/norm, ayn = ay/norm, azn = az/norm;

        // Estimated gravity direction from quaternion
        float vx = 2.0f*(q1*q3 - q0*q2);
        float vy = 2.0f*(q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        // Cross-product error
        ex = ayn*vz - azn*vy;
        ey = azn*vx - axn*vz;
        ez = axn*vy - ayn*vx;

        // Integral feedback — فقط عند المسرّع الموثوق
        mex_int += Ki_mahony * ex * dt;
        mey_int += Ki_mahony * ey * dt;
        mez_int += Ki_mahony * ez * dt;
        // Clamp mahony integrals
        mex_int = fclamp(mex_int, -0.1f, 0.1f);
        mey_int = fclamp(mey_int, -0.1f, 0.1f);
        mez_int = fclamp(mez_int, -0.1f, 0.1f);
    } else {
        // المسرّع غير موثوق (حركة) → الجيروسكوب فقط
        // الـ integral يبقى كما هو، لا تصحيح جديد
        mex_int *= 0.999f;  // تصريف بطيء جداً
        mey_int *= 0.999f;
        mez_int *= 0.999f;
    }

    // طبّق التصحيح على الجيروسكوب
    gx += Kp_mahony*ex + mex_int;
    gy += Kp_mahony*ey + mey_int;
    gz += Kp_mahony*ez + mez_int;

    // Quaternion rate integration
    float h = 0.5f * dt;
    float dq0 = (-q1*gx - q2*gy - q3*gz)*h;
    float dq1 = ( q0*gx + q2*gz - q3*gy)*h;
    float dq2 = ( q0*gy - q1*gz + q3*gx)*h;
    float dq3 = ( q0*gz + q1*gy - q2*gx)*h;
    q0+=dq0; q1+=dq1; q2+=dq2; q3+=dq3;

    norm = sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
    if (norm < 1e-6f) { q0=1;q1=q2=q3=0; return; }
    q0/=norm; q1/=norm; q2/=norm; q3/=norm;

    // Euler extraction (ZYX convention) – raw values
    float r_raw = atan2f(2.0f*(q0*q1+q2*q3),
                         1.0f-2.0f*(q1*q1+q2*q2)) * 57.29578f;
    float p_raw = asinf(fclamp(2.0f*(q0*q2-q3*q1),-1.0f,1.0f))
                  * 57.29578f;
    float y_raw = atan2f(2.0f*(q0*q3+q1*q2),
                         1.0f-2.0f*(q2*q2+q3*q3)) * 57.29578f;

    // Subtract boot-time surface offset so angles always start at 0
    roll_deg  = r_raw - roll_offset;
    pitch_deg = p_raw - pitch_offset;

    // Yaw difference (wrap to ±180)
    float yd = y_raw - yaw_offset;
    while (yd >  180.0f) yd -= 360.0f;
    while (yd < -180.0f) yd += 360.0f;
    yaw_deg = yd;
}

// ════════════════════════════════════════════════════
//  SECTION 8 – RPM ESTIMATION & DYNAMIC NOTCH
// ════════════════════════════════════════════════════
void estimateRPM(int pw[4]) {
    for (int i=0; i<4; i++) {
        float n = (float)(pw[i] - PWM_MIN) / (float)(PWM_MAX - PWM_MIN);
        n = fclamp(n, 0.0f, 1.0f);
        motor_rpm[i] = n * KV_RPM * V_BATT;
    }
}

// Apply 4-stage dynamic notch bank to one gyro axis signal
float applyDynamicNotch(BiquadNotch* notch, float sig, float fs_hz) {
    float out = sig;
    fs_hz = fclamp(fs_hz, 200.0f, 4000.0f);
    for (int i=0; i<N_NOTCH; i++) {
        float f0 = (motor_rpm[i] / 60.0f) * (float)PROP_BLADES;
        notch[i].setFreq(f0, fs_hz, NOTCH_Q);
        out = notch[i].process(out);
    }
    return out;
}
// ════════════════════════════════════════════════════
//  SECTION 9 – AIR MODEL  (ISA + Sutherland viscosity)
// ════════════════════════════════════════════════════
void updateAirModel(float T_C, float P_hPa) {
    float T_K  = T_C + 273.15f;
    float P_Pa = P_hPa * 100.0f;
    air_rho       = P_Pa / (R_AIR * T_K);
    air_rho_ratio = air_rho / RHO_SEA;

    // Sutherland viscosity law
    const float mu_ref = 1.716e-5f;
    const float T_ref  = 273.15f;
    const float C_s    = 110.4f;
    float mu = mu_ref * powf(T_K/T_ref, 1.5f)
             * (T_ref+C_s)/(T_K+C_s);

    const float r_tip  = PROP_DIAM_M / 2.0f;
    const float chord  = 0.03f;
    for (int i=0; i<4; i++) {
        float omega = motor_rpm[i] * (2.0f*(float)M_PI/60.0f);
        float v_tip = omega * r_tip;
        reynoldsNumber[i] = (air_rho * v_tip * chord) / mu;
    }
}

void updateThrustTorque() {
    for (int i=0; i<4; i++) {
        float n = motor_rpm[i] / 60.0f;
        motor_thrust[i] = KT_COEFF * air_rho * n * n;
        motor_torque[i] = KQ_COEFF * air_rho * n * n;
    }
}

// ════════════════════════════════════════════════════
//  SECTION 10 – PID COMPUTE
//  Features: D-on-measurement, feed-forward,
//            back-calculation anti-windup
// ════════════════════════════════════════════════════
float pidCompute(float sp, float meas, PIDState &s,
                 float dt, const PIDConfig &c, float sp_prev)
{
    if (dt < 1e-6f) return 0.0f;
    float err   = sp - meas;
    float dMeas = (meas - s.prevMeas) / dt;
    s.prevMeas  = meas;

    // D-on-measurement with PT1 low-pass filter
    // dCutoff = 30 Hz → D term only responds to events < 30 Hz
    // This is the primary weapon against high-frequency oscillation
    float dRaw  = -dMeas;
    float dFilt = (c.dCutoff > 1.0f)
                  ? s.dFilter.process(dRaw, c.dCutoff, dt)
                  : dRaw;

    // Feed-forward REMOVED — Kff/dt at 2kHz amplifies stick noise
    float out        = c.Kp*err + c.Ki*s.integral + c.Kd*dFilt;
    float outClamped = fclamp(out, c.outMin, c.outMax);

    // ── Conditional Integration (أفضل من anti-windup) ──
    // الـ integral يتراكم فقط إذا تحققت الشروط الثلاثة معاً:
    // 1. الطائرة في الهواء
    // 2. الإخراج لم يتشبع (not saturated)
    // 3. الـ integral والـ error في نفس الاتجاه (منع التراكم الزائد)
    bool notSaturated = (out >= c.outMin) && (out <= c.outMax);
    bool sameSign     = (err * s.integral) >= 0.0f;

    if (pid_integrators_active && c.Ki > 1e-9f
        && notSaturated && sameSign)
    {
        s.integral += err * dt;
        // حد صارم: الـ integral لا يتجاوز 20% من نطاق الإخراج
        float ilim = (fabsf(c.outMax) * 0.20f) / c.Ki;
        s.integral = fclamp(s.integral, -ilim, ilim);
    } else if (!pid_integrators_active) {
        s.integral *= 0.98f;  // تصريف بطيء على الأرض
    }
    s.prevError = err;
    return outClamped;
}

// ════════════════════════════════════════════════════
//  SECTION 11 – ADAPTIVE GAIN SCALING
//  CH6 <= 1300: FIX (base gains)
//  CH6  > 1300: ADAPTIVE (scale by 1/air_rho_ratio)
//  Physical basis: Thrust ∝ ρ·n²
//  → if ρ drops, multiply gains by 1/ρ_ratio to keep
//    same closed-loop bandwidth
// ════════════════════════════════════════════════════
PIDConfig scaledCfg(const PIDConfig &base) {
    if (!adaptiveMode) return base;
    float s = 1.0f / fclamp(air_rho_ratio, 0.6f, 1.2f);
    PIDConfig c = base;
    c.Kp  *= s;
    c.Ki  *= s;
    c.Kd  *= s;
    c.Kff *= s;
    return c;
}

// ════════════════════════════════════════════════════
//  SECTION 12 – MIXER  (X-frame)
//  M1=Rear-Right CCW  M2=Front-Right CW
//  M3=Rear-Left CW    M4=Front-Left CCW
// ════════════════════════════════════════════════════
void mixMotors(int baseOut, float mixAlpha, int &t1, int &t2, int &t3, int &t4) {
    // mixAlpha: 0→1 (يمنع قفزة تصحيح كبيرة لحظة الإقلاع)
    int r = (int)(uRoll  * mixAlpha);
    int p = (int)(uPitch * mixAlpha);
    int y = (int)(uYaw   * mixAlpha);
    t1 = baseOut - r - p + y;
    t2 = baseOut - r + p - y;
    t3 = baseOut + r - p - y;
    t4 = baseOut + r + p + y;
}
// ════════════════════════════════════════════════════
//  SECTION 13 – iBUS PARSER
// ════════════════════════════════════════════════════
bool ibusReadFrame() {
    const int FL = 32;
    if (IBUS_S.available() < FL) return false;
    if (IBUS_S.peek() != 0x20) { IBUS_S.read(); return false; }
    uint8_t buf[FL];
    IBUS_S.readBytes(buf, FL);
    if (buf[0]!=0x20 || buf[1]!=0x40) return false;
    uint16_t sum=0;
    for (int i=0;i<FL-2;i++) sum+=buf[i];
    uint16_t ex=0xFFFF-sum;
    uint16_t got=(uint16_t)buf[FL-2]|((uint16_t)buf[FL-1]<<8);
    if (got!=ex) return false;
    for (int ch=0;ch<14;ch++) {
        int idx=2+ch*2;
        ibus_ch[ch]=(uint16_t)buf[idx]|((uint16_t)buf[idx+1]<<8);
    }
    lastIbusMs=millis();
    return true;
}

uint16_t ibusGet(int ch) {
    return (ch>=0 && ch<14) ? ibus_ch[ch] : 1500;
}

// ════════════════════════════════════════════════════
//  SECTION 14 – SETUP
// ════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n[BOOT] Quad Research Firmware v4.0");

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    // MPU6050
    if (!mpu.begin()) {
        while(1) { Serial.println("[ERR] MPU6050 not found!"); delay(1000); }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
    Serial.println("[OK] MPU6050");

    // Gyro calibration
    Serial.println("[CAL] Keep still 4 sec...");
    const int N = 4000;
    float gxs=0, gys=0, gzs=0;
    for (int i=0; i<N; i++) {
        sensors_event_t a,g,t;
        mpu.getEvent(&a,&g,&t);
        gxs+=g.gyro.x; gys+=g.gyro.y; gzs+=g.gyro.z;
        delay(1);
    }
    gx_offset=gxs/N; gy_offset=gys/N; gz_offset=gzs/N;
    Serial.printf("[CAL] gx=%.5f gy=%.5f gz=%.5f\n",
                  gx_offset,gy_offset,gz_offset);

    // AHRS warm-up: run 500 iterations (~1 sec) so the quaternion
    // converges fully to the real resting orientation,
    // then lock those angles as the zero reference.
    // Result: Roll/Pitch/Yaw always start at exactly 0.0 deg
    // regardless of surface tilt.
    Serial.println("[CAL] AHRS zeroing (1 sec)...");
    for (int i=0; i<500; i++) {
        sensors_event_t a,g,t;
        mpu.getEvent(&a,&g,&t);
        float gxc = g.gyro.x - gx_offset;
        float gyc = g.gyro.y - gy_offset;
        float gzc = g.gyro.z - gz_offset;
        mahonyUpdate(gxc, gyc, gzc,
                     a.acceleration.x,
                     a.acceleration.y,
                     a.acceleration.z, 0.002f);
        delay(2);
    }
    // Capture raw quaternion → Euler as the surface offset
    roll_offset  = atan2f(2.0f*(q0*q1+q2*q3),
                          1.0f-2.0f*(q1*q1+q2*q2)) * 57.29578f;
    pitch_offset = asinf(fclamp(2.0f*(q0*q2-q3*q1),
                                -1.0f, 1.0f))       * 57.29578f;
    yaw_offset   = atan2f(2.0f*(q0*q3+q1*q2),
                          1.0f-2.0f*(q2*q2+q3*q3))  * 57.29578f;
    ahrs_zeroed  = true;
    Serial.printf("[CAL] Zero offset: R=%.2f  P=%.2f  Y=%.2f\n",
                  roll_offset, pitch_offset, yaw_offset);

    // BME280
    if (bme.begin(0x76) || bme.begin(0x77)) {
        bme_ok = true;
        delay(200);
        bme_temp_C   = bme.readTemperature();
        bme_pres_hPa = bme.readPressure() / 100.0f;
        bme_humidity = bme.readHumidity();
        bme_alt0_m   = bme.readAltitude(SEALEVEL_HPA);
        updateAirModel(bme_temp_C, bme_pres_hPa);
        Serial.printf("[OK] BME280 T=%.1fC P=%.1fhPa rho=%.4f\n",
                      bme_temp_C, bme_pres_hPa, air_rho);
    } else {
        Serial.println("[WARN] BME280 missing – using defaults");
    }

    // GPS
    GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("[OK] GPS UART");

    // iBUS
    IBUS_S.begin(115200, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);
    IBUS_S.setTimeout(1);

    // ESC
    ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
    esc1.setPeriodHertz(50); esc2.setPeriodHertz(50);
    esc3.setPeriodHertz(50); esc4.setPeriodHertz(50);
    esc1.attach(ESC_M1_PIN,PWM_MIN,PWM_MAX);
    esc2.attach(ESC_M2_PIN,PWM_MIN,PWM_MAX);
    esc3.attach(ESC_M3_PIN,PWM_MIN,PWM_MAX);
    esc4.attach(ESC_M4_PIN,PWM_MIN,PWM_MAX);
    esc1.writeMicroseconds(PWM_MIN);
    esc2.writeMicroseconds(PWM_MIN);
    esc3.writeMicroseconds(PWM_MIN);
    esc4.writeMicroseconds(PWM_MIN);

    // Init notch banks at 200 Hz default
    for (int i=0; i<N_NOTCH; i++) {
        notchRoll[i].setFreq(200.0f,  fs_hz_est, NOTCH_Q);
        notchPitch[i].setFreq(200.0f, fs_hz_est, NOTCH_Q);
        notchYaw[i].setFreq(200.0f,   fs_hz_est, NOTCH_Q);
    }

    Serial.println("[OK] Setup complete. REMOVE PROPS BEFORE TEST!");

    // Bluetooth — يُبدأ آخراً بعد كل الـ calibration (مثل الكود الشغال)
    if (!BT.begin("Quadcopter_Adam")) {
        Serial.println("[BT] فشل التشغيل — تحقق من Partition Scheme");
    } else {
        Serial.println("[BT] جاهز: Quadcopter_Adam");
        Serial.println("[BT] موبايل: Serial Bluetooth Terminal → Quadcopter_Adam");
    }
}

// ════════════════════════════════════════════════════
//  SECTION 15 – MAIN LOOP
// ════════════════════════════════════════════════════
void loop() {
    static uint32_t lastLoop = micros();
    uint32_t now = micros();
    if (now - lastLoop < 500) return;   // ~2 kHz
    float dt = (now - lastLoop) * 1e-6f;
    if (dt > 0.05f) dt = 0.05f;
    lastLoop = now;

    // تقدير fs الحقيقي للحلقة (مهم لتصميم الـ Notch بشكل صحيح)
    float fs_now = (dt > 1e-6f) ? (1.0f / dt) : fs_hz_est;
    fs_hz_est += 0.02f * (fs_now - fs_hz_est);

    // ── 1. IMU read ──────────────────────────────────
    sensors_event_t evA, evG, evT;
    mpu.getEvent(&evA, &evG, &evT);

    float gx_raw = evG.gyro.x - gx_offset - gx_bias_if;
    float gy_raw = evG.gyro.y - gy_offset - gy_bias_if;
    float gz_raw = evG.gyro.z - gz_offset - gz_bias_if;

    // ── 2. Mahony AHRS ───────────────────────────────
    mahonyUpdate(gx_raw, gy_raw, gz_raw,
                 evA.acceleration.x,
                 evA.acceleration.y,
                 evA.acceleration.z, dt);
    yawRate_dps = gz_raw * 57.29578f;

    // ── 3. RPM estimate → Dynamic Notch ─────────────
    int pw_now[4] = {m1, m2, m3, m4};
    estimateRPM(pw_now);

    // Convert raw gyro rad/s → deg/s, then notch-filter
    float gx_dps = gx_raw * 57.29578f;
    float gy_dps = gy_raw * 57.29578f;
    float gz_dps = gz_raw * 57.29578f;

    // Stage 1: Dynamic Notch — removes motor vibration frequencies
    float gx_notch = applyDynamicNotch(notchRoll,  gx_dps, fs_hz_est);
    float gy_notch = applyDynamicNotch(notchPitch, gy_dps, fs_hz_est);
    float gz_notch = applyDynamicNotch(notchYaw,   gz_dps, fs_hz_est);

    // Stage 2: PT1 Low-Pass — removes residual broadband noise
    // Two-stage filtering = clean signal for PID, minimal phase lag
    float gx_filt = pt1Roll.process (gx_notch, GYRO_LPF_HZ, dt);
    float gy_filt = pt1Pitch.process(gy_notch, GYRO_LPF_HZ, dt);
    float gz_filt = pt1Yaw.process  (gz_notch, GYRO_LPF_HZ, dt);

    // ── 4. GPS & RC ──────────────────────────────────
    while (GPS_Serial.available()) gps.encode(GPS_Serial.read());
    while (ibusReadFrame()) {}

    uint16_t thr = ibusGet(CH_THR);
    uint16_t ail = (uint16_t)iDeadband(ibusGet(CH_ROLL));
    uint16_t ele = (uint16_t)iDeadband(ibusGet(CH_PITCH));
    uint16_t rud = (uint16_t)iDeadband(ibusGet(CH_YAW));
    uint16_t sw  = ibusGet(CH_ARM);
    uint16_t swM = ibusGet(CH_MODE);

    // CH6 mode switch
    adaptiveMode = (swM > 1300);

    // onGround flag
    static const int THR_FLY = 1250;
    bool onGround = (!armed) || ((int)thr <= THR_FLY);
    pid_integrators_active = !onGround;
    // Mahony Kp: ينتقل بسلاسة من 2.0 إلى 0.4
    // القفز المفاجئ كان يُدخل صدمة في الكواترنيون عند الإقلاع
    float kp_target = onGround ? 2.0f : 0.4f;
    Kp_mahony += (kp_target - Kp_mahony) * 0.01f; // slew ~100 loops = 50ms

    // ── In-Flight Bias Estimator ──────────────────────
    // When all filtered rates are small the craft is hovering
    // quietly — residual reading equals thermal bias.
    // We track it with a very slow LP (τ≈10s) and subtract it
    // next iteration. Fixes drift that appears after 30+ sec flight.
    if (!onGround &&
        fabsf(gx_filt) < BIAS_STILL_TH &&
        fabsf(gy_filt) < BIAS_STILL_TH &&
        fabsf(gz_filt) < BIAS_STILL_TH)
    {
        // raw rad/s values, not deg/s
        gx_bias_if += BIAS_ALPHA * (evG.gyro.x - gx_offset - gx_bias_if);
        gy_bias_if += BIAS_ALPHA * (evG.gyro.y - gy_offset - gy_bias_if);
        gz_bias_if += BIAS_ALPHA * (evG.gyro.z - gz_offset - gz_bias_if);
        gx_bias_if = fclamp(gx_bias_if, -0.5f, 0.5f);
        gy_bias_if = fclamp(gy_bias_if, -0.5f, 0.5f);
        gz_bias_if = fclamp(gz_bias_if, -0.5f, 0.5f);
    }

    bool wantArm = (sw  > 1700);
    bool thrLow  = (thr < (uint16_t)(PWM_IDLE + 50));

    // ── 5. Arm / Disarm ──────────────────────────────
    if (wantArm && !armed && thrLow &&
        fabsf(roll_deg)  <= ARM_LIM &&
        fabsf(pitch_deg) <= ARM_LIM)
    {
        armed          = true;
        yawHoldTarget  = yaw_deg;
        altHoldTarget  = bme_alt_m;
        if (gps.location.isValid()) {
            homeLat = gps.location.lat();
            homeLon = gps.location.lng();
        }
        for (int us=PWM_MIN; us<=PWM_IDLE; us+=5) {
            esc1.writeMicroseconds(us);
            esc2.writeMicroseconds(us);
            esc3.writeMicroseconds(us);
            esc4.writeMicroseconds(us);
            delay(3);
        }
        pidAngR.reset();  pidAngP.reset();
        pidRateR.reset(); pidRateP.reset();
        pidYaw.reset();   pidAlt.reset();
        pidGpsX.reset();  pidGpsY.reset();
        // Reset in-flight bias — fresh start each flight
        gx_bias_if = gy_bias_if = gz_bias_if = 0.0f;
        // Reset gyro PT1 filters
        pt1Roll.reset(gx_filt);
        pt1Pitch.reset(gy_filt);
        pt1Yaw.reset(gz_filt);
        // Seed prevMeas with current values to prevent derivative
        // kick on the first PID iteration after arming
        pidAngR.prevMeas  = roll_deg;
        pidAngP.prevMeas  = pitch_deg;
        pidRateR.prevMeas = gx_filt;
        pidRateP.prevMeas = gy_filt;
        pidYaw.prevMeas   = gz_filt;
    }
    else if (!wantArm && armed) {
        armed = false;
        esc1.writeMicroseconds(PWM_MIN);
        esc2.writeMicroseconds(PWM_MIN);
        esc3.writeMicroseconds(PWM_MIN);
        esc4.writeMicroseconds(PWM_MIN);
    }

    // ── 6. BME280 update (50 ms) ─────────────────────
    static uint32_t lastBme = 0;
    if (bme_ok && (millis()-lastBme >= 50)) {
        lastBme      = millis();
        bme_temp_C   = bme.readTemperature();
        bme_pres_hPa = bme.readPressure() / 100.0f;
        bme_alt_m    = bme.readAltitude(SEALEVEL_HPA) - bme_alt0_m;
        bme_humidity = bme.readHumidity();
        updateAirModel(bme_temp_C, bme_pres_hPa);
        updateThrustTorque();
    }

    // ── 7. Control ───────────────────────────────────
    baseThrottle = constrain((int)thr, PWM_MIN, PWM_MAX);

    float spRoll  = fmap((float)ail, 1000,2000,
                         -MAX_ANGLE, +MAX_ANGLE);
    float spPitch = fmap((float)ele, 1000,2000,
                         -MAX_ANGLE, +MAX_ANGLE);
    float spYawR  = fmap((float)rud, 1000,2000,
                         -MAX_YAW_RATE, +MAX_YAW_RATE);

    // Altitude hold
    float altCorr = 0.0f;
    if (altHoldEnabled && bme_ok) {
        static float spAlt_prev = 0.0f;
        PIDConfig ca = scaledCfg(cfgAlt);
        altCorr = pidCompute(altHoldTarget, bme_alt_m,
                             pidAlt, dt, ca, spAlt_prev);
        spAlt_prev = altHoldTarget;
        baseThrottle = constrain(baseThrottle+(int)altCorr,
                                 PWM_MIN, PWM_MAX);
    }

    // GPS hold
    if (gpsHoldEnabled && gps.location.isValid()) {
        double dLat = (gps.location.lat()-homeLat)*111320.0;
        double dLon = (gps.location.lng()-homeLon)*111320.0
                      *cos(homeLat*(float)DEG_TO_RAD);
        static float spGX_prev=0, spGY_prev=0;
        PIDConfig cg = scaledCfg(cfgGps);
        float pc = pidCompute(0.0f,(float)dLat,
                              pidGpsX,dt,cg,spGX_prev);
        float rc = pidCompute(0.0f,(float)dLon,
                              pidGpsY,dt,cg,spGY_prev);
        spGX_prev=0; spGY_prev=0;
        spPitch += pc;
        spRoll  += rc;
    }

    // ── Ground Effect Compensation ────────────────────
    // تحت 40سم: الطائرة في منطقة Ground Effect — الهواء مضغوط تحتها
    // فوق 40سم: ديناميكيات طبيعية — نفس الـ Kp يصبح aggressive
    // الحل: نخفض KpAng تدريجياً كلما ارتفعت الطائرة
    // alt_m من BME280 نسبي من نقطة الإقلاع
    float gndFactor = 1.0f;
    if (bme_ok && !onGround) {
        float alt = fabsf(bme_alt_m);
        // من 0 إلى 0.4م: عامل يتراجع من 1.0 إلى 0.70
        if (alt < 0.4f)
            gndFactor = 1.0f - (alt / 0.4f) * 0.30f;
        else
            gndFactor = 0.70f;  // ثابت فوق 40سم
    }

    // Cascade PID – outer angle loop (مع تعديل Ground Effect)
    static float spR_prev=0.0f, spP_prev=0.0f;
    PIDConfig cAR = scaledCfg(cfgAngR);
    PIDConfig cAP = scaledCfg(cfgAngP);
    // طبّق Ground Effect factor على Kp الحلقة الخارجية فقط
    cAR.Kp *= gndFactor;
    cAP.Kp *= gndFactor;
    float rateR_sp = pidCompute(spRoll,  roll_deg,
                                pidAngR, dt, cAR, spR_prev);
    float rateP_sp = pidCompute(spPitch, pitch_deg,
                                pidAngP, dt, cAP, spP_prev);
    spR_prev = spRoll;
    spP_prev = spPitch;

    // Cascade PID – inner rate loop
    PIDConfig cRR = scaledCfg(cfgRateR);
    PIDConfig cRP = scaledCfg(cfgRateP);
    uRoll  =  pidCompute(rateR_sp, gx_filt,
                         pidRateR, dt, cRR, 0.0f);
    uPitch = -pidCompute(rateP_sp, gy_filt,
                         pidRateP, dt, cRP, 0.0f);

    // ── Yaw Control ───────────────────────────────────
    // مشكلة الـ drift: يawHoldTarget يتحرك مع كل اضطراب صغير
    // الحل: عندما العصا في المنتصف → نُرسل setpoint = 0 للـ rate
    //       أي drift في gz_filt يُصحَّح مباشرة بدون حاجة لـ heading
    static float yaw_sp_prev = 0.0f;
    float yaw_sp;
    if (fabsf(spYawR) > 2.0f) {
        // عصا Yaw متحركة → نتبع الأمر
        yawHoldTarget = yaw_deg;
        yaw_sp = spYawR;
    } else {
        // عصا في المنتصف → هدف معدل الدوران = صفر (يمنع drift)
        // أي عزم خارجي يُرى كـ gz_filt ≠ 0 ويُصحَّح بالـ Kp مباشرة
        float yawErr = yaw_deg - yawHoldTarget;
        while (yawErr >  180.0f) yawErr -= 360.0f;
        while (yawErr < -180.0f) yawErr += 360.0f;
        // ضبط بطيء للـ heading error عبر rate setpoint
        yaw_sp = -yawErr * 0.3f;
    }
    PIDConfig cY = scaledCfg(cfgYaw);
    uYaw = pidCompute(yaw_sp, gz_filt,
                      pidYaw, dt, cY, yaw_sp_prev);
    yaw_sp_prev = yaw_sp;

    // ── TILT SAFETY ───────────────────────────────────
    // إذا مالت > 50° → أوقف الموتورات فوراً
    if (armed && !onGround &&
        (fabsf(roll_deg) > 50.0f || fabsf(pitch_deg) > 50.0f)) {
        armed = false;
        esc1.writeMicroseconds(PWM_MIN);
        esc2.writeMicroseconds(PWM_MIN);
        esc3.writeMicroseconds(PWM_MIN);
        esc4.writeMicroseconds(PWM_MIN);
        Serial.println("[SAFETY] TILT > 50deg — DISARMED");
        return;
    }

    // ── 8. Mix, slew, clamp ───────────────────────────

    // GROUND RESONANCE FIX:
    if (onGround) {
        uRoll  = 0.0f;
        uPitch = 0.0f;
        uYaw   = 0.0f;
        yawHoldTarget = yaw_deg; // يمنع الالتفاف عند الإقلاع
    }
    // ── 8. Mixer + Smooth Takeoff ───────────────────
    // قبل كان في قفزة: على الأرض PID=0 ثم فجأة PID=100% عند THR_FLY.
    // الآن: mixAlpha يرفع سلطة الـ PID تدريجياً من 0→1.
    float mixAlpha = 0.0f;
    if (armed) {
        const float mixStart = (float)(PWM_IDLE + 20);
        const float mixFull  = (float)THR_FLY;
        mixAlpha = fclamp(((float)thr - mixStart) / (mixFull - mixStart), 0.0f, 1.0f);
    }

    int baseOut = armed ? max(baseThrottle, PWM_IDLE) : PWM_MIN;
    int t1, t2, t3, t4;
    mixMotors(baseOut, mixAlpha, t1, t2, t3, t4);

    // Slew rate limit (prevents sudden ESC commands)
    const int SLEW = 8;
    auto slew = [&](int target, int &current) {
        int d = target - current;
        if (d > SLEW) d = SLEW;
        else if (d < -SLEW) d = -SLEW;
        current += d;
    };
    slew(t1, m1); slew(t2, m2); slew(t3, m3); slew(t4, m4);
    m1 = constrain(m1, PWM_MIN, PWM_MAX);
    m2 = constrain(m2, PWM_MIN, PWM_MAX);
    m3 = constrain(m3, PWM_MIN, PWM_MAX);
    m4 = constrain(m4, PWM_MIN, PWM_MAX);
    // ── 9. Write ESC ──────────────────────────────────
    static uint32_t lastEsc = 0;
    if (micros()-lastEsc >= 1000) {
        lastEsc = micros();
        if (armed) {
            // onGround/air: mixAlpha يتحكم بمدى الـ PID؛ هنا فقط نكتب الـ PWM
            esc1.writeMicroseconds(constrain(m1, PWM_IDLE, PWM_MAX));
            esc2.writeMicroseconds(constrain(m2, PWM_IDLE, PWM_MAX));
            esc3.writeMicroseconds(constrain(m3, PWM_IDLE, PWM_MAX));
            esc4.writeMicroseconds(constrain(m4, PWM_IDLE, PWM_MAX));
        } else {
            esc1.writeMicroseconds(PWM_MIN);
            esc2.writeMicroseconds(PWM_MIN);
            esc3.writeMicroseconds(PWM_MIN);
            esc4.writeMicroseconds(PWM_MIN);
        }
}

    // ── 10. Telemetry 80 ms ───────────────────────────
    static uint32_t lastPrint = 0;
    if (millis()-lastPrint >= 80) {
        lastPrint = millis();

        float T_total = motor_thrust[0]+motor_thrust[1]
                       +motor_thrust[2]+motor_thrust[3];
        float Q_net   = motor_torque[0]-motor_torque[1]
                       +motor_torque[2]-motor_torque[3];

        float bx = isfinite(gx_bias_if) ? gx_bias_if : 0.0f;
        float by = isfinite(gy_bias_if) ? gy_bias_if : 0.0f;
        float bz = isfinite(gz_bias_if) ? gz_bias_if : 0.0f;

        char buf[512];
        snprintf(buf, sizeof(buf),
            "ARM=%s MODE=%s "
            "R/P/Y=%.1f/%.1f/%.1f "
            "gxyz=%.1f/%.1f/%.1f "
            "THR=%d M=%d,%d,%d,%d "
            "uRPY=%.0f,%.0f,%.0f "
            "T=%.1fC P=%.1f Alt=%.2fm H=%.1f%% "
            "rho=%.4f ratio=%.3f "
            "RPM=%d,%d,%d,%d "
            "Re=%.0f,%.0f,%.0f,%.0f "
            "Thr=%.3f,%.3f,%.3f,%.3f Tot=%.3f "
            "Tq=%.5f,%.5f,%.5f,%.5f Qnet=%.6f "
            "GPS=%s "
            "BiasIF=%.4f,%.4f,%.4f\n",
            armed?"ON":"OFF",
            adaptiveMode?"ADAPT":"FIX",
            roll_deg, pitch_deg, yaw_deg,
            gx_filt, gy_filt, gz_filt,
            baseThrottle, m1, m2, m3, m4,
            uRoll, uPitch, uYaw,
            bme_temp_C, bme_pres_hPa, bme_alt_m, bme_humidity,
            air_rho, air_rho_ratio,
            (int)motor_rpm[0],(int)motor_rpm[1],
            (int)motor_rpm[2],(int)motor_rpm[3],
            reynoldsNumber[0],reynoldsNumber[1],
            reynoldsNumber[2],reynoldsNumber[3],
            motor_thrust[0],motor_thrust[1],
            motor_thrust[2],motor_thrust[3], T_total,
            motor_torque[0],motor_torque[1],
            motor_torque[2],motor_torque[3], Q_net,
            gps.location.isValid()?"OK":"NO",
            bx, by, bz
        );

        Serial.print(buf);
        BT.print(buf);   // إرسال مباشر بدون hasClient() — مثل الكود الشغال
    }
}

