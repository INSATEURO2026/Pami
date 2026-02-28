/* encoder_tuning.ino
   Standalone encoder & odometry test for the SIMA (Ninja).
   
   PURPOSE:
   Since encoders and motors share the same power rail, the motors
   will be energized when the board is powered. This script explicitly
   BRAKES all motors at startup and keeps them braked, so you can
   safely push the robot by hand while reading encoder values.

   WHAT IT DOES:
   - Forces all motor outputs LOW and PWM to 0 (hard brake).
   - Reads encoders using 4x quadrature decoding.
   - Computes odometry (x, y, teta, dist_p) every 50ms.
   - Prints everything to Serial at 115200 baud.
   - Also provides interactive Serial commands for calibration.

   SERIAL COMMANDS (type in Serial Monitor, press Enter):
   r  — Reset odometry to (0, 0, 0)
   d  — Print raw encoder counts (for debugging direction)
   c  — Print current calibration constants
   h  — Print help

   HOW TO USE:
   1. Flash this sketch to the ESP32.
   2. Open Serial Monitor at 115200.
   3. Push the robot forward exactly 1000mm → read 'x'.
   4. Rotate the robot exactly 360° on the spot → read 'teta' (should be ~0).
   5. Adjust ENCODER_COEFFICIENT and ALPHA_CORRECTION accordingly.
   6. Use 'r' to reset between tests.
*/

#include <math.h>

// ===================== PIN DEFINITIONS =====================
// Encoder pins (same as main code)
#define ENCODER_RA 18
#define ENCODER_RB 17
#define ENCODER_LA 16
#define ENCODER_LB 15

// Motor driver pins — we set these to brake the motors
#define IN1 9
#define IN2 46
#define ENA 8
#define IN3 10
#define IN4 12
#define ENB 6

// Pump pins — force OFF
#define P1  35
#define P2  36
#define ENP 37

// Stepper pins — force OFF
#define DIR_PIN  20
#define STEP_PIN 19

// ===================== CALIBRATION CONSTANTS =====================
// >>> EDIT THESE to match your robot, then reflash <<<

// 4x quadrature: 350 CPR × 4 = 1400 counts/rev
#define WHEEL_PERIMETER       138.23    // mm (measure your wheel!)
#define RESOLUTION            1400      // counts per revolution (4x mode)
#define ENCODER_COEFFICIENT   0.098736  // WHEEL_PERIMETER / RESOLUTION
#define WHEEL_BASE            119.0     // mm  (center-to-center of wheels)
#define ALPHA_CORRECTION      1.05      // rotation correction factor

// Print interval
#define PRINT_INTERVAL_MS     50        // print odometry every 50ms

// ===================== ENCODER VARIABLES =====================
volatile long encoderCount_R = 0;
volatile long encoderCount_L = 0;

// ===================== ODOMETRY STATE =====================
double x = 0, y = 0, teta = 0;
double x_prev = 0, y_prev = 0, teta_prev = 0;
double dist_p = 0;       // cumulative forward distance
double linearVel = 0;
double encR_prev_mm = 0, encL_prev_mm = 0;

// Timing
unsigned long lastPrint = 0;
unsigned long lastOdom  = 0;

// ===================== 4x QUADRATURE ISRs =====================
void IRAM_ATTR encoderISR_RA() {
  encoderCount_R += (digitalRead(ENCODER_RA) == digitalRead(ENCODER_RB)) ? 1 : -1;
}
void IRAM_ATTR encoderISR_RB() {
  encoderCount_R += (digitalRead(ENCODER_RA) == digitalRead(ENCODER_RB)) ? -1 : 1;
}
void IRAM_ATTR encoderISR_LA() {
  encoderCount_L += (digitalRead(ENCODER_LA) == digitalRead(ENCODER_LB)) ? 1 : -1;
}
void IRAM_ATTR encoderISR_LB() {
  encoderCount_L += (digitalRead(ENCODER_LA) == digitalRead(ENCODER_LB)) ? -1 : 1;
}

// ===================== UTILITY =====================
double normalizeAngle(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// ===================== FORCE MOTORS OFF =====================
void brakeAllMotors() {
  // Left motor: brake (both LOW) + PWM 0
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  // Right motor: brake (both LOW) + PWM 0
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);

  // Pump OFF
  digitalWrite(P1, LOW);
  digitalWrite(P2, LOW);
  digitalWrite(ENP, LOW);

  // Stepper OFF
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
}

// ===================== RESET ODOMETRY =====================
void resetOdometry() {
  noInterrupts();
  encoderCount_R = 0;
  encoderCount_L = 0;
  interrupts();

  x = 0;  y = 0;  teta = 0;
  x_prev = 0;  y_prev = 0;  teta_prev = 0;
  dist_p = 0;
  encR_prev_mm = 0;
  encL_prev_mm = 0;
  linearVel = 0;

  Serial.println(">>> Odometry RESET to (0, 0, 0)");
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }  // wait for USB serial

  Serial.println("==============================================");
  Serial.println("  SIMA Encoder Tuning & Odometry Test");
  Serial.println("  Motors are BRAKED — push the robot by hand");
  Serial.println("==============================================");
  Serial.println("Commands: r=reset, d=raw counts, c=constants, h=help");
  Serial.println();

  // --- Motor pins: OUTPUT, then immediately brake ---
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(P1,  OUTPUT);
  pinMode(P2,  OUTPUT);
  pinMode(ENP, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);

  brakeAllMotors();   // <-- CRITICAL: motors braked before encoders start

  // --- Encoder pins ---
  pinMode(ENCODER_RA, INPUT_PULLUP);
  pinMode(ENCODER_RB, INPUT_PULLUP);
  pinMode(ENCODER_LA, INPUT_PULLUP);
  pinMode(ENCODER_LB, INPUT_PULLUP);

  // --- 4x quadrature interrupts ---
  attachInterrupt(digitalPinToInterrupt(ENCODER_RA), encoderISR_RA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RB), encoderISR_RB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LA), encoderISR_LA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LB), encoderISR_LB, CHANGE);

  lastPrint = millis();
  lastOdom  = millis();

  Serial.println("Ready. Push the robot to see encoder data.\n");
}

// ===================== MAIN LOOP =====================
void loop() {
  unsigned long now = millis();

  // --- Ensure motors remain braked every loop (safety) ---
  // This costs almost nothing and guarantees motors never turn on
  // even if some other code or noise toggles a pin.
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  // --- Update odometry at ~200 Hz (every 5ms) ---
  if (now - lastOdom >= 5) {
    double dt = (now - lastOdom) / 1000.0;
    lastOdom = now;

    // Atomic snapshot
    noInterrupts();
    long snapR = encoderCount_R;
    long snapL = encoderCount_L;
    interrupts();

    // Convert to mm
    double encR_mm = snapR * ENCODER_COEFFICIENT;
    double encL_mm = snapL * ENCODER_COEFFICIENT;

    double deltaR = encR_mm - encR_prev_mm;
    double deltaL = encL_mm - encL_prev_mm;
    encR_prev_mm = encR_mm;
    encL_prev_mm = encL_mm;

    // Forward displacement & rotation
    double D     = (deltaR + deltaL) / 2.0;
    double alpha = ALPHA_CORRECTION * ((deltaR - deltaL) / WHEEL_BASE);

    linearVel = (dt > 0) ? (D / dt) : 0;

    // Mid-angle integration
    teta = normalizeAngle(teta_prev + alpha);
    double teta_mid = teta_prev + alpha / 2.0;
    x = x_prev + cos(teta_mid) * D;
    y = y_prev + sin(teta_mid) * D;
    dist_p += D;

    x_prev    = x;
    y_prev    = y;
    teta_prev = teta;
  }

  // --- Print at PRINT_INTERVAL_MS ---
  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;

    // Only print if the robot has moved (avoid flooding)
    static double last_printed_dist = 0;
    if (abs(dist_p - last_printed_dist) > 0.1) {  // 0.1mm threshold
      last_printed_dist = dist_p;

      Serial.print("x=");        Serial.print(x, 1);
      Serial.print(" mm  y=");   Serial.print(y, 1);
      Serial.print(" mm  teta="); Serial.print(teta * 180.0 / M_PI, 2);
      Serial.print(" deg (");    Serial.print(teta, 4);
      Serial.print(" rad)  dist="); Serial.print(dist_p, 1);
      Serial.print(" mm  vel="); Serial.print(linearVel, 0);
      Serial.println(" mm/s");
    }
  }

  // --- Handle Serial commands ---
  if (Serial.available()) {
    char c = Serial.read();

    switch (c) {
      case 'r':
      case 'R':
        resetOdometry();
        break;

      case 'd':
      case 'D': {
        noInterrupts();
        long rr = encoderCount_R;
        long ll = encoderCount_L;
        interrupts();
        Serial.print("Raw counts → R="); Serial.print(rr);
        Serial.print("  L="); Serial.print(ll);
        Serial.print("  (R mm="); Serial.print(rr * ENCODER_COEFFICIENT, 2);
        Serial.print("  L mm="); Serial.print(ll * ENCODER_COEFFICIENT, 2);
        Serial.println(")");
      } break;

      case 'c':
      case 'C':
        Serial.println("--- Current calibration ---");
        Serial.print("  WHEEL_PERIMETER     = "); Serial.print(WHEEL_PERIMETER); Serial.println(" mm");
        Serial.print("  RESOLUTION (4x)     = "); Serial.println(RESOLUTION);
        Serial.print("  ENCODER_COEFFICIENT = "); Serial.println(ENCODER_COEFFICIENT, 6);
        Serial.print("  WHEEL_BASE          = "); Serial.print(WHEEL_BASE); Serial.println(" mm");
        Serial.print("  ALPHA_CORRECTION    = "); Serial.println(ALPHA_CORRECTION, 4);
        Serial.println("---------------------------");
        break;

      case 'h':
      case 'H':
        Serial.println("Commands:");
        Serial.println("  r — Reset odometry to (0,0,0)");
        Serial.println("  d — Print raw encoder counts");
        Serial.println("  c — Print calibration constants");
        Serial.println("  h — This help");
        break;

      default:
        break;  // ignore newlines, spaces, etc.
    }
  }
}
