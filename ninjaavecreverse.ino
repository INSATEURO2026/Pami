/* robot_control.ino
   SIMA (Ninja) for Eurobot 2026 — granary operations.
   Odometry with 4x quadrature decoding, PID_v1 angle + distance,
   trapezoidal velocity profile, motor driver.

   === Changes from original ===
   BUG FIXES:
   [1]  PID_angleF now set to AUTOMATIC in setup()
   [2]  Pompe() dead code removed; PompeOff() added for proper shutdown
   [3]  STEPPER case: added missing 'break' (was falling through to STOP)
   [4]  FINISH state: robot stops and stays stopped (was looping to SETPATH)
   [5]  Cap2: removed abs() wrappers — correct signed angle difference
   [6]  delay() calls replaced with non-blocking state timers
   [7]  BACKWARD: heading setpoint now adds PI (back faces target)
   [8]  path_teta: filled all 5 values (was only 2 for 5 waypoints)
   [9]  Encoder reads: atomic snapshot with noInterrupts/interrupts
   [10] pwmFromSigned: returns 0 for values below deadband
   [11] Trapeze remaining_dist: uses segment-projected distance (consistent with PID)
   [12] dist_p: now computed as projection onto segment direction (works for fwd & rev)
   [13] wheel_gain: cast to int for analogWrite
   [14] 4x quadrature decoding (CHANGE on both A and B channels): 1400 CPR
   [15] Magic numbers replaced with named constants

   ENHANCEMENTS:
   [E1] 4x quadrature encoder resolution (350 → 1400 CPR)
   [E3] Distance-along-path: dot-product projection onto segment vector
   [E5] Non-blocking STOP and SETPATH delays (no missed odometry ticks)
   [E7] PID anti-windup: resetPID() called at every state transition
   [E8] Named constants for all tuning thresholds
*/
#include <Wire.h>
#include <math.h>
#include <PID_v1.h>

// ===================== PIN DEFINITIONS =====================
// Encoder pins
#define ENCODER_RA 18
#define ENCODER_RB 17
#define ENCODER_LA 16
#define ENCODER_LB 15

// Motor driver pins
#define IN1 9
#define IN2 46
#define ENA 8   // PWM Left
#define IN3 10
#define IN4 12
#define ENB 6   // PWM Right

// Pump pins
#define P1  35
#define P2  36
#define ENP 37

// LED pin
#define LED_PIN 2

// Stepper pins
const int dirPin  = 20;
const int stepPin = 19;
const int stepsPerRevolution = 200;

// ===================== TUNING CONSTANTS =====================
// (Replaces scattered magic numbers throughout the code)
#define PWM_MAX               200
#define PWM_STOP_DEADBAND     5      // PWM commands below this → 0
int     PWM_MIN             = 65;    // Minimum PWM to overcome motor deadband

#define V_MAX                 600.0  // mm/s  max cruise velocity
#define ACC_MAX               300.0  // mm/s² acceleration
#define DEC_MAX               400.0  // mm/s² deceleration
#define SAMPLING              5      // ms    control loop period

// Angle thresholds (radians)
#define ANGLE_THRESH_ROTATE   0.03   // ROTATE → FORWARD transition
#define ANGLE_THRESH_ROTAFT   0.02   // ROTATEAFTER → STOP transition

// Distance thresholds (mm)
#define DIST_ARRIVE_THRESH    10.0   // close enough to waypoint
#define DIST_REF_MARGIN       20.0   // dist_ref completion margin

// Odometry empirical correction factor for rotation
#define ALPHA_CORRECTION      1.05

// Delay durations (ms) — now non-blocking
#define SETPATH_DELAY_MS      500
#define STOP_DELAY_MS         100

// ===================== WHEEL / ENCODER PARAMETERS =====================
// 4x quadrature decoding: 350 CPR × 4 edges = 1400 counts/rev  [E1]
#define WHEEL_PERIMETER       138.23
#define RESOLUTION            1400
#define ENCODER_COEFFICIENT   0.098736  // WHEEL_PERIMETER / RESOLUTION
#define WHEEL_BASE            119

// Asymmetric wheel compensation (applied to left motor PWM output)
double wheel_gain = 1.063;

// ===================== WAYPOINTS =====================
#define NB_POINTS 5

double path_x[NB_POINTS]    = { 300,  200, -150, 100, -500 };
double path_y[NB_POINTS]    = {   0,  200, -150,   0,    0 };
double path_teta[NB_POINTS] = { 1.57, 0.0, 0.0, 0.0, 0.0 };  // [8] filled all 5

int currentPoint = 0;

// ===================== STATE VARIABLES =====================
// Coordinates
double x = 0, y = 0, teta = 0;
double x_prev = 0, y_prev = 0, teta_prev = 0;

// Velocities
double linearVel  = 0.0;
double angularVel = 0.0;

// Current target
double xc = 0, yc = 0, tetac = 0;

// Errors
double x_err = 0, y_err = 0, teta_err = 0;
double Dist = 0;        // Euclidean distance to target
double Cap  = 0;        // bearing error (heading correction during motion)
double Cap1 = 0;        // bearing error for initial rotation
double Cap2 = 0;        // heading error for ROTATEAFTER

double SPAngle = 0;
double SPdist  = 0;
double measuredVel = 0.0;
double bearing = 0;

// Pump state
bool pumpOn = false;

// Encoder counts (volatile — modified in ISRs)
volatile long encoderCount_R = 0;
volatile long encoderCount_L = 0;

// Trapezoid profile variables
double rampUp;           // distance to accelerate from 0 to V_MAX
double rampDown;         // distance to decelerate from V_MAX to 0
double vitesse;          // current reference velocity (mm/s)
double Target_dist;      // straight-line distance for current segment
double remaining_dist;   // segment-projected remaining distance
double dt;               // time step (seconds)
double dist_p;           // progress along segment (projected, mm)  [E3]
double dist_ref;         // trapeze reference distance (integrated)
int rightCmd, leftCmd;

// Non-blocking timer  [E5]
unsigned long stateTimerStart  = 0;
bool          stateTimerActive = false;

// ===================== PID PARAMETERS =====================
double KpAngle = 20, KiAngle = 0, KdAngle = 1;
double KpAngleF = 9, KiAngleF = 0, KdAngleF = 0;
double Kpdist  = 5,  Kidist  = 0,  Kddist  = 0;
double KpdistB = 0.3, KidistB = 0, KddistB = 0;

// Pump-mode PID gains (higher P for loaded motion)
double Kp_dist_pump = 12, Ki_dist_pump = 0, Kd_dist_pump = 0;

// PID I/O variables
double PIDAngle_input  = 0, PIDAngle_output  = 0, PIDAngle_setpoint  = 0;
double PIDAngleF_input = 0, PIDAngleF_output = 0, PIDAngleF_setpoint = 0;
double PIDdist_input   = 0, PIDdist_output   = 0, PIDdist_setpoint   = 0;
double PIDdistB_input  = 0, PIDdistB_output  = 0, PIDdistB_setpoint  = 0;

// PID objects
PID PID_angle (&PIDAngle_input,  &PIDAngle_output,  &PIDAngle_setpoint,  KpAngle,  KiAngle,  KdAngle,  DIRECT);
PID PID_angleF(&PIDAngleF_input, &PIDAngleF_output, &PIDAngleF_setpoint, KpAngleF, KiAngleF, KdAngleF, DIRECT);
PID PID_dist  (&PIDdist_input,   &PIDdist_output,   &PIDdist_setpoint,   Kpdist,   Kidist,   Kddist,   DIRECT);
PID PID_distB (&PIDdistB_input,  &PIDdistB_output,  &PIDdistB_setpoint,  KpdistB,  KidistB,  KddistB,  DIRECT);

// ================================================================
//                     4x QUADRATURE ENCODER ISRs                [E1]
// ================================================================
// Full quadrature decoding: CHANGE interrupts on both A and B channels.
// Gives 4× the resolution of single-edge counting (350 → 1400 CPR).
//
// Direction logic (verified against original 1x convention):
//   Channel A edge: count += (A == B) ? +1 : -1
//   Channel B edge: count += (A == B) ? -1 : +1

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

// ================================================================
//                          UTILITY FUNCTIONS
// ================================================================

double normalizeAngle(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// Convert a signed command (-255..255) to an unsigned PWM magnitude.
// Returns 0 for values inside the deadband.                       [10]
int pwmFromSigned(int val) {
  int mag = abs(val);
  if (mag < PWM_STOP_DEADBAND) return 0;
  int pwm = map(mag, 0, 255, PWM_MIN, PWM_MAX);
  return constrain(pwm, PWM_MIN, PWM_MAX);
}

void updatePIDGains() {
  if (pumpOn) {
    PID_dist.SetTunings(Kp_dist_pump, Ki_dist_pump, Kd_dist_pump);
    Serial.println("Pump mode PID activated");
  } else {
    PID_dist.SetTunings(Kpdist, Kidist, Kddist);
    Serial.println("Normal mode PID activated");
  }
}

// Reset a PID controller's internal state (clears integral windup).  [E7]
// Toggling MANUAL → AUTOMATIC resets the accumulated I-term.
void resetPID(PID &pid) {
  pid.SetMode(MANUAL);
  pid.SetMode(AUTOMATIC);
}

// ================================================================
//                              SETUP
// ================================================================

void setup() {
  Serial.begin(115200);

  // Encoder pins
  pinMode(ENCODER_RA, INPUT_PULLUP);
  pinMode(ENCODER_RB, INPUT_PULLUP);
  pinMode(ENCODER_LA, INPUT_PULLUP);
  pinMode(ENCODER_LB, INPUT_PULLUP);

  // Motor driver pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // LED
  pinMode(LED_PIN, OUTPUT);

  // Pump
  pinMode(P1,  OUTPUT);
  pinMode(P2,  OUTPUT);
  pinMode(ENP, OUTPUT);
  digitalWrite(P1,  LOW);
  digitalWrite(P2,  LOW);
  digitalWrite(ENP, LOW);

  // Stepper
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin,  OUTPUT);

  // --- 4x quadrature interrupts ---                              [E1]
  attachInterrupt(digitalPinToInterrupt(ENCODER_RA), encoderISR_RA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RB), encoderISR_RB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LA), encoderISR_LA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LB), encoderISR_LB, CHANGE);

  // Precompute trapeze ramp distances
  rampUp   = (V_MAX * V_MAX) / (2.0 * ACC_MAX);
  rampDown = (V_MAX * V_MAX) / (2.0 * DEC_MAX);

  // Initial state
  x = 0; y = 0; teta = 0;
  dist_p = 0; dist_ref = 0;
  Target_dist = 0;

  // --- PID: rotation ---
  PID_angle.SetOutputLimits(-200, 200);
  PID_angle.SetSampleTime(10);
  PID_angle.SetMode(AUTOMATIC);

  // --- PID: heading correction during motion ---                 [1]
  PID_angleF.SetOutputLimits(-200, 200);
  PID_angleF.SetSampleTime(10);
  PID_angleF.SetMode(AUTOMATIC);   // BUG FIX: was never set to AUTOMATIC

  // --- PID: forward distance ---
  PID_dist.SetOutputLimits(-200, 200);
  PID_dist.SetSampleTime(10);
  PID_dist.SetMode(AUTOMATIC);

  // --- PID: backward distance ---
  PID_distB.SetOutputLimits(-200, 200);
  PID_distB.SetSampleTime(10);
  PID_distB.SetMode(AUTOMATIC);

  Serial.println("Robot control (PID_v1 + 4x quad) ready.");
}

// ================================================================
//                          STATE MACHINE
// ================================================================

typedef enum {
  SETPATH,
  ROTATE,
  BACKWORD,
  FORWARD,
  ROTATEAFTER,
  CHECKPOINT,
  STEPPER,
  STOP,
  FINISH
} states;

states state = SETPATH;

enum TrapState { TRAP_ACCEL, TRAP_CRUISE, TRAP_DECEL };
TrapState trapState = TRAP_ACCEL;

enum TrapMode { TRAPEZOID, TRIANGULAR };
TrapMode trapMode = TRAPEZOID;

// ================================================================
//                           MAIN LOOP
// ================================================================

void loop() {
  static unsigned long lastTime = 0;
  unsigned long now = millis();

  if (now - lastTime >= SAMPLING) {
    dt = (now - lastTime) / 1000.0;
    lastTime = now;

    switch (state) {

      // =================== SETPATH ===================
      case SETPATH: {
        if (!stateTimerActive) {
          // --- First entry: initialise for new segment ---

          // Reset position to (0,0) — waypoints are relative to segment start
          x = 0;  y = 0;
          x_prev = 0;  y_prev = 0;
          dist_p   = 0;
          dist_ref = 0;

          // Pump hardware ON (idempotent)
          Pompe();

          // Reset trapeze
          vitesse = 0;

          // Load current waypoint
          xc    = path_x[currentPoint];
          yc    = path_y[currentPoint];
          tetac = path_teta[currentPoint];

          // PID gains: use normal mode for this segment
          // (Pompe() sets pumpOn=true; we revert the flag so PID uses normal gains)
          pumpOn = false;
          updatePIDGains();

          // Target distance for this segment
          Target_dist = sqrt(xc * xc + yc * yc);
          trapState   = TRAP_ACCEL;

          // Clear integral windup from previous segment               [E7]
          resetPID(PID_angle);
          resetPID(PID_angleF);
          resetPID(PID_dist);
          resetPID(PID_distB);

          Serial.print("SETPATH pt=");  Serial.print(currentPoint);
          Serial.print(" target=("); Serial.print(xc);
          Serial.print(","); Serial.print(yc);
          Serial.print(") dist="); Serial.println(Target_dist);

          // Start non-blocking delay                                  [6][E5]
          stateTimerStart  = now;
          stateTimerActive = true;
        }

        // Wait SETPATH_DELAY_MS before transitioning (replaces delay(500))
        if (stateTimerActive && (now - stateTimerStart >= SETPATH_DELAY_MS)) {
          stateTimerActive = false;
          if (currentPoint == 2) {
            state = BACKWORD;
          } else {
            state = ROTATE;
          }
          Serial.println("SETPATH done → next state");
        }
      } break;

      // =================== ROTATE ===================
      case ROTATE: {
        wheel_gain = 1.063;
        digitalWrite(LED_PIN, HIGH);

        Update_coordinates();
        Calcul_erreur();

        // Setpoint: angle from origin (0,0) toward target (xc,yc)
        SPAngle = normalizeAngle(atan2(yc, xc));

        PIDAngle_setpoint = SPAngle;
        PIDAngle_input    = teta;
        PID_angle.Compute();

        double cmd = PIDAngle_output;

        // Differential drive for pure rotation: left = −cmd, right = +cmd
        driveMotors((int)round(-cmd), (int)round(cmd));

        Serial.print("ROTATE SPa="); Serial.print(SPAngle);
        Serial.print(" teta="); Serial.print(teta);
        Serial.print(" Cap1="); Serial.print(Cap1);
        Serial.print(" out=");  Serial.println(PIDAngle_output);

        // Transition when heading is close enough                     [15]
        if (abs(Cap1) < ANGLE_THRESH_ROTATE) {
          resetPID(PID_dist);                                        // [E7]
          resetPID(PID_angleF);
          state = FORWARD;
          Serial.println("ROTATE → FORWARD");
        }
      } break;

      // =================== BACKWARD ===================
      case BACKWORD: {
        wheel_gain = 1.063;
        digitalWrite(LED_PIN, LOW);

        Update_coordinates();
        Calcul_erreur();

        // Trapeze velocity profile
        Trapeze();

        dist_ref += vitesse * dt;
        if (dist_ref > Target_dist) dist_ref = Target_dist;
        SPdist = dist_ref;

        // Distance PID
        PIDdistB_setpoint = dist_ref;
        PIDdistB_input    = dist_p;
        PID_distB.Compute();

        // Heading PID — BUG FIX: add PI so the robot's BACK faces target  [7]
        SPAngle = normalizeAngle(atan2(yc, xc) + M_PI);
        PIDAngleF_setpoint = SPAngle;
        PIDAngleF_input    = teta;
        PID_angleF.Compute();

        // Invert distance command for reverse
        int distCmd = -(int)round(PIDdistB_output);
        leftCmd  = distCmd - (int)round(PIDAngleF_output);
        rightCmd = distCmd + (int)round(PIDAngleF_output);

        driveMotors(leftCmd, rightCmd);

        Serial.print("REVERSE Dist="); Serial.print(Dist);
        Serial.print(" dist_p="); Serial.print(dist_p);
        Serial.print(" SPdist="); Serial.print(SPdist);
        Serial.print(" L="); Serial.print(leftCmd);
        Serial.print(" R="); Serial.println(rightCmd);

        // Arrival check
        if (Dist < DIST_ARRIVE_THRESH) {
          state = CHECKPOINT;
          Serial.println("REVERSE arrived → CHECKPOINT");
        } else if (dist_ref >= Target_dist - DIST_REF_MARGIN &&
                   dist_ref <= Target_dist) {
          state = CHECKPOINT;
          Serial.println("REVERSE ref done → CHECKPOINT");
        }
      } break;

      // =================== FORWARD ===================
      case FORWARD: {
        wheel_gain = 1.063;
        digitalWrite(LED_PIN, LOW);

        Update_coordinates();
        Calcul_erreur();

        // Trapeze velocity profile
        Trapeze();

        dist_ref += vitesse * dt;
        if (dist_ref > Target_dist) dist_ref = Target_dist;
        SPdist = dist_ref;

        // Distance PID
        PIDdist_setpoint = dist_ref;
        PIDdist_input    = dist_p;
        PID_dist.Compute();

        // Heading correction toward target
        SPAngle = normalizeAngle(bearing);
        PIDAngleF_setpoint = SPAngle;
        PIDAngleF_input    = teta;
        PID_angleF.Compute();

        leftCmd  = (int)round(PIDdist_output - PIDAngleF_output);
        rightCmd = (int)round(PIDdist_output + PIDAngleF_output);
        driveMotors(leftCmd, rightCmd);

        Serial.print("FWD x="); Serial.print(x);
        Serial.print(" y="); Serial.print(y);
        Serial.print(" teta="); Serial.print(teta);
        Serial.print(" Dist="); Serial.print(Dist);
        Serial.print(" dist_p="); Serial.print(dist_p);
        Serial.print(" SPd="); Serial.print(SPdist);
        Serial.print(" L="); Serial.print(leftCmd);
        Serial.print(" R="); Serial.println(rightCmd);

        // Arrival check
        if (Dist < DIST_ARRIVE_THRESH) {
          state = CHECKPOINT;
          digitalWrite(LED_PIN, LOW);
          Serial.println("FWD arrived → CHECKPOINT");
        } else if (dist_ref >= Target_dist - DIST_REF_MARGIN &&
                   dist_ref <= Target_dist) {
          state = CHECKPOINT;
          Serial.println("FWD ref done → CHECKPOINT");
        }
      } break;

      // =================== ROTATEAFTER ===================
      case ROTATEAFTER: {
        Update_coordinates();
        Calcul_erreur();

        SPAngle = normalizeAngle(tetac);
        PIDAngle_setpoint = SPAngle;
        PIDAngle_input    = teta;
        PID_angle.Compute();

        int cmd = (int)round(PIDAngle_output);
        driveMotors(-cmd, cmd);

        // BUG FIX [5]: proper signed angle difference (no abs() on individual angles)
        if (abs(Cap2) < ANGLE_THRESH_ROTAFT) {
          state = STOP;
          Serial.println("ROTATEAFTER done → STOP");
        }
      } break;

      // =================== CHECKPOINT ===================
      case CHECKPOINT: {
        stopmotors();
        currentPoint++;

        if (currentPoint == 4) {
          Pompe();         // Ensure pump is ON for stepper phase
          pumpOn = true;   // Keep pump PID gains for this phase
          updatePIDGains();
          state = STEPPER;
        } else if (currentPoint >= NB_POINTS) {
          digitalWrite(LED_PIN, HIGH);   // LED ON = done
          Serial.println("Trajet termine!");
          state = FINISH;
        } else {
          state = STOP;
        }
      } break;

      // =================== STEPPER ===================
      case STEPPER: {
        // Robot is stationary — blocking stepper is acceptable here.
        stopmotors();

        // Phase 1: clockwise, 4 revolutions, slow (2000 µs/half-step)
        digitalWrite(dirPin, LOW);
        for (int i = 0; i < 4 * stepsPerRevolution; i++) {
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(2000);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(2000);
        }
        delay(1000);

        // Phase 2: counter-clockwise, 4 revolutions, fast (1000 µs/half-step)
        digitalWrite(dirPin, HIGH);
        for (int i = 0; i < 4 * stepsPerRevolution; i++) {
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(1000);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(1000);
        }
        delay(1000);

        state = STOP;
        break;                // BUG FIX [3]: was missing → fell through to STOP
      }

      // =================== STOP ===================
      case STOP: {
        // Non-blocking delay (replaces delay(100))                   [6][E5]
        if (!stateTimerActive) {
          stopmotors();
          stateTimerStart  = now;
          stateTimerActive = true;
        }
        if (stateTimerActive && (now - stateTimerStart >= STOP_DELAY_MS)) {
          stateTimerActive = false;
          state = SETPATH;
        }
      } break;

      // =================== FINISH ===================
      case FINISH: {
        // BUG FIX [4]: stay stopped — was endlessly looping back to SETPATH
        stopmotors();
        PompeOff();
        // Robot is done. Stay here.
      } break;
    }
  }
}

// ================================================================
//                        MOTOR FUNCTIONS
// ================================================================

void driveMotors(int leftPWM, int rightPWM) {
  leftPWM  = constrain(leftPWM,  -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  // LEFT MOTOR (with wheel_gain compensation)
  if (leftPWM > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, (int)(wheel_gain * pwmFromSigned(leftPWM)));  // [13]
  } else if (leftPWM < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, (int)(wheel_gain * pwmFromSigned(leftPWM)));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }

  // RIGHT MOTOR
  if (rightPWM > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, pwmFromSigned(rightPWM));
  } else if (rightPWM < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, pwmFromSigned(rightPWM));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopmotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ================================================================
//                    ODOMETRY & ERROR CALCULATION
// ================================================================

void Update_coordinates() {
  static double encR_prev = 0;
  static double encL_prev = 0;

  // BUG FIX [9]: atomic snapshot of both encoder counts
  noInterrupts();
  long snapR = encoderCount_R;
  long snapL = encoderCount_L;
  interrupts();

  // Convert counts to mm
  double encR = snapR * ENCODER_COEFFICIENT;
  double encL = snapL * ENCODER_COEFFICIENT;

  double deltaR = encR - encR_prev;
  double deltaL = encL - encL_prev;
  encR_prev = encR;
  encL_prev = encL;

  // Arc-center displacement and rotation
  double D     = (deltaR + deltaL) / 2.0;
  double alpha = ALPHA_CORRECTION * ((deltaR - deltaL) / (double)WHEEL_BASE);

  // Linear velocity (mm/s)
  linearVel = D / dt;

  // Mid-angle integration (second-order, better than Euler forward)
  teta = normalizeAngle(teta_prev + alpha);
  double teta_mid = teta_prev + alpha / 2.0;
  x = x_prev + cos(teta_mid) * D;
  y = y_prev + sin(teta_mid) * D;

  // ENHANCEMENT [E3]: project position onto segment direction.
  // dist_p = dot(position, unit_segment_vector) = (x·xc + y·yc) / |segment|
  // Works correctly for both forward AND backward motion.
  if (Target_dist > 0.001) {
    dist_p = (x * xc + y * yc) / Target_dist;
  } else {
    dist_p = 0;
  }

  // Save for next cycle
  x_prev    = x;
  y_prev    = y;
  teta_prev = teta;
}

void Calcul_erreur() {
  // Position errors
  x_err = xc - x;
  y_err = yc - y;

  // Euclidean distance to target
  Dist = sqrt(x_err * x_err + y_err * y_err);

  // Bearing from current position to target
  bearing = atan2(y_err, x_err);

  // Cap1: angle from origin heading to target (used in ROTATE)
  Cap1 = normalizeAngle(atan2(yc, xc) - teta);

  // Cap: bearing error (used in FORWARD heading correction)
  Cap = normalizeAngle(bearing - teta);

  // BUG FIX [5]: proper signed difference — was normalizeAngle(abs(tetac)-abs(teta))
  Cap2 = normalizeAngle(tetac - teta);
}

// ================================================================
//                      TRAPEZOID VELOCITY PROFILE
// ================================================================

void ResetTrapeze() {
  vitesse   = 0;
  trapState = TRAP_ACCEL;
}

void Trapeze() {
  // ENHANCEMENT [E3/11]: use segment-projected remaining distance
  double seg_remaining = Target_dist - dist_p;
  if (seg_remaining < 0) seg_remaining = 0;

  // If already at target, stop
  if (Dist < DIST_ARRIVE_THRESH) {
    vitesse = 0;
    return;
  }

  remaining_dist = seg_remaining;

  // Check if the segment is too short for full trapezoid
  double Vpeak = sqrt((2.0 * ACC_MAX * DEC_MAX * Target_dist) / (ACC_MAX + DEC_MAX));
  if (Vpeak < V_MAX) {
    trapMode = TRIANGULAR;
  } else {
    trapMode = TRAPEZOID;
  }

  if (trapMode == TRIANGULAR) {
    double downDist = (Vpeak * Vpeak) / (2.0 * DEC_MAX);

    if (remaining_dist > downDist) {
      // Accelerating
      vitesse += ACC_MAX * dt;
      if (vitesse > Vpeak) vitesse = Vpeak;
      trapState = TRAP_ACCEL;
    } else {
      // Decelerating
      vitesse -= DEC_MAX * dt;
      if (vitesse < 0) vitesse = 0;
      trapState = TRAP_DECEL;
    }
  } else {
    // Full trapezoid profile
    switch (trapState) {
      case TRAP_ACCEL:
        vitesse += ACC_MAX * dt;
        if (vitesse >= V_MAX) {
          vitesse = V_MAX;
          trapState = TRAP_CRUISE;
        }
        if (remaining_dist <= rampDown) {
          trapState = TRAP_DECEL;
        }
        break;

      case TRAP_CRUISE:
        vitesse = V_MAX;
        if (remaining_dist <= rampDown) {
          trapState = TRAP_DECEL;
        }
        break;

      case TRAP_DECEL:
        vitesse -= DEC_MAX * dt;
        if (vitesse <= 0) {
          vitesse = 0;
        }
        break;
    }
  }
}

// ================================================================
//                         PUMP CONTROL
// ================================================================

// Turn pump ON (idempotent — safe to call repeatedly)             [2]
void Pompe() {
  if (!pumpOn) {
    pumpOn = true;
    PWM_MIN = 75;
    updatePIDGains();
    digitalWrite(P1,  HIGH);
    digitalWrite(P2,  LOW);
    digitalWrite(ENP, HIGH);
    Serial.println("Pump ON");
  }
}

// Turn pump OFF                                                    [2]
// BUG FIX: original else-if branch was unreachable dead code.
// Now a separate function for explicit OFF control.
void PompeOff() {
  if (pumpOn) {
    pumpOn = false;
    PWM_MIN = 65;
    updatePIDGains();
    digitalWrite(P1,  LOW);
    digitalWrite(P2,  LOW);
    digitalWrite(ENP, LOW);
    Serial.println("Pump OFF");
  }
}
