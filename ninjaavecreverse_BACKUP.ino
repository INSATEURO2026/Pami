/* robot_control.ino
   Implémentation complète : PID_v1 angle + distance, odométrie, trapeze, moteurs.
*/
#include <Wire.h>


#include <math.h>
#include <PID_v1.h>


// --- Encoder pins ---
#define ENCODER_RA 18
#define ENCODER_RB 17
#define ENCODER_LA 16
#define ENCODER_LB 15

#define IN1 9
#define IN2 46
#define ENA 8   // PWM Left
#define IN3 10
#define IN4 12
#define ENB 6  // PWM Right
#define REVERSE_THRESHOLD 1.57
//ponmpe
#define P1 35
#define P2 36
#define ENP 37
int PWM_MIN= 65;

#define PWM_MAX 200
#define base_speed 50
#define V_MAX 600
#define ACC_MAX 300
#define DEC_MAX 400
#define SAMPLING 5     // ms
#define PWM_STOP_DEADBAND 5
#define NB_POINTS 5

double path_x[NB_POINTS] ={300,200,-150,100,-500};//{300,250,0,200
double path_y[NB_POINTS] = {0,200,-150,0,0};// {0,250,-200,0}
double path_teta[NB_POINTS] = {1.57, 0.0};

int currentPoint = 0;

bool useReverse = false;
//stepper
const int dirPin = 20;
const int stepPin = 19;
const int stepsPerRevolution = 200;

//---------------- Coordinates ------------------
double x = 0, y = 0, teta = 0;
double x_prev = 0, y_prev = 0, teta_prev = 0;

//---------------- Velocities ------------------
double linearVel;
double angularVel = 0.0;

//--------------- Target -------------
double xc ;
double yc ;  
double tetac ;

double x_err = 0;
double y_err = 0;
double teta_err = 0;

double Dist = 0;
double Cap = 0;
double Cap1 = 0;
double Cap2 = 0;

double SPAngle = 0;
double SPdist = 0;
int leftPWM=0;
double measuredVel = 0.0;
int leftt ;
// Wheel parameters
#define WHEEL_PERIMETER 138.23
#define RESOLUTION 350
#define ENCODER_COEFFICIENT 0.39494
#define WHEEL_BASE 119
bool pumpOn = false;

double wheel_gain=1.063;//0.84

volatile long encoderCount_R = 0;
volatile long encoderCount_L = 0;

// trapeze vars
double rampUp;
double rampDown;
double vitesse;
double Target_dist;
double remaining_dist = 0;
double bearing ;
double dt ; // seconds
double dist_p;
double dist_ref;
int rightCmd;
int leftCmd;
//==================== PID PARAMETERS ==================
double KpAngle =20, KiAngle =0, KdAngle =1;//20 0 1
double KpAngleF =9, KiAngleF= 0, KdAngleF =0;//9 0 0
double Kpdist=5, Kidist= 0.0, Kddist  =0;//5
double KpdistB=0.3, KidistB= 0.0, KddistB  =0;//5

// ----- PUMP MODE -----
double Kp_dist_pump = 12;   // example higher P
double Ki_dist_pump = 0;
double Kd_dist_pump = 0;
// PID INPUTS / OUTPUTS
double PIDAngle_input = 0, PIDAngle_output = 0, PIDAngle_setpoint = 0;
double PIDAngleF_input = 0, PIDAngleF_output = 0, PIDAngleF_setpoint = 0;
double PIDdist_input = 0,  PIDdist_output = 0,  PIDdist_setpoint = 0;
double PIDdistB_input = 0,  PIDdistB_output = 0,  PIDdistB_setpoint = 0;

//---------- PID OBJECTS ---------
PID PID_angle(&PIDAngle_input, &PIDAngle_output, &PIDAngle_setpoint,KpAngle, KiAngle, KdAngle, DIRECT);
PID PID_angleF(&PIDAngleF_input, &PIDAngleF_output, &PIDAngleF_setpoint,KpAngleF, KiAngleF, KdAngleF, DIRECT);

PID PID_dist(&PIDdist_input, &PIDdist_output, &PIDdist_setpoint,Kpdist, Kidist, Kddist, DIRECT);
PID PID_distB(&PIDdistB_input, &PIDdistB_output, &PIDdistB_setpoint,KpdistB, KidistB, KddistB, DIRECT);


//================ ENCODERS ISR ======================
void IRAM_ATTR encoderISR_R() {
  // On rising edge of encoder A right, read B to determine direction
  encoderCount_R += (digitalRead(ENCODER_RB) == HIGH ? 1 : -1);
}

void IRAM_ATTR encoderISR_L() {
  encoderCount_L += (digitalRead(ENCODER_LB) == HIGH ? 1 : -1);
}


//============== UTILS ===================
double normalizeAngle(double a) {
  while (a > M_PI)  a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

int pwmFromSigned(int val) {
  // val in -255..255 -> return pwm value to send to analogWrite (0..255)
  int mag = abs(val);
  //if (mag < PWM_STOP_DEADBAND) return 0;
  // map magnitude to [PWM_MIN, PWM_MAX] to overcome motor deadband
  int pwm = map(mag, 0, 255, PWM_MIN, PWM_MAX);
  //pwm=constrain(pwm, PWM_MIN, PWM_MAX);
  return pwm;
}
void updatePIDGains() {

  if (pumpOn) {
    PID_dist.SetTunings(Kp_dist_pump, Ki_dist_pump, Kd_dist_pump);
    Serial.println("Pump mode PID activated");
  } 
  else {
    PID_dist.SetTunings(Kpdist, Kidist, Kddist);
    Serial.println("Normal mode PID activated");
  }
}

//============== ROBOT INIT ==================
void setup() {
  Serial.begin(115200);



  pinMode(ENCODER_RA, INPUT_PULLUP);
  pinMode(ENCODER_RB, INPUT_PULLUP);
  pinMode(ENCODER_LA, INPUT_PULLUP);
  pinMode(ENCODER_LB, INPUT_PULLUP);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // PWM
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(2,OUTPUT);
  //Pompe
  pinMode(P1, OUTPUT);
  pinMode(P2, OUTPUT);
  pinMode(ENP, OUTPUT);

  //stepper
	pinMode(stepPin, OUTPUT);
	pinMode(dirPin, OUTPUT);

  // Pompe OFF au début
  digitalWrite(P1, LOW);
  digitalWrite(P2, LOW);
  digitalWrite(ENP, LOW);
 

  attachInterrupt(digitalPinToInterrupt(ENCODER_RA), encoderISR_R, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LA), encoderISR_L, RISING);
 

  // compute trapeze precomputed distances
  rampUp = (V_MAX * V_MAX) / (2.0 * ACC_MAX);
  rampDown = (V_MAX*V_MAX)/(2.0*DEC_MAX);

  // initialize target distance (euclid)
  x = 0;
  y = 0;
  teta = 0;
  dist_p   = 0;
  dist_ref = 0;
 // xc = path_x[currentPoint];
 // yc = path_y[currentPoint];
  //tetac = path_teta[currentPoint];
  Target_dist = sqrt(xc*xc+ yc*yc);

  //=========== PID ANGLE ============
  PID_angle.SetOutputLimits(-200, 200); // PID outputs in this range
  PID_angle.SetSampleTime(10);
  PID_angle.SetMode(AUTOMATIC);

  //=========== PID DIST ============
  PID_dist.SetOutputLimits(-200, 200);
  PID_dist.SetSampleTime(10);
  PID_dist.SetMode(AUTOMATIC);

  PID_distB.SetOutputLimits(-200, 200);
  PID_distB.SetSampleTime(10);
  PID_distB.SetMode(AUTOMATIC);
  Serial.println("Robot control (PID_v1) ready.");
  pinMode(2, OUTPUT);
  

}


//============== MAIN LOOP ==================
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
enum TrapState { 
  TRAP_ACCEL, 
  TRAP_CRUISE, 
  TRAP_DECEL
  };
TrapState trapState = TRAP_ACCEL;
enum TrapMode { 
  TRAPEZOID, 
  TRIANGULAR 
};
TrapMode trapMode = TRAPEZOID;

void loop() {
  
  static unsigned long lastTime = 0;
  unsigned long now = millis();
  if (now - lastTime >= SAMPLING) {
    
    dt = (now - lastTime) / 1000.0;
    lastTime = now;
    
    switch(state) {
      case SETPATH: {
        //PID_dist.SetMode(AUTOMATIC);
        //PID_angle.SetMode(AUTOMATIC);

        // 🔹 Initialisation pose
        x = 0;
        y = 0;
        //teta = 0;
        dist_p   = 0;
        dist_ref = 0;

        x_prev = 0;
        y_prev = 0;
        //teta_prev = 0;

        // 🔹 Reset encodeurs
       // noInterrupts();
        //encoderCount_L = 0;
        //encoderCount_R = 0;
        //interrupts();
        Pompe();
        // 🔹 Reset trapèze
       vitesse = 0;
       // charger la cible courante
        xc = path_x[currentPoint];
        yc = path_y[currentPoint];
        tetac = path_teta[currentPoint];
        pumpOn = false;
        // 🔹 Calcul distance cible (IMPORTANT)
        Target_dist = sqrt(xc*xc + yc*yc);
        trapState = TRAP_ACCEL;
        Serial.println("SETPATH done → ROTATE");
        delay(500);
        if(currentPoint==2)
        {
          state=BACKWORD;
        }
        else
          state = ROTATE;
      }
      break;
      //=================== ROTATE ===================
      case ROTATE: {
      wheel_gain=1.063;
       digitalWrite(2,HIGH);
       Update_coordinates();
       Calcul_erreur();
       // setpoint toward goal
        SPAngle = normalizeAngle(atan2(yc, xc));

        double angleError = normalizeAngle(SPAngle - teta);
        PIDAngle_setpoint = SPAngle;
        PIDAngle_input = teta;
        PID_angle.Compute();
        double cmd = PIDAngle_output;
        leftt=pwmFromSigned(leftPWM);
        // drive for rotation: left = +cmd, right = -cmd
        driveMotors(-cmd, cmd);
        Serial.print(" spangle "); Serial.print(SPAngle);
        Serial.print("  |x: "); Serial.print(x);
        Serial.print("  |y: "); Serial.print(y);
        Serial.print("  |xc: "); Serial.print(xc);
        Serial.print("  |yc: "); Serial.print(yc);
        Serial.print("  |teta: "); Serial.print(teta);
        Serial.print("  leftt"); Serial.print(leftt);
        Serial.print("ROTATE outA="); Serial.print(PIDAngle_output);
        Serial.print("   Cap1="); Serial.println(Cap1);
        

        // threshold to go forward (Cap close to zero)
        if (abs(Cap1) < 0.03) {
         state = FORWARD;  
          //digitalWrite(2,HIGH); // ~0.08 rad ~4.6 deg
          //state = FORWARD;
          Serial.println("Switching to FORWARD");
        }
      }
      break;
      case BACKWORD: {
      wheel_gain = 1.063;
      digitalWrite(2, LOW); // LED OFF, comme dans FORWARD
      Update_coordinates();
      Calcul_erreur();

      // Trapeze pour vitesse
      Trapeze();
      
      dist_ref += vitesse * dt;
      if (dist_ref > Target_dist) dist_ref = Target_dist;
      SPdist = dist_ref;

      // PID distance : on recule → inversion
      PIDdistB_setpoint = dist_ref;
      PIDdistB_input = dist_p;
      PID_distB.Compute();

      // Angle PID pour rester aligné
      SPAngle = normalizeAngle(atan2(yc,xc)); // on recule donc on ajoute π
      PIDAngleF_setpoint = SPAngle;
      PIDAngleF_input = teta;
      PID_angleF.Compute();

      // Calcul commandes moteurs inversées
      int distCmd = -PIDdistB_output;  // inversion pour reculer
      leftCmd  = (int)round(distCmd - PIDAngleF_output);
      rightCmd = (int)round(distCmd + PIDAngleF_output);

      driveMotors(leftCmd, rightCmd);

      // Debug
      Serial.print("REVERSE | Dist="); Serial.print(Dist);
      Serial.print(" SPdist="); Serial.print(SPdist);
      Serial.print(" leftCmd="); Serial.print(leftCmd);
      Serial.print(" rightCmd="); Serial.println(rightCmd);

      // Passage au checkpoint si proche de la cible
      if (Dist < 10) {
          state = CHECKPOINT;
          digitalWrite(2, LOW);
          Serial.println("REVERSE complete, going to checkpoint");
      }
      if ((SPdist<=Target_dist) && (SPdist>(Target_dist-20))) {
           //PIDAngle_output=0;
           // PIDdist_output=0;
           //digitalWrite(2,HIGH);
           //PID_dist.SetMode(MANUAL);
           //PID_angle.SetMode(MANUAL);
          
         state = CHECKPOINT;
          
        Serial.println("ndourou taw");
        }
      } break;
      //=================== FORWARD ===================
      case FORWARD: {
        wheel_gain=1.063;
        digitalWrite(2,LOW);
        Update_coordinates();
        Calcul_erreur();
        // update trapeze setpoint velocity (mm/s)
        Trapeze();
        
        dist_ref += vitesse * dt;
        if (dist_ref > Target_dist)
            dist_ref = Target_dist;
        SPdist=dist_ref;
        // distance PID : setpoint = SPdist (mm/s)
        
        PIDdist_setpoint =dist_ref;        // objectif = distance restante = 0
        PIDdist_input = dist_p;        // distance restante à la cible
        PID_dist.Compute();
        
        // angle correction still active
        SPAngle=normalizeAngle(bearing);
        PIDAngleF_setpoint = SPAngle;
        PIDAngleF_input = teta;
        PID_angleF.Compute();
      
        leftCmd = (int)round((PIDdist_output- PIDAngleF_output));//
         rightCmd = (int)round(((PIDdist_output+ PIDAngleF_output))); //
        driveMotors(leftCmd, rightCmd);
        Serial.print("trapMode"); Serial.print(trapMode);
        Serial.print(" teta"); Serial.print(teta);
        Serial.print(" trapState"); Serial.print(trapState);
        Serial.print(" x="); Serial.print(x);
        Serial.print("y="); Serial.print(y);
        Serial.print(" leftcmd="); Serial.print(leftCmd);
        Serial.print(" rightcmd="); Serial.print(rightCmd);
        Serial.print("FORWARD | Dist="); Serial.print(Dist);
        Serial.print(" SPdist="); Serial.print(SPdist);
        Serial.print(" linVel="); Serial.print(linearVel);
        Serial.print(" outV="); Serial.print(PIDdist_output);
        Serial.print(" outA="); Serial.println(PIDAngleF_output);
       
         if(Dist<10)
         {
         // PID_dist.SetMode(MANUAL);
          //PID_angle.SetMode(MANUAL);
           state = CHECKPOINT;
           digitalWrite(2,LOW);
           Serial.println("enough");
        }
        if ((SPdist<=Target_dist) && (SPdist>(Target_dist-20))) {
           //PIDAngle_output=0;
           // PIDdist_output=0;
           //digitalWrite(2,HIGH);
           //PID_dist.SetMode(MANUAL);
           //PID_angle.SetMode(MANUAL);
          
         state = CHECKPOINT;
          
        Serial.println("ndourou taw");
        }
      }
      break;
       // ================= ROTATEAFTER ===================
      case ROTATEAFTER: {
        PID_angle.SetMode(AUTOMATIC);
        Update_coordinates();
        Calcul_erreur();
        // setpoint toward goal
        SPAngle = normalizeAngle(tetac);
        PIDAngle_setpoint = SPAngle;
        PIDAngle_input =teta;  
        PID_angle.Compute();
        int cmd = (int)round(PIDAngle_output);
        // drive for rotation: left = +cmd, right = -cmd
        driveMotors(-cmd, cmd);
        // Serial.print(" spangle "); Serial.print(SPAngle);
        // Serial.print("  |x: "); Serial.print(x);
        // Serial.print("  |y: "); Serial.print(y);
        // Serial.print("  |teta: "); Serial.print(teta);
        // Serial.print("  |Dist: "); Serial.print(Dist);
        // Serial.print("ROTATE outA="); Serial.print(PIDAngle_output);
        // Serial.print("Cap2="); Serial.println(Cap2);
         if (abs(Cap2)< 0.02)
         { // ~0.08 rad ~4.6 deg
          // reset trapeze before go forward
          state = STOP;
          Serial.println("Dorna");
          //  Serial.print("  |x: "); Serial.print(x);
          //  Serial.print("  |y: "); Serial.print(y);
         }
       
      } break;
      case CHECKPOINT:{
      currentPoint++;
      if(currentPoint==4)
      {
        Pompe();
          state=STEPPER;
      }
       
      else if (currentPoint >= NB_POINTS) {
        digitalWrite(2,HIGH);
        delay(100);
        digitalWrite(2,LOW);
        delay(100);
        Serial.println("Trajet terminé 🎯");
        state=FINISH;
      }
      else 
       state=STOP;
      }break;
      case STEPPER:
      {
        stopmotors();
        //Pompe();
        // Set motor direction clockwise
        digitalWrite(dirPin, LOW);

        // Spin motor 4 revolutions slowly
        for(int x = 0; x < 4* stepsPerRevolution; x++)
        {
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(2000);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(2000);
        }
        delay(1000); // Wait a second
        
        // Set motor direction counterclockwise
        digitalWrite(dirPin, HIGH);

        // Spin motor 4 revolutions quickly
        for(int x = 0; x < 4*stepsPerRevolution; x++)//5
        {
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(1000);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(1000);
        }
        delay(1000); // Wait a second
        state=STOP;
      }
      //=================== STOP ===================
      case STOP:
      stopmotors();
      delay(100);
       // stopmotors();
       state=SETPATH;
        break;
      case FINISH:
      {
       state=SETPATH;
      }
      break;
    }
  }
}


//============== MOTOR FUNCTIONS ================
void driveMotors(int leftPWM, int rightPWM) {
  leftPWM  = constrain(leftPWM,  -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  // LEFT MOTOR
  if (leftPWM > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA,(wheel_gain* pwmFromSigned(leftPWM)));
  } 
  else if (leftPWM < 0) {
    digitalWrite(IN1, LOW );
    digitalWrite(IN2, HIGH);
    analogWrite(ENA,(wheel_gain* pwmFromSigned(leftPWM)));
  } 
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
   analogWrite(ENA,0);
  }

  // RIGHT MOTOR
  
  if (rightPWM > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, ((pwmFromSigned(rightPWM))));
  } 
  else if (rightPWM < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB,((pwmFromSigned(rightPWM))));
  } 
  else {
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



//============== ODOM, TRAPEZE, ERRORS ===============
void Update_coordinates() {
  static double encR_prev = 0;
  static double encL_prev = 0;

  // convert counts to mm
  double encR = encoderCount_R * ENCODER_COEFFICIENT;
  double encL =  encoderCount_L * ENCODER_COEFFICIENT;//wheel_gain*

  double deltaR = encR - encR_prev;
  double deltaL = encL - encL_prev;

  encR_prev = encR;
  encL_prev = encL;
  
  // forward movement (mm) and rotation (rad)
  double D =(deltaR + deltaL) / 2.0;// 1.054*
  double alpha =1.05*((deltaR - deltaL) / (double)WHEEL_BASE);//1.03*

  // linear velocity in mm/s
  linearVel = D /dt;

  // update orientation and position
  teta =normalizeAngle(teta_prev + alpha);
  double teta_mid=teta_prev + alpha/2.0;
  x = x_prev + cos(teta_mid) * D;
  y = y_prev + sin(teta_mid) * D;
  dist_p+=D;

  // save for next
  x_prev = x;
  y_prev = y;
  teta_prev = teta;
}

void Calcul_erreur() {
  // Errors
  x_err = xc - x;
  y_err = yc - y;

  // Euclidean distance
  Dist = sqrt(x_err * x_err + y_err * y_err);

  // Bearing from robot to target minus robot heading
  bearing = atan2(y_err, x_err);
  Cap1 = normalizeAngle(atan2(yc,xc)-teta); 
  Cap= normalizeAngle(bearing - teta);
  Cap2=normalizeAngle(abs(tetac)-abs(teta));

}
void ResetTrapeze() {
  vitesse = 0;
  trapState = TRAP_ACCEL;
}
void Trapeze() {
  if (Dist < 10.0) {
    vitesse = 0;
    SPdist = 0;
    return;
}
  remaining_dist =Dist;
 if (remaining_dist < 0) remaining_dist = 0;


  double Vpeak = sqrt((2.0 * ACC_MAX * DEC_MAX * Target_dist) / (ACC_MAX + DEC_MAX));
 if (Vpeak < V_MAX) {
    trapMode = TRIANGULAR;
  } else {
    trapMode = TRAPEZOID;
  }

 if (trapMode == TRIANGULAR) {
    // distances needed to accel/decel to Vpeak
    double upDist = (Vpeak * Vpeak) / (2.0 * ACC_MAX);
    double downDist = (Vpeak * Vpeak) / (2.0 * DEC_MAX);

    // If still in accel phase (compare cumulative_traveled from start of segment)
    // We use remaining_dist to decide:
    if (remaining_dist > downDist) {
      // accelerating
      // increase internal vitesse toward Vpeak using ACC
      vitesse += ACC_MAX * dt;
      if (vitesse > Vpeak) vitesse = Vpeak;
      trapState = TRAP_ACCEL;
    } else {
      // decelerating
      vitesse -= DEC_MAX * dt;
      if (vitesse < 0) vitesse = 0;
      trapState = TRAP_DECEL;
    }
  }
  else{
    switch (trapState) {

    // ─────────────────────────────
    // 1) ACCELÉRATION
    // ─────────────────────────────
    case TRAP_ACCEL:
      vitesse += ACC_MAX *dt;
      if (vitesse >= V_MAX) {
        vitesse = V_MAX;
        trapState = TRAP_CRUISE;      // passage croisière
      }
      // si proche de la fin → décélération directe
      if (remaining_dist <= rampDown) {
        trapState = TRAP_DECEL;
      }
      break;

    // ─────────────────────────────
    // 2) CROISIÈRE
    // ─────────────────────────────
    case TRAP_CRUISE:
    digitalWrite(2,HIGH);
      vitesse = V_MAX;

      // passage en décélération si on approche de la cible
      if (remaining_dist <= rampDown) {
        trapState = TRAP_DECEL;
      }
      break;

    // ─────────────────────────────
    // 3) DÉCÉLÉRATION
    // ─────────────────────────────
    case TRAP_DECEL:
      digitalWrite(2,LOW);
      vitesse -= DEC_MAX * dt;
      if (vitesse <= 0) {
        //PID_dist.SetMode(MANUAL);
        //PID_angle.SetMode(MANUAL);
        vitesse = 0;
        // on reste en DECEL (robot s'arrête)
      }
      if(Dist<(rampUp+5))
      {
        vitesse=0;
      }
      break;
    
  }
  } 
  // sortie
 SPdist = vitesse;
}
void Pompe()
{
  // Pompe active entre waypoint 1 et 2
 
      if(!pumpOn)   // éviter de reconfigurer à chaque cycle
      {
        pumpOn = true;
        PWM_MIN = 75;
        updatePIDGains();
        digitalWrite(P1, HIGH);
        digitalWrite(P2, LOW);
        digitalWrite(ENP, HIGH);
        Serial.println("Pump ON");
      }
  
  else if (pumpOn==false)
 {
    pumpOn = false;
    digitalWrite(ENP, LOW);
    Serial.println("Pump OFF");

  }
}
