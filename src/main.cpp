#include <ESP32Servo.h>
#include <math.h>

// DABBLE phone app (GamePad only) 
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>


// CONFIG


#define DEG_TO_RAD 0.017453292519943295
#define RAD_TO_DEG 57.29577951308232

// Link lengths in mm
const double FEMUR_LEN = 63.0;
const double TIBIA_LEN = 95.0;
const double COXA_LEN  = 19.0;   

// Mechanical neutral offsets
const double Y_Rest = 34.0;
const double Z_Rest = -80.0;
const double J3_LegAngle = 15.4;

// Tuning

float  servoStep    = 4.4f;   
int    stepDelayMs  = 12;      // lower = smoother/faster animation
double strideX      = 30.0;    // mm
double liftZ        = 50.0;    // mm positive = up
int    phasePauseMs = 0;

// New smooth gait tuning
int swingFrames  = 30;         // higher = smoother swing
int glideFrames  = 20;         // higher = smoother support shift
double supportShiftRatio = 0.50; // how much planted legs shift during a step
double swingReachRatio   = 0.73; // how far swing leg reaches forward

// LEG STRUCT / IDs


enum LegId {
  FL_ID = 0,
  FR_ID = 1,
  BL_ID = 2,
  BR_ID = 3
};

struct Leg {
  int coxaPin;
  int femurPin;
  int tibiaPin;

  Servo coxa;
  Servo femur;
  Servo tibia;

  int sideSign;   // +1 = right, -1 = left
  LegId id;
};

Leg FL  = {  2,  4,  16, Servo(), Servo(), Servo(), -1, FL_ID };  // Front Left
Leg FR  = { 17,  5,  18, Servo(), Servo(), Servo(), +1, FR_ID };  // Front Right
Leg BL  = { 12, 13,  14, Servo(), Servo(), Servo(), -1, BL_ID };  // Back Left
Leg BRL = { 25, 26,  27, Servo(), Servo(), Servo(), +1, BR_ID };  // legBack Right

Leg* legs[4] = { &FL, &FR, &BL, &BRL };

// Per-joint inversion, index = FL, FR, BL, BR
bool invertCoxa [4] = { true,  false, false, true  };
bool invertFemur[4] = { true,  false, false, true  };
bool invertTibia[4] = { true,  false, false, true  };

// Forward X direction for each leg
int forwardSign[4] = { +1, +1, -1, -1 };

// X stance neutral positions
double NX[4];
double NY[4];
const double NZ_NEUTRAL = 0.0;

// Current tracked foot positions
double curX[4] = {0, 0, 0, 0};
double curY[4] = {0, 0, 0, 0};
double curZ[4] = {0, 0, 0, 0};


// STATE / MODES


volatile bool walking_crawler = false;
volatile bool inGait = false;
volatile bool stopRequested = false;
volatile bool inDemoMove = false;

enum MotionMode {
  IDLE = 0,
  FORWARD,
  BACKWARD,
  TURN_LEFT,
  TURN_RIGHT
};

volatile MotionMode currentMode = IDLE;

// Button edge tracking for demo moves
bool prevTriangle = false;
bool prevCircle   = false;
bool prevSquare   = false;
bool prevCross    = false;

// FUNCTION DECLARATIONS

void calcIK(double inX, double inY, double inZ, int sideSign,
            float &coxaTarget, float &femurTarget, float &tibiaTarget);

void writeLegInstant(Leg &leg, double X, double Y, double Z);
void rampLegToTargets(Leg &L, float c, float f, float t);
void moveLegTo(Leg &leg, double X, double Y, double Z);
void moveLegArc(Leg &leg, double targetX, double targetY, double targetZ, double arcHeight, int frames);
void moveAllLegsTo(double targetX[4], double targetY[4], double targetZ[4], int frames);

void applyXPreset(int mode, double frontX = 15.0, double rearX = -15.0, double spreadY = 10.0);
void standNeutralX();
void softStartStandNeutralX();

void crawlerStepOnce(Leg &L, MotionMode mode);
void crawlerWalk();
void handleGamepadHeldWalk();
void handleDemoButtons();

void danceWave();
void danceBounce();
void danceTap();
void danceSpin();
void danceCombo();

void printHelp();
void xshow();
void handleTuningCommand(const String& input);

MotionMode readHeldDirection();
void updateStopRequestFromGamepad();

// HELPERS

static inline float clampAngle(float a) {
  return constrain(a, 0.0f, 180.0f);
}

static inline double clampDouble(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline double lerpD(double a, double b, double t) {
  return a + (b - a) * t;
}

// Smoothstep
static inline double easeSmooth(double t) {
  t = clampDouble(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

bool anyDpadHeld() {
  return GamePad.isUpPressed() ||
         GamePad.isDownPressed() ||
         GamePad.isLeftPressed() ||
         GamePad.isRightPressed();
}

MotionMode readHeldDirection() {
  if (GamePad.isUpPressed())    return FORWARD;
  if (GamePad.isDownPressed())  return BACKWARD;
  if (GamePad.isLeftPressed())  return TURN_LEFT;
  if (GamePad.isRightPressed()) return TURN_RIGHT;
  return IDLE;
}

// During smooth movement...keep BLE open and check for stop request from Dabble GamePad.

void updateStopRequestFromGamepad() {
  Dabble.processInput();

  if (walking_crawler && !anyDpadHeld()) {
    stopRequested = true;
  }
}

// Inverse kinematics for a single leg. Returns angles in degrees via reference parameters.

void calcIK(double inX, double inY, double inZ, int sideSign, float &coxaTarget, float &femurTarget, float &tibiaTarget)
{
  const double X = inX;
  const double Y = (double)sideSign * inY + Y_Rest;
  const double Z = inZ + Z_Rest;

  const double J1 = atan2(X, Y) * RAD_TO_DEG;
  const double H  = sqrt((Y * Y) + (X * X));
  const double L  = sqrt((H * H) + (Z * Z));

  double c1 = ((FEMUR_LEN * FEMUR_LEN) + (TIBIA_LEN * TIBIA_LEN) - (L * L)) / (2.0 * FEMUR_LEN * TIBIA_LEN);
  double c2 = ((L * L) + (FEMUR_LEN * FEMUR_LEN) - (TIBIA_LEN * TIBIA_LEN)) / (2.0 * L * FEMUR_LEN);

  c1 = fmax(-1.0, fmin(1.0, c1));
  c2 = fmax(-1.0, fmin(1.0, c2));

  const double J3 = acos(c1) * RAD_TO_DEG;
  const double B  = acos(c2) * RAD_TO_DEG;
  const double A  = atan2(Z, H) * RAD_TO_DEG;
  const double J2 = B + A;

  float coxa  = 90.0f - (float)J1;
  float femur = 90.0f - (float)J2;
  float tibia = (float)J3 + (float)J3_LegAngle - 90.0f;

  coxa  = clampAngle(coxa);
  femur = clampAngle(femur);
  tibia = clampAngle(tibia);

  coxaTarget  = coxa;
  femurTarget = femur;
  tibiaTarget = tibia;
}
// Write angles to a leg's servos immediately, without smoothing. (DEBUGGING / TESTING)
void writeLegInstant(Leg &leg, double X, double Y, double Z) {
  float c, f, t;
  calcIK(X, Y, Z, leg.sideSign, c, f, t);

  const int idx = (int)leg.id;

  if (invertCoxa[idx])  c = 180.0f - c;
  if (invertFemur[idx]) f = 180.0f - f;
  if (invertTibia[idx]) t = 180.0f - t;

  leg.coxa.write(clampAngle(c));
  leg.femur.write(clampAngle(f));
  leg.tibia.write(clampAngle(t));

  curX[idx] = X;
  curY[idx] = Y;
  curZ[idx] = Z;
}

// Startup angle ramp. Kept because it avoids harsh boot snapping.
void rampLegToTargets(Leg &L, float c, float f, float t) {
  float cc = L.coxa.read();
  float ff = L.femur.read();
  float tt = L.tibia.read();

  c = clampAngle(c);
  f = clampAngle(f);
  t = clampAngle(t);

  bool done = false;

  while (!done) {
    Dabble.processInput();

    done = true;

    if (fabs(c - cc) >= servoStep) {
      cc += (c > cc ? servoStep : -servoStep);
      L.coxa.write(cc);
      done = false;
    } else {
      L.coxa.write(c);
      cc = c;
    }

    if (fabs(f - ff) >= servoStep) {
      ff += (f > ff ? servoStep : -servoStep);
      L.femur.write(ff);
      done = false;
    } else {
      L.femur.write(f);
      ff = f;
    }

    if (fabs(t - tt) >= servoStep) {
      tt += (t > tt ? servoStep : -servoStep);
      L.tibia.write(tt);
      done = false;
    } else {
      L.tibia.write(t);
      tt = t;
    }

    delay(stepDelayMs);
  }
}

// Smooth XYZ movement to target.
void moveLegTo(Leg &leg, double X, double Y, double Z) {
  const int idx = (int)leg.id;

  double sx = curX[idx];
  double sy = curY[idx];
  double sz = curZ[idx];

  for (int i = 1; i <= swingFrames; i++) {
    updateStopRequestFromGamepad();

    double u = (double)i / (double)swingFrames;
    double e = easeSmooth(u);

    double px = lerpD(sx, X, e);
    double py = lerpD(sy, Y, e);
    double pz = lerpD(sz, Z, e);

    writeLegInstant(leg, px, py, pz);
    delay(stepDelayMs);
  }

  writeLegInstant(leg, X, Y, Z);
}

// Smooth lifted arc movement.
void moveLegArc(Leg &leg, double targetX, double targetY, double targetZ, double arcHeight, int frames) {
  const int idx = (int)leg.id;

  double sx = curX[idx];
  double sy = curY[idx];
  double sz = curZ[idx];

  frames = max(4, frames);

  for (int i = 1; i <= frames; i++) {
    updateStopRequestFromGamepad();

    double u = (double)i / (double)frames;
    double e = easeSmooth(u);

    double px = lerpD(sx, targetX, e);
    double py = lerpD(sy, targetY, e);

    // Sine arc for motion smoothing (credit to reddit for this idea)
    double baseZ = lerpD(sz, targetZ, e);
    double arcZ  = sin(PI * u) * arcHeight;
    double pz    = baseZ + arcZ;

    writeLegInstant(leg, px, py, pz);
    delay(stepDelayMs);
  }

  writeLegInstant(leg, targetX, targetY, targetZ); // call again to ensure final position is exact
}

// Move all four feet smoothly together 
void moveAllLegsTo(double targetX[4], double targetY[4], double targetZ[4], int frames) {
  frames = max(4, frames);

  double sx[4], sy[4], sz[4];

  for (int i = 0; i < 4; i++) {
    sx[i] = curX[i];
    sy[i] = curY[i];
    sz[i] = curZ[i];
  }

  for (int frame = 1; frame <= frames; frame++) {
    updateStopRequestFromGamepad();

    double u = (double)frame / (double)frames;
    double e = easeSmooth(u);

    for (int i = 0; i < 4; i++) {
      double px = lerpD(sx[i], targetX[i], e);
      double py = lerpD(sy[i], targetY[i], e);
      double pz = lerpD(sz[i], targetZ[i], e);

      writeLegInstant(*legs[i], px, py, pz);
    }

    delay(stepDelayMs);
  }

  for (int i = 0; i < 4; i++) {
    writeLegInstant(*legs[i], targetX[i], targetY[i], targetZ[i]);
  }
}

// ======================================================
// X STANCE
// ======================================================

void applyXPreset(int mode, double frontX, double rearX, double spreadY) {
  if (mode == 2) {
    double tmp = frontX;
    frontX = rearX;
    rearX = tmp;
  }

  NX[FL_ID] = frontX;
  NX[FR_ID] = frontX;
  NX[BL_ID] = rearX;
  NX[BR_ID] = rearX;

  NY[FL_ID] = +spreadY;
  NY[FR_ID] = -spreadY;
  NY[BL_ID] = +spreadY;
  NY[BR_ID] = -spreadY;
}

void standNeutralX() {
  double tx[4] = { NX[0], NX[1], NX[2], NX[3] };
  double ty[4] = { NY[0], NY[1], NY[2], NY[3] };
  double tz[4] = { 0, 0, 0, 0 };

  moveAllLegsTo(tx, ty, tz, glideFrames + 8);
}

void softStartStandNeutralX() {
  for (int i = 0; i < 4; i++) {
    float c, f, t;
    calcIK(NX[i], NY[i], 0.0, legs[i]->sideSign, c, f, t);

    if (invertCoxa[i])  c = 180.0f - c;
    if (invertFemur[i]) f = 180.0f - f;
    if (invertTibia[i]) t = 180.0f - t;

    rampLegToTargets(*legs[i], c, f, t);

    curX[i] = NX[i];
    curY[i] = NY[i];
    curZ[i] = 0.0;
  }
}

// CRAWLER GAIT


void getStepVector(Leg &L, MotionMode mode, double &dx, double &dy) {
  const int idx = (int)L.id;

  dx = 0.0;
  dy = 0.0;

  if (mode == FORWARD) {
    dx = forwardSign[idx] * strideX;
  }
  else if (mode == BACKWARD) {
    dx = forwardSign[idx] * -strideX;
  }
  else if (mode == TURN_LEFT) {
    dx = -L.sideSign * strideX * 0.50;
    dy =  L.sideSign * strideX * 0.50;
  }
  else if (mode == TURN_RIGHT) {
    dx =  L.sideSign * strideX * 0.50;
    dy = -L.sideSign * strideX * 0.50;
  }
}

void crawlerStepOnce(Leg &L, MotionMode mode) {
  const int idx = (int)L.id;

  double dx, dy;
  getStepVector(L, mode, dx, dy);

  // 1. Support legs shift slightly opposite the intended movement.
  // This makes the walk animation look like the body is transferring weight.
  double tx[4];
  double ty[4];
  double tz[4];

  for (int i = 0; i < 4; i++) {
    tx[i] = NX[i];
    ty[i] = NY[i];
    tz[i] = 0.0;
  }

  for (int i = 0; i < 4; i++) {
    if (i != idx) {
      tx[i] = NX[i] - dx * supportShiftRatio;
      ty[i] = NY[i] - dy * supportShiftRatio;
    }
  }

  // Keep the swing leg where it is while the body/supports settle.
  tx[idx] = curX[idx];
  ty[idx] = curY[idx];
  tz[idx] = curZ[idx];

  moveAllLegsTo(tx, ty, tz, glideFrames);

  if (phasePauseMs) delay(phasePauseMs);

  // 2. Swing selected leg forward using a smooth arc.
  double swingX = NX[idx] + dx * swingReachRatio;
  double swingY = NY[idx] + dy * swingReachRatio;

  moveLegArc(L, swingX, swingY, 0.0, liftZ, swingFrames);

  if (phasePauseMs) delay(phasePauseMs);

  // 3. Glide everything back to neutral.
  // This keeps the robot from slowly walking itself into weird stretched poses.
  for (int i = 0; i < 4; i++) {
    tx[i] = NX[i];
    ty[i] = NY[i];
    tz[i] = 0.0;
  }

  moveAllLegsTo(tx, ty, tz, glideFrames);

  if (phasePauseMs) delay(phasePauseMs);
}

void crawlerWalk() {
  if (currentMode == IDLE) return;

  MotionMode cycleMode = currentMode;

  // Diagonal ripple order for stability
  static Leg* order[4] = { &FL, &BRL, &FR, &BL };

  inGait = true;

  for (int i = 0; i < 4 && walking_crawler; i++) {
    crawlerStepOnce(*order[i], cycleMode);

    if (stopRequested) {
      walking_crawler = false;
      currentMode = IDLE;
      stopRequested = false;
      break;
    }
  }

  inGait = false;

  if (!walking_crawler) {
    standNeutralX();
    Serial.println("Stopped after current leg -> Stand X");
  }
}

// Hold D-pad to walk. Release = finish current leg, then stop.
void handleGamepadHeldWalk() {
  if (inDemoMove) return;

  Dabble.processInput();

  MotionMode held = readHeldDirection();

  if (held == IDLE) {
    if (walking_crawler) stopRequested = true;
    return;
  }

  currentMode = held;

  if (!walking_crawler) {
    walking_crawler = true;
    stopRequested = false;
    Serial.println("D-pad held -> walking");
  }
}


// DANCE / DEMO MOVES

void danceWave() {
  if (walking_crawler) return;

  inDemoMove = true;
  Serial.println("Dance: wave");

  standNeutralX();

  // Front-left wave
  moveLegArc(FL, NX[FL_ID] + 8, NY[FL_ID] + 18, 18, liftZ + 10, 18);

  for (int i = 0; i < 3; i++) {
    moveLegTo(FL, NX[FL_ID] + 8, NY[FL_ID] + 28, 28);
    moveLegTo(FL, NX[FL_ID] + 8, NY[FL_ID] + 10, 28);
  }

  moveLegArc(FL, NX[FL_ID], NY[FL_ID], 0, liftZ, 16);

  standNeutralX();
  inDemoMove = false;
}

void danceBounce() {
  if (walking_crawler) return;

  inDemoMove = true;
  Serial.println("Dance: bounce");

  standNeutralX();

  double tx[4] = { NX[0], NX[1], NX[2], NX[3] };
  double ty[4] = { NY[0], NY[1], NY[2], NY[3] };
  double tz[4];

  for (int r = 0; r < 3; r++) {
    tz[0] = 12; tz[1] = 12; tz[2] = 12; tz[3] = 12;
    moveAllLegsTo(tx, ty, tz, 10);

    tz[0] = -6; tz[1] = -6; tz[2] = -6; tz[3] = -6;
    moveAllLegsTo(tx, ty, tz, 10);
  }

  tz[0] = 0; tz[1] = 0; tz[2] = 0; tz[3] = 0;
  moveAllLegsTo(tx, ty, tz, 12);

  standNeutralX();
  inDemoMove = false;
}

void danceTap() {
  if (walking_crawler) return;

  inDemoMove = true;
  Serial.println("Dance: tap");

  standNeutralX();

  for (int i = 0; i < 3; i++) {
    moveLegArc(FL, NX[FL_ID] + 6, NY[FL_ID], 0, liftZ + 5, 12);
    moveLegTo(FL, NX[FL_ID] + 6, NY[FL_ID], -8);
    moveLegTo(FL, NX[FL_ID] + 6, NY[FL_ID], 0);

    moveLegArc(FR, NX[FR_ID] + 6, NY[FR_ID], 0, liftZ + 5, 12);
    moveLegTo(FR, NX[FR_ID] + 6, NY[FR_ID], -8);
    moveLegTo(FR, NX[FR_ID] + 6, NY[FR_ID], 0);
  }

  standNeutralX();
  inDemoMove = false;
}

void danceSpin() {
  if (walking_crawler) return;

  inDemoMove = true;
  Serial.println("Dance: spin");

  standNeutralX();

  double oldStride = strideX;
  strideX = oldStride * 0.85;

  static Leg* order[4] = { &FL, &BRL, &FR, &BL };

  for (int r = 0; r < 2; r++) {
    for (int i = 0; i < 4; i++) {
      crawlerStepOnce(*order[i], TURN_RIGHT);
    }
  }

  strideX = oldStride;

  standNeutralX();
  inDemoMove = false;
}

void danceCombo() {
  if (walking_crawler) return;

  inDemoMove = true;
  Serial.println("Dance: combo");

  inDemoMove = false;
  danceBounce();
  danceWave();
  danceTap();
  danceSpin();
}

void handleDemoButtons() {
  if (walking_crawler || inDemoMove) return;

  Dabble.processInput();

  bool tri = GamePad.isTrianglePressed();
  bool cir = GamePad.isCirclePressed();
  bool sqr = GamePad.isSquarePressed();
  bool cro = GamePad.isCrossPressed();

  if (tri && !prevTriangle) {
    danceWave();
  }
  else if (cir && !prevCircle) {
    danceBounce();
  }
  else if (sqr && !prevSquare) {
    danceTap();
  }
  else if (cro && !prevCross) {
    danceSpin();
  }

  prevTriangle = tri;
  prevCircle   = cir;
  prevSquare   = sqr;
  prevCross    = cro;
}


// Help and tuning commands. These are for debugging and testing, not required for normal operation :) 

void printHelp() {
  Serial.println("\nCommands:");
  Serial.println("  standx          -> go to X stance");
  Serial.println("  xmode N         -> X stance preset 1 or 2");
  Serial.println("  xfront <mm>     -> set front legs X");
  Serial.println("  xrear  <mm>     -> set rear legs X");
  Serial.println("  xspread <mm>    -> set outward Y spread");
  Serial.println("  xshow           -> show current X stance");
  Serial.println("  leg N X Y Z     -> move single leg N: 0=FL,1=FR,2=BL,3=BR");
  Serial.println("  all X Y Z       -> move all legs");
  Serial.println("  stride <mm>     -> set stride length");
  Serial.println("  lift <mm>       -> set lift height");
  Serial.println("  step <deg>      -> set startup servo angle step");
  Serial.println("  delay <ms>      -> set frame delay");
  Serial.println("  pace <ms>       -> pause between gait phases");
  Serial.println("  swing <frames>  -> swing smoothness");
  Serial.println("  glide <frames>  -> support glide smoothness");
  Serial.println("  dance wave      -> wave demo");
  Serial.println("  dance bounce    -> bounce demo");
  Serial.println("  dance tap       -> tap demo");
  Serial.println("  dance spin      -> spin demo");
  Serial.println("  dance combo     -> combo demo");
  Serial.println();
  Serial.println("Dabble:");
  Serial.println("  Hold D-pad to walk");
  Serial.println("  Triangle = wave");
  Serial.println("  Circle   = bounce");
  Serial.println("  Square   = tap");
  Serial.println("  Cross    = spin");
  Serial.println("----------------------------");
}

void xshow() {
  Serial.println("--- X stance NX, NY ---");
  Serial.print("FL: "); Serial.print(NX[0]); Serial.print(", "); Serial.println(NY[0]);
  Serial.print("FR: "); Serial.print(NX[1]); Serial.print(", "); Serial.println(NY[1]);
  Serial.print("BL: "); Serial.print(NX[2]); Serial.print(", "); Serial.println(NY[2]);
  Serial.print("BR: "); Serial.print(NX[3]); Serial.print(", "); Serial.println(NY[3]);

  Serial.println("--- Gait ---");
  Serial.print("strideX: "); Serial.println(strideX);
  Serial.print("liftZ: "); Serial.println(liftZ);
  Serial.print("stepDelayMs: "); Serial.println(stepDelayMs);
  Serial.print("swingFrames: "); Serial.println(swingFrames);
  Serial.print("glideFrames: "); Serial.println(glideFrames);
  Serial.print("supportShiftRatio: "); Serial.println(supportShiftRatio, 2);
  Serial.print("swingReachRatio: "); Serial.println(swingReachRatio, 2);
  Serial.println("-----------------------");
}

void handleTuningCommand(const String& input) {
  double dval;

  if (input.equalsIgnoreCase("help")) {
    printHelp();
  }
  else if (input.startsWith("stride ")) {
    if (sscanf(input.c_str(), "stride %lf", &dval) == 1) {
      strideX = dval;
      Serial.println("stride set to " + String(strideX, 1));
    }
  }
  else if (input.startsWith("lift ")) {
    if (sscanf(input.c_str(), "lift %lf", &dval) == 1) {
      liftZ = dval;
      Serial.println("lift set to " + String(liftZ, 1));
    }
  }
  else if (input.startsWith("step ")) {
    if (sscanf(input.c_str(), "step %lf", &dval) == 1) {
      servoStep = (float)clampDouble(dval, 0.1, 20.0);
      Serial.println("step set to " + String(servoStep, 1));
    }
  }
  else if (input.startsWith("delay ")) {
    long ival;
    if (sscanf(input.c_str(), "delay %ld", &ival) == 1) {
      stepDelayMs = (int)clampDouble(ival, 1, 100);
      Serial.println("delay set to " + String(stepDelayMs));
    }
  }
  else if (input.startsWith("pace ")) {
    long ival;
    if (sscanf(input.c_str(), "pace %ld", &ival) == 1) {
      phasePauseMs = (int)clampDouble(ival, 0, 1000);
      Serial.println("pace set to " + String(phasePauseMs));
    }
  }
  else if (input.startsWith("swing ")) {
    long ival;
    if (sscanf(input.c_str(), "swing %ld", &ival) == 1) {
      swingFrames = (int)clampDouble(ival, 4, 60);
      Serial.println("swingFrames set to " + String(swingFrames));
    }
  }
  else if (input.startsWith("glide ")) {
    long ival;
    if (sscanf(input.c_str(), "glide %ld", &ival) == 1) {
      glideFrames = (int)clampDouble(ival, 4, 60);
      Serial.println("glideFrames set to " + String(glideFrames));
    }
  }
  else if (input.equalsIgnoreCase("xshow")) {
    xshow();
  }
  else if (input.startsWith("xmode ")) {
    int m;
    if (sscanf(input.c_str(), "xmode %d", &m) == 1) {
      applyXPreset((m == 2) ? 2 : 1);
      Serial.print("xmode set to ");
      Serial.println((m == 2) ? 2 : 1);
      standNeutralX();
    }
  }
  else if (input.startsWith("xfront ")) {
    if (sscanf(input.c_str(), "xfront %lf", &dval) == 1) {
      NX[FL_ID] = dval;
      NX[FR_ID] = dval;
      standNeutralX();
      xshow();
    }
  }
  else if (input.startsWith("xrear ")) {
    if (sscanf(input.c_str(), "xrear %lf", &dval) == 1) {
      NX[BL_ID] = dval;
      NX[BR_ID] = dval;
      standNeutralX();
      xshow();
    }
  }
  else if (input.startsWith("xspread ")) {
    if (sscanf(input.c_str(), "xspread %lf", &dval) == 1) {
      NY[FL_ID] = +dval;
      NY[FR_ID] = -dval;
      NY[BL_ID] = +dval;
      NY[BR_ID] = -dval;
      standNeutralX();
      xshow();
    }
  }
  else if (input.equalsIgnoreCase("dance wave")) {
    danceWave();
  }
  else if (input.equalsIgnoreCase("dance bounce")) {
    danceBounce();
  }
  else if (input.equalsIgnoreCase("dance tap")) {
    danceTap();
  }
  else if (input.equalsIgnoreCase("dance spin")) {
    danceSpin();
  }
  else if (input.equalsIgnoreCase("dance combo")) {
    danceCombo();
  }
  else {
    Serial.println("Unknown command. Type help.");
  }
}



// SETUP
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20000); 
  delay(300);

  // Attach servos
  FL.coxa.attach(FL.coxaPin);
  FL.femur.attach(FL.femurPin);
  FL.tibia.attach(FL.tibiaPin);

  FR.coxa.attach(FR.coxaPin);
  FR.femur.attach(FR.femurPin);
  FR.tibia.attach(FR.tibiaPin);

  BL.coxa.attach(BL.coxaPin);
  BL.femur.attach(BL.femurPin);
  BL.tibia.attach(BL.tibiaPin);

  BRL.coxa.attach(BRL.coxaPin);
  BRL.femur.attach(BRL.femurPin);
  BRL.tibia.attach(BRL.tibiaPin);

  // Your working stance preset
  applyXPreset(/*mode=*/1, /*frontX=*/15.0, /*rearX=*/15.0, /*spreadY=*/10.0);

  // Safe start
  FL.coxa.write(90);  FR.coxa.write(90);  BL.coxa.write(90);  BRL.coxa.write(90);
  FL.femur.write(90); FR.femur.write(90); BL.femur.write(90); BRL.femur.write(90);
  FL.tibia.write(90); FR.tibia.write(90); BL.tibia.write(90); BRL.tibia.write(90);

  delay(400);

  // Smooth move into IK neutral
  softStartStandNeutralX();

  // Dabble BLE
  Dabble.begin("SpiderBot");

  Serial.println("SpiderBot ready.");
  printHelp();
}

// LOOP


void loop() {
  Dabble.processInput();

  handleDemoButtons();
  handleGamepadHeldWalk();

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("standx")) {
      walking_crawler = false;
      currentMode = IDLE;
      stopRequested = false;
      standNeutralX();
    }
    else if (input.startsWith("leg")) {
      int n;
      double x, y, z;

      if (sscanf(input.c_str(), "leg %d %lf %lf %lf", &n, &x, &y, &z) == 4) {
        if (n >= 0 && n <= 3) {
          moveLegTo(*legs[n], x, y, z);
        } else {
          Serial.println("Bad leg index. Use 0..3.");
        }
      }
    }
    else if (input.startsWith("all")) {
      double x, y, z;

      if (sscanf(input.c_str(), "all %lf %lf %lf", &x, &y, &z) == 3) {
        double tx[4] = { x, x, x, x };
        double ty[4] = { y, y, y, y };
        double tz[4] = { z, z, z, z };
        moveAllLegsTo(tx, ty, tz, glideFrames + 8);
      }
    }
    else if (input.length() > 0) {
      handleTuningCommand(input);
    }
  }

  if (walking_crawler && !inDemoMove) {
    crawlerWalk();
  }
}