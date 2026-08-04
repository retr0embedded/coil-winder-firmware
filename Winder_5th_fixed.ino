#include <LiquidCrystal.h>

// =====================================================
// LCD
// =====================================================

LiquidCrystal lcd(A0, A1, A2, A3, A4, A5); // RS, EN, D4, D5, D6, D7

// =====================================================
// PIN DEFINITIONS
// =====================================================

// MAIN SHAFT STEPPER DRIVER
#define STEP_PIN      9
#define DIR_PIN       8
#define ENABLE_PIN   10

// WIRE GUIDE STEPPER DRIVER
#define WG_STEP_PIN      11
#define WG_DIR_PIN        6
#define WG_ENABLE_PIN     5

// INPUTS - Encoder and Limit Switch
#define ENC_BTN       2
#define ENC_A         3
#define ENC_B         4

#define LIMIT_SWITCH  7

// =====================================================
// MOTOR PARAMETERS
// =====================================================

#define MOTOR_STEPS   200
#define MICROSTEPS   16

#define WG_STEPS_PER_MM 1600.0  // 200 steps/rev × 8 microsteps, 1mm pitch lead screw

float targetRPS = 1.0;  // adjustable 0.5–3.0 via encoder during run

const float STEPS_PER_TURN =
  MOTOR_STEPS * MICROSTEPS;

uint16_t targetSPS =
  (uint16_t)(MOTOR_STEPS * MICROSTEPS * targetRPS);

// =====================================================
// SYSTEM MODES
// =====================================================

enum SystemMode
{
  MENU_MODE,
  MANUAL_MODE,
  SEMI_AUTO_MODE,
  AUTO_MODE,
  SETUP_MODE,
  HOMING_MODE,
  ZERO_SET_MODE
};

volatile SystemMode currentMode =
  MENU_MODE;

// =====================================================
// MENU
// =====================================================

const char* menuItems[] =
{
  "MANUAL",
  "SEMI-AUTO",
  "AUTOMATIC",
  "SETUP"
};

volatile int menuIndex = 0;

// =====================================================
// USER PARAMETERS
// =====================================================

float wireDiameter    = 0.25;   // mm
float coilLengthMM    = 10.0;   // mm
float targetTurnsValue = 100.0;
bool  windLeftToRight  = true;  // true = wind left-to-right (default), false = right-to-left

float zeroOffsetMM  = 0.0; // mm to travel from limit switch to virtual zero (0.1 mm steps)
                            // in R->L mode this is the distance from the switch to the FAR
                            // logical-zero point; coilLengthMM must not exceed it
int zeroEncoderBase = 0;   // encoder snapshot on ZERO_SET_MODE entry

int setupParamIndex  = 0;  // which param is being edited: 0=wireDia 1=coilLen 2=turns 3=windDir
int setupEncoderBase = 0;  // encoder snapshot on SETUP_MODE entry / param advance
int speedEncoderBase = 0;  // encoder snapshot for in-run speed adjustment
bool freshRun        = false;  // true only when entering a run mode fresh from menu
bool windingComplete = false;  // set when auto-stop fires; clears on menu re-entry
volatile bool mainMotorForward = true;  // true=forward, false=reverse; applied to DIR_PIN on motor start

// =====================================================
// MAIN MOTOR STATE
// =====================================================

volatile bool motorRunning = false;
volatile bool targetState  = false;

volatile bool stepState = false;

volatile uint16_t stepIntervalUs = 3000;

volatile int32_t stepCount = 0;

float turns      = 0.0;
float currentSPS = 0;

bool ramping  = false;
bool rampUp   = true;

unsigned long rampStart = 0;

// =====================================================
// WIRE GUIDE STATE
// =====================================================

volatile long wireGuidePosition    = 0;
volatile bool wireGuideDir         = true;   // current traverse direction; reset from windLeftToRight on fresh run start

volatile long wgAccumulator        = 0;
volatile long wgRatio              = 0;

volatile long layerSteps           = 0;
volatile long currentLayerPosition = 0;

volatile bool layerBoundaryReached = false;

int layerCount = 1;  // current winding layer, always >= 1

// =====================================================
// ENCODER
// =====================================================

volatile int encoderPosition = 0;

// =====================================================
// BUTTON
// =====================================================

volatile bool buttonPressed = false;

// =====================================================
// BUTTON ISR
// =====================================================

void buttonISR()
{
  static unsigned long lastInterrupt = 0;

  unsigned long now = millis();

  if (now - lastInterrupt > 150)
  {
    buttonPressed = true;
  }

  lastInterrupt = now;
}

// =====================================================
// ENCODER ISR
// =====================================================

void encoderISR()
{
  if (digitalRead(ENC_B))
    encoderPosition--;
  else
    encoderPosition++;
}

// =====================================================
// TIMER1 ISR
// =====================================================

ISR(TIMER1_COMPA_vect)
{
  static uint16_t counter = 0;

  if (!motorRunning)
    return;

  counter += 10;

  if (counter >= stepIntervalUs)
  {
    counter = 0;

    stepState = !stepState;

    digitalWrite(STEP_PIN, stepState);

    // count only rising edges
    if (stepState)
    {
      if (mainMotorForward) stepCount++;
      else                  stepCount--;

      // ===============================================
      // BRESENHAM WIRE GUIDE SYNC
      // ===============================================

      if (currentMode == MANUAL_MODE    ||
          currentMode == SEMI_AUTO_MODE ||
          currentMode == AUTO_MODE)
      {
        wgAccumulator += wgRatio;

        // while (not if): if wgRatio ever exceeds STEPS_PER_TURN (e.g. from
        // higher WG microstepping or a larger wire gauge), drain the excess
        // fully instead of losing it and letting the guide fall behind.
        while (wgAccumulator >=
            (long)STEPS_PER_TURN)
        {
          wgAccumulator -=
            (long)STEPS_PER_TURN;

          // reverse main motor → traverse wire guide in opposite direction
          bool actualWGDir =
            mainMotorForward ?
            wireGuideDir     :
            !wireGuideDir;

          digitalWrite(
            WG_DIR_PIN,
            actualWGDir);

          digitalWrite(
            WG_STEP_PIN,
            HIGH);

          delayMicroseconds(3);

          digitalWrite(
            WG_STEP_PIN,
            LOW);

          wireGuidePosition +=
            (actualWGDir ? 1 : -1);

          currentLayerPosition +=
            (actualWGDir ? 1 : -1);

          if (abs(currentLayerPosition)
              >= layerSteps)
          {
            layerBoundaryReached = true;
          }
        }
      }
    }
  }
}

// =====================================================
// TIMER SETUP
// =====================================================

void setupTimer1()
{
  cli();

  TCCR1A = 0;
  TCCR1B = 0;

  // CTC mode
  TCCR1B |= (1 << WGM12);

  // prescaler 8 → 10 µs tick
  TCCR1B |= (1 << CS11);

  OCR1A = 19;

  TIMSK1 |= (1 << OCIE1A);

  sei();
}

// =====================================================
// SMOOTHSTEP
// =====================================================

float smoothstep(float x)
{
  return (3.0 * x * x)
       - (2.0 * x * x * x);
}

// =====================================================
// UPDATE MOTION
// =====================================================

void updateMotion()
{
  if (!ramping)
    return;

  float t =
    (millis() - rampStart)
    / 1000.0;

  if (t >= 1.0)
  {
    t = 1.0;
    ramping = false;
  }

  float curve = smoothstep(t);

  if (!rampUp)
    curve = 1.0 - curve;

  currentSPS = targetSPS * curve;

  if (currentSPS < 1)
    currentSPS = 0;

  if (currentSPS > 0)
  {
    stepIntervalUs =
      1000000.0 /
      (currentSPS * 2.0);
  }

  if (!ramping && !rampUp)
  {
    motorRunning = false;
    currentSPS   = 0;
  }
}

// =====================================================
// HOMING
// =====================================================

void homeWireGuide()
{
  digitalWrite(WG_DIR_PIN, LOW);

  while (digitalRead(LIMIT_SWITCH))
  {
    digitalWrite(WG_STEP_PIN, HIGH);

    delayMicroseconds(10);   // pulse width (DRV8825 min ~2 µs)

    digitalWrite(WG_STEP_PIN, LOW);

    delayMicroseconds(302);  // 10 + 302 = 312 µs/step → 3200 steps/sec = 2.0 mm/sec
  }

  wireGuidePosition = 0;
}

// =====================================================
// MOVE WIRE GUIDE — blocking, used for zero offset travel
// =====================================================

void moveWireGuideSteps(long steps, bool dir)
{
  digitalWrite(WG_DIR_PIN, dir);

  for (long i = 0; i < steps; i++)
  {
    digitalWrite(WG_STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(WG_STEP_PIN, LOW);
    delayMicroseconds(302);  // 1.5 mm/sec
  }
}

// =====================================================
// UPDATE WIRE GUIDE RATIO
// =====================================================


void updateWireGuideRatio()
{
  float guideStepsPerTurn =
    wireDiameter * WG_STEPS_PER_MM;

  // round instead of truncate: guards against float representation error
  // (e.g. 0.23*1600 landing at 367.9999 instead of 368.0) biasing wgRatio low
  wgRatio =
    (long)(guideStepsPerTurn + 0.5f);

  layerSteps =
    (long)(coilLengthMM * WG_STEPS_PER_MM);
}

// =====================================================
// HANDLE LAYER LOGIC
// =====================================================

void handleLayerLogic()
{
  if (!layerBoundaryReached)
    return;

  layerBoundaryReached = false;

  wireGuideDir = !wireGuideDir;
  layerCount++;

  currentLayerPosition = 0;

  if (currentMode == SEMI_AUTO_MODE)
  {
    motorRunning = false;
  }
}

// =====================================================
// HANDLE MENU
// =====================================================

void handleMenu()
{
  static int lastEncoder = 0;

  if (encoderPosition != lastEncoder)
  {
    if (encoderPosition > lastEncoder)
      menuIndex++;
    else
      menuIndex--;

    if (menuIndex < 0) menuIndex = 3;
    if (menuIndex > 3) menuIndex = 0;

    lastEncoder = encoderPosition;
  }
}

// =====================================================
// HANDLE BUTTON
// =====================================================

void handleButtonEvent()
{
  if (!buttonPressed)
    return;

  buttonPressed = false;

  // =========================================
  // MENU
  // =========================================

  if (currentMode == MENU_MODE)
  {
    switch (menuIndex)
    {
      case 0:
        speedEncoderBase = encoderPosition;
        freshRun    = true;
        currentMode = MANUAL_MODE;
        break;
      case 1:
        speedEncoderBase = encoderPosition;
        freshRun    = true;
        currentMode = SEMI_AUTO_MODE;
        break;
      case 2:
        speedEncoderBase = encoderPosition;
        freshRun    = true;
        currentMode = AUTO_MODE;
        break;
      case 3:
        setupParamIndex  = 0;
        setupEncoderBase = encoderPosition;
        currentMode      = SETUP_MODE;
        break;
    }

    return;
  }

  // =========================================
  // HOMING — FIX: triggered by user, not blocking setup()
  // =========================================

  if (currentMode == HOMING_MODE)
  {
    homeWireGuide();

    zeroOffsetMM    = 0;
    zeroEncoderBase = encoderPosition;

    currentMode = ZERO_SET_MODE;

    return;
  }

  // =========================================
  // ZERO SET — confirm offset, drive to virtual zero
  // =========================================

  if (currentMode == ZERO_SET_MODE)
  {
    long targetSteps =
      (long)(zeroOffsetMM * WG_STEPS_PER_MM);

    moveWireGuideSteps(targetSteps, HIGH);

    wireGuidePosition = 0;

    setupParamIndex  = 0;
    setupEncoderBase = encoderPosition;
    currentMode      = SETUP_MODE;

    return;
  }

  // =========================================
  // SETUP — advance param or exit on last
  // =========================================

  if (currentMode == SETUP_MODE)
  {
    setupParamIndex++;

    if (setupParamIndex > 3)
    {
      setupParamIndex = 0;
      updateWireGuideRatio();
      currentMode = MENU_MODE;
    }
    else
    {
      setupEncoderBase = encoderPosition;
    }

    return;
  }

  // =========================================
  // MANUAL / SEMI / AUTO
  // =========================================

  if (currentMode == MANUAL_MODE    ||
      currentMode == SEMI_AUTO_MODE ||
      currentMode == AUTO_MODE)
  {
    // If winding finished, exit to menu instead of restarting
    if (windingComplete &&
        (currentMode == AUTO_MODE ||
         currentMode == SEMI_AUTO_MODE))
    {
      windingComplete = false;
      freshRun        = false;
      currentMode     = MENU_MODE;
      return;
    }

    targetState = !targetState;

    ramping    = true;
    rampStart  = millis();

    if (targetState)
    {
      if (freshRun)           // only reset counters on a brand-new run, not SEMI-AUTO resume
      {
        cli(); stepCount = 0; sei();
        layerCount   = 1;
        wireGuideDir = windLeftToRight;  // apply configured initial wind direction
        freshRun     = false;
      }
      digitalWrite(DIR_PIN,
        mainMotorForward ? HIGH : LOW);  // apply direction before ramp
      rampUp       = true;
      motorRunning = true;
    }
    else
    {
      rampUp = false;
    }
  }
}

// =====================================================
// LCD UPDATE
// =====================================================

void updateLCD()
{
  static unsigned long last        = 0;
  static SystemMode    lastMode       = (SystemMode)-1;  // invalid sentinel
  static int           lastMenuIdx    = -1;
  static int           lastSetupParam = -1;

  if (millis() - last < 200)
    return;

  last = millis();

  // only lcd.clear() on mode change, menu item change, or setup param change
  bool needsClear =
    (currentMode != lastMode) ||
    (currentMode == MENU_MODE  && menuIndex       != lastMenuIdx)    ||
    (currentMode == SETUP_MODE && setupParamIndex != lastSetupParam);

  if (needsClear)
  {
    lcd.clear();
    lastMode       = currentMode;
    lastMenuIdx    = menuIndex;
    lastSetupParam = setupParamIndex;
  }

  // =========================================
  // HOMING
  // =========================================

  if (currentMode == HOMING_MODE)
  {
    lcd.setCursor(0, 0);
    lcd.print("HOMING");
    lcd.setCursor(0, 1);
    lcd.print("Press to start");
    return;
  }

  // =========================================
  // MENU
  // =========================================

  if (currentMode == MENU_MODE)
  {
    lcd.setCursor(0, 0);
    lcd.print(">");
    lcd.print(menuItems[menuIndex]);
    return;
  }

  // =========================================
  // MANUAL
  // =========================================

  if (currentMode == MANUAL_MODE)
  {
    lcd.setCursor(0, 0);
    lcd.print("                ");  // clear first — sign char changes line width
    lcd.setCursor(0, 0);
    lcd.print("MAN S:");
    lcd.print(targetRPS, 1);
    lcd.print(" RPS");

    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.print(turns, 1);
    lcd.print("            ");
    return;
  }

  // =========================================
  // SEMI AUTO
  // =========================================

  if (currentMode == SEMI_AUTO_MODE)
  {
    lcd.setCursor(0, 0);
    lcd.print("SEMI S:");
    lcd.print(targetRPS, 1);
    lcd.print(" RPS ");

    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.print(turns, 1);
    lcd.print(" L:");
    lcd.print(layerCount);
    return;
  }

  // =========================================
  // AUTO
  // =========================================

  if (currentMode == AUTO_MODE)
  {
    lcd.setCursor(0, 0);
    lcd.print("AUTO S:");
    lcd.print(targetRPS, 1);
    lcd.print(" RPS ");

    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.print(turns, 1);
    lcd.print(" L:");
    lcd.print(layerCount);
    return;
  }

  // =========================================
  // SETUP
  // =========================================

  if (currentMode == SETUP_MODE)
  {
    lcd.setCursor(0, 0);
    if (setupParamIndex == 0)
      lcd.print(">Wire diam.[1/4]");
    else if (setupParamIndex == 1)
      lcd.print(">Coil len. [2/4]");
    else if (setupParamIndex == 2)
      lcd.print(">Turns     [3/4]");
    else
      lcd.print(">Wind dir. [4/4]");

    // clear line 1 then print value (avoids stale digits on value width change)
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);

    if (setupParamIndex == 0)
    {
      lcd.print(wireDiameter, 2);
      lcd.print(" mm");
    }
    else if (setupParamIndex == 1)
    {
      lcd.print(coilLengthMM, 1);
      lcd.print(" mm");
    }
    else if (setupParamIndex == 2)
    {
      int tpl = (wireDiameter > 0.0f)
        ? (int)(coilLengthMM / wireDiameter) : 1;
      int numLayers = (tpl > 0)
        ? ((int)targetTurnsValue + tpl - 1) / tpl : 1;
      lcd.print((int)targetTurnsValue);
      lcd.print("t ");
      lcd.print(numLayers);
      lcd.print("L");
    }
    else  // wind direction
    {
      if (windLeftToRight)
      {
        lcd.print("L ");
        lcd.write((byte)0x7E);  // HD44780 built-in right arrow
        lcd.print(" R");
      }
      else
      {
        lcd.print("R ");
        lcd.write((byte)0x7F);  // HD44780 built-in left arrow
        lcd.print(" L");
      }
    }
    return;
  }

  // =========================================
  // ZERO SET
  // =========================================

  if (currentMode == ZERO_SET_MODE)
  {
    lcd.setCursor(0, 0);
    lcd.print("ZERO OFFSET:    ");

    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print(zeroOffsetMM, 1);
    lcd.print(" mm");
    return;
  }
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  // MAIN MOTOR
  pinMode(STEP_PIN,   OUTPUT);
  pinMode(DIR_PIN,    OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(DIR_PIN,    HIGH);

  // WIRE GUIDE
  pinMode(WG_STEP_PIN,   OUTPUT);
  pinMode(WG_DIR_PIN,    OUTPUT);
  pinMode(WG_ENABLE_PIN, OUTPUT);

  digitalWrite(WG_ENABLE_PIN, LOW);

  // INPUTS
  pinMode(ENC_BTN,      INPUT_PULLUP);
  pinMode(ENC_A,        INPUT_PULLUP);
  pinMode(ENC_B,        INPUT_PULLUP);
  pinMode(LIMIT_SWITCH, INPUT_PULLUP);

  // INTERRUPTS
  attachInterrupt(
    digitalPinToInterrupt(ENC_BTN),
    buttonISR,
    FALLING);

  attachInterrupt(
    digitalPinToInterrupt(ENC_A),
    encoderISR,
    FALLING);  // FALLING fires once per detent; CHANGE cancelled itself out

  // LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Coil Winder");

  // TIMER
  setupTimer1();

  // FIX: start in HOMING_MODE; homing runs when user presses button
  // (previously homeWireGuide() was called here unconditionally,
  //  which hung forever if no limit switch was connected)
  currentMode = HOMING_MODE;

  // INITIAL RATIO
  updateWireGuideRatio();
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
  // atomic read of stepCount (2-byte access on AVR)
  int32_t steps;
  cli(); steps = stepCount; sei();

  turns = (float)steps / STEPS_PER_TURN;

  // =========================================
  // AUTO STOP — target turns reached
  // =========================================

  if ((currentMode == AUTO_MODE ||
       currentMode == SEMI_AUTO_MODE) &&
      motorRunning                    &&
      !ramping                        &&
      turns >= targetTurnsValue)
  {
    windingComplete = true;
    targetState = false;
    ramping     = true;
    rampUp      = false;
    rampStart   = millis();
  }

  // =========================================
  // MANUAL REVERSE FLOOR — stop at turns = 0
  // =========================================

  if (currentMode == MANUAL_MODE &&
      motorRunning               &&
      !mainMotorForward          &&
      turns <= 0.0f)
  {
    cli(); stepCount = 0; sei();
    turns        = 0.0f;
    motorRunning = false;
    targetState  = false;
    ramping      = false;
    currentSPS   = 0;
  }

  // =========================================
  // ZERO SET MODE
  // =========================================

  if (currentMode == ZERO_SET_MODE)
  {
    int raw = encoderPosition - zeroEncoderBase;
    if (raw < 0)    raw = 0;
    if (raw > 2000) raw = 2000;   // 0–200.0 mm in 0.1 mm steps
    zeroOffsetMM = raw * 0.1f;
  }

  // =========================================
  // SETUP MODE — encoder adjusts current parameter
  // =========================================

  if (currentMode == SETUP_MODE)
  {
    int delta = encoderPosition - setupEncoderBase;
    setupEncoderBase = encoderPosition;

    if (delta != 0)
    {
      if (setupParamIndex == 0)
      {
        wireDiameter += delta * 0.01f;
        if (wireDiameter < 0.05f) wireDiameter = 0.05f;
        if (wireDiameter > 1.0f)  wireDiameter = 1.0f;  // wire >1.00mm is wound manually
      }
      else if (setupParamIndex == 1)
      {
        coilLengthMM += delta;
        if (coilLengthMM < 1.0f)   coilLengthMM = 1.0f;
        if (coilLengthMM > 500.0f) coilLengthMM = 500.0f;

        // R->L: logical zero is the FAR point, switch is at distance 0, so the
        // coil can't be longer than the offset without crashing into the switch.
        if (!windLeftToRight && coilLengthMM > zeroOffsetMM)
          coilLengthMM = zeroOffsetMM;
      }
      else if (setupParamIndex == 2)
      {
        targetTurnsValue += delta;
        if (targetTurnsValue < 1.0f)    targetTurnsValue = 1.0f;
        if (targetTurnsValue > 9999.0f) targetTurnsValue = 9999.0f;
      }
      else // setupParamIndex == 3 -- wind direction (L->R / R->L)
      {
        windLeftToRight = (delta > 0);

        // re-validate coil length against the far-point offset when switching to R->L
        if (!windLeftToRight && coilLengthMM > zeroOffsetMM)
          coilLengthMM = zeroOffsetMM;
      }
    }
  }

  if (currentMode == MENU_MODE)
  {
    handleMenu();
  }

  // =========================================
  // RUNNING MODES — encoder adjusts speed
  // =========================================

  if (currentMode == MANUAL_MODE    ||
      currentMode == SEMI_AUTO_MODE ||
      currentMode == AUTO_MODE)
  {
    int delta = encoderPosition - speedEncoderBase;
    speedEncoderBase = encoderPosition;

    if (delta != 0)
    {
      targetRPS += delta * 0.1f;
      if (targetRPS < -3.0f) targetRPS = -3.0f;
      if (targetRPS >  3.0f) targetRPS =  3.0f;

      if (fabsf(targetRPS) < 0.05f)
      {
        // Dialled to 0.0 RPS: hard stop
        targetRPS    = 0.0f;
        targetSPS    = 0;
        motorRunning = false;
        targetState  = false;
        ramping      = false;
        currentSPS   = 0;
      }
      else
      {
        bool prevForward  = mainMotorForward;
        mainMotorForward  = (targetRPS >= 0.0f);
        targetSPS = (uint16_t)(MOTOR_STEPS * MICROSTEPS * fabsf(targetRPS));

        // If direction sign changed while running, update DIR_PIN immediately.
        // DRV8825 samples DIR at STEP rising edge; our min step interval (~52 µs
        // at the 3.0 RPS cap) is far longer than the 650 ns DIR setup time, so this is safe.
        if (mainMotorForward != prevForward && motorRunning)
        {
          digitalWrite(DIR_PIN, mainMotorForward ? HIGH : LOW);
        }

        // Apply speed magnitude immediately when at steady state
        if (!ramping && motorRunning && targetSPS > 0)
        {
          currentSPS = targetSPS;
          uint16_t newInterval =
            (uint16_t)(1000000.0 / (currentSPS * 2.0));
          cli(); stepIntervalUs = newInterval; sei();
        }
      }
    }
  }

  handleButtonEvent();

  handleLayerLogic();

  updateMotion();

  updateLCD();
}
