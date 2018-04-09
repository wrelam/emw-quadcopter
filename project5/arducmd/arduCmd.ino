/*******************************************************************************
 *
 * arduCmd.ino
 *
 * Arduino code for receiving commands over the i2c bus and executing the
 * commands by lighting various LEDs or spinning a motor up/down.
 *
*******************************************************************************/
#include <Wire.h>

/* GPIO pinouts */
#define LED_RED             (12)
#define LED_BLUE            (11)
#define LED_GREEN           (10)
#define LED_YELLOW          (9)
#define MOTOR_PIN           (8)

/* i2c device address, must match the definition in common.py */
#define I2C_DEVICE_ADDR     (0x10)

/* Commands, must match the definitions in common.py */
#define CMD_START           (0x41)  // Spin motors
#define CMD_STOP            (0x42)  // Turn off motors
#define CMD_THRUST_POS      (0x43)  // Up
#define CMD_THRUST_NEG      (0x44)  // Down
#define CMD_PITCH_POS       (0x45)  // Backward
#define CMD_PITCH_NEG       (0x46)  // Forward
#define CMD_ROLL_POS        (0x47)  // Right
#define CMD_ROLL_NEG        (0x48)  // Left
#define CMD_YAW_POS         (0x49)  // Spin counter clockwise
#define CMD_YAW_NEG         (0x4A)  // Spin clockwise
#define CMD_PITCH_POS_TRIM  (0x4B)  // Trim pitch positively
#define CMD_PITCH_NEG_TRIM  (0x4C)  // Trim pitch negatively
#define CMD_ROLL_POS_TRIM   (0x4D)  // Trim roll positively
#define CMD_ROLL_NEG_TRIM   (0x4E)  // Trim roll negatively
#define CMD_YAW_POS_TRIM    (0x4F)  // Trim yaw positively
#define CMD_YAW_NEG_TRIM    (0x50)  // Trim yaw negatively

#define CMD_MAX             (0x51)  // Set this to +1 of the last command

#define CMD_BUF_LEN         (32)
#define EXEC_BUF_LEN        (2 * CMD_BUF_LEN)

typedef void (*cmdFunc_t)(void);

cmdFunc_t cmdTable[CMD_MAX];

unsigned char cmdBuf[CMD_BUF_LEN];
unsigned char execBuf[EXEC_BUF_LEN];
size_t cmdBufIdx;
size_t execBufIdx;

/* Brightness of an LED when lit, 0-255 */
unsigned char ledBrightness = 255;

/* Step increase/decrease when changing LED brightness */
const unsigned char ledStep = 10;

/* Speed of the motor */
unsigned char motorSpeed = 0;

/* Step increase/decrease when changing motor speed */
const unsigned char motorStep = 10;

/* Helper functions for turning LEDs on/off */
void redOn(void){       digitalWrite(LED_RED, ledBrightness); }
void redOff(void){      digitalWrite(LED_RED, LOW); }
void blueOn(void){      digitalWrite(LED_BLUE, ledBrightness); }
void blueOff(void){     digitalWrite(LED_BLUE, LOW); }
void greenOn(void){     digitalWrite(LED_GREEN, ledBrightness); }
void greenOff(void){    digitalWrite(LED_GREEN, LOW); }
void yellowOn(void){    digitalWrite(LED_YELLOW, ledBrightness); }
void yellowOff(void){   digitalWrite(LED_YELLOW, LOW); }


void
allOn(void)
{
    redOn();
    blueOn();
    greenOn();
    yellowOn();
}


void
allOff(void)
{
    redOff();
    blueOff();
    greenOff();
    yellowOff();
}


void
funcCmdStart(void)
{
    allOn();
}


void
funcCmdStop(void)
{
    allOff();
}


void
funcCmdThrustPos(void)
{
    if (motorSpeed < 0xFF)
    {
        // prevent wrapping
        if ((unsigned char) (motorSpeed + motorStep) < motorSpeed)
        {
            motorSpeed = 0xFF;
        }
        else
        {
            motorSpeed += motorStep;
        }

        analogWrite(MOTOR_PIN, motorSpeed);
    }
}


void
funcCmdThrustNeg(void)
{
    if (motorSpeed)
    {
        // prevent wrapping
        if ((unsigned char) (motorSpeed - motorStep) > motorSpeed)
        {
            motorSpeed = 0;
        }
        else
        {
            motorSpeed -= motorStep;
        }
        analogWrite(MOTOR_PIN, motorSpeed);
    }

}


void
funcCmdPitchPos(void)
{
    allOff();
    yellowOn();
}


void
funcCmdPitchNeg(void)
{
    allOff();
    blueOn();
}


void
funcCmdRollPos(void)
{
    allOff();
    greenOn();
}


void
funcCmdRollNeg(void)
{
    allOff();
    redOn();
}


void
funcCmdYawPos(void)
{
    allOff();
    blueOn();
    greenOn();
    yellowOn();
}


void
funcCmdYawNeg(void)
{
    allOff();
    blueOn();
    redOn();
    yellowOn();
}


void
funcCmdPitchPosTrim(void)
{
    allOff();
    yellowOn();
    delay(50);
    yellowOff();
}


void
funcCmdPitchNegTrim(void)
{
    allOff();
    blueOn();
    delay(50);
    blueOff();
}


void
funcCmdRollPosTrim(void)
{
    allOff();
    greenOn();
    delay(50);
    greenOff();
}


void
funcCmdRollNegTrim(void)
{
    allOff();
    redOn();
    delay(50);
    redOff();
}


void
funcCmdYawPosTrim(void)
{
    allOff();
    blueOn();
    greenOn();
    yellowOn();
    delay(50);
    blueOff();
    greenOff();
    yellowOff();
}


void
funcCmdYawNegTrim(void)
{
    allOff();
    blueOn();
    redOn();
    yellowOn();
    delay(50);
    blueOff();
    redOff();
    yellowOff();
}


/*  recvEvent
 *  n   Number of bytes read from the master device
 *
 *  Receives an event from the i2c master device and writes it to the command
 *  buffer.
 */
void
recvEvent(int n)
{
    unsigned char x = 0;

    n = n;  // get rid of the warning

    while (Wire.available())
    {
        x = Wire.read();
        if (cmdBufIdx < CMD_BUF_LEN)
        {
            cmdBuf[cmdBufIdx++] = x;
        }
        else
        {
            // Ideally this is never printed, in testing I haven't seen it yet!
            Serial.println("X");
        }
    }
}


/*  setupCmdTable
 *
 *  Sets up the command table with command handlers.
 */
void
setupCmdTable(void)
{
    size_t i = 0;

    for (i = 0; i < sizeof(cmdTable) / sizeof(cmdTable[0]); i++)
    {
        cmdTable[i] = NULL;
    }

    cmdTable[CMD_START] = funcCmdStart;
    cmdTable[CMD_STOP] = funcCmdStop;
    cmdTable[CMD_THRUST_POS] = funcCmdThrustPos;
    cmdTable[CMD_THRUST_NEG] = funcCmdThrustNeg;
    cmdTable[CMD_PITCH_POS] = funcCmdPitchPos;
    cmdTable[CMD_PITCH_NEG] = funcCmdPitchNeg;
    cmdTable[CMD_ROLL_POS] = funcCmdRollPos;
    cmdTable[CMD_ROLL_NEG] = funcCmdRollNeg;
    cmdTable[CMD_YAW_POS] = funcCmdYawPos;
    cmdTable[CMD_YAW_NEG] = funcCmdYawNeg;
    cmdTable[CMD_PITCH_POS_TRIM] = funcCmdPitchPosTrim;
    cmdTable[CMD_PITCH_NEG_TRIM] = funcCmdPitchNegTrim;
    cmdTable[CMD_ROLL_POS_TRIM] = funcCmdRollPosTrim;
    cmdTable[CMD_ROLL_NEG_TRIM] = funcCmdRollNegTrim;
    cmdTable[CMD_YAW_POS_TRIM] = funcCmdYawPosTrim;
    cmdTable[CMD_YAW_NEG_TRIM] = funcCmdYawNegTrim;
}


/*  setup
 *
 *  Establishes i2c connection and receive callback and configures the LED and
 *  motor GPIO pins.
 */
void
setup(void)
{
    Wire.begin(I2C_DEVICE_ADDR);
    Wire.onReceive(recvEvent);
    Serial.begin(9600);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(MOTOR_PIN, OUTPUT);
    memset(cmdBuf, 0, sizeof(cmdBuf));
    memset(execBuf, 0, sizeof(execBuf));
    cmdBufIdx = 0;
    execBufIdx = 0;
    setupCmdTable();
}


/*  execCmds
 *
 *  Executes commands from the execute buffer.
 *
 *  This solves the shared data problem by having separate buffers. One buffer
 *  has received commands written to it in an interrupt context and we copy out
 *  of that buffer only when interrupts are disabled. We then executed commands
 *  from this copy so that we never "write to" or "execute from" at the same
 *  time.
 */
void
execCmds(void)
{
    size_t i = 0;

    // Execute the minimum number of commands to free the exec buffer for a full
    // cmd buffer
    for (i = 0; i < CMD_BUF_LEN; i++)
    {
        if (execBuf[i] == 0)
        {
            break;
        }

        cmdTable[execBuf[i]]();
    }

    // Slide off executed commands
    memmove(execBuf, execBuf + i, i);
    execBufIdx -= i;
}


/*  loop
 *
 *  Delays briefly.
 */
void
loop(void)
{
    noInterrupts();
    // If new commands exist and we have room for them in the execute buffer
    if (cmdBufIdx &&
        ((EXEC_BUF_LEN - execBufIdx) >= cmdBufIdx))
    {
        // Copy new commands to the end of the execute buffer
        memcpy(execBuf + execBufIdx, cmdBuf, cmdBufIdx * sizeof(execBuf[0]));
        execBufIdx += cmdBufIdx;
        cmdBufIdx = 0;
    }
    interrupts();

    if (execBufIdx)
    {
        execCmds();
    }
}

