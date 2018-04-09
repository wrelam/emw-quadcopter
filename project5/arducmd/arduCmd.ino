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

/*  recvEvent
 *  n   Number of bytes read from the master device
 *
 *  Receives an event from the i2c master device and executes it.
 */
void
recvEvent(int n)
{
    unsigned char x = 0;
    while (Wire.available())
    {
        x = Wire.read();
        Serial.println(x);

        switch (x)
        {
        case CMD_START:
            allOn();
            break;

        case CMD_PITCH_NEG:
            allOff();
            blueOn();
            break;

        case CMD_PITCH_POS:
            allOff();
            yellowOn();
            break;

        case CMD_ROLL_NEG:
            allOff();
            redOn();
            break;

        case CMD_ROLL_POS:
            allOff();
            greenOn();
            break;

        case CMD_YAW_POS:
            allOff();
            blueOn();
            greenOn();
            yellowOn();
            break;

        case CMD_YAW_NEG:
            allOff();
            blueOn();
            redOn();
            yellowOn();
            break;

        case CMD_THRUST_POS:
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
            break;

        case CMD_THRUST_NEG:
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
            break;

        case CMD_PITCH_POS_TRIM:
            allOff();
            yellowOn();
            delay(50);
            yellowOff();
            break;

        case CMD_PITCH_NEG_TRIM:
            allOff();
            blueOn();
            delay(50);
            blueOff();
            break;

        case CMD_ROLL_POS_TRIM:
            allOff();
            greenOn();
            delay(50);
            greenOff();
            break;

        case CMD_ROLL_NEG_TRIM:
            allOff();
            redOn();
            delay(50);
            redOff();
            break;

        case CMD_YAW_POS_TRIM:
            allOff();
            blueOn();
            greenOn();
            yellowOn();
            delay(50);
            blueOff();
            greenOff();
            yellowOff();
            break;

        case CMD_YAW_NEG_TRIM:
            allOff();
            blueOn();
            redOn();
            yellowOn();
            delay(50);
            blueOff();
            redOff();
            yellowOff();
            break;

        case CMD_STOP:
        default:
            allOff();
            break;
        }
    }
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
}


/*  loop
 *
 *  Delays briefly.
 */
void
loop(void)
{
    delay(100);
}

