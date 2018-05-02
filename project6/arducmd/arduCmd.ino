/*******************************************************************************
 *
 * arduCmd.ino
 *
 * Arduino code to utilize an ESP8266 WiFi module to receiving commands and
 * display an HTTP status page. Uses an LED array and motor to simulate the
 * commands being executed. Orientation information is obtained from a BNO055
 * IMU connected using i2c.
*******************************************************************************/
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SoftwareSerial.h>
#include <utility/imumaths.h>
#include <Wire.h>

/* GPIO pinouts */
#define LED_RED             (12)
#define LED_BLUE            (11)
#define LED_GREEN           (10)
#define LED_YELLOW          (9)
#define MOTOR_PIN           (8)
#define WIFI_TX             (3)
#define WIFI_RX             (2)

/* Commands, must match the definitions in common.py */
#define CMD_MIN             (0x40)  // -1 of the first command

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

#define ROLL_STEP           (10)
#define PITCH_STEP          (10)
#define YAW_STEP            (10)

/* Be careful increasing this, you'll either saturate the Serial buffer at ~50
 * samples or run out of memory at ~100.
 */
#define ORIENTATION_SAMPLES (30)
#define ORIENTATION_DELAY   (5000)

typedef void (*cmdFunc_t)(void);

cmdFunc_t cmdTable[CMD_MAX];

unsigned char cmdBuf[CMD_BUF_LEN];
unsigned char execBuf[EXEC_BUF_LEN];
size_t cmdBufIdx;
size_t execBufIdx;

/* Roll/Pitch/Yaw values */
unsigned char rollPwm = 255;
unsigned char pitchPwm = 255;
unsigned char yawPwm = 255;

/* Circular buffers for orientation information */
float xVals[ORIENTATION_SAMPLES];
float yVals[ORIENTATION_SAMPLES];
float zVals[ORIENTATION_SAMPLES];
size_t xIdx = 0;
size_t yIdx = 0;
size_t zIdx = 0;

unsigned long orientationTime;
bool readIMU = false;

/* Speed of the motor */
unsigned char motorPwm = 0;

/* Step increase/decrease when changing motor speed */
const unsigned char motorStep = 10;

/* Networking */
SoftwareSerial esp(WIFI_TX, WIFI_RX);
int responseTime = 500;

/* IMU sensor */
Adafruit_BNO055 bno = Adafruit_BNO055(55);

/* Helper functions for turning LEDs on/off */
void redOn(void){       analogWrite(LED_RED, 255); }
void redOff(void){      analogWrite(LED_RED, 0); }
void blueOn(void){      analogWrite(LED_BLUE, 255); }
void blueOff(void){     analogWrite(LED_BLUE, 0); }
void greenOn(void){     analogWrite(LED_GREEN, 255); }
void greenOff(void){    analogWrite(LED_GREEN, 0); }
void yellowOn(void){    analogWrite(LED_YELLOW, 255); }
void yellowOff(void){   analogWrite(LED_YELLOW, 0); }

#if 0
/* strCmd
 *
 * Translates a command value into it corresponding string
 */
const char *
strCmd(int cmd)
{
    switch (cmd)
    {
    case CMD_START:             return "CMD_START";             break;
    case CMD_STOP:              return "CMD_STOP";              break;
    case CMD_THRUST_POS:        return "CMD_THRUST_POS";        break;
    case CMD_THRUST_NEG:        return "CMD_THRUST_NEG";        break;
    case CMD_PITCH_POS:         return "CMD_PITCH_POS";         break;
    case CMD_PITCH_NEG:         return "CMD_PITCH_NEG";         break;
    case CMD_ROLL_POS:          return "CMD_ROLL_POS";          break;
    case CMD_ROLL_NEG:          return "CMD_ROLL_NEG";          break;
    case CMD_YAW_POS:           return "CMD_YAW_POS";           break;
    case CMD_YAW_NEG:           return "CMD_YAW_NEG";           break;
    case CMD_PITCH_POS_TRIM:    return "CMD_PITCH_POS_TRIM";    break;
    case CMD_PITCH_NEG_TRIM:    return "CMD_PITCH_NEG_TRIM";    break;
    case CMD_ROLL_POS_TRIM:     return "CMD_ROLL_POS_TRIM";     break;
    case CMD_ROLL_NEG_TRIM:     return "CMD_ROLL_NEG_TRIM";     break;
    case CMD_YAW_POS_TRIM:      return "CMD_YAW_POS_TRIM";      break;
    case CMD_YAW_NEG_TRIM:      return "CMD_YAW_NEG_TRIM";      break;
    default:                    return "<unknown cmd>";         break;
    }

    return "<unknown cmd>";
}
#endif


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
    if (motorPwm < 0xFF)
    {
        // prevent wrapping
        if ((unsigned char) (motorPwm + motorStep) < motorPwm)
        {
            motorPwm = 0xFF;
        }
        else
        {
            motorPwm += motorStep;
        }

        analogWrite(MOTOR_PIN, motorPwm);
    }
}


void
funcCmdThrustNeg(void)
{
    if (motorPwm)
    {
        // prevent wrapping
        if ((unsigned char) (motorPwm - motorStep) > motorPwm)
        {
            motorPwm = 0;
        }
        else
        {
            motorPwm -= motorStep;
        }
        analogWrite(MOTOR_PIN, motorPwm);
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
    if (pitchPwm < 0xFF)
    {
        // prevent wrapping
        if ((unsigned char) (pitchPwm + PITCH_STEP) < pitchPwm)
        {
            pitchPwm = 0xFF;
        }
        else
        {
            pitchPwm += PITCH_STEP;
        }
    }
    delay(50);
    yellowOff();
}


void
funcCmdPitchNegTrim(void)
{
    allOff();
    blueOn();
    if (pitchPwm)
    {
        // prevent wrapping
        if ((unsigned char) (pitchPwm - PITCH_STEP) > pitchPwm)
        {
            pitchPwm = 0;
        }
        else
        {
            pitchPwm -= PITCH_STEP;
        }
    }
    delay(50);
    blueOff();
}


void
funcCmdRollPosTrim(void)
{
    allOff();
    greenOn();
    if (rollPwm < 0xFF)
    {
        // prevent wrapping
        if ((unsigned char) (rollPwm + ROLL_STEP) < rollPwm)
        {
            rollPwm = 0xFF;
        }
        else
        {
            rollPwm += ROLL_STEP;
        }
    }
    delay(50);
    greenOff();
}


void
funcCmdRollNegTrim(void)
{
    allOff();
    redOn();
    if (rollPwm)
    {
        // prevent wrapping
        if ((unsigned char) (rollPwm - ROLL_STEP) > rollPwm)
        {
            rollPwm = 0;
        }
        else
        {
            rollPwm -= ROLL_STEP;
        }
    }
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
    if (yawPwm < 0xFF)
    {
        // prevent wrapping
        if ((unsigned char) (yawPwm + YAW_STEP) < yawPwm)
        {
            yawPwm = 0xFF;
        }
        else
        {
            yawPwm += YAW_STEP;
        }
    }
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
    if (yawPwm)
    {
        // prevent wrapping
        if ((unsigned char) (yawPwm - YAW_STEP) > yawPwm)
        {
            yawPwm = 0;
        }
        else
        {
            yawPwm -= YAW_STEP;
        }
    }
    delay(50);
    blueOff();
    redOff();
    yellowOff();
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


/* sendToWifi
 *
 * Send a command to the WiFi module
 */
int
sendToWifi(String cmd, const unsigned int timeout, boolean debug)
{
    String response = "";
    unsigned long int time = 0;
    char c = '\0';

    if (debug)
    {
        Serial.print(F("Sending: "));
        Serial.println(cmd);
    }

    esp.println(cmd);
    delay(50);

    time = millis();
    while ((time + timeout) > millis())
    {
        while (esp.available())
        {
            c = esp.read();
            response += c;
            if (c == '\r')
            {
                c = esp.read();
                response += c;
                if (c == '\n')
                {
                    goto done_reading;
                }
            }
        }
    }

done_reading:

    if (debug)
    {
        Serial.print(F("Response:"));
        Serial.println(response);
    }

    if ( strstr(response.c_str(), "busy"))
    {
        delay(100);
    }

    if ( strstr(response.c_str(), "ERROR"))
    {
        Serial.println(F("Command failed\n"));
        return -1;
    }

    return 0;
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

    // Slide off executed commands, scrub the rest
    memmove(execBuf, execBuf + i, sizeof(execBuf) - i);
    execBufIdx -= i;
    memset(execBuf + execBufIdx, 0, sizeof(execBuf) - execBufIdx);
}


/* TIMER1_COMPA_vect
 *
 * Indicates that the IMU should be sampled
 * ISR for Timer1 Channel A
 */
ISR(TIMER1_COMPA_vect)
{
    readIMU = true;
}


/*  setup
 *
 *  Establishes i2c connection and receive callback and configures the LED and
 *  motor GPIO pins.
 */
void
setup(void)
{
    Serial.begin(57600);
    esp.begin(115200);
    //esp.println("AT+UART_DEF=9600,8,1,0,0");  // Change baud rate
    //sendToWifi("AT+RST", responseTime, true); // Reset module
    delay(100);
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

    // IMU configuration
    if ( !bno.begin())
    {
        Serial.println(F("BNO failed to initialize"));
        while (1)
        {
            allOn();
            delay(250);
            allOff();
            delay(250);
        }
    }

    bno.setExtCrystalUse(true);
    for (int i = 0; i < ORIENTATION_SAMPLES; i++)
    {
        xVals[i] = 0.0F;
        yVals[i] = 0.0F;
        zVals[i] = 0.0F;
    }
    orientationTime = millis();

    // Configure Timer1 for 100Hz interrupt
    // http://www.instructables.com/id/Arduino-Timer-Interrupts/
    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 155;   // (16 000 000 / (100 * 1024)) - 1
    //OCR1A = 15624;   // (16 000 000 / (100 * 1024)) - 1
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS12) | (1 << CS10);
    TIMSK1 |= (1 << OCIE1A);
    sei();


    // "Prime" the AT commands to get the errors out of the way
    sendToWifi(F("AT"), responseTime, true);

    // Don't echo back the commands we send
    sendToWifi(F("ATE0"), responseTime, true);

    // Set to access point mode
    sendToWifi(F("AT+CWMODE=2"), responseTime, true);

    // Configure access point:
    // SSID: ArduNet
    // Password: ardupass
    // Channel: 1
    // Encryption: WPA PSK (2)
    // Max connections: 10
    // Hidden SSID: No (0)
    sendToWifi(F("AT+CWSAP=\"ArduNet\",\"ardupass\",1,2,10,0"), responseTime, true);
    sendToWifi(F("AT+CWSAP?"), responseTime, true);

    // Set network to 192.168.6.1/24
    sendToWifi(F("AT+CIPAP=\"192.168.6.1\",\"192.168.6.1\",\"255.255.255.0\""),
               responseTime,
               true);

    // Display IP/MAC address
    sendToWifi(F("AT+CIFSR"), responseTime, true);

    // Allow multiple connections, required for servers
    sendToWifi(F("AT+CIPMUX=1"), responseTime, true);

    // Display remote IP/port with +IPD
    sendToWifi(F("AT+CIPDINFO=1"), responseTime, true);

    // Start server listening on TCP port 80
    sendToWifi(F("AT+CIPSERVER=1,80"), responseTime, true);

    // Close any existing connections (5 means close all)
    sendToWifi(F("AT+CIPCLOSE=5"), responseTime, true);

    // Start a UDP connection for commands
    // Link ID: 1   (so we know when *not* to response with a web page)
    // Type: UDP
    // Remote IP: 0.0.0.0
    // Remote Port: 0
    // Local port: 5000
    // Mode: Destination may change (2)
    sendToWifi(F("AT+CIPSTART=1,\"UDP\",\"0.0.0.0\",0,5000,2"),
               responseTime,
               true);

    Serial.println(F("Wifi connection running!"));
    Serial.flush();
    delay(1000);
}


/*  loop
 *
 *  Looks for commands/requests over WiFi and then responds to or executes them.
 */
void
loop(void)
{
    String page = "";
    int webId = 0;
    bool sendWeb = false;

    /* Handle web requests */
    if (esp.available())
    {
        if (esp.find((char *) "+IPD,"))
        {
            // Delay slightly to allow the serial buffer to fill
            delay(50);
            int id = esp.read() - '0';

            /* Connection ID 0 indicates an HTTP request, respond with the
             * status page regardless of what page was actually requested.
             */
            if (0 == id)
            {
                webId = id;     // This is redundant
                sendWeb = true;
            }
            /* Otherwise this is a command so add it to the buffer */
            else
            {
                esp.find((char *) ":");
                char cmd = esp.read();
                if ((cmdBufIdx < sizeof(cmdBuf)) &&
                    (cmd > CMD_MIN) &&
                    (cmd < CMD_MAX))
                {
                    cmdBuf[cmdBufIdx++] = cmd;
                }
            }
        }
    }

    /* This seems to work better */
    if (sendWeb)
    {
        Serial.println(F("Responding to web request"));
        String page = F("<html><h1>Quadcopter Status</h1>");
        page += F("</br>Motor PWM: ");
        page += motorPwm;
        page += F("</br>Roll PWM: ");
        page += rollPwm;
        page += F("</br>Pitch PWM: ");
        page += pitchPwm;
        page += F("</br>Yaw PWM: ");
        page += yawPwm;

        page += F("</html>");

        String resp = F("AT+CIPSEND=");
        resp += webId;
        resp += F(",");
        resp += page.length();
        resp += F("\r\n");

        String clos = F("AT+CIPCLOSE=");
        clos += webId;
        clos += F("\r\n");

        sendToWifi(resp, responseTime, false);
        esp.print(page);
    }

    /* Get positional information if enough time has passed */
    bool takeSample = false;
    noInterrupts();
    if (readIMU)
    {
        takeSample = true;
        readIMU = false;
    }
    interrupts();

    if (takeSample)
    {
        sensors_event_t event;
        bno.getEvent(&event);

        /* Write orientation information to a circular buffer */
        xVals[xIdx] = event.orientation.x;
        yVals[yIdx] = event.orientation.y;
        zVals[zIdx] = event.orientation.z;
        xIdx = (xIdx + 1) % ORIENTATION_SAMPLES;
        yIdx = (yIdx + 1) % ORIENTATION_SAMPLES;
        zIdx = (zIdx + 1) % ORIENTATION_SAMPLES;
    }

    /* Only display every ORIENTATION_DELAY milliseconds */
    if ((orientationTime + ORIENTATION_DELAY) <= millis())
    {
        orientationTime = millis();
        int i = 0;
        float xSamples[ORIENTATION_SAMPLES];
        float ySamples[ORIENTATION_SAMPLES];
        float zSamples[ORIENTATION_SAMPLES];
        size_t xSampleIdx = 0;
        size_t ySampleIdx = 0;
        size_t zSampleIdx = 0;

        noInterrupts();
        memcpy(xSamples, xVals, sizeof(xSamples));
        memcpy(ySamples, yVals, sizeof(ySamples));
        memcpy(zSamples, zVals, sizeof(zSamples));
        xSampleIdx = xIdx;
        ySampleIdx = yIdx;
        zSampleIdx = zIdx;
        interrupts();

        /* Iterate circularly from the oldest to newest sample, leaving
         * the last for after the loop so an extra comma isn't printed.
         */
        Serial.print(F("X: "));
        for (i = ORIENTATION_SAMPLES - 1; i >= 1; i--)
        {
            Serial.print(xSamples[(xSampleIdx + i) % ORIENTATION_SAMPLES], 4);
            Serial.print(F(", "));
            Serial.flush();
        }
        Serial.println(xSamples[(xSampleIdx + i) % ORIENTATION_SAMPLES], 4);

        Serial.flush();

        Serial.print(F("Y: "));
        for (i = ORIENTATION_SAMPLES - 1; i >= 1; i--)
        {
            Serial.print(ySamples[(ySampleIdx + i) % ORIENTATION_SAMPLES], 4);
            Serial.print(F(", "));
            Serial.flush();
        }
        Serial.println(ySamples[(ySampleIdx + i) % ORIENTATION_SAMPLES], 4);

        Serial.flush();

        Serial.print(F("Z: "));
        for (i = ORIENTATION_SAMPLES - 1; i >= 1; i--)
        {
            Serial.print(zSamples[(zSampleIdx + i) % ORIENTATION_SAMPLES], 4);
            Serial.print(F(", "));
            Serial.flush();
        }
        Serial.println(zSamples[(zSampleIdx + i) % ORIENTATION_SAMPLES], 4);
        Serial.println("");
    }

    // If new commands exist and we have room for them in the execute buffer,
    // add them
    noInterrupts();
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

