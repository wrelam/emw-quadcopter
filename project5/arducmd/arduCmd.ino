/*******************************************************************************
 *
 * arduCmd.ino
 *
 * Arduino code for receiving commands over the i2c bus and executing the
 * commands by lighting various LEDs or spinning a motor up/down.
 *
*******************************************************************************/
#include <string.h>
#include <SoftwareSerial.h>
#include <Wire.h>

/* GPIO pinouts */
#define LED_RED             (12)
#define LED_BLUE            (11)
#define LED_GREEN           (10)
#define LED_YELLOW          (9)
#define MOTOR_PIN           (8)
#define WIFI_TX             (3)
#define WIFI_RX             (2)

/* i2c device address, must match the definition in common.py */
#define I2C_DEVICE_ADDR     (0x10)

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

typedef void (*cmdFunc_t)(void);

cmdFunc_t cmdTable[CMD_MAX];

unsigned char cmdBuf[CMD_BUF_LEN];
unsigned char execBuf[EXEC_BUF_LEN];
size_t cmdBufIdx;
size_t execBufIdx;

/* Brightness of an LED when lit, 0-255 */
unsigned char rollPwm = 255;
unsigned char rollStep = 10;
unsigned char pitchPwm = 255;
unsigned char pitchStep = 10;
unsigned char yawPwm = 255;
unsigned char yawStep = 10;

/* Step increase/decrease when changing LED brightness */
const unsigned char ledStep = 51;

/* Speed of the motor */
unsigned char motorSpeed = 0;

/* Step increase/decrease when changing motor speed */
const unsigned char motorStep = 10;

/* Networking */
SoftwareSerial esp(WIFI_TX, WIFI_RX);
int responseTime = 500;

/* Helper functions for turning LEDs on/off */
void redOn(void){       analogWrite(LED_RED, 255); }
void redOff(void){      analogWrite(LED_RED, 0); }
void blueOn(void){      analogWrite(LED_BLUE, 255); }
void blueOff(void){     analogWrite(LED_BLUE, 0); }
void greenOn(void){     analogWrite(LED_GREEN, 255); }
void greenOff(void){    analogWrite(LED_GREEN, 0); }
void yellowOn(void){    analogWrite(LED_YELLOW, 255); }
void yellowOff(void){   analogWrite(LED_YELLOW, 0); }

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
    if (pitchPwm < 0xFF)
    {
        // prevent wrapping
        if ((unsigned char) (pitchPwm + pitchStep) < pitchPwm)
        {
            pitchPwm = 0xFF;
        }
        else
        {
            pitchPwm += pitchStep;
        }
    }
}


void
funcCmdPitchNegTrim(void)
{
    allOff();
    blueOn();
    if (pitchPwm)
    {
        // prevent wrapping
        if ((unsigned char) (pitchPwm - pitchStep) > pitchPwm)
        {
            pitchPwm = 0;
        }
        else
        {
            pitchPwm -= pitchStep;
        }
    }
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
        if ((unsigned char) (rollPwm + rollStep) < rollPwm)
        {
            rollPwm = 0xFF;
        }
        else
        {
            rollPwm += rollStep;
        }
    }
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
        if ((unsigned char) (rollPwm - rollStep) > rollPwm)
        {
            rollPwm = 0;
        }
        else
        {
            rollPwm -= rollStep;
        }
    }
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
        if ((unsigned char) (yawPwm + yawStep) < yawPwm)
        {
            yawPwm = 0xFF;
        }
        else
        {
            yawPwm += yawStep;
        }
    }
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
        if ((unsigned char) (yawPwm - yawStep) > yawPwm)
        {
            yawPwm = 0;
        }
        else
        {
            yawPwm -= yawStep;
        }
    }
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


int
sendToWifi(String cmd, const unsigned int timeout, boolean debug)
{
    String response = "";
    unsigned long int time = 0;
    char c = '\0';

    if (debug)
    {
        Serial.print("Sending: ");
        Serial.println(cmd);
    }

    esp.println(cmd);
    delay(100);

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
        Serial.print("Response:");
        Serial.println(response);
    }

    if ( strstr(response.c_str(), "busy"))
    {
        delay(500);
    }

    if ( strstr(response.c_str(), "ERROR"))
    {
        Serial.println("Command failed\n");
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

    //Serial.println("Executing");

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
    //memmove(execBuf, execBuf + i, i);
    memmove(execBuf, execBuf + i, sizeof(execBuf) - i);
    execBufIdx -= i;
    memset(execBuf + execBufIdx, 0, sizeof(execBuf) - execBufIdx);
}

#if 0
void
handleConnections(void)
{
    //int res = -1;
    String response = "";
    bool stop = false;
    unsigned char msgBuf[BUFSIZ] = { 0 };

    //esp.println("AT+IPD,0,50");
    readUntil(msgBuf, sizeof(msgBuf), '\n');
    while (!stop)
    {
        while (esp.available())
        {
            char c = esp.read();
            response += c;
            if ('\n' == c)
            {
                stop = true;
            }
        }
    }

    //if ( strlen(response.c_str()) &&
         //!strstr(response.c_str(), "ERROR"))
    {
        Serial.println("Received:");
        Serial.println(response);
    }
}

uint64_t
parseDecimal(char *buf, size_t buflen, int stopchar)
{
    uint64_t num = 0;
    char numBuf[50] = { 0 };
    size_t nIdx = 0;
    size_t i = 0;
    uint32_t multiplier = 0;
    char c = 0;

    Serial.println("Parsing decimal");
    return 0;

    /* Read chars into buffer */
    c = esp.read();
    while (c != stopchar)
    {
        Serial.print("Read: ");
        Serial.println(c);
        numBuf[nIdx++] = c;
        c = esp.read();
    }

    multiplier = pow(10, strlen(numBuf) - 1);
    for (i = 0; i < nIdx; i++)
    {
        num += multiplier * (numBuf[i] - '0');
        multiplier /= 10;
    }

    return num;
}


void
parseRequest(
    char *buf,
    size_t buflen,
    int *pId,
    size_t *pLen,
    uint32_t *pIp,
    uint16_t *pPort,
    char *req,
    size_t reqLen,
    char *name,
    size_t nameLen)
{
    size_t i = 0;
    Serial.println("Parsing request");

    if (!pId || !pLen || !pIp || !req || !name)
    {
        Serial.println("Bad pointer");
        return;
    }

    *pId = parseDecimal(',');

    *pLen = parseDecimal(',');

    *pIp = parseDecimal('.');
    *pIp <<= 24;
    *pIp |= parseDecimal('.');
    *pIp <<= 16;
    *pIp |= parseDecimal('.');
    *pIp <<= 8;
    *pIp |= parseDecimal(',');

    *pPort = parseDecimal(':');

        //readUntil((unsigned char *) req, reqLen, ' ');
        //readUntil((unsigned char *) name, nameLen, ' ');
    }
}
#endif

int
readUntil(unsigned char *buf, size_t buflen, int stopchar)
{
    unsigned char ch = 0;
    int res = -1;

    if (!buf || (buflen <= 1))
    {
        return res;
    }

    while (1)
    {
        if (0 == buflen)
        {
            res = -1;
            break;
        }

        ch = esp.read();
        *buf++ = ch;
        buflen--;

        if (ch == stopchar)
        {
            if (buflen)
            {
                *buf = '\0';
            }
            else
            {
                *--buf = '\0';
            }
            break;
        }
    }

    return res;
}


/*  setup
 *
 *  Establishes i2c connection and receive callback and configures the LED and
 *  motor GPIO pins.
 */
void
setup(void)
{
    Serial.begin(9600);
    //esp.begin(115200);
    //esp.println("AT+UART_DEF=9600,8,1,0,0");
    esp.begin(9600);
    //sendToWifi("AT+RST", responseTime, true);
    delay(1000);
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

    // "Prime" the AT commands to get the errors out of the way
    sendToWifi("AT", responseTime, true);
    sendToWifi("ATE0", responseTime, true);

    // Set to access point mode
    sendToWifi("AT+CWMODE=2", responseTime, true);

    // Configure access point:
    // SSID: ArduNet
    // Password: ardupass
    // Channel: 1
    // Encryption: WPA PSK (2)
    // Max connections: 10
    // Hidden SSID: No (0)
    sendToWifi("AT+CWSAP=\"ArduNet\",\"ardupass\",1,2,10,0", responseTime, true);
    sendToWifi("AT+CWSAP?", responseTime, true);

    // Set network to 192.168.6.1/24
    sendToWifi("AT+CIPAP=\"192.168.6.1\",\"192.168.6.1\",\"255.255.255.0\"",
               responseTime,
               true);

    // Display IP/MAC
    sendToWifi("AT+CIFSR", responseTime, true);

    // Allow multiple connections, required for servers
    sendToWifi("AT+CIPMUX=1", responseTime, true);

    // Display remote IP/port with +IPD
    sendToWifi("AT+CIPDINFO=1", responseTime, true);

    // Start server listening on TCP port 80
    sendToWifi("AT+CIPSERVER=1,80", responseTime, true);

    // Close any stale connections
    sendToWifi("AT+CIPCLOSE=5", responseTime, true);

    // Start a UDP connection for commands
    sendToWifi("AT+CIPSTART=1,\"UDP\",\"0.0.0.0\",0,5000,2", responseTime, true);

    Serial.println("Wifi connection running!");
}


/*  loop
 *
 *  Delays briefly.
 */
void
loop(void)
{
    /* Handle web requests */
    if (esp.available())
    {
        if (esp.find((char *) "+IPD,"))
        {
            delay(35);
            int id = esp.read() - '0';

            if (0 == id)
            {
                Serial.println("Responding to web request");
                String page = "<html><h1>Quadcopter Status</h1>";
                page += "</br>Motor Speed: ";
                page += motorSpeed;
                page += "</br>Roll PWM: ";
                page += rollPwm;
                page += "</br>Pitch PWM: ";
                page += pitchPwm;
                page += "</br>Yaw PWM: ";
                page += yawPwm;
                page += "</html>";

                String resp = "AT+CIPSEND=";
                resp += id;
                resp += ",";
                resp += page.length();
                resp += "\r\n";

                String clos = "AT+CIPCLOSE=";
                clos += id;
                clos += "\r\n";

                sendToWifi(resp, responseTime, false);
                sendToWifi(page, responseTime, false);
                sendToWifi(clos, responseTime, false);
            }
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

    // If new commands exist and we have room for them in the execute buffer
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

