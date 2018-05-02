/*******************************************************************************
 *
 * arduCmd.ino
 *
 * Arduino code to utilize an ESP8266 WiFi module to receiving commands and
 * display an HTTP status page. Uses an LED array and motor to simulate the
 * commands being executed.
*******************************************************************************/
#include <SoftwareSerial.h>

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

typedef void (*cmdFunc_t)(void);

cmdFunc_t cmdTable[CMD_MAX];

unsigned char cmdBuf[CMD_BUF_LEN];
unsigned char execBuf[EXEC_BUF_LEN];
size_t cmdBufIdx;
size_t execBufIdx;

/* Roll/Pitch/Yaw values and steps */
unsigned char rollPwm = 255;
unsigned char rollStep = 10;
unsigned char pitchPwm = 255;
unsigned char pitchStep = 10;
unsigned char yawPwm = 255;
unsigned char yawStep = 10;

/* Speed of the motor */
unsigned char motorPwm = 0;

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

#if 0
/*******************************************************************************
    strCmd
*//**
    @brief  Returns the string corresponding to a command value
    @param  cmd Command to be translated

    Useful for debugging.

    @return String representation of a command
    @retval "<unknown cmd>" Command is unknown
*******************************************************************************/
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
        if ((unsigned char) (pitchPwm + pitchStep) < pitchPwm)
        {
            pitchPwm = 0xFF;
        }
        else
        {
            pitchPwm += pitchStep;
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
        if ((unsigned char) (pitchPwm - pitchStep) > pitchPwm)
        {
            pitchPwm = 0;
        }
        else
        {
            pitchPwm -= pitchStep;
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
        if ((unsigned char) (rollPwm + rollStep) < rollPwm)
        {
            rollPwm = 0xFF;
        }
        else
        {
            rollPwm += rollStep;
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
        if ((unsigned char) (rollPwm - rollStep) > rollPwm)
        {
            rollPwm = 0;
        }
        else
        {
            rollPwm -= rollStep;
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
        if ((unsigned char) (yawPwm + yawStep) < yawPwm)
        {
            yawPwm = 0xFF;
        }
        else
        {
            yawPwm += yawStep;
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
        if ((unsigned char) (yawPwm - yawStep) > yawPwm)
        {
            yawPwm = 0;
        }
        else
        {
            yawPwm -= yawStep;
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


/*******************************************************************************
    sendToWifi
*//**
    @brief  Sends a command to the WiFi module
    @param  cmd     Command string to be sent
    @param  timeout # of milliseconds to wait for a response
    @param  debug   To print debug information or not

    @return Success indication
    @retval 0   Success
    @retval -1  Error
*******************************************************************************/
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
        Serial.print("Response:");
        Serial.println(response);
    }

    if ( strstr(response.c_str(), "busy"))
    {
        delay(100);
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


/*  setup
 *
 *  Establishes i2c connection and receive callback and configures the LED and
 *  motor GPIO pins.
 */
void
setup(void)
{
    Serial.begin(9600);
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

    // "Prime" the AT commands to get the errors out of the way
    sendToWifi("AT", responseTime, true);

    // Don't echo back the commands we send
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

    // Display IP/MAC address
    sendToWifi("AT+CIFSR", responseTime, true);

    // Allow multiple connections, required for servers
    sendToWifi("AT+CIPMUX=1", responseTime, true);

    // Display remote IP/port with +IPD
    sendToWifi("AT+CIPDINFO=1", responseTime, true);

    // Start server listening on TCP port 80
    sendToWifi("AT+CIPSERVER=1,80", responseTime, true);

    // Close any existing connections (5 means close all)
    sendToWifi("AT+CIPCLOSE=5", responseTime, true);

    // Start a UDP connection for commands
    // Link ID: 1   (so we know when *not* to response with a web page)
    // Type: UDP
    // Remote IP: 0.0.0.0
    // Remote Port: 0
    // Local port: 5000
    // Mode: Destination may change (2)
    sendToWifi("AT+CIPSTART=1,\"UDP\",\"0.0.0.0\",0,5000,2", responseTime, true);

    Serial.println("Wifi connection running!");
}


/*  loop
 *
 *  Looks for commands/requests over WiFi and then responds to or executes them.
 */
void
loop(void)
{
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
                Serial.println("Responding to web request");
                String page = "<html><h1>Quadcopter Status</h1>";
                page += "</br>Motor PWM: ";
                page += motorPwm;
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

