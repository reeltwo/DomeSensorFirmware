///////////////////////////////////

#if __has_include("build_version.h")
#include "build_version.h"
#endif

#if __has_include("reeltwo_build_version.h")
#include "reeltwo_build_version.h"
#endif

///////////////////////////////////

// #define USE_DEBUG
// #define USE_DOME_SENSOR_DEBUG
#include "ReelTwo.h"
#include "drive/DomeSensorRing.h"
#include "core/EEPROMSettings.h"
#include "core/StringUtils.h"
#include "Adafruit_MCP4725.h"
#include <EEPROM.h>

//Define for Stef's full size dome sensor ring 
//#define DOME_SENSOR_RING_FULL_SIZE

///////////////////////////////////
// CONFIGURABLE OPTIONS
///////////////////////////////////

// If debug is enabled the serial baud rate will be 57600
#define SERIAL_BAUD_RATE                    57600
#define CONSOLE_BUFFER_SIZE                 300
#define COMMAND_BUFFER_SIZE                 256

#ifndef DOME_SENSOR_RING_FULL_SIZE
#define DAC_I2C                             0x60
#endif
#ifdef DAC_I2C
#define DEFAULT_ANALOG_MIN_REFERENCE        0
#define DEFAULT_ANALOG_MAX_REFERENCE        1023
#endif

#define POSITION_RESEND_INTERVAL            1000  // milliseconds
#define TEST_DURATION_SECONDS               60

///////////////////////////////////

struct DomeSensorSettings
{
    uint32_t fBaudRate = SERIAL_BAUD_RATE;
#ifdef DAC_I2C
    uint16_t fMinAnalogOut = DEFAULT_ANALOG_MIN_REFERENCE;
    uint16_t fMaxAnalogOut = DEFAULT_ANALOG_MAX_REFERENCE;
#endif
};

EEPROMSettings<DomeSensorSettings> sSettings;

///////////////////////////////////

static bool sNextCommand;
static bool sProcessing;
static unsigned sPos;
static uint32_t sWaitNextSerialCommand;
static char sBuffer[CONSOLE_BUFFER_SIZE];
static bool sCmdNextCommand;
static char sCmdBuffer[COMMAND_BUFFER_SIZE];
static uint32_t sEndTesting;
static bool sTestOne;
static unsigned sSensorMask;

///////////////////////////////////

#ifdef DAC_I2C
Adafruit_MCP4725 sDAC;
#endif
DomeSensorRing sDomePosition;

///////////////////////////////////

void setup()
{
    REELTWO_READY();

    if (sSettings.read())
    {
    #ifndef USE_DEBUG
        Serial.begin(sSettings.fBaudRate);
    #endif
        PrintReelTwoInfo(Serial, "Dome Sensor");
        Serial.println(F("Settings Restored"));
    }
    else
    {
    #ifndef USE_DEBUG
        Serial.begin(sSettings.fBaudRate);
    #endif
        PrintReelTwoInfo(Serial, "Dome Sensor");
        Serial.println(F("First Time Settings"));
        sSettings.write();
        if (sSettings.read())
        {
            Serial.println(F("Readback Success"));
        }
    }

    SetupEvent::ready();
#ifdef DAC_I2C
    sDAC.begin(DAC_I2C);
#endif
}

///////////////////////////////////

static void updateSettings()
{
    sSettings.write();
    Serial.println(F("Updated"));
}

///////////////////////////////////

static void runSerialCommand()
{
    sWaitNextSerialCommand = 0;
    sProcessing = true;
}

static void resetSerialCommand()
{
    sWaitNextSerialCommand = 0;
    sNextCommand = false;
    sProcessing = (sCmdBuffer[0] == ':');
    sPos = 0;
}

bool processDomeRingCommand(const char* cmd)
{
    switch (*cmd++)
    {
        default:
            // No dome ring commands
            break;
    }
    return true;
}

void processConfigureCommand(const char* cmd)
{
    if (startswith(cmd, "#DPZERO"))
    {
        DomeSensorSettings defaultSettings;
        *sSettings.data() = defaultSettings;
        updateSettings();
    }
    else if (startswith(cmd, "#DPCONFIG"))
    {
        Serial.print("BaudRate="); Serial.println(sSettings.fBaudRate);
    #ifdef DAC_I2C
        Serial.print("MinAnalog="); Serial.println(sSettings.fMinAnalogOut);
        Serial.print("MaxAnalog="); Serial.println(sSettings.fMaxAnalogOut);
    #endif
    }
    else if (startswith(cmd, "#DPTEST"))
    {
        sTestOne = (*cmd == '1');
        if (sTestOne)
        {
            Serial.print("Test one sensor at a time. ");
        }
        else
        {
            Serial.print("Testing. Rotate dome 360 degrees. ");
        }
        Serial.println("Test ends in "+String(TEST_DURATION_SECONDS)+" seconds or when all sensors have registered.");
        sEndTesting = millis() + TEST_DURATION_SECONDS * 1000L;
        sSensorMask = 0;
    }
    else if (startswith(cmd, "#DPBAUD"))
    {
        uint32_t baudrate = strtolu(cmd, &cmd);
        if (baudrate > 1200 && sSettings.fBaudRate != baudrate)
        {
            sSettings.fBaudRate = baudrate;
            Serial.print("Reboot baud rate: "); Serial.println(sSettings.fBaudRate);
            updateSettings();
        }
    }
#ifdef DAC_I2C
    else if (startswith(cmd, "#DPMIN"))
    {
        uint32_t analogout = strtolu(cmd, &cmd);
        if (analogout > 0 && analogout <= 4096 && sSettings.fMinAnalogOut != analogout)
        {
            sSettings.fMinAnalogOut = analogout;
            Serial.print("Reboot min analog out: "); Serial.println(sSettings.fMinAnalogOut);
            updateSettings();
        }
    }
    else if (startswith(cmd, "#DPMAX"))
    {
        uint32_t analogout = strtolu(cmd, &cmd);
        if (analogout > 0 && analogout <= 4096 && sSettings.fMaxAnalogOut != analogout)
        {
            sSettings.fMaxAnalogOut = analogout;
            Serial.print("Reboot max analog out: "); Serial.println(sSettings.fMaxAnalogOut);
            updateSettings();
        }
    }
#endif
}

bool processCommand(const char* cmd, bool firstCommand)
{
    sWaitNextSerialCommand = 0;
    if (*cmd == '\0')
        return true;
    if (!firstCommand)
    {
        if (cmd[0] != ':')
        {
            Serial.println(F("Invalid"));
            return false;
        }
        return processDomeRingCommand(cmd+1);
    }
    switch (cmd[0])
    {
        case ':':
            if (cmd[1] == 'D')
                return processDomeRingCommand(cmd+2);
            break;
        case '#':
            processConfigureCommand(cmd);
            return true;
        default:
            Serial.println(F("Invalid"));
            break;
    }
    return false;
}

///////////////////////////////////

static unsigned countChangedBits(unsigned a, unsigned b)
{
    unsigned n = 0;
    for (unsigned i = 0; i < 9; i++) 
    {
        if ((a & (1 << i)) != (b & (1 << i)))
            n++;
    }
    return n;
}

static void printBinary(unsigned num, unsigned places)
{
    if (places)
        printBinary(num >> 1, places-1);
    Serial.print((num & 1) ? '1' : '0');
}

void loop()
{
    static short sLastAngle = -1;
    static uint32_t sLastReport = 0;
    short angle = sDomePosition.getAngle();
    if (sDomePosition.ready())
    {
        // Output the position if it changed or one second has passed
        if (angle != sLastAngle || millis() - sLastReport > POSITION_RESEND_INTERVAL)
        {
        #ifndef USE_DEBUG
            Serial.print(String("#DP@") + String(angle) + String("\r"));
        #endif
            sLastAngle = angle;
        #ifdef DAC_I2C
            sDAC.setVoltage(map(angle, 0, 359,
                sSettings.fMinAnalogOut, sSettings.fMaxAnalogOut), false);
        #endif
            sLastReport = millis();
        }
    }

    if (sEndTesting > 0)
    {
        unsigned sensors = (~sDomePosition.readSensors() & 0x1FF);
        sSensorMask |= sensors;
        if (sTestOne && countChangedBits(0, sensors) > 1)
        {
            Serial.print("Test failed. These sensors are reading at the same time: ");
            sEndTesting = 0;
            for (unsigned i = 0; i < 9; i++)
            {
                if ((sensors & (1<<i)) != 0)
                {
                    Serial.print(i+1);
                    Serial.print(" ");
                }
            }
            Serial.println();
        }
        if (sSensorMask == 0x1FF)
        {
            Serial.println("All sensors registered. Ending test.");
            sEndTesting = 0;
        }
        else if (sEndTesting < millis())
        {
            Serial.print("Test timeout. Following sensors did not register: ");
            for (unsigned i = 0; i < 9; i++)
            {
                if ((sSensorMask & (1<<i)) == 0)
                {
                    Serial.print(i+1);
                    Serial.print(" ");
                }
            }
            Serial.println();
            sEndTesting = 0;
        }
    }
    // append commands to command buffer
    if (Serial.available())
    {
        int ch = Serial.read();
        if (ch == '\r' || ch == '\n')
        {
            runSerialCommand();
        }
        else if (sPos < SizeOfArray(sBuffer)-1)
        {
            sBuffer[sPos++] = ch;
            sBuffer[sPos] = '\0';
        }
    }
    if (sProcessing && millis() > sWaitNextSerialCommand)
    {
        if (sCmdBuffer[0] == ':')
        {
            char* end = strchr(sCmdBuffer+1, ':');
            if (end != nullptr)
                *end = '\0';
            if (!processCommand(sCmdBuffer, !sCmdNextCommand))
            {
                // command invalid abort buffer
                DEBUG_PRINT(F("Unrecognized: ")); DEBUG_PRINTLN(sCmdBuffer);
                sWaitNextSerialCommand = 0;
                end = nullptr;
            }
            if (end != nullptr)
            {
                *end = ':';
                strcpy(sCmdBuffer, end);
                DEBUG_PRINT(F("REMAINS: "));
                DEBUG_PRINTLN(sCmdBuffer);
                sCmdNextCommand = true;
            }
            else
            {
                sCmdBuffer[0] = '\0';
                sCmdNextCommand = false;
            }
        }
        else if (sBuffer[0] == ':')
        {
            char* end = strchr(sBuffer+1, ':');
            if (end != nullptr)
                *end = '\0';
            if (!processCommand(sBuffer, !sNextCommand))
            {
                // command invalid abort buffer
                DEBUG_PRINT(F("Unrecognized: ")); DEBUG_PRINTLN(sBuffer);
                sWaitNextSerialCommand = 0;
                end = nullptr;
            }
            if (end != nullptr)
            {
                *end = ':';
                strcpy(sBuffer, end);
                sPos = strlen(sBuffer);
                DEBUG_PRINT(F("REMAINS: "));
                DEBUG_PRINTLN(sBuffer);
                sNextCommand = true;
            }
            else
            {
                resetSerialCommand();
                sBuffer[0] = '\0';
            }
        }
        else
        {
            processCommand(sBuffer, true);
            resetSerialCommand();
        }
    }
}
