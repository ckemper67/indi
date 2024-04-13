/*
    DarkSkyGeek Focuser (darkskygeek) INDI Focuser

    Based on aaf2 and DarkSkyGeek ASCOM Driver
    
    Copyright (C) 2026 Christian Kemper (ckemper@gmail.com)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include "darkskygeek.h"
#include "connectionplugins/connectionserial.h"

#include "indicom.h"

#include <cstring>
#include <memory>

#include <termios.h>
#include <unistd.h>

static std::unique_ptr<DarkSkyGeek> darkSkyGeek(new DarkSkyGeek());

// Constants used to communicate with the device
// Make sure those values are identical to those in the Arduino Firmware.
//static const char* SEPARATOR = "\n";
//static const char* DEVICE_GUID = "6e18ce4b-0d7b-4850-8470-80df623bf0a4";

static const char* OK = "OK";

static const char* TRUE = "TRUE";
static const char* FALSE = "FALSE";

static const char* COMMAND_PING = "COMMAND:PING";
static const char* RESULT_PING = "RESULT:PING:OK:6e18ce4b-0d7b-4850-8470-80df623bf0a4";

static const char* COMMAND_INFO = "COMMAND:INFO";
static const char* RESULT_INFO = "RESULT:INFO:";

static const char* COMMAND_FOCUSER_GETPOSITION = "COMMAND:FOCUSER:GETPOSITION";
static const char* RESULT_FOCUSER_POSITION = "RESULT:FOCUSER:POSITION:";

static const char* COMMAND_FOCUSER_ISMOVING = "COMMAND:FOCUSER:ISMOVING";
static const char* RESULT_FOCUSER_ISMOVING = "RESULT:FOCUSER:ISMOVING:";


static const char* COMMAND_FOCUSER_SETZEROPOSITION = "COMMAND:FOCUSER:SETZEROPOSITION";
static const char* RESULT_FOCUSER_SETZEROPOSITION = "RESULT:FOCUSER:SETZEROPOSITION:";

static const char* COMMAND_FOCUSER_MOVE = "COMMAND:FOCUSER:MOVE:";
static const char* RESULT_FOCUSER_MOVE = "RESULT:FOCUSER:MOVE:";

static const char* COMMAND_FOCUSER_HALT = "COMMAND:FOCUSER:HALT";
static const char* RESULT_FOCUSER_HALT = "RESULT:FOCUSER:HALT:";

DarkSkyGeek::DarkSkyGeek()
{
    // Absolute, Abort, and Sync
    FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT | FOCUSER_CAN_SYNC);

    setVersion(1, 0);
}

bool DarkSkyGeek::initProperties()
{
    INDI::Focuser::initProperties();

    // No speed for DarkSkyGeek
    FocusSpeedNP[0].setMin(1);
    FocusSpeedNP[0].setMax(1);
    FocusSpeedNP[0].setValue(1);

    // Relative and absolute movement
    FocusRelPosNP[0].setMin(0.);
    FocusRelPosNP[0].setMax(FocusMaxPosNP[0].getValue());
    FocusRelPosNP[0].setValue(0);
    FocusRelPosNP[0].setStep(1);

    FocusAbsPosNP[0].setMin(0.);
    FocusAbsPosNP[0].setMax(FocusMaxPosNP[0].getValue());
    FocusAbsPosNP[0].setValue(0);
    FocusAbsPosNP[0].setStep(1);

    addDebugControl();
    // Set default baud rate to 57600
    serialConnection->setDefaultBaudRate(Connection::Serial::B_57600);
    serialConnection->setWordSize(8);
    return true;
}

bool DarkSkyGeek::updateProperties()
{
    INDI::Focuser::updateProperties();

    if (isConnected())
    {
        LOG_INFO("Focuser ready.");
    }
    else
    {
    }

    return true;
}

bool DarkSkyGeek::Handshake()
{
    if (Ack())
    {
        LOG_INFO("DarkSkyGeek Focuser is online.");

        readVersion();

        return true;
    }

    LOG_INFO("Error retrieving data from DarkSkyGeek Focuser, please ensure DarkSkyGeek is powered and the port is correct.");
    return false;
}

const char * DarkSkyGeek::getDefaultName()
{
    return "DarkSkyGeek Focuser";
}

bool DarkSkyGeek::Ack()
{
    int nbytes_written = 0, nbytes_read = 0, rc = -1;
    char errstr[MAXRBUF];
    char resp[MAXRBUF] = {0};

    tcflush(PortFD, TCIOFLUSH);
    int numChecks = 0;
    bool success = false;
    while (numChecks < 3 && !success)
    {
        numChecks++;
        //wait 1 second between each test.
        sleep(1);

        bool transmissionSuccess = (rc = tty_write_string(PortFD, COMMAND_PING, &nbytes_written)) == TTY_OK;
        if(!transmissionSuccess)
        {
            tty_error_msg(rc, errstr, MAXRBUF);
            LOGF_ERROR("Handshake Attempt %i, tty transmission error: %s.", numChecks, errstr);
        }

        bool responseSuccess = (rc = tty_nread_section(PortFD, resp, MAXRBUF, DRIVER_DEL, DRIVER_TIMEOUT, &nbytes_read)) == TTY_OK;
        if(!responseSuccess)
        {
            tty_error_msg(rc, errstr, MAXRBUF);
            LOGF_ERROR("Handshake Attempt %i, updatePosition response error: %s,%s.", numChecks, errstr, resp);
        }

        success = transmissionSuccess && responseSuccess;
    }

    if(!success)
    {
        LOG_INFO("Handshake failed after 3 attempts");
        return false;
    }
    tcflush(PortFD, TCIOFLUSH);
    if (strncmp(resp, RESULT_PING, strlen(RESULT_PING))!= 0)
    {
        LOGF_ERROR("Invalid response from device: %s.", resp);
        return false;
    }
    return true;
}



bool DarkSkyGeek::readVersion()
{
    char res[MAXRBUF] = {0};

    if (sendCommand(COMMAND_INFO, RESULT_INFO, res) == false)
        return false;

    LOGF_INFO("Detected %s", res);

    return true;
}

bool DarkSkyGeek::readPosition()
{
    char res[MAXRBUF] = {0};

    if (sendCommand(COMMAND_FOCUSER_GETPOSITION, RESULT_FOCUSER_POSITION, res) == false)
        return false;

    int32_t pos;
    int rc = sscanf(res, "%d", &pos);

    if (rc > 0)
        FocusAbsPosNP[0].setValue(pos + positionOffset);
    else
    {
        LOGF_ERROR("Unknown error: focuser position value (%s)", res);
        return false;
    }

    return true;
}

bool DarkSkyGeek::isMoving()
{
    char res[MAXRBUF] = {0};

    if (sendCommand(COMMAND_FOCUSER_ISMOVING, RESULT_FOCUSER_ISMOVING, res) == false)
        return false;
    LOGF_DEBUG("isMoving %s", res);

    if (strcmp(res, TRUE) == 0)
        return true;
    else if (strcmp(res, FALSE) == 0)
        return false;

    LOGF_ERROR("Unknown error: isMoving value (%s)", res);
    return false;
}


bool DarkSkyGeek::SyncFocuser(uint32_t ticks)
{
    if (ticks == 0)
    {
        // Use the firmware's native zero-reset to keep driver and firmware in sync.
        char res[MAXRBUF] = {0};
        if (sendCommand(COMMAND_FOCUSER_SETZEROPOSITION, RESULT_FOCUSER_SETZEROPOSITION, res) == false)
            return false;
        positionOffset = 0;
        FocusAbsPosNP[0].setValue(0);
        FocusAbsPosNP.apply();
        return true;
    }

    // For non-zero sync, track the delta in the driver.
    char res[MAXRBUF] = {0};
    if (sendCommand(COMMAND_FOCUSER_GETPOSITION, RESULT_FOCUSER_POSITION, res) == false)
        return false;
    int32_t rawPos;
    if (sscanf(res, "%d", &rawPos) <= 0)
        return false;
    positionOffset = static_cast<int32_t>(ticks) - rawPos;
    FocusAbsPosNP[0].setValue(ticks);
    FocusAbsPosNP.apply();
    return true;
}

IPState DarkSkyGeek::MoveAbsFocuser(uint32_t targetTicks)
{
    char cmd[MAXRBUF] = {0}, res[MAXRBUF] = {0};
    int32_t rawTarget = static_cast<int32_t>(targetTicks) - positionOffset;
    if (rawTarget < 0 || rawTarget > static_cast<int32_t>(FocusMaxPosNP[0].getValue()))
    {
        LOGF_ERROR("Move error: position %d maps to raw target %d, outside firmware range [0, %g]",
                   targetTicks, rawTarget, FocusMaxPosNP[0].getValue());
        return IPS_ALERT;
    }
    snprintf(cmd, MAXRBUF, "%s%d", COMMAND_FOCUSER_MOVE, rawTarget);
    if (sendCommand(cmd, RESULT_FOCUSER_MOVE, res) == false)
        return IPS_ALERT;

    targetPos = targetTicks;
    LOGF_DEBUG("Move %d (raw %d): %s", targetTicks, rawTarget, res);
    if (strcmp(res, OK) == 0)
        return IPS_BUSY;
    else
        return IPS_ALERT;
}

IPState DarkSkyGeek::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    int32_t newPosition = 0;
    if (dir == FOCUS_INWARD)
        newPosition = FocusAbsPosNP[0].getValue() - ticks;
    else
        newPosition = FocusAbsPosNP[0].getValue() + ticks;

    // Clamp
    newPosition = std::max(0, std::min(static_cast<int32_t>(FocusAbsPosNP[0].getMax()), newPosition));
    if (MoveAbsFocuser(newPosition) != IPS_BUSY)
        return IPS_ALERT;

    FocusRelPosNP[0].setValue(ticks);
    FocusRelPosNP.setState(IPS_BUSY);

    return IPS_BUSY;
}

void DarkSkyGeek::TimerHit()
{
    if (!isConnected())
    {
        SetTimer(getCurrentPollingPeriod());
        return;
    }

    if (readPosition())
    {
        if (FocusAbsPosNP.getState() == IPS_BUSY || FocusRelPosNP.getState() == IPS_BUSY)
        {
            if (!isMoving())
            {
                FocusAbsPosNP.setState(IPS_OK);
                FocusRelPosNP.setState(IPS_OK);
                FocusAbsPosNP.apply();
                FocusRelPosNP.apply();
                lastPos = FocusAbsPosNP[0].getValue();
                LOG_INFO("Focuser reached requested position.");
            }
            else
            {
                FocusAbsPosNP.apply();
            }
        }
        else if (FocusAbsPosNP[0].getValue() != lastPos)
        {
            lastPos = FocusAbsPosNP[0].getValue();
            FocusAbsPosNP.apply();
        }
    }
     
    SetTimer(getCurrentPollingPeriod());
}

bool DarkSkyGeek::AbortFocuser()
{
    return sendCommand(COMMAND_FOCUSER_HALT, RESULT_FOCUSER_HALT);
    // Ignore whether the firmware responded with OK or NOK.
    // If the firmware responded with NOK, it's likely because
    // the focuser was not moving when the command was sent...
}


bool DarkSkyGeek::sendCommand(const char * cmd, const char * resultPrefix, char * res)
{
    int nbytes_written = 0, nbytes_read = 0, rc = -1;
    char cmdbuf[MAXRBUF] = {0};

    tcflush(PortFD, TCIOFLUSH);

    LOGF_DEBUG("CMD <%s>", cmd);

    snprintf(cmdbuf, MAXRBUF, "%s\n", cmd);

    if ((rc = tty_write_string(PortFD, cmdbuf, &nbytes_written)) != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial write error: %s.", errstr);
        return false;
    }

    if (res == nullptr)
        return true;

    if ((rc = tty_nread_section(PortFD, res, MAXRBUF, DRIVER_DEL, DRIVER_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial read error: %s.", errstr);
        return false;
    }
    res[nbytes_read - 1] = 0;
    // Arduino Serial.println() writes CR/LF
    if (nbytes_read >= 2 && res[nbytes_read - 2] == '\r')
        res[nbytes_read - 2] = 0;

    LOGF_DEBUG("RES <%s>", res);

    tcflush(PortFD, TCIOFLUSH);
    if (strncmp(res, resultPrefix, strlen(resultPrefix)) != 0)
    {
        LOGF_ERROR("Invalid response from device: %s.", res);
        return false;
    }
    // trim off the prefix to get the result
    size_t prefixLength = strlen(resultPrefix);
    memmove(res, res + prefixLength, nbytes_read - prefixLength);
    res[nbytes_read - prefixLength] = 0;
    return true;
}
