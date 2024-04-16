/*
    DarkSkyGeek Focuser (darkskygeek) INDI Focuser

    Based on aaf2 and DarkSkyGeek ASCOM Driver
    
    Copyright (C) 2024 Christian Kemper (ckemper@gmail.com)

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

#include <cmath>
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
    FocusSpeedN[0].min = 1;
    FocusSpeedN[0].max = 1;
    FocusSpeedN[0].value = 1;
    IUUpdateMinMax(&FocusSpeedNP);

    // Relative and absolute movement
    FocusRelPosN[0].min   = 0.;
    FocusRelPosN[0].max   = FocusMaxPosN[0].value; //MaxPositionN[0].value;
    FocusRelPosN[0].value = 0;
    FocusRelPosN[0].step  = 1;

    FocusAbsPosN[0].min   = 0.;
    FocusAbsPosN[0].max   = FocusMaxPosN[0].value; //MaxPositionN[0].value;
    FocusAbsPosN[0].value = 0;
    FocusAbsPosN[0].step  = 1;

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
    tty_set_debug(1);
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
        FocusAbsPosN[0].value = pos;
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
    LOGF_INFO("isMoving %s", res);

    if (strcmp(res, TRUE) == 0)
        return true;
    else if (strcmp(res, FALSE) == 0)
        return false;

    LOGF_ERROR("Unknown error: isMoving value (%s)", res);
    return false;
}


bool DarkSkyGeek::SyncFocuser(uint32_t ticks)
{
    if (ticks != 0) {
      LOGF_ERROR("Sync error: requested %d != 0", ticks);
      return false;
    }
    if (sendCommand(COMMAND_FOCUSER_SETZEROPOSITION, RESULT_FOCUSER_SETZEROPOSITION) == false)
      return false;
    
    return true;
}

IPState DarkSkyGeek::MoveAbsFocuser(uint32_t targetTicks)
{
    char cmd[MAXRBUF] = {0}, res[MAXRBUF] = {0};
    if (targetTicks > maxPosition) {
      LOGF_ERROR("Move error: requested %d outside of [0, %d]", targetTicks, maxPosition);
      return IPS_ALERT;
    }
    snprintf(cmd, MAXRBUF, "%s%d", COMMAND_FOCUSER_MOVE, targetTicks);
    if (sendCommand(cmd, RESULT_FOCUSER_MOVE, res) == false)
        return IPS_ALERT;

    targetPos = targetTicks;
    LOGF_INFO("Move %d:%s", targetTicks, res);
    if (strcmp(res, OK) == 0)
        return IPS_BUSY;
    else
        return IPS_ALERT;
}

IPState DarkSkyGeek::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    int32_t newPosition = 0;
    if (dir == FOCUS_INWARD)
        newPosition = FocusAbsPosN[0].value - ticks;
    else
        newPosition = FocusAbsPosN[0].value + ticks;

    // Clamp
    newPosition = std::max(0, std::min(static_cast<int32_t>(FocusAbsPosN[0].max), newPosition));
    if (MoveAbsFocuser(newPosition) != IPS_BUSY)
        return IPS_ALERT;

    FocusRelPosN[0].value = ticks;
    FocusRelPosNP.s       = IPS_BUSY;

    return IPS_BUSY;
}

void DarkSkyGeek::TimerHit()
{
    if (!isConnected())
    {
        SetTimer(getCurrentPollingPeriod());
        return;
    }

    readPosition();

    if (FocusAbsPosNP.s == IPS_BUSY || FocusRelPosNP.s == IPS_BUSY)
    {
        if (!isMoving())
        {
            FocusAbsPosNP.s = IPS_OK;
            FocusRelPosNP.s = IPS_OK;
            IDSetNumber(&FocusAbsPosNP, nullptr);
            IDSetNumber(&FocusRelPosNP, nullptr);
            lastPos = FocusAbsPosN[0].value;
            LOG_INFO("Focuser reached requested position.");
        }
	else {
            IDSetNumber(&FocusAbsPosNP, nullptr);
	}
    }
    else if (fabs(FocusAbsPosN[0].value - lastPos) > 0)
    {
        lastPos = FocusAbsPosN[0].value;
        IDSetNumber(&FocusAbsPosNP, nullptr);
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
    char cmdbuf[MAXRBUF]={0};
    char result[MAXRBUF]={0};
      
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

    if ((rc = tty_nread_section(PortFD, result, MAXRBUF, DRIVER_DEL, DRIVER_TIMEOUT, &nbytes_read)) != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial read error: %s.", errstr);
        return false;
    }
    result[nbytes_read - 1] = 0;
    // Arduino Serial.println() writes CR/LF
    if( result[nbytes_read - 2] == '\r') res[nbytes_read - 2] = 0;

    LOGF_DEBUG("RES <%s>", result);

    tcflush(PortFD, TCIOFLUSH);
    if (strncmp(result, resultPrefix, strlen(resultPrefix))!= 0)
    {
        LOGF_ERROR("Invalid response from device: %s.", result);
        return false;
    }
    size_t prefixLength = strlen(resultPrefix);
    strncpy(res, result +  prefixLength, nbytes_read);
    return true;
}
