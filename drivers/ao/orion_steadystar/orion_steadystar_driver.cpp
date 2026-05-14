/*******************************************************************************
  Copyright(c) 2026 Christian Kemper. All rights reserved.

  Based on the work of Benoit Schillings (https://github.com/BenoitSchillings/orion_steadystar)

  This program is free software; you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free
  Software Foundation; either version 2.1 of the License, or (at your option)
  any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc., 51
  Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*******************************************************************************/

#include "orion_steadystar_driver.h"
#include <indicom.h>
#include <memory>
#include <cstring>

static std::unique_ptr<OrionSteadyStarDriver> driver(new OrionSteadyStarDriver);

OrionSteadyStarDriver::OrionSteadyStarDriver() : INDI::GuiderInterface(this), INDI::RotatorInterface(this)
{
}

const char *OrionSteadyStarDriver::getDefaultName()
{
    return "Orion SteadyStar";
}

bool OrionSteadyStarDriver::initProperties()
{
    INDI::DefaultDevice::initProperties();
    INDI::GuiderInterface::initProperties(GUIDE_CONTROL_TAB);
    INDI::RotatorInterface::initProperties(MAIN_CONTROL_TAB);
    SetCapability(ROTATOR_CAN_SYNC | ROTATOR_CAN_REVERSE | ROTATOR_CAN_ABORT);

    IUFillNumber(&AONS[0], "AO_N", "North (steps)", "%g", 0, 80, 1, 0);
    IUFillNumber(&AONS[1], "AO_S", "South (steps)", "%g", 0, 80, 1, 0);
    IUFillNumberVector(&AONSNP, AONS, 2, getDeviceName(), "AO_NS", "AO Tilt North/South", GUIDE_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

    IUFillNumber(&AOWE[0], "AO_E", "East (steps)", "%g", 0, 80, 1, 0);
    IUFillNumber(&AOWE[1], "AO_W", "West (steps)", "%g", 0, 80, 1, 0);
    IUFillNumberVector(&AOWENP, AOWE, 2, getDeviceName(), "AO_WE", "AO Tilt East/West", GUIDE_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

    IUFillSwitch(&Center[0], "CENTER", "Center", ISS_OFF);
    IUFillSwitch(&Center[1], "UNJAM", "Unjam", ISS_OFF);
    IUFillSwitchVector(&CenterP, Center, 2, getDeviceName(), "AO_CENTER", "AO Center", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    IUFillLight(&AtLimitL[0], "AT_LIMIT_N", "North", IPS_IDLE);
    IUFillLight(&AtLimitL[1], "AT_LIMIT_S", "South", IPS_IDLE);
    IUFillLight(&AtLimitL[2], "AT_LIMIT_E", "East", IPS_IDLE);
    IUFillLight(&AtLimitL[3], "AT_LIMIT_W", "West", IPS_IDLE);
    IUFillLightVector(&AtLimitLP, AtLimitL, 4, getDeviceName(), "AT_LIMIT", "At limit", MAIN_CONTROL_TAB, IPS_IDLE);

    IUFillNumber(&AOLimitN[0], "LIMIT", "Max steps", "%g", 0, 200, 1, 80);
    IUFillNumberVector(&AOLimitNP, AOLimitN, 1, getDeviceName(), "AO_LIMIT", "AO Limit", GUIDE_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    IUFillText(&FWT[0], "VERSION", "Driver version", "1.0");
    IUFillTextVector(&FWTP, FWT, 1, getDeviceName(), "INFO", "Info", OPTIONS_TAB, IP_RO, 60, IPS_IDLE);

    IUFillSwitch(&RotatorEnabledS[0], "ENABLED", "Enabled", ISS_OFF);
    IUFillSwitch(&RotatorEnabledS[1], "DISABLED", "Disabled", ISS_ON);
    IUFillSwitchVector(&RotatorEnabledSP, RotatorEnabledS, 2, getDeviceName(), "ROTATOR_ENABLED", "Rotator support", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    IUFillNumber(&DerotateN[0], "RATE", "Rate (arcsec/sec)", "%.4f", -1000, 1000, 0, 0);
    IUFillNumberVector(&DerotateNP, DerotateN, 1, getDeviceName(), "ROTATOR_DEROTATE", "Derotation", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

    IUFillNumber(&GuideN[0], "DELTA", "Correction (deg)", "%.4f", -100, 100, 0, 0);
    IUFillNumberVector(&GuideNP, GuideN, 1, getDeviceName(), "ROTATOR_GUIDE", "Guiding", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

    IUFillSwitch(&DebugPacketsS[0], "ENABLED", "Enabled", ISS_OFF);
    IUFillSwitch(&DebugPacketsS[1], "DISABLED", "Disabled", ISS_ON);
    IUFillSwitchVector(&DebugPacketsSP, DebugPacketsS, 2, getDeviceName(), "DEBUG_PACKETS", "Debug Messages", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    addDebugControl();
    addSimulationControl();

    setDriverInterface(AO_INTERFACE | GUIDER_INTERFACE);

    return true;
}

bool OrionSteadyStarDriver::updateProperties()
{
    INDI::DefaultDevice::updateProperties();

    if (isConnected())
    {
        defineProperty(&AONSNP);
        defineProperty(&AOWENP);
        defineProperty(&CenterP);
        defineProperty(&AtLimitLP);
        defineProperty(&AOLimitNP);
        defineProperty(&FWTP);
        defineProperty(&RotatorEnabledSP);
        defineProperty(&DebugPacketsSP);

        if (RotatorEnabledS[0].s == ISS_ON)
        {
            INDI::RotatorInterface::updateProperties();
            defineProperty(&DerotateNP);
            defineProperty(&GuideNP);
            setDriverInterface(AO_INTERFACE | GUIDER_INTERFACE | ROTATOR_INTERFACE);
        }
        else
        {
            setDriverInterface(AO_INTERFACE | GUIDER_INTERFACE);
        }
        syncDriverInfo();
        SetTimer(100);
    }
    else
    {
        deleteProperty(AONSNP.name);
        deleteProperty(AOWENP.name);
        deleteProperty(CenterP.name);
        deleteProperty(AtLimitLP.name);
        deleteProperty(AOLimitNP.name);
        deleteProperty(FWTP.name);
        deleteProperty(RotatorEnabledSP.name);
        deleteProperty(DebugPacketsSP.name);
        deleteProperty(DerotateNP.name);
        deleteProperty(GuideNP.name);
        INDI::RotatorInterface::updateProperties();
    }

    return true;
}

bool OrionSteadyStarDriver::Connect()
{
    m_Hardware.SetSimulation(isSimulation());
    m_Hardware.SetDebug(DebugPacketsS[0].s == ISS_ON);
    if (m_Hardware.Connect())
    {
        if (m_Hardware.Home())
        {
            m_ax = 0;
            m_ay = 0;
            m_LastWatchdogTime = getTime();
            if (RotatorEnabledS[0].s == ISS_ON)
            {
                double angle = 0;
                if (m_Hardware.GetRotatorAngle(angle))
                {
                    GotoRotatorNP[0].setValue(angle);
                }
                m_DerotationStartTime = getTime();
                m_DerotationStartAngle = GotoRotatorNP[0].getValue();
                m_DerotationGuideOffset = 0;
            }
            else
            {
                // Rotator disabled: sync hardware state to the last known property value.
                m_Hardware.SyncRotation(GotoRotatorNP[0].getValue());
            }

            CheckLimit(true);
            return true;
        }
    }
    return false;
}

bool OrionSteadyStarDriver::Disconnect()
{
    m_Hardware.Disconnect();
    INDI::RotatorInterface::updateProperties();
    return true;
}

void OrionSteadyStarDriver::simulationTriggered(bool enable)
{
    m_Hardware.SetSimulation(enable);
}

bool OrionSteadyStarDriver::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    INDI_UNUSED(dev);

    // Check guider interface
    if (INDI::GuiderInterface::processNumber(dev, name, values, names, n))
        return true;

    // Check rotator interface
    if (RotatorEnabledS[0].s == ISS_ON && INDI::RotatorInterface::processNumber(dev, name, values, names, n))
        return true;

    if (strcmp(name, DerotateNP.name) == 0)
    {
        auto now = getTime();
        double elapsed_sec = std::chrono::duration<double>(now - m_DerotationStartTime).count();
        // Calculate the exact theoretical position reached with the OLD rate and offset
        double current_theoretical = range360(m_DerotationStartAngle + m_DerotationGuideOffset + (elapsed_sec * DerotateN[0].value / 3600.0));

        IUUpdateNumber(&DerotateNP, values, names, n);

        m_DerotationStartTime = now;
        m_DerotationStartAngle = current_theoretical;
        m_DerotationGuideOffset = 0;

        DerotateNP.s = (std::abs(DerotateN[0].value) > 1e-6) ? IPS_BUSY : IPS_OK;
        IDSetNumber(&DerotateNP, nullptr);
        return true;
    }

    if (strcmp(name, GuideNP.name) == 0)
    {
        IUUpdateNumber(&GuideNP, values, names, n);
        m_DerotationGuideOffset += GuideN[0].value;
        GuideN[0].value = 0;
        IDSetNumber(&GuideNP, nullptr);
        return true;
    }

    if (strcmp(name, AONSNP.name) == 0)
    {
        AONSNP.s = IPS_BUSY;
        IUUpdateNumber(&AONSNP, values, names, n);
        IDSetNumber(&AONSNP, nullptr);
        if (AONS[0].value != 0)
            AONSNP.s = AONorth(static_cast<int>(AONS[0].value)) ? IPS_OK : IPS_ALERT;
        else if (AONS[1].value != 0)
            AONSNP.s = AOSouth(static_cast<int>(AONS[1].value)) ? IPS_OK : IPS_ALERT;
        // Show accumulated absolute position, not a reset-to-zero step counter.
        AONS[0].value = (m_ay > 0) ? m_ay : 0;
        AONS[1].value = (m_ay < 0) ? -m_ay : 0;
        IDSetNumber(&AONSNP, nullptr);
        CheckLimit(false);
        return true;
    }
    else if (strcmp(name, AOWENP.name) == 0)
    {
        AOWENP.s = IPS_BUSY;
        IUUpdateNumber(&AOWENP, values, names, n);
        IDSetNumber(&AOWENP, nullptr);
        if (AOWE[0].value != 0)
            AOWENP.s = AOEast(static_cast<int>(AOWE[0].value)) ? IPS_OK : IPS_ALERT;
        else if (AOWE[1].value != 0)
            AOWENP.s = AOWest(static_cast<int>(AOWE[1].value)) ? IPS_OK : IPS_ALERT;
        // Show accumulated absolute position.
        AOWE[0].value = (m_ax > 0) ? m_ax : 0;
        AOWE[1].value = (m_ax < 0) ? -m_ax : 0;
        IDSetNumber(&AOWENP, nullptr);
        CheckLimit(false);
        return true;
    }

    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool OrionSteadyStarDriver::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    INDI_UNUSED(dev);

    // Check rotator interface
    if (RotatorEnabledS[0].s == ISS_ON && INDI::RotatorInterface::processSwitch(dev, name, states, names, n))
        return true;

    if (strcmp(name, RotatorEnabledSP.name) == 0)
    {
        IUUpdateSwitch(&RotatorEnabledSP, states, names, n);
        RotatorEnabledSP.s = IPS_OK;
        IDSetSwitch(&RotatorEnabledSP, nullptr);

        if (RotatorEnabledS[0].s == ISS_ON)
        {
            INDI::RotatorInterface::updateProperties();
            defineProperty(&DerotateNP);
            defineProperty(&GuideNP);
            setDriverInterface(AO_INTERFACE | GUIDER_INTERFACE | ROTATOR_INTERFACE);
        }
        else
        {
            deleteProperty(GotoRotatorNP);
            if (CanSync())
                deleteProperty(SyncRotatorNP);
            if (CanReverse())
                deleteProperty(ReverseRotatorSP);
            deleteProperty(RotatorLimitsNP);
            deleteProperty(DerotateNP.name);
            deleteProperty(GuideNP.name);
            setDriverInterface(AO_INTERFACE | GUIDER_INTERFACE);
        }
        syncDriverInfo();
        return true;
    }

    if (strcmp(name, DebugPacketsSP.name) == 0)
    {
        IUUpdateSwitch(&DebugPacketsSP, states, names, n);
        m_Hardware.SetDebug(DebugPacketsS[0].s == ISS_ON);
        DebugPacketsSP.s = IPS_OK;
        IDSetSwitch(&DebugPacketsSP, nullptr);
        return true;
    }

    if (strcmp(name, "AO_CENTER") == 0)
    {
        CenterP.s = IPS_BUSY;
        IDSetSwitch(&CenterP, nullptr);
        IUUpdateSwitch(&CenterP, states, names, n);
        if (Center[0].s == ISS_ON)
        {
            AOCenter();
            Center[0].s = ISS_OFF;
        }
        else if (Center[1].s == ISS_ON)
        {
            AOUnjam();
            Center[1].s = ISS_OFF;
        }
        CenterP.s = IPS_OK;
        IDSetSwitch(&CenterP, nullptr);
        CheckLimit(true);
        return true;
    }

    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

IPState OrionSteadyStarDriver::GuideNorth(uint32_t ms)
{
    // Following sxao pattern: convert ms to steps. 
    // sxao uses ms / 10.
    return AONorth(ms / 10) ? IPS_OK : IPS_ALERT;
}

IPState OrionSteadyStarDriver::GuideSouth(uint32_t ms)
{
    return AOSouth(ms / 10) ? IPS_OK : IPS_ALERT;
}

IPState OrionSteadyStarDriver::GuideEast(uint32_t ms)
{
    return AOEast(ms / 10) ? IPS_OK : IPS_ALERT;
}

IPState OrionSteadyStarDriver::GuideWest(uint32_t ms)
{
    return AOWest(ms / 10) ? IPS_OK : IPS_ALERT;
}

bool OrionSteadyStarDriver::AONorth(int steps)
{
    if (m_Hardware.SetAO(m_ax, m_ay + steps))
    {
        m_ay += steps;
        return true;
    }
    return false;
}

bool OrionSteadyStarDriver::AOSouth(int steps)
{
    if (m_Hardware.SetAO(m_ax, m_ay - steps))
    {
        m_ay -= steps;
        return true;
    }
    return false;
}

bool OrionSteadyStarDriver::AOEast(int steps)
{
    if (m_Hardware.SetAO(m_ax + steps, m_ay))
    {
        m_ax += steps;
        return true;
    }
    return false;
}

bool OrionSteadyStarDriver::AOWest(int steps)
{
    if (m_Hardware.SetAO(m_ax - steps, m_ay))
    {
        m_ax -= steps;
        return true;
    }
    return false;
}

bool OrionSteadyStarDriver::AOCenter()
{
    if (m_Hardware.SetAO(0, 0))
    {
        m_ax = 0;
        m_ay = 0;
        return true;
    }
    return false;
}

bool OrionSteadyStarDriver::AOUnjam()
{
    if (!m_Hardware.Home())
        return false;
    m_ax = 0;
    m_ay = 0;
    return true;
}

void OrionSteadyStarDriver::CheckLimit(bool force)
{
    uint8_t status = 0;
    if (m_Hardware.GetHomingStatus(status))
    {
        if (force || status != lastLimit)
        {
            AtLimitL[0].s = (status & 0x10) ? IPS_ALERT : IPS_IDLE;
            AtLimitL[1].s = (status & 0x20) ? IPS_ALERT : IPS_IDLE;
            AtLimitL[2].s = (status & 0x40) ? IPS_ALERT : IPS_IDLE;
            AtLimitL[3].s = (status & 0x80) ? IPS_ALERT : IPS_IDLE;
            AtLimitLP.s   = (status & 0xF0) ? IPS_ALERT : IPS_IDLE;
            IDSetLight(&AtLimitLP, nullptr);
            lastLimit = status;
        }
    }
}

IPState OrionSteadyStarDriver::MoveRotator(double angle)
{
    if (m_Hardware.StartRotation(angle))
    {
        m_DerotationStartTime = getTime();
        m_DerotationStartAngle = angle;
        m_DerotationGuideOffset = 0;
        SetTimer(100);
        return IPS_BUSY;
    }
    return IPS_ALERT;
}

bool OrionSteadyStarDriver::SyncRotator(double angle)
{
    bool rc = m_Hardware.SyncRotation(angle);
    if (rc)
    {
        m_DerotationStartTime = getTime();
        m_DerotationStartAngle = angle;
        m_DerotationGuideOffset = 0;
    }
    return rc;
}

bool OrionSteadyStarDriver::ReverseRotator(bool enabled)
{
    return m_Hardware.ReverseRotation(enabled);
}

bool OrionSteadyStarDriver::AbortRotator()
{
    return m_Hardware.AbortRotation();
}

void OrionSteadyStarDriver::TimerHit()
{
    if (!isConnected())
        return;

    bool pollHardware = false;
    auto now = getTime();

    // 1. Watchdog: check connection every 5 seconds
    auto watchdogElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - m_LastWatchdogTime).count();
    if (watchdogElapsed >= 5)
    {
        m_LastWatchdogTime = now;
        uint8_t status = 0;
        if (!m_Hardware.GetHomingStatus(status))
        {
            LOG_WARN("Watchdog: connection lost, attempting auto-reconnect...");
            m_Hardware.Disconnect();
            if (m_Hardware.Connect())
            {
                LOG_INFO("Watchdog: auto-reconnect successful.");
            }
            else
            {
                LOG_ERROR("Watchdog: auto-reconnect failed. Disconnecting.");
                setConnected(false);
                return;
            }
        }
    }

    // 2. Handle Goto or idle position refresh (always poll when rotator is enabled,
    //    matching the ASI reference driver pattern so KStars always sees current angle).
    if (RotatorEnabledS[0].s == ISS_ON)
    {
        if (GotoRotatorNP.getState() == IPS_BUSY)
        {
            pollHardware = true;
        }
        else
        {
            // Poll at a reduced rate (every 2 s) when idle to keep position fresh.
            auto rotatorElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - m_LastRotatorPollTime).count();
            if (rotatorElapsed >= 2)
                pollHardware = true;
        }
    }

    // 3. Handle software derotation
    if (RotatorEnabledS[0].s == ISS_ON && (std::abs(DerotateN[0].value) > 1e-6 || std::abs(m_DerotationGuideOffset) > 1e-6) &&
        GotoRotatorNP.getState() != IPS_BUSY)
    {
        constexpr double STEPS_PER_DEGREE = 29.0;
        constexpr int STEPS_PER_CIRCLE = 10440; // 360 * 29

        double elapsed_sec = std::chrono::duration<double>(now - m_DerotationStartTime).count();
        double theoretical_angle = range360(m_DerotationStartAngle + m_DerotationGuideOffset + (elapsed_sec * DerotateN[0].value / 3600.0));

        // Convert current hardware position and theoretical position to absolute steps
        int current_steps = static_cast<int>(GotoRotatorNP[0].getValue() * STEPS_PER_DEGREE + 0.5) % STEPS_PER_CIRCLE;
        int theoretical_steps = static_cast<int>(theoretical_angle * STEPS_PER_DEGREE + 0.5) % STEPS_PER_CIRCLE;

        if (current_steps != theoretical_steps)
        {
            if (m_Hardware.StartRotation(theoretical_angle))
                pollHardware = true;
        }
    }

    if (pollHardware)
    {
        m_LastRotatorPollTime = now;
        bool busy = false;
        double current_angle = 0;
        if (m_Hardware.PollRotation(busy, current_angle))
        {
            GotoRotatorNP[0].setValue(current_angle);
            if (!busy && GotoRotatorNP.getState() == IPS_BUSY)
            {
                GotoRotatorNP.setState(IPS_OK);
            }
            GotoRotatorNP.apply();
        }
        else
        {
            GotoRotatorNP.setState(IPS_ALERT);
            GotoRotatorNP.apply();
        }
    }

    // Always keep timer running if we are connected (for watchdog)
    if (isConnected())
    {
        SetTimer(100);
    }
}
