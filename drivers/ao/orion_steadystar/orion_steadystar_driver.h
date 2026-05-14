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

#pragma once

#include <defaultdevice.h>
#include <indiguiderinterface.h>
#include <indirotatorinterface.h>
#include "orion_steadystar_hardware.h"

class OrionSteadyStarDriver : public INDI::DefaultDevice, public INDI::GuiderInterface, public INDI::RotatorInterface
{
public:
    OrionSteadyStarDriver();
    virtual ~OrionSteadyStarDriver() = default;

    virtual std::chrono::steady_clock::time_point getTime() const { return std::chrono::steady_clock::now(); }

    const char *getDefaultName() override;
    bool initProperties() override;
    bool updateProperties() override;
    bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;

    IPState GuideNorth(uint32_t ms) override;
    IPState GuideSouth(uint32_t ms) override;
    IPState GuideEast(uint32_t ms) override;
    IPState GuideWest(uint32_t ms) override;

    bool AONorth(int steps);
    bool AOSouth(int steps);
    bool AOEast(int steps);
    bool AOWest(int steps);

    bool AOCenter();
    bool AOUnjam();

    void CheckLimit(bool force);

    // Rotator Implementation
    IPState MoveRotator(double angle) override;
    bool SyncRotator(double angle) override;
    bool ReverseRotator(bool enabled) override;
    bool AbortRotator() override;

protected:
    bool Connect() override;
    bool Disconnect() override;
    void simulationTriggered(bool enable) override;
    void TimerHit() override;

    OrionSteadyStarHardware m_Hardware;

    INumber AONS[2];
    INumberVectorProperty AONSNP;

    INumber AOWE[2];
    INumberVectorProperty AOWENP;

    ISwitch Center[2];
    ISwitchVectorProperty CenterP;

    IText FWT[1];
    ITextVectorProperty FWTP;

    ILight AtLimitL[4];
    ILightVectorProperty AtLimitLP;

    INumber AOLimitN[1];
    INumberVectorProperty AOLimitNP;

    ISwitch RotatorEnabledS[2];
    ISwitchVectorProperty RotatorEnabledSP;

    ISwitch DebugPacketsS[2];
    ISwitchVectorProperty DebugPacketsSP;

    INumber DerotateN[1];
    INumberVectorProperty DerotateNP;

    INumber GuideN[1];
    INumberVectorProperty GuideNP;

    uint8_t lastLimit = 0xFF;

    int m_ax = 0, m_ay = 0;
    double m_DerotationStartAngle = 0;
    double m_DerotationGuideOffset = 0;
    std::chrono::steady_clock::time_point m_DerotationStartTime;
    std::chrono::steady_clock::time_point m_LastWatchdogTime;
    std::chrono::steady_clock::time_point m_LastRotatorPollTime;

    static constexpr const char *GUIDE_CONTROL_TAB = "Guider Control";
};
