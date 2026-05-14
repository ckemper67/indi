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

#include <libusb-1.0/libusb.h>
#include <cstdint>
#include <vector>
#include <string>
#include <mutex>

class OrionSteadyStarHardware
{
public:
    OrionSteadyStarHardware();
    ~OrionSteadyStarHardware();

    bool Connect();
    void Disconnect();
    bool IsConnected() const { return m_Simulation || m_DeviceHandle != nullptr; }

    void SetSimulation(bool sim) { m_Simulation = sim; }
    void SetDebug(bool enabled) { m_Debug = enabled; }

    bool GetPosition(int8_t &m1, int8_t &m2, int8_t &m3, int8_t &m4);
    bool GetHomingStatus(uint8_t &status);
    bool Home();

    bool MoveMotor(uint8_t motor_idx, int8_t delta);
    bool MoveMotors(int8_t m1, int8_t m2, int8_t m3, int8_t m4);
    bool SetAO(int tx, int ty);

    bool StartRotation(double angle);
    bool PollRotation(bool &busy, double &current_angle);
    bool GetRotatorAngle(double &angle);
    bool AbortRotation();
    bool SyncRotation(double angle);
    bool ReverseRotation(bool enabled);

    void syncState(double angle)
    {
        m_CurrentAngle = angle;
        m_TargetAngle = angle;
        m_Rotating = false;
    }

    double GetTargetAngle() const { return m_TargetAngle; }

    const char *getDeviceName() const { return "Orion SteadyStar"; }

protected:
    bool ConnectOnce();
    bool InterruptWrite(const uint8_t *data);
    bool InterruptRead(uint8_t *data);

    libusb_context *m_Context = nullptr;
    libusb_device_handle *m_DeviceHandle = nullptr;
    mutable std::mutex m_UsbMutex;
    bool m_Simulation = false;
    bool m_Debug = false;

    int8_t m_m1 = 0, m_m2 = 0, m_m3 = 0, m_m4 = 0;
    double m_TargetAngle = 0;
    double m_CurrentAngle = 0;
    bool m_ReverseRotation = false;
    std::chrono::steady_clock::time_point m_LastPollTime;
    bool m_Rotating = false;

    static constexpr uint16_t VENDOR_ID = 0x03EB;
    static constexpr uint16_t PRODUCT_ID = 0x2013;
    static constexpr uint8_t ENDPOINT_OUT = 0x02;
    static constexpr uint8_t ENDPOINT_IN  = 0x81;
    static constexpr int TIMEOUT_MS = 1000;

    uint8_t c2_byte(int8_t v);
    int8_t b_2_i(uint8_t v);
};
