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

#include "orion_steadystar_hardware.h"
#include <thread>
#include <chrono>
#include <cstring>
#include <cmath>
#include <indicom.h>
#include <indilogger.h>

OrionSteadyStarHardware::OrionSteadyStarHardware()
{
    libusb_init(&m_Context);
}

OrionSteadyStarHardware::~OrionSteadyStarHardware()
{
    Disconnect();
    if (m_Context)
        libusb_exit(m_Context);
}

bool OrionSteadyStarHardware::ConnectOnce()
{
    // Enumerate all USB devices to find ours and report the exact open error.
    libusb_device **list = nullptr;
    ssize_t cnt = libusb_get_device_list(m_Context, &list);
    if (cnt < 0)
    {
        LOGF_ERROR("libusb_get_device_list failed: %s", libusb_error_name(static_cast<int>(cnt)));
        return false;
    }

    libusb_device *found = nullptr;
    for (ssize_t i = 0; i < cnt; i++)
    {
        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(list[i], &desc) != 0)
            continue;
        LOGF_DEBUG("USB device: VID=0x%04x PID=0x%04x bus=%d addr=%d",
                   desc.idVendor, desc.idProduct,
                   libusb_get_bus_number(list[i]),
                   libusb_get_device_address(list[i]));
        if (desc.idVendor == VENDOR_ID && desc.idProduct == PRODUCT_ID)
            found = list[i];
    }

    if (!found)
    {
        LOGF_ERROR("Device VID=0x%04x PID=0x%04x not visible to this process. "
                   "On macOS, IOHIDFamily hides HID devices from non-root processes. "
                   "Run: sudo indiserver -v indi_orion_steadystar",
                   VENDOR_ID, PRODUCT_ID);
        libusb_free_device_list(list, 1);
        return false;
    }

    int rc = libusb_open(found, &m_DeviceHandle);
    libusb_free_device_list(list, 1);

    if (rc != LIBUSB_SUCCESS)
    {
        if (rc != LIBUSB_ERROR_NO_DEVICE)
            LOGF_ERROR("libusb_open failed: %s — on macOS try: sudo indiserver -v indi_orion_steadystar",
                       libusb_error_name(rc));
        m_DeviceHandle = nullptr;
        return false;
    }

    // Auto-detach any kernel HID driver when claiming (Linux); no-op on macOS.
    libusb_set_auto_detach_kernel_driver(m_DeviceHandle, 1);

    rc = libusb_claim_interface(m_DeviceHandle, 0);
    if (rc != LIBUSB_SUCCESS)
    {
        LOGF_ERROR("libusb_claim_interface failed: %s — on macOS try: sudo indiserver -v indi_orion_steadystar",
                   libusb_error_name(rc));
        libusb_close(m_DeviceHandle);
        m_DeviceHandle = nullptr;
        return false;
    }

    return true;
}

bool OrionSteadyStarHardware::Connect()
{
    std::lock_guard<std::mutex> lock(m_UsbMutex);
    if (m_Simulation)
        return true;
    if (m_DeviceHandle)
        return true;

    // On macOS, when another process (e.g. status.py) holds or just released the
    // USB interface, IOHIDFamily re-attaches and the device briefly re-registers
    // with the IOKit service.  libusb_open returns LIBUSB_ERROR_NO_DEVICE during
    // that window even though the device appears in the enumeration list.
    // Resetting the libusb context between retries gives a completely fresh view
    // of the bus and typically resolves the issue within 1-2 seconds.
    static constexpr int MAX_RETRIES = 5;
    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++)
    {
        if (ConnectOnce())
            return true;

        if (attempt < MAX_RETRIES)
        {
            LOGF_INFO("Connect attempt %d/%d failed, retrying in 1 s (device may be re-enumerating)...",
                      attempt, MAX_RETRIES);
            // Release the stale context so the next attempt gets a fresh device list.
            libusb_exit(m_Context);
            m_Context = nullptr;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            int init_rc = libusb_init(&m_Context);
            if (init_rc != LIBUSB_SUCCESS)
            {
                LOGF_ERROR("libusb_init failed: %s", libusb_error_name(init_rc));
                m_Context = nullptr;
                return false;
            }
        }
    }

    LOGF_ERROR("libusb_open failed after %d attempts: device found but not accessible. "
               "On macOS try: sudo indiserver -v indi_orion_steadystar", MAX_RETRIES);
    return false;
}

void OrionSteadyStarHardware::Disconnect()
{
    std::lock_guard<std::mutex> lock(m_UsbMutex);
    if (m_DeviceHandle)
    {
        libusb_release_interface(m_DeviceHandle, 0);
        libusb_close(m_DeviceHandle);
        m_DeviceHandle = nullptr;
    }
}

bool OrionSteadyStarHardware::InterruptWrite(const uint8_t *data)
{
    if (m_Debug && !m_Simulation)
    {
        LOGF_DEBUG("CMD: %02x %02x %02x %02x %02x %02x %02x %02x",
                   data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    }

    if (m_Simulation)
    {
        return true;
    }

    int actual = 0;
    int retries = 3;
    while (retries-- > 0)
    {
        int rc = libusb_interrupt_transfer(m_DeviceHandle, ENDPOINT_OUT,
                                           const_cast<uint8_t *>(data), 8, &actual, TIMEOUT_MS);
        if (rc == LIBUSB_SUCCESS && actual == 8)
            return true;
        LOGF_WARN("InterruptWrite: %s (sent %d), retrying...", libusb_error_name(rc), actual);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    LOG_ERROR("InterruptWrite failed after retries");
    return false;
}

bool OrionSteadyStarHardware::InterruptRead(uint8_t *data)
{
    if (m_Simulation)
    {
        memset(data, 0, 8);
        return true;
    }

    int actual = 0;
    int retries = 3;
    while (retries-- > 0)
    {
        int rc = libusb_interrupt_transfer(m_DeviceHandle, ENDPOINT_IN,
                                           data, 8, &actual, TIMEOUT_MS);
        if (rc == LIBUSB_SUCCESS && actual == 8)
        {
            if (m_Debug)
                LOGF_DEBUG("RES: %02x %02x %02x %02x %02x %02x %02x %02x",
                           data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
            return true;
        }
        LOGF_WARN("InterruptRead: %s (got %d), retrying...", libusb_error_name(rc), actual);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    LOG_ERROR("InterruptRead failed after retries");
    return false;
}

bool OrionSteadyStarHardware::GetPosition(int8_t &m1, int8_t &m2, int8_t &m3, int8_t &m4)
{
    if (m_Simulation)
    {
        m1 = m2 = m3 = m4 = 0;
        return true;
    }
    std::lock_guard<std::mutex> lock(m_UsbMutex);
    if (!m_DeviceHandle) return false;

    uint8_t cmd1[8] = {0x30, 0x00, 0x00, 0x00, 0xc7, 0xdd, 0x42, 0x1a};
    uint8_t cmd2[8] = {0x39, 0x16, 0x42, 0x1a, 0xc7, 0xdd, 0x42, 0x1a};
    uint8_t cmd3[8] = {0x62, 0x00, 0x00, 0x00, 0xc7, 0xdd, 0x42, 0x1a};
    uint8_t result[8] = {0};

    if (!InterruptWrite(cmd1)) return false;
    if (!InterruptWrite(cmd2)) return false;
    if (!InterruptWrite(cmd3)) return false;

    if (!InterruptRead(result)) return false;

    m1 = b_2_i(result[0]);
    m2 = b_2_i(result[1]);
    m3 = b_2_i(result[2]);
    m4 = b_2_i(result[3]);

    return true;
}

bool OrionSteadyStarHardware::GetHomingStatus(uint8_t &status)
{
    if (m_Simulation)
    {
        status = 0;
        return true;
    }
    std::lock_guard<std::mutex> lock(m_UsbMutex);
    if (!m_DeviceHandle) return false;

    uint8_t cmd[8] = {0x34, 0x00, 0x00, 0x00, 0xc7, 0xdd, 0x42, 0x1a};
    uint8_t result[8] = {0};

    if (!InterruptWrite(cmd)) return false;
    if (!InterruptRead(result)) return false;

    status = result[0];
    return true;
}

bool OrionSteadyStarHardware::Home()
{
    // Matches Python home() exactly:
    // 1. Read current positions and replay them as deltas (firmware position-counter sync).
    // 2. Poll/clear limit switches.
    // 3. Read positions again and replay to complete settling.
    int8_t m1, m2, m3, m4;
    if (!GetPosition(m1, m2, m3, m4)) return false;
    if (!MoveMotors(m1, m2, m3, m4)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (!GetPosition(m1, m2, m3, m4)) return false;

    uint8_t status = 0;
    if (!GetHomingStatus(status)) return false;

    int timeout = 200;
    while (status != 0 && timeout-- > 0)
    {
        int8_t move_m1 = (status & 0x10) ? -7 : 2;
        int8_t move_m2 = (status & 0x20) ? -7 : 2;
        int8_t move_m3 = (status & 0x40) ? -7 : 2;
        int8_t move_m4 = (status & 0x80) ? -7 : 2;

        if (!MoveMotors(move_m1, move_m2, move_m3, move_m4)) return false;
        if (!GetHomingStatus(status)) return false;
    }

    if (!GetPosition(m1, m2, m3, m4)) return false;
    if (!MoveMotors(m1, m2, m3, m4)) return false;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (!GetPosition(m1, m2, m3, m4)) return false;
    m_m1 = m_m2 = m_m3 = m_m4 = 0;
    return (timeout > 0);
}

bool OrionSteadyStarHardware::MoveMotor(uint8_t motor_idx, int8_t delta)
{
    if (m_Simulation)
        return true;
    std::lock_guard<std::mutex> lock(m_UsbMutex);
    if (!m_DeviceHandle) return false;

    uint8_t cmd[8] = {0x61, c2_byte(delta), c2_byte(static_cast<int8_t>(motor_idx)), 0x00, 0xc7, 0xdd, 0x42, 0x1a};
    return InterruptWrite(cmd);
}

bool OrionSteadyStarHardware::MoveMotors(int8_t m1, int8_t m2, int8_t m3, int8_t m4)
{
    if (m1 != 0 && !MoveMotor(0, m1)) return false;
    if (m2 != 0 && !MoveMotor(1, m2)) return false;
    if (m3 != 0 && !MoveMotor(2, m3)) return false;
    if (m4 != 0 && !MoveMotor(3, m4)) return false;
    return true;
}

bool OrionSteadyStarHardware::SetAO(int tx, int ty)
{
    // Keep all intermediate math in int to avoid int8_t overflow before clipping.
    int ddx = tx / 2;
    int ddy = ty / 2;

    int itm1 = tx - ddx;
    int itm2 = ty - ddy;
    int itm3 = 0 - ddx;
    int itm4 = 0 - ddy;

    auto clip = [](int v) -> int8_t {
        if (v < -80) v = -80;
        if (v > 80)  v = 80;
        return static_cast<int8_t>(v);
    };

    int8_t target_m1 = clip(itm1);
    int8_t target_m2 = clip(itm2);
    int8_t target_m3 = clip(itm3);
    int8_t target_m4 = clip(itm4);

    int8_t delta_m1 = target_m1 - m_m1;
    int8_t delta_m2 = target_m2 - m_m2;
    int8_t delta_m3 = target_m3 - m_m3;
    int8_t delta_m4 = target_m4 - m_m4;

    if (!MoveMotors(delta_m1, delta_m2, delta_m3, delta_m4)) return false;

    m_m1 = target_m1;
    m_m2 = target_m2;
    m_m3 = target_m3;
    m_m4 = target_m4;

    return true;
}

bool OrionSteadyStarHardware::StartRotation(double angle)
{
    constexpr double STEPS_PER_DEGREE = 29.0;
    constexpr int STEPS_PER_CIRCLE = 10440;

    // Orion protocol angle * 29
    double effectiveAngle = m_ReverseRotation ? range360(360.0 - angle) : range360(angle);
    int iangle = static_cast<int>(effectiveAngle * STEPS_PER_DEGREE + 0.5) % STEPS_PER_CIRCLE;

    // Update target angle to the actual quantized step the hardware will reach
    m_TargetAngle = iangle / STEPS_PER_DEGREE;
    // If we are reversing, we need to convert back to the user's coordinate space
    if (m_ReverseRotation)
        m_TargetAngle = range360(360.0 - m_TargetAngle);

    m_Rotating = true;
    m_LastPollTime = std::chrono::steady_clock::now();

    if (m_Simulation)
    {
        LOGF_DEBUG("simulation: starting rotation to %.2f (quantized)", m_TargetAngle);
        return true;
    }

    std::lock_guard<std::mutex> lock(m_UsbMutex);
    if (!m_DeviceHandle) return false;

    uint8_t cmd_init[8] = {0x67, 0x27, 0x74, 0x03, 0x8f, 0xb0, 0x27, 0x95};
    if (!InterruptWrite(cmd_init)) return false;

    uint8_t result[8];
    if (!InterruptRead(result)) return false;

    uint8_t low_byte = iangle & 0xFF;
    uint8_t high_byte = (iangle >> 8) & 0xFF;
    uint8_t cmd_rot[8] = {0x6b, low_byte, high_byte, 0x0d, low_byte, high_byte, 0x00, 0x00};
    if (!InterruptWrite(cmd_rot)) return false;

    return true;
}

bool OrionSteadyStarHardware::PollRotation(bool &busy, double &current_angle)
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_LastPollTime).count();
    m_LastPollTime = now;

    if (m_Simulation)
    {
        // Simulating 10 degrees per second
        double rotation_rate = 10.0 * (elapsed / 1000.0);
        
        double diff = std::fmod(m_TargetAngle - m_CurrentAngle + 360.0, 360.0);
        if (diff > 180.0) diff = 360.0 - diff;
        if (diff <= rotation_rate)
        {
            m_CurrentAngle = m_TargetAngle;
            m_Rotating = false;
        }
        else
        {
            double a = m_TargetAngle;
            double b = m_CurrentAngle;
            int sign = (a - b >= 0 && a - b <= 180) || (a - b <= -180 && a - b >= -360) ? 1 : -1;
            m_CurrentAngle = range360(m_CurrentAngle + (rotation_rate * sign));
        }
        // Round m_CurrentAngle to nearest hardware step for simulator consistency
        m_CurrentAngle = static_cast<int>(m_CurrentAngle * 29.0 + 0.5) / 29.0;
    }
    else
    {
        std::lock_guard<std::mutex> lock(m_UsbMutex);
        if (!m_DeviceHandle) return false;

        uint8_t cmd_poll[8] = {0x68, 0x75, 0xf9, 0x05, 0x38, 0xdb, 0x76, 0x0b};
        uint8_t result[8];

        if (!InterruptWrite(cmd_poll)) return false;
        if (!InterruptRead(result)) return false;

        // Based on the protocol reference and Python script, 
        // rotation takes approx 11 seconds. 
        // We'll update m_CurrentAngle incrementally toward m_TargetAngle 
        // based on time, and stop when the hardware stops returning non-zero.
        
        busy = false;
        for (int i = 0; i < 8; ++i)
        {
            if (result[i] != 0)
            {
                busy = true;
                break;
            }
        }

        // Query absolute rotator position (0x70 confirmed from USB capture).
        uint8_t cmd_pos[8] = {0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20};
        if (InterruptWrite(cmd_pos) && InterruptRead(result))
        {
            uint16_t pos = result[0] | (static_cast<uint16_t>(result[1]) << 8);
            double read_angle = static_cast<double>(pos) / 29.0;
            m_CurrentAngle = m_ReverseRotation ? range360(360.0 - read_angle) : range360(read_angle);
        }
        else
        {
            // Fallback to time-based estimation if position query fails
            // Approx 360 degrees / 11 seconds = ~32 deg/sec
            double rotation_rate = 32.0 * (elapsed / 1000.0);
            double fb_diff = std::fmod(m_TargetAngle - m_CurrentAngle + 360.0, 360.0);
            if (fb_diff > 180.0) fb_diff = 360.0 - fb_diff;
            if (fb_diff <= rotation_rate || !busy)
            {
                m_CurrentAngle = m_TargetAngle;
            }
            else
            {
                double a = m_TargetAngle;
                double b = m_CurrentAngle;
                int sign = (a - b >= 0 && a - b <= 180) || (a - b <= -180 && a - b >= -360) ? 1 : -1;
                m_CurrentAngle = range360(m_CurrentAngle + (rotation_rate * sign));
            }
        }

        if (!busy)
        {
            m_Rotating = false;
        }
    }

    busy = m_Rotating;
    current_angle = m_CurrentAngle;
    return true;
}

bool OrionSteadyStarHardware::GetRotatorAngle(double &angle)
{
    if (m_Simulation)
    {
        angle = m_CurrentAngle;
        return true;
    }

    std::lock_guard<std::mutex> lock(m_UsbMutex);
    if (!m_DeviceHandle) return false;

    uint8_t cmd[8] = {0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20};
    uint8_t result[8] = {0};

    if (!InterruptWrite(cmd)) return false;
    if (!InterruptRead(result)) return false;

    uint16_t pos = result[0] | (static_cast<uint16_t>(result[1]) << 8);
    double read_angle = static_cast<double>(pos) / 29.0;
    
    angle = m_ReverseRotation ? range360(360.0 - read_angle) : range360(read_angle);
    m_CurrentAngle = angle;
    return true;
}

bool OrionSteadyStarHardware::AbortRotation()
{
    // The Orion SteadyStar protocol has no mid-rotation stop command; the
    // firmware always runs to the commanded position. Clearing m_Rotating
    // stops the driver from issuing further derotation commands but does not
    // halt the current hardware move.
    m_Rotating = false;
    if (m_Simulation)
        LOGF_DEBUG("%s", "simulation: rotation aborted");
    return true;
}

bool OrionSteadyStarHardware::SyncRotation(double angle)
{
    m_CurrentAngle = range360(angle);
    m_TargetAngle = m_CurrentAngle;
    m_Rotating = false;
    return true;
}

bool OrionSteadyStarHardware::ReverseRotation(bool enabled)
{
    m_ReverseRotation = enabled;
    return true;
}

uint8_t OrionSteadyStarHardware::c2_byte(int8_t v)
{
    // Orion firmware uses sign-magnitude encoding for motor move deltas:
    // bit 7 = sign (1 = negative), bits 0-6 = magnitude.
    // This matches Python: c2_byte = 0x80 - v for v < 0.
    if (v < 0)
        return 0x80 | static_cast<uint8_t>(-v);
    return static_cast<uint8_t>(v);
}

int8_t OrionSteadyStarHardware::b_2_i(uint8_t v)
{
    return static_cast<int8_t>(v);
}

