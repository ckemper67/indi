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

#pragma once

#include "indifocuser.h"

class DarkSkyGeek : public INDI::Focuser
{
    public:
        DarkSkyGeek();
        virtual ~DarkSkyGeek() override = default;

        const char * getDefaultName() override;
        virtual bool initProperties() override;
        virtual bool updateProperties() override;

    protected:
        /**
         * @brief Handshake Try to communicate with Focuser and see if there is a valid response.
         * @return True if communication is successful, false otherwise.
         */
        virtual bool Handshake() override;

        /**
         * @brief MoveAbsFocuser Move to an absolute target position
         * @param targetTicks target position
         * @return IPS_BUSY if motion is in progress. IPS_OK if motion is small and already complete. IPS_ALERT for trouble.
         */
        virtual IPState MoveAbsFocuser(uint32_t targetTicks) override;

        /**
         * @brief MoveRelFocuser Move focuser for a relative amount of ticks in a specific direction
         * @param dir Directoin of motion
         * @param ticks steps to move
         * @return IPS_BUSY if motion is in progress. IPS_OK if motion is small and already complete. IPS_ALERT for trouble.
         */
        virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;

        /**
         * @brief SyncFocuser Set the supplied position as the current focuser position
         * @param ticks target position
         * @return IPS_OK if focuser position is now set to ticks. IPS_ALERT for problems.
         */
        virtual bool SyncFocuser(uint32_t ticks) override;

        virtual bool AbortFocuser() override;
        virtual void TimerHit() override;

    private:
        bool Ack();
        /**
         * @brief sendCommand Send a string command to DarkSkyGeek.
         * @param cmd Command to be sent, The function will add the necessary delimiter ('\n')
         * @param resultPefix expected prefix of the result. The function will read the result and compare it to the prefix
         * @param res If not nullptr, the function will read until it detects the default delimiter ('\n') up to ML_RES length.
         *        if nullptr, no read back is done and the function returns true.
         * @return True if successful, false otherwise.
         */
        bool sendCommand(const char * cmd, const char * resultPrefix, char * res = nullptr);

        // Read and update Position
        bool readPosition();
        // Read Version
        bool readVersion();
        // Are we moving?
        bool isMoving();

        // Read Only Temperature Reporting
        INumber TemperatureN[1];
        INumberVectorProperty TemperatureNP;

        double targetPos { 0 }, lastPos { 0 }, lastTemperature { 0 };
        uint32_t maxPosition {4600};
        // DarkSkyGeek buffer size
        static const int DRIVER_RES {256} ;
        // DarkSkyGeek command and result delimiter
        static const char DRIVER_DEL {'\n'};
        // DarkSkyGeek timeout
        static const int DRIVER_TIMEOUT {3} ;
};
