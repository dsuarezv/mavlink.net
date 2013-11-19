/*
The MIT License (MIT)

Copyright (c) 2013, David Suarez

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
using System;
using System.Collections.Generic;

namespace MavLinkNet
{
    /// <summary>
    /// Holds message objects representing state. Meant for status messages, not communication objects
    /// </summary>
    public class MavLinkState
    {
        private Dictionary<string, UasMessage> mState = new Dictionary<string, UasMessage>();
        private List<UasMessage> mHeartBeatMessages = new List<UasMessage>();

        public MavLinkState()
        {
            mState.Add("Heartbeat", new UasHeartbeat
            {
                Type = MavType.Quadrotor,
                Autopilot = MavAutopilot.Ardupilotmega,
                BaseMode = MavModeFlag.AutoEnabled,
                CustomMode = 0,
                SystemStatus = MavState.Active,
                MavlinkVersion = (byte)3,
            });

            mState.Add("SysStatus", new UasSysStatus
            {
                Load = 500,
                VoltageBattery = 11000,
                CurrentBattery = -1,
                BatteryRemaining = -1
            });

            mState.Add("LocalPositionNed", new UasLocalPositionNed());

            mState.Add("Attitude", new UasAttitude());
        }

        public UasMessage Get(string mavlinkObjectName)
        {
            return mState[mavlinkObjectName];
        }

        public List<UasMessage> GetHeartBeatObjects()
        {
            if (mHeartBeatMessages.Count == 0)
            {
                mHeartBeatMessages.Add(Get("Heartbeat"));
                mHeartBeatMessages.Add(Get("SysStatus"));
                mHeartBeatMessages.Add(Get("LocalPositionNed"));
                mHeartBeatMessages.Add(Get("Attitude"));
            }

            return mHeartBeatMessages;
        }
    }
}
