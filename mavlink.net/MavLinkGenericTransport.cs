using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MavLinkNet
{
    public abstract class MavLinkGenericTransport: IDisposable
    {
        public byte MavlinkSystemId = 200;
        public byte MavlinkComponentId = 1;
        public MavLinkState UavState = new MavLinkState();

        public event PacketReceivedDelegate OnPacketReceived;

        public abstract void Initialize();
        public abstract void Dispose();
        public abstract void SendMessage(UasMessage msg);


        // __ MavLink events __________________________________________________


        protected void HandlePacketReceived(object sender, MavLinkPacket e)
        {
            if (OnPacketReceived != null) OnPacketReceived(sender, e);
        }
    }
}
