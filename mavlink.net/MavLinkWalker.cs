using System;
using System.IO;

namespace MavLinkNet
{
    public class MavLinkWalker
    {
        public MavLinkPacket GetNextPacket(BinaryReader s)
        {
            while (true)
            {
                SyncStream(s);
                MavLinkPacket packet = MavLinkPacket.Deserialize(s);

                if (packet.IsValid)
                {
                    return packet;
                }
                else
                { 
                    // notify discarded packet. Keep going.
                }
            }
        }

        private void SyncStream(BinaryReader s)
        {
            while (s.ReadByte() != 0xFE)
            {
                // Skip bytes until a packet start is found
            }
        }
    }
}
