using System;
using System.IO;
using System.Threading;
using System.Collections.Generic;

namespace MavLinkNet
{
    public class MavLinkLogFileTransport: MavLinkGenericTransport
    {
        private string mLogFileName;

        public MavLinkLogFileTransport(string logFileName)
        {
            mLogFileName = logFileName;
        }

        public override void Initialize()
        {
            Parse();
        }

        public override void Dispose()
        {
            
        }

        public override void SendMessage(UasMessage msg)
        {
            // No messages are sent on this transport (only read from the logfile)
        }


        // __ Impl ____________________________________________________________


        private void Parse()
        {
            try
            {
                using (FileStream s = new FileStream(mLogFileName, FileMode.Open))
                {
                    using (BinaryReader reader = new BinaryReader(s))
                    {
                        while (true)
                        {
                            SyncStream(reader);
                            MavLinkPacket packet = MavLinkPacket.Deserialize(reader, 0);

                            if (packet.IsValid)
                            {
                                HandlePacketReceived(this, packet);
                            }
                        }
                    }
                }
            }
            catch (EndOfStreamException)
            { 
                
            }

            HandleReceptionEnded(this);
        }

        private void SyncStream(BinaryReader s)
        {
            while (s.ReadByte() != MavLinkGenericPacketWalker.PacketSignalByte)
            {
                // Skip bytes until a packet start is found
            }
        }
    }
}
