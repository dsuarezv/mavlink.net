using System;
using System.IO;
using System.Threading;
using System.Collections.Generic;

namespace MavLinkNet
{
    public class MavLinkLogFileTransport: MavLinkGenericTransport
    {
        private string mLogFileName;
        private bool mWrite;
        private BinaryWriter binWriter;
        private FileStream logFile;
        private MavLinkAsyncWalker walker;

        public MavLinkLogFileTransport(string logFileName, bool write=false)
        {
            mLogFileName = logFileName;
            mWrite = write; // Are we writing to the file? Reading is the default.

            walker = new MavLinkAsyncWalker();
        }

        public override void Initialize()
        {
            if (!mWrite) {
                Parse(); // If we're reading the file, parse it and be done.
                return;
            }

            // Ok, lets open up a file for writing.
            logFile = File.Open(mLogFileName, File.Exists(mLogFileName) ? FileMode.Append : FileMode.Create);
            // And create a binary writer we can throw bytes at.
            binWriter = new BinaryWriter(logFile);

        }

        public override void Dispose()
        {
            if (mWrite)
            {
                if (binWriter != null)
                {
                    binWriter.Flush();
                    binWriter.Close();
                    binWriter.Dispose();
                }

                if (logFile != null)
                {
                    logFile.Close();
                    logFile.Dispose();
                }
            }
            binWriter = null;
            logFile = null;
        }

        public byte MavlinkSequenceNumber
        {
            get;
            set;
        }

        public override void SendMessage(UasMessage msg)
        {
            if (mWrite)
            {
                // Serialize the message and write it to the log file
                byte[] packet_bytes = walker.SerializeMessage(msg, MavlinkSystemId, MavlinkComponentId, true, MavlinkSequenceNumber);
                binWriter.Write(packet_bytes);
            }
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
