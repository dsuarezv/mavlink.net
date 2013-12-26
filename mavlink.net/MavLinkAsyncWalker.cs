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
using System.IO;
using System.Threading;

namespace MavLinkNet
{
    /// <summary>
    /// Process incoming packets and prepare outgoing messages for sending.
    /// </summary>
    /// <remarks>
    /// This class maintains a circular buffer that gets the byte input from the wire,
    /// and a background thread that processes the circular buffer as soon as data 
    /// is received.
    /// </remarks>
    public class MavLinkAsyncWalker: MavLinkGenericPacketWalker
    {
        public const int DefaultCircularBufferSize = 4096;

        private BlockingCircularStream mProcessStream;


        public MavLinkAsyncWalker()
        {
            mProcessStream = new BlockingCircularStream(DefaultCircularBufferSize);

            ThreadPool.QueueUserWorkItem(new WaitCallback(PacketProcessingWorker));
        }

        /// <summary>
        /// Add bytes to the processing queue.
        /// </summary>
        /// <param name="buffer">The buffer to process</param>
        public override void ProcessReceivedBytes(byte[] buffer, int start, int length)
        {
            mProcessStream.Write(buffer, 0, buffer.Length);
        }


        // __ Impl ____________________________________________________________


        private void PacketProcessingWorker(object state)
        {
            using (BinaryReader reader = MavLinkPacket.GetBinaryReader(mProcessStream))
            {
                while (true)
                {
                    SyncStream(reader);
                    MavLinkPacket packet = MavLinkPacket.Deserialize(reader, 0);

                    if (packet.IsValid)
                    {
                        NotifyPacketReceived(packet);
                    }
                    else
                    {
                        NotifyPacketDiscarded(packet);
                    }
                }
            }
        }

        private void SyncStream(BinaryReader s)
        {
            while (s.ReadByte() != PacketSignalByte)
            {
                // Skip bytes until a packet start is found
            }
        }
    }
}
