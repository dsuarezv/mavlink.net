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
using System.Text;

namespace MavLinkNet
{
    public class MavLinkPacket
    {
        public const int PacketOverheadNumBytes = 7;

        public bool IsValid = false;

        public byte PayLoadLength;
        public byte PacketSequenceNumber;
        public byte SystemId;
        public byte ComponentId;
        public byte MessageId;
        public byte[] Payload;
        public byte Checksum1;
        public byte Checksum2;

        public UasMessage Message;

        // __ Deserialization _________________________________________________

        /*
         * Byte order:
         * 
         * 0  Packet start sign	
         * 1	 Payload length	 0 - 255
         * 2	 Packet sequence	 0 - 255
         * 3	 System ID	 1 - 255
         * 4	 Component ID	 0 - 255
         * 5	 Message ID	 0 - 255
         * 6 to (n+6)	 Data	 (0 - 255) bytes
         * (n+7) to (n+8)	 Checksum (high byte, low byte) for v0.9, lowbyte, highbyte for 1.0
         *
         */
        public static MavLinkPacket Deserialize(BinaryReader s, byte payloadLength)
        {
            MavLinkPacket result = new MavLinkPacket()
            {
                PayLoadLength = (payloadLength == 0) ? s.ReadByte() : payloadLength,
                PacketSequenceNumber = s.ReadByte(),
                SystemId = s.ReadByte(),
                ComponentId = s.ReadByte(),
                MessageId = s.ReadByte(),
            };

            // Read the payload instead of deserializing so we can validate CRC.
            result.Payload = s.ReadBytes(result.PayLoadLength);
            result.Checksum1 = s.ReadByte();
            result.Checksum2 = s.ReadByte();

            if (result.IsValidCrc())
            {
                result.DeserializeMessage();
            }

            return result;
        }

        public int GetPacketSize()
        {
            return PacketOverheadNumBytes + PayLoadLength;
        }

        private bool IsValidCrc()
        {
            UInt16 crc = GetPacketCrc(this);

            return ( ((byte)(crc & 0xFF) == Checksum1) &&
                     ((byte)(crc >> 8) == Checksum2) );
        }

        private void DeserializeMessage()
        {
            UasMessage result = UasSummary.CreateFromId(MessageId);

            if (result == null) return;  // Unknown type

            using (MemoryStream ms = new MemoryStream(Payload))
            {
                using (BinaryReader br = GetBinaryReader(ms))
                {
                    result.DeserializeBody(br);
                }
            }

            Message = result;
            IsValid = true;
        }

        public static BinaryReader GetBinaryReader(Stream s)
        {
            return new BinaryReader(s, Encoding.ASCII);
        }


        // __ Serialization ___________________________________________________


        public static MavLinkPacket GetPacketForMessage(
            UasMessage msg, byte systemId, byte componentId, byte sequenceNumber)
        {
            MavLinkPacket result = new MavLinkPacket()
            {
                SystemId = systemId,
                ComponentId = componentId,
                PacketSequenceNumber = sequenceNumber,
                MessageId = msg.MessageId,
                Message = msg
            };

            using (MemoryStream ms = new MemoryStream())
            {
                using (BinaryWriter bw = new BinaryWriter(ms))
                {
                    msg.SerializeBody(bw);
                }

                result.Payload = ms.ToArray();
                result.PayLoadLength = (byte)result.Payload.Length;
                result.UpdateCrc();
            }

            return result;
        }

        public static byte[] GetBytesForMessage(
            UasMessage msg, byte systemId, byte componentId, byte sequenceNumber, byte signalMark)
        {
            MavLinkPacket p = MavLinkPacket.GetPacketForMessage(
                                 msg, systemId, componentId, sequenceNumber);

            int bufferSize = p.GetPacketSize();

            if (signalMark != 0) bufferSize++;

            byte[] result = new byte[bufferSize];

            using (MemoryStream s = new MemoryStream(result))
            {
                using (BinaryWriter w = new BinaryWriter(s))
                {
                    if (signalMark != 0) w.Write(signalMark);
                    p.Serialize(w);
                }
            }

            return result;
        }


        public void Serialize(BinaryWriter w)
        {
            w.Write(PayLoadLength);
            w.Write(PacketSequenceNumber);
            w.Write(SystemId);
            w.Write(ComponentId);
            w.Write(MessageId);
            w.Write(Payload);
            w.Write(Checksum1);
            w.Write(Checksum2);
        }

        private void UpdateCrc()
        {
            UInt16 crc = GetPacketCrc(this);
            Checksum1 = (byte)(crc & 0xFF);
            Checksum2 = (byte)(crc >> 8);
        }

        public static UInt16 GetPacketCrc(MavLinkPacket p)
        {
            UInt16 crc = X25CrcSeed;

            crc = X25CrcAccumulate(p.PayLoadLength, crc);
            crc = X25CrcAccumulate(p.PacketSequenceNumber, crc);
            crc = X25CrcAccumulate(p.SystemId, crc);
            crc = X25CrcAccumulate(p.ComponentId, crc);
            crc = X25CrcAccumulate(p.MessageId, crc);

            for (int i = 0; i < p.Payload.Length; ++i)
            {
                crc = X25CrcAccumulate(p.Payload[i], crc);
            }

            crc = X25CrcAccumulate(UasSummary.GetCrcExtraForId(p.MessageId), crc);

            return crc;
        }



        // __ CRC _____________________________________________________________


        // CRC code adapted from Mavlink C# generator (https://github.com/mavlink/mavlink)

        const UInt16 X25CrcSeed = 0xffff;

        public static UInt16 X25CrcAccumulate(byte b, UInt16 crc)
        {
            unchecked
            {
                byte ch = (byte)(b ^ (byte)(crc & 0x00ff));
                ch = (byte)(ch ^ (ch << 4));
                return (UInt16)((crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4));
            }
        }
    }
}
