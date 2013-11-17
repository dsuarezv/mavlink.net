using System;
using System.IO;

namespace MavLinkNet
{
    public class MavLinkPacket
    {
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
        public static MavLinkPacket Deserialize(BinaryReader s)
        {
            MavLinkPacket result = new MavLinkPacket()
            {
                PayLoadLength = s.ReadByte(),
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

        private bool IsValidCrc()
        {
            // Validate checksum

            return true;
        }

        private void DeserializeMessage()
        {
            UasMessage result = UasSummary.CreateFromId(MessageId);

            if (result == null) return;  // Unknown type

            using (MemoryStream ms = new MemoryStream(Payload))
            {
                using (BinaryReader br = new BinaryReader(ms))
                {
                    result.DeserializeBody(br);
                }
            }

            Message = result;
            IsValid = true;
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
            // Updates the CRC field using the other packet fields and payload.
            throw new NotImplementedException("CRC calculation not implemented yet");
        }
    }
}
