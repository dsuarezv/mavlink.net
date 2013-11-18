using System;
using System.IO;

namespace MavLinkObjectGenerator
{
    public abstract class GenericGenerator
    {
        public abstract void Write(ProtocolData data, TextWriter w);


        // __ CRC _____________________________________________________________


        // Adapted from python Mavlink generator (https://github.com/mavlink/mavlink)

        protected static byte GetMessageExtraCrc(MessageData m)
        {
            /*
            def message_checksum(msg):
                from mavcrc import x25crc
                
                crc = x25crc(msg.name + ' ')
                for f in msg.ordered_fields:
                    crc.accumulate(f.type + ' ')
                    crc.accumulate(f.name + ' ')
                    if f.array_length:
                        crc.accumulate(chr(f.array_length))
                return (crc.crc&0xFF) ^ (crc.crc>>8)
             */

            UInt16 crc = X25Crc.X25CrcCalculate(m.Name + ' ');

            foreach (FieldData f in m.Fields)
            {
                crc = X25Crc.X25CrcAccumulate(XmlParser.GetBasicFieldTypeFromString(f.TypeString) + ' ', crc);
                crc = X25Crc.X25CrcAccumulate(f.Name + ' ', crc);

                if (f.NumElements > 1)
                {
                    crc = X25Crc.X25CrcAccumulate((byte)f.NumElements, crc);
                }
            }

            byte result = (byte)((crc & 0xFF) ^ (crc >> 8));

            //Console.WriteLine("{0}  {1}", m.Name, result);
            return result;
        }
    }
}
