using System;
using System.Text;

namespace MavLinkObjectGenerator
{
    public class X25Crc
    {

        // Crc code copied/adapted from ardumega planner code / Mavlink C# generator (https://github.com/mavlink/mavlink)

        private const UInt16 X25_INIT_CRC = 0xffff;

        public static UInt16 X25CrcCalculate(string s)
        {
            return X25CrcAccumulate(s, X25_INIT_CRC);
        }

        public static UInt16 X25CrcAccumulate(string s, UInt16 crc)
        {
            // 28591 = ISO-8859-1 = latin1
            byte[] bytes = Encoding.GetEncoding(28591).GetBytes(s);

            foreach (byte b in bytes)
            {
                crc = X25CrcAccumulate(b, crc);
            }

            return crc;
        }

        public static UInt16 X25CrcAccumulate(byte b, UInt16 crc)
        {
            unchecked
            {
                byte ch = (byte)(b ^ (byte)(crc & 0x00ff));
                ch = (byte)(ch ^ (ch << 4));
                return (UInt16)((crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4));
            }
        }

        public static UInt16 X25CrcCalculate(byte[] buffer)
        {
            UInt16 crc = X25_INIT_CRC;

            for (int i = 0; i < buffer.Length; ++i)
            {
                crc = X25CrcAccumulate(buffer[i], crc);
            }

            return crc;
        }
    }
}
