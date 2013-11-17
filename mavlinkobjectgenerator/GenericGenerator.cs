using System;
using System.IO;

namespace MavLinkObjectGenerator
{
    public abstract class GenericGenerator
    {
        public abstract void Write(ProtocolData data, TextWriter w);
    }
}
