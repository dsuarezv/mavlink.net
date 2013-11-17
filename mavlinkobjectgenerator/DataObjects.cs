using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace MavLinkObjectGenerator
{
    public enum FieldDataType
    {
        INT8 = 0,
        INT16,
        INT32,
        INT64,
        UINT8,
        UINT16,
        UINT32,
        FLOAT32,
        UINT64,
        CHAR,
        ENUM,
        NONE
    };

    public class ProtocolData
    {
        public int Version;
        public Dictionary<string, EnumData> Enumerations = new Dictionary<string, EnumData>();
        public Dictionary<string, MessageData> Messages = new Dictionary<string, MessageData>();
    }

    public class ProtocolObject
    {
        public string Name;
        public string Description;
    }

    public class MessageData : ProtocolObject
    {
        public int Id;
        public List<FieldData> Fields = new List<FieldData>();
    }

    [DebuggerDisplay("{Name}: [type: {TypeString}] [enum: {EnumType}]")]
    public class FieldData : ProtocolObject
    {
        public string TypeString;
        public FieldDataType Type;
        public int NumElements;
        public bool IsEnum = false;
        public string EnumType;
    }

    public class EnumData : ProtocolObject
    {
        public List<EnumEntry> Entries = new List<EnumEntry>();
    }

    public class EnumEntry : ProtocolObject
    {
        public int Value;
        public List<EnumEntryParameter> Parameters = new List<EnumEntryParameter>();
    }

    public class EnumEntryParameter
    {
        public int Index;
        public string Description;
    }
}

