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
        DOUBLE,
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

