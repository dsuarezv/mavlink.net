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
using System.Collections.Generic;
using System.Text;

namespace MavLinkNet
{
    public class UasMessage
    {
        public byte CrcExtra 
        { 
            get; 
            protected set; 
        }

        public byte MessageId
        {
            get { return mMessageId; }
        }


        public UasMessageMetadata GetMetadata()
        {
            if (mMetadata == null)
            {
                InitMetadata();
            }

            return mMetadata;
        }


        protected void NotifyUpdated()
        { 
            
        }

        internal virtual void SerializeBody(BinaryWriter s)
        { 
        
        }

        internal virtual void DeserializeBody(BinaryReader stream)
        { 
        
        }

        protected virtual void InitMetadata()
        {

        }


        protected byte mMessageId;
        protected UasMessageMetadata mMetadata;
    }

    
    public class UasMessageMetadata
    {
        public string Description;
        public List<UasFieldMetadata> Fields = new List<UasFieldMetadata>();
    }


    public class UasFieldMetadata
    {
        public string Name;
        public string Description;
        public int NumElements = 1;
        public UasEnumMetadata EnumMetadata;
    }


    public class UasEnumMetadata
    {
        public string Name;
        public string Description;
        public List<UasEnumEntryMetadata> Entries = new List<UasEnumEntryMetadata>();
    }


    public class UasEnumEntryMetadata
    {
        public int Value;
        public string Name;
        public string Description;
        public List<string> Params;
    }
}
