using System;
using System.IO;
using System.Collections.Generic;
using System.Text;

namespace MavLinkNet
{
    public class UasMessage
    {
        public byte CrcExtra { get; protected set; }

        public byte MessageId
        {
            get { return mMessageId; }
        }

        protected virtual void NotifyUpdated()
        { 
            
        }

        internal virtual void SerializeBody(BinaryWriter s)
        { 
        
        }

        internal virtual void DeserializeBody(BinaryReader stream)
        { 
        
        }

        protected byte mMessageId;
    }
}
