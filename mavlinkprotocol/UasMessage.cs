using System;
using System.IO;
using System.Collections.Generic;
using System.Text;

namespace MavLinkNet
{
    public class UasMessage
    {
        protected virtual void NotifyUpdated()
        { 
            
        }

        internal virtual void SerializeBody(BinaryWriter s)
        { 
        
        }

        internal virtual void DeserializeBody(BinaryReader stream)
        { 
        
        }
    }
}
