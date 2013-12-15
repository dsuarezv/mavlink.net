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
using System.Threading;
using System.Collections.Generic;


namespace System.IO
{
    /// <summary>
    /// A blocking circular stream is an in-memory circular buffer that provides
    /// Stream methods to read and write. "Blocking" means that it will block the Read thread
    /// until the requested buffer length is filled (by writing from a different thread).
    /// </summary>
    public class BlockingCircularStream : Stream
    {
        private byte[] mBuffer;
        private int mWritePosition;
        private int mReadPosition;
        private int mCapacity;
        private AutoResetEvent mBlockSignal = new AutoResetEvent(false);
        private bool mReadIsAborted = false;
        private object mAccessLock = new object();


        public BlockingCircularStream(int bufferCapacity)
        {
            mCapacity = bufferCapacity;
            mBuffer = new byte[bufferCapacity];
        }

        protected override void Dispose(bool disposing)
        {
            base.Dispose(disposing);

            AbortRead();

            mBlockSignal.Dispose();
        }

        public override bool CanRead
        {
            get { return true; }
        }

        public override bool CanSeek
        {
            get { return false; }
        }

        public override bool CanWrite
        {
            get { return true; }
        }

        public override void Flush()
        {

        }

        public override long Length
        {
            get
            {
                lock (mAccessLock)
                {
                    if (mWritePosition >= mReadPosition)
                    {
                        return mWritePosition - mReadPosition;
                    }
                    else
                    {
                        return mCapacity - mReadPosition + mWritePosition;
                    }
                }
            }
        }

        public override long Position
        {
            get
            {
                throw new NotImplementedException();
            }
            set
            {
                throw new NotImplementedException();
            }
        }

        /// <summary>
        /// Returns 'count' bytes from the stream into 'buffer'.
        /// </summary>
        /// <param name="buffer">The buffer that will receive the read bytes</param>
        /// <param name="offset">offset in the result buffer to start copying read bytes</param>
        /// <param name="count">Number of bytes to be copied from the stream into buffer</param>
        /// <returns>The number of bytes read. or -1 if the Read operation is 
        /// aborted (by calling AbortRead() from another thread)</returns>
        /// <remarks>
        /// The read operation will return 'count' bytes from the stream if they
        /// are available. Otherwise, this method will block until enough bytes
        /// are written through the Write() method from another thread.
        /// </remarks>
        public override int Read(byte[] buffer, int offset, int count)
        {
            if (count > mCapacity)
                throw new IndexOutOfRangeException("Tried to read more bytes than the capacity of the circular buffer.");

            mReadIsAborted = false;

            // wait until the buffer has enough bytes available
            while (Length < count)
            {
                mBlockSignal.WaitOne();
                if (mReadIsAborted) return -1;
            }

            lock (mAccessLock)
            {
                if (mReadPosition + count <= mCapacity)
                {
                    // Direct copy
                    CopyBytes(mBuffer, buffer, mReadPosition, offset, count);
                    mReadPosition += count;
                }
                else
                {
                    // Split buffer (new round)
                    int len1 = mCapacity - mReadPosition;
                    int len2 = count - len1;

                    CopyBytes(mBuffer, buffer, mReadPosition, offset, len1);
                    CopyBytes(mBuffer, buffer, 0, offset + len1, len2);

                    mReadPosition = len2;
                }

                return count;
            }
        }

        /// <summary>
        /// Aborts a pending Read() operation if one is in progress. 
        /// </summary>
        /// <remarks>If no Read() method is waiting for bytes to arrive in the stream, 
        /// this method does nothing. In any case, this method returns inmediately.</remarks>
        public void AbortRead()
        {
            mReadIsAborted = true;
            mBlockSignal.Set();
        }

        public override int ReadByte()
        {
            while (mReadPosition == mWritePosition)
            {
                mBlockSignal.WaitOne();
            }

            lock (mAccessLock)
            {
                if (mReadPosition == mCapacity) mReadPosition = 0;

                return mBuffer[mReadPosition++];
            }
        }

        public override long Seek(long offset, SeekOrigin origin)
        {
            throw new NotImplementedException();
        }

        public override void SetLength(long value)
        {
            throw new NotImplementedException();
        }

        public override void Write(byte[] buffer, int offset, int count)
        {
            if (count > mCapacity)
                throw new IndexOutOfRangeException("Tried to write more bytes than the capacity of the circular buffer.");

            lock (mAccessLock)
            {
                if (mWritePosition + count <= mCapacity)
                {
                    // Direct copy
                    CopyBytes(buffer, mBuffer, offset, mWritePosition, count);
                    mWritePosition += count;
                }
                else
                {
                    // Split buffer (new round)
                    int len1 = mCapacity - mWritePosition;
                    int len2 = count - len1;

                    if (len2 > mReadPosition)
                    {
                        throw new InternalBufferOverflowException("Data is being overwritten without being read. May need to increase capacity.");
                    }

                    CopyBytes(buffer, mBuffer, offset, mWritePosition, len1);
                    CopyBytes(buffer, mBuffer, offset + len1, 0, len2);

                    mWritePosition = len2;
                }
            }

            mBlockSignal.Set();
        }

        public override string ToString()
        {
            return string.Format("{0}: r:{1} w:{2} l:{3}",
                System.Text.Encoding.UTF8.GetString(mBuffer),
                mReadPosition,
                mWritePosition,
                Length);
        }


        // __ Impl ____________________________________________________________


        private void CopyBytes(byte[] source, byte[] destination, int sourceStart, int destinationStart, int count)
        {
            for (int i = 0; i < count; ++i)
            {
                destination[destinationStart + i] = source[sourceStart + i];
            }
        }
    }
}
