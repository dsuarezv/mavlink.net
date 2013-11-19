using System;
using System.IO;
using System.Collections.Generic;


namespace CircularStreamSample
{
    public class CircularStream : Stream
    {
        private byte[] mBuffer;
        private int mWritePosition;
        private int mReadPosition;
        private int mCapacity;


        public CircularStream(int bufferCapacity)
        {
            mCapacity = bufferCapacity;
            mBuffer = new byte[bufferCapacity];

            Reset();
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
                if (mWritePosition > mReadPosition)
                {
                    return mWritePosition - mReadPosition;
                }
                else
                {
                    return mCapacity - mReadPosition + mWritePosition;
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

        public override int Read(byte[] buffer, int offset, int count)
        {
            if (count > mCapacity)
                throw new IndexOutOfRangeException("Tried to read more bytes than the capacity of the circular buffer.");

            int bytesToCopy = (Length <= count) ? count : (int)Length;

            if (mReadPosition + bytesToCopy <= mCapacity)
            {
                // Direct copy
                CopyBytes(mBuffer, buffer, mReadPosition, offset, bytesToCopy);
                mReadPosition += bytesToCopy;
            }
            else
            {
                // Split buffer (new round)
                int len1 = mCapacity - mReadPosition;
                int len2 = bytesToCopy - len1;

                CopyBytes(mBuffer, buffer, mReadPosition, offset, len1);
                CopyBytes(mBuffer, buffer, 0, offset + len1, len2);

                mReadPosition = len2;
            }

            return bytesToCopy;
        }

        public override int ReadByte()
        {
            if (mReadPosition == mWritePosition) return -1;
            if (mReadPosition == mCapacity) mReadPosition = 0;

            return mBuffer[mReadPosition++];
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
                    throw new Exception("Data is being overwritten without being read. May need to increase capacity.");

                CopyBytes(buffer, mBuffer, offset, mWritePosition, len1);
                CopyBytes(buffer, mBuffer, offset + len1, 0, len2);

                mWritePosition = len2;
            }
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


        private void Reset()
        {
            mWritePosition = 0;
            mReadPosition = 0;
        }

        private void CopyBytes(byte[] source, byte[] destination, int sourceStart, int destinationStart, int count)
        {
            for (int i = 0; i < count; ++i)
            {
                destination[destinationStart + i] = source[sourceStart + i];
            }
        }
    }
}
