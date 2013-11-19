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
using MavLinkObjectGenerator;

namespace MavLinkGen
{
    public class Configuration
    {
        public List<string> Files
        {
            get { return mFiles; }
        }

        public Dictionary<string, string> Options
        {
            get { return mOptions; }
        }

        public string OutputFile
        {
            get
            {
                return GetOption("output");
            }
        }



        public Configuration(string[] args)
        {
            Parse(args);
        }


        // __ API _____________________________________________________________


        public void CheckValid()
        {
            if (mFiles.Count == 0)
            {
                throw new Exception(GetUsage());
            }
        }

        public string GetOption(string opt)
        {
            string result;

            if (mOptions.TryGetValue(opt, out result))
                return result;

            return "";
        }

        public GenericGenerator GetGenerator()
        {
            switch (GetOption("type").ToLower())
            { 
                case "c":
                    throw new NotImplementedException("C generation not supported yet");
                default:
                    return new CSharpGenerator();
            }
        }

        // __ Impl ____________________________________________________________


        private string GetUsage()
        {
            return
                "Usage: mavlinkgen [options] <xml definition file>\n" + 
                "Options:\n" +
                "  --output=<output directory>\n" + 
                "  [--type=<language>]: CSharp is the default and only option for now.\n" +
                "  \n" + 
                "";
        }

        private void Parse(string[] args)
        {
            foreach (string s in args)
            {
                if (IsOption(s))
                {
                    ParseOption(s);
                }
                else
                {
                    ParseFileName(s);
                }
            }
        }

        private bool IsOption(string s)
        {
            return s.StartsWith("-");
        }

        private void ParseFileName(string s)
        {
            mFiles.Add(s);
        }

        private void ParseOption(string s)
        {
            string removedDashes = s.TrimStart('-');
            string[] parts = removedDashes.Split('=');

            if (parts.Length != 2) throw new Exception("Invalid option: " + s);

            mOptions.Add(parts[0].Trim().ToLower(), parts[1].Trim());
        }




        private List<string> mFiles = new List<string>();
        private Dictionary<string, string> mOptions = new Dictionary<string, string>();
    }
}

