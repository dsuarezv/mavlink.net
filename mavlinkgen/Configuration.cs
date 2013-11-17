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

