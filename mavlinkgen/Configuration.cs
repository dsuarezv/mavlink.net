using System;
using System.Collections.Generic;

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


        public void CheckValid()
        {
            if (mFiles.Count == 0)
            {
                throw new Exception(GetUsage());
            }
        }

        private string GetUsage()
        {
            return "Usage: mavlinkgen [--output=<output directory>]  <xml definition file>";
        }

        public string GetOption(string opt)
        {
            string result;

            if (mOptions.TryGetValue(opt, out result))
                return result;

            return "";
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

