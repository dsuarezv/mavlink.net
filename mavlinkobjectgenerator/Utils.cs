using System;
using System.Text;
using System.Collections.Generic;

namespace MavLinkObjectGenerator
{
    public class Utils
    {   
        private static void ParseItemsIntoList(string items, List<string> target)
        {
            if (items == null || items == "") return;

            string[] ss = items.Split(',');

            foreach (string s in ss)
            {
                target.Add(s);
            }
        }

        public static string GetPascalStyleString(string str)
        {
            // time_boot_ms -> TimeBootMs

            string[] parts = str.Split('_', '-');
            if (parts.Length == 0) return "";

            StringBuilder sb = new StringBuilder();

            foreach (string s in parts) sb.Append(GetPascalStyleWord(s));

            return GetEscapedItemName(sb.ToString());
        }

        public static string GetEscapedItemName(string s)
        {
            if (s == null || s == "") return "";

            //string old = s;

            if (s[0] >= '0' && s[0] <= '9') s = '_' + s;

            s = s.Trim();
            s = s.Replace(' ', '_');
            s = s.Replace('+', '_');
            s = s.Replace('.', '_');
            s = s.Replace('(', '_');
            s = s.Replace(')', '_');
            s = s.Replace('-', '_');

            //Console.WriteLine("D: initial: [{0}] processed: [{1}]", old, s);

            return s;
        }

        private static string GetPascalStyleWord(string word)
        {
            if (word.Length == 0) return "";

            char[] cs = word.ToLower().ToCharArray();
            cs[0] = new String(cs[0], 1).ToUpper()[0];

            return new String(cs);
        }
    }
}

