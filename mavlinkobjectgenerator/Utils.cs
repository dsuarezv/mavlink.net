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

