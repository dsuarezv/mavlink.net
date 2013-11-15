using System;
using System.Text;
using System.Collections.Generic;

namespace UavObjectGenerator
{
    public enum FieldDataType{
        INT8 = 0,
        INT16,
        INT32,
        UINT8,
        UINT16,
        UINT32,
        FLOAT32,
        ENUM, 
        CHAR,
        CHAR_ARRAY
    };

    public class FieldData
    {   
        // Anything added here should be added as well in the CloneFrom method
        public string Name { get; set; }
        public string TypeString { get; set; }
        public FieldDataType Type { get; set; }
        public string Description { get; set; }

        public void CloneFrom(FieldData f)
        {
            this.Type = f.Type;
            this.TypeString = f.TypeString;
            this.Name = f.Name;
            this.Description = f.Description;
        }




        private static void ParseItemsIntoList(string items, List<string> target)
        {
            if (items == null || items == "")
                return;

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

            return sb.ToString();
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

