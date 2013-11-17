using System;
using System.IO;
using System.Text;
using System.Collections.Generic;

namespace MavLinkObjectGenerator
{
    public class CSharpGenerator: GenericGenerator
    {

        // __ Config consts ______________________________________________


        public const string Namespace = "MavLinkNet";

        private ProtocolData mProtocolData;
        private TextWriter mWriter;

        /// <summary>
        /// Generates the C# code for the given protocol data.
        /// </summary>
        /// <param name="data">Protocol data to generate code from.</param>
        /// <param name="w">The stream to write the code to.</param>
        /// <remarks>This method is non-reentrant (i.e. not thread safe)</remarks>
        public override void Write(ProtocolData data, TextWriter w)
        {
            mProtocolData = data;
            mWriter = w;

            WriteHeader();
            WriteEnums();
            WriteClasses();
            WriteFooter();
        }


        // __ Code generators global __________________________________________


        private void WriteHeader()
        {
            WL("using System;");
            WL("using System.IO;");
            WL();
            WL("namespace {0}", Namespace);
            WL("{");
            WL();
        }
       
        private void WriteEnums()
        {
            foreach (EnumData e in mProtocolData.Enumerations.Values)
            {
                if (e.Description != null)
                {
                    WL("    /// <summary>");
                    WL("    /// {0}", GetSanitizedComment(e.Description));
                    WL("    /// </summary>");
                }

                WL("    public enum {0} {{ {1} }};", GetEnumName(e.Name), GetEnumItems(e));
                WL();                
            }

        }

        private void WriteClasses()
        {
            foreach (MessageData m in mProtocolData.Messages.Values)
            {
                WriteClassHeader(m);
                WriteProperties(m);
                WriteSerialize(m);
                WriteDeserialize(m);
                WriteToString(m);
                WritePrivateFields(m);
                WriteClassFooter(m);
            }
        }

        private void WriteFooter()
        {
            WL("}");
        }


        // __ Code generators class ___________________________________________


        private void WriteClassHeader(MessageData m)
        {
            WL();
            WL("    // ___________________________________________________________________________________");
            WL();
            WL();

            if (m.Description != null)
            {
                WL("    /// <summary>");
                WL("    /// {0}", GetSanitizedComment(m.Description));
                WL("    /// </summary>");
            }

            WL("    public class Uas{0}: UasMessage", Utils.GetPascalStyleString(m.Name));
            WL("    {");
        }


        private void WriteProperties(MessageData m)
        {
            foreach (FieldData f in m.Fields)
            {
                if (f.Description != null)
                {
                    WL("        /// <summary>");
                    WL("        /// {0}", GetSanitizedComment(f.Description));
                    WL("        /// </summary>");
                }

                WL("        public {0}{1} {2} {{", GetCSharpType(f), GetArrayModifier(f, false), GetFieldName(f));
                WL("            get {{ return {0}; }}", GetPrivateFieldName(f));
                WL("            set {{ {0} = value; NotifyUpdated(); }}", GetPrivateFieldName(f));
                WL("        }");
                WL();
            }
        }



        //private static void WriteConstructor(TextWriter w, ProtocolData obj)
        //{
        //    WL(w, "        public {0}()", obj.Name);
        //    WL(w, "        {");
        //    WL(w, "            IsSingleInstance = {0};", (obj.IsSingleInstInt == 1) ? "true" : "false");
        //    WL(w, "            ObjectId = 0x{0:x8};", Hasher.CalculateId(obj));
        //    WL(w, "        }");
        //    WL(w);
        //}


        private void WriteSerialize(MessageData m)
        {
            WL("        internal override void SerializeBody(BinaryWriter s)");
            WL("        {");

            foreach (FieldData f in m.Fields)
            {
                if (f.NumElements <= 1)
                {
                    WL("            s.Write({0}{1});", GetSerializeTypeCast(f), GetPrivateFieldName(f));
                }
                else
                {
                    for (int i = 0; i < f.NumElements; ++i)
                    {
                        WL("            s.Write({0}{1}[{2}]); ",
                           GetSerializeTypeCast(f), GetPrivateFieldName(f), i);
                    }
                }
            }

            WL("        }");
            WL();
        }

        private void WriteDeserialize(MessageData m)
        {
            WL("        internal override void DeserializeBody(BinaryReader s)");
            WL("        {");

            foreach (FieldData f in m.Fields)
            {
                int numElements = f.NumElements;

                if (numElements <= 1)
                {
                    WL("            this.{0} = {1}s.{2}();",
                       GetPrivateFieldName(f), GetEnumTypeCast(f), GetReadOperation(f));
                }
                else
                {
                    for (int i = 0; i < numElements; ++i)
                    {
                        WL("            this.{0}[{1}] = {2}s.{3}();",
                           GetPrivateFieldName(f), i, GetEnumTypeCast(f), GetReadOperation(f));
                    }
                }
            }

            WL("        }");
            WL();
        }

        private void WriteToString(MessageData m)
        {
            WL("        public override string ToString()");
            WL("        {");
            WL("            System.Text.StringBuilder sb = new System.Text.StringBuilder();");
            WL();
            WL("            sb.Append(\"{0} \\n\");", Utils.GetPascalStyleString(m.Name));

            foreach (FieldData f in m.Fields)
            {
                if (f.NumElements == 1)
                {
                    WL("            sb.AppendFormat(\"    {0}: {{0}}\\n\", {1});", GetFieldName(f), GetPrivateFieldName(f));
                }
                else
                {
                    WL("            sb.Append(\"    {0}\\n\");", GetFieldName(f));
                    for (int i = 0; i < f.NumElements; ++i)
                    {
                        //string elemName = (f.ElementNames.Count == f.NumElements) ? f.ElementNames[i] : "";
                        WL("            sb.AppendFormat(\"        [{1}]: {{0}}\\n\", {2}[{1}]);", GetFieldName(f), i, GetPrivateFieldName(f));
                    }
                }
            }

            WL();
            WL("            return sb.ToString();");
            WL("        }");
            WL();
        }

        private void WritePrivateFields(MessageData m)
        {
            foreach (FieldData f in m.Fields)
            {
                WL("        private {0}{1} {2}{3};",
                   GetCSharpType(f), GetArrayModifier(f, false),
                   GetPrivateFieldName(f), GetDefaultValue(f));
            }
        }

        private void WriteClassFooter(MessageData m)
        {
            WL("    }");
            WL();
        }


        // __ Helpers _____________________________________________________________


        private static string GetEnumName(string enumName)
        {
            return Utils.GetPascalStyleString(enumName);
        }

        private static string GetEnumItems(EnumData en)
        {
            List<string> escapedEnum = new List<string>();

            foreach (EnumEntry entry in en.Entries)
            {
                escapedEnum.Add(string.Format("\r\n\r\n        /// <summary> {0} </summary>\r\n        {1} = {2}",
                    GetSanitizedComment(entry.Description),
                    Utils.GetPascalStyleString(GetStrippedEnumName(en.Name, entry.Name)),
                    entry.Value));
            }

            return GetCommaSeparatedValues(escapedEnum, "");
        }

        private static string GetStrippedEnumName(string enumName, string entryName)
        {
            if (!entryName.StartsWith(enumName)) return entryName;

            return entryName.Substring(enumName.Length + 1);
        }

        private static string GetCommaSeparatedValues(List<string> list, string suffix)
        {
            StringBuilder result = new StringBuilder();
            bool isFirst = true;

            foreach (String s in list)
            {
                if (isFirst)
                    isFirst = false;
                else
                    result.Append(", ");

                result.Append(s + suffix);
            }

            return result.ToString();
        }

        private static string GetDefaultValue(FieldData f)
        {
            if (f.NumElements > 1)
            {
                // Array value
                return string.Format(" = new {0}[{1}]", GetCSharpType(f), f.NumElements);
            }

            return "";
        }

        //private static string GetFormattedDefaultValue(ProtocolData obj, FieldData f, int index)
        //{
        //    if (f.IsEnum)
        //    {
        //        return string.Format("{0}.{1}", GetEnumName(obj, f, false), FieldData.GetEscapedItemName(f.DefaultValues[index]));
        //    }
        //    else
        //    {
        //        return string.Format("{0}{1}", f.DefaultValues[index], GetFieldTypeSuffix(f));
        //    }
        //}

        //private static string GetBracketedString(string s)
        //{
        //    return string.Format("{{ {0} }}", s);
        //}

        //private static string GetDefaultValuesList(ProtocolData obj, FieldData f)
        //{

        //    // Case 0: No default values: just return empty.

        //    if (f.DefaultValues.Count == 0) return "";

        //    // Case 1: there is a default value for every item

        //    if (f.DefaultValues.Count == f.NumElements)
        //    {
        //        if (f.IsEnum)
        //            return GetBracketedString(GetEnumCommaSeparatedValues(GetEnumName(obj, f, false), f.DefaultValues));
        //        else
        //            return GetBracketedString(GetCommaSeparatedValues(f.DefaultValues, GetFieldTypeSuffix(f)));
        //    }

        //    return "";
        //}

        //private static string GetFieldTypeSuffix(FieldData f)
        //{
        //    switch (f.Type)
        //    {
        //        case FieldDataType.FLOAT32:
        //            return "f";
        //        default:
        //            return "";
        //    }
        //}

        //private static string GetEnumCommaSeparatedValues(string enumName, List<string> list)
        //{
        //    List<string> result = new List<string>();

        //    for (int i = 0; i < list.Count; ++i)
        //    {
        //        result.Add(string.Format("{0}.{1}", enumName, FieldData.GetEscapedItemName(list[i])));
        //    }

        //    return GetCommaSeparatedValues(result, "");
        //}

        private static string GetFieldName(FieldData f)
        {
            return Utils.GetPascalStyleString(f.Name);
        }

        private static string GetArrayModifier(FieldData f, bool withNumberOfElements)
        {
            int numElements = f.NumElements;

            if (numElements <= 1) return "";

            return string.Format("[{0}]", (withNumberOfElements ? numElements.ToString() : ""));
        }

        private static string GetCSharpType(FieldData f)
        {
            if (f.IsEnum) return GetEnumName(f.EnumType);

            return GetBaseCSharpType(f.Type);
        }

        private static string GetBaseCSharpType(FieldDataType t)
        {
            switch (t)
            {
                case FieldDataType.FLOAT32: return "float";
                case FieldDataType.INT8: return "SByte";
                case FieldDataType.UINT8: return "byte";
                case FieldDataType.INT16: return "Int16";
                case FieldDataType.UINT16: return "UInt16";
                case FieldDataType.INT32: return "Int32";
                case FieldDataType.UINT32: return "UInt32";
                case FieldDataType.INT64: return "Int64";
                case FieldDataType.UINT64: return "UInt64";
                case FieldDataType.CHAR: return "char";
                default:
                    return "!!!!";
            }
        }

        private static string GetSerializeTypeCast(FieldData f)
        {
            if (!f.IsEnum) return "";

            // Field is enum, use the declared type
            return string.Format("({0})", 
                GetBaseCSharpType(XmlParser.GetFieldTypeFromString(f.TypeString)));
        }

        private static string GetEnumTypeCast(FieldData f)
        {
            if (!f.IsEnum)
                return "";

            return String.Format("({0})", GetEnumName(f.EnumType));
        }

        private static string GetReadOperation(FieldData f)
        {
            if (f.IsEnum)
            {
                return GetBaseReadOperation(XmlParser.GetFieldTypeFromString(f.TypeString));
            }

            return GetBaseReadOperation(f.Type);
        }

        private static string GetBaseReadOperation(FieldDataType t)
        {
            switch (t)
            {
                case FieldDataType.FLOAT32: return "ReadSingle";
                case FieldDataType.INT8: return "ReadSByte";
                case FieldDataType.UINT8: return "ReadByte";
                case FieldDataType.INT16: return "ReadInt16";
                case FieldDataType.UINT16: return "ReadUInt16";
                case FieldDataType.INT32: return "ReadInt32";
                case FieldDataType.UINT32: return "ReadUInt32";
                case FieldDataType.INT64: return "ReadInt64";
                case FieldDataType.UINT64: return "ReadUInt64";
                case FieldDataType.CHAR: return "ReadChar";

                default:
                    Console.WriteLine("ERROR: Unknown uavType: " + t);
                    return "UNKNOWN_UAS_TYPE";
            }
        }

        private static string GetPrivateFieldName(FieldData f)
        {
            return string.Format("m{0}", Utils.GetPascalStyleString(f.Name));
        }

        private static string GetSanitizedComment(string comment)
        {
            return comment.Replace('\n', ' ').Replace('\r', ' ');
        }

        // __ Output __________________________________________________________


        private void WL()
        {
            WL(mWriter);
        }

        private void WL(string s, params object[] args)
        {
            WL(mWriter, s, args);
        }

        internal static void WL(TextWriter w)
        {
            w.WriteLine();
        }

        internal static void WL(TextWriter w, string s, params object[] args)
        {
            if (args.Length == 0)
                w.WriteLine(s);
            else
                w.WriteLine(string.Format(s, args));
        }

    }
}

