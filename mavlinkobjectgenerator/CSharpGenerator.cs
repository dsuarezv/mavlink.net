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
            WL("    public class Uas{0}: UasMessage", Utils.GetPascalStyleString(m.Name));
            WL("    {");
        }


        private void WriteProperties(MessageData m)
        {
            foreach (FieldData f in m.Fields)
            {
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


        //private static void WriteSerialize(TextWriter w, ProtocolData obj)
        //{
        //    WL(w, "        internal override void SerializeBody(BinaryWriter s)");
        //    WL(w, "        {");

        //    foreach (FieldData f in obj.Fields)
        //    {
        //        int numElements = f.NumElements;

        //        if (numElements <= 1)
        //        {
        //            WL(w, "            s.Write({0}{1});", GetSerializeTypeCast(obj, f), GetPrivateFieldName(f));
        //        }
        //        else
        //        {
        //            for (int i = 0; i < numElements; ++i)
        //            {
        //                WL(w, "            s.Write({0}{1}[{2}]);  // {3}", 
        //                   GetSerializeTypeCast(obj, f), GetPrivateFieldName(f), i, GetElementNameAt(f, i));
        //            }
        //        }

        //    }

        //    WL(w, "        }\n");
        //    WL(w);
        //}

        //private static void WriteDeserialize(TextWriter w, ProtocolData obj)
        //{
        //    WL(w, "        internal override void DeserializeBody(BinaryReader stream)", obj.Name);
        //    WL(w, "        {");

        //    foreach (FieldData f in obj.Fields)
        //    {
        //        int numElements = f.NumElements;

        //        if (numElements <= 1)
        //        {
        //            WL(w, "            this.{0} = {1}stream.{2}();", 
        //               GetPrivateFieldName(f), GetEnumTypeCast(obj, f), GetReadOperation(f));
        //        }
        //        else
        //        {
        //            for (int i = 0; i < numElements; ++i)
        //            {
        //                WL(w, "            this.{0}[{1}] = {2}stream.{3}();  // {4}", 
        //                   GetPrivateFieldName(f), i, GetEnumTypeCast(obj, f), GetReadOperation(f), GetElementNameAt(f, i));
        //            }
        //        }
        //    }

        //    WL(w, "        }\n");
        //    WL(w);
        //}

        //private static void WriteToString(TextWriter w, ProtocolData obj)
        //{
        //    WL(w, "        public override string ToString()");
        //    WL(w, "        {");
        //    WL(w, "            System.Text.StringBuilder sb = new System.Text.StringBuilder();");
        //    WL(w);
        //    WL(w, "            sb.Append(\"{0} \\n\");", obj.Name);

        //    foreach (FieldData f in obj.Fields)
        //    {
        //        if (f.NumElements == 1)
        //        {
        //            WL(w, "            sb.AppendFormat(\"    {0}: {{0}} {1}\\n\", {0});", f.Name, f.Units);
        //        }
        //        else
        //        {
        //            WL(w, "            sb.Append(\"    {0}\\n\");", f.Name);
        //            for (int i = 0; i < f.NumElements; ++i)
        //            {
        //                string elemName = (f.ElementNames.Count == f.NumElements) ? f.ElementNames[i] : "";
        //                WL(w, "            sb.AppendFormat(\"        {1}: {{0}} {3}\\n\", {0}[{2}]);", f.Name, elemName, i, f.Units);
        //            }
        //        }
        //    }

        //    WL(w);
        //    WL(w, "            return sb.ToString();");
        //    WL(w, "        }");
        //    WL(w);
        //}

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
                escapedEnum.Add(Utils.GetPascalStyleString(
                    GetStrippedEnumName(en.Name, entry.Name)));
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



        //private static string GetElementNameAt(FieldData f, int index)
        //{
        //    if (index < f.ElementNames.Count) return f.ElementNames[index];

        //    return "NO_ELEMENT_NAME";
        //}


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
            switch (f.Type)
            {
                case FieldDataType.FLOAT32: return "float";
                case FieldDataType.INT8: return "SByte";
                case FieldDataType.UINT8: return "byte";
                case FieldDataType.INT16: return "Int16";
                case FieldDataType.UINT16: return "UInt16";
                //case FieldDataType.enum: return GetEnumName(obj, f, false);
                case FieldDataType.INT32: return "Int32";
                case FieldDataType.UINT32: return "UInt32";
                case FieldDataType.INT64: return "Int64"; 
                case FieldDataType.UINT64: return "UInt64";
                case FieldDataType.CHAR: return "char";
                default:
                    Console.WriteLine("ERROR: Unknown field type: " + f.TypeString);
                    return "!!!!";
            }
        }

        //private static string GetSerializeTypeCast(ProtocolData obj, FieldData f)
        //{
        //    if (!f.IsEnum)
        //        return "";

        //    return "(byte)";
        //}

        //private static string GetEnumTypeCast(ProtocolData obj, FieldData f)
        //{
        //    if (!f.IsEnum)
        //        return "";

        //    return String.Format("({0})", GetEnumName(obj, f, false));
        //}

        //private static string GetReadOperation(FieldData f)
        //{
        //    switch (f.TypeString)
        //    {
        //        case "float":  return "ReadSingle";
        //        case "int8":   return "ReadSByte";
        //        case "uint8":  return "ReadByte";
        //        case "int16":  return "ReadInt16";
        //        case "uint16": return "ReadUInt16";
        //        case "int32":  return "ReadInt32";
        //        case "uint32": return "ReadUInt32";
        //        case "enum":   return "ReadByte";
        //        default:
        //            Console.WriteLine("ERROR: Unknown uavType: " + f.TypeString);
        //            return "UNKNOWN_UAV_TYPE";
        //    }
        //}

        private static string GetPrivateFieldName(FieldData f)
        {
            return string.Format("m{0}", Utils.GetPascalStyleString(f.Name));
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

