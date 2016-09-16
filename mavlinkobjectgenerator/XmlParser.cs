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
using System.IO;
using System.Xml;
using System.Collections.Generic;

namespace MavLinkObjectGenerator
{
    public class XmlParser
    {
        public XmlParser(string fileName)
        {
            mSourceFileName = fileName;
        }

        public void Generate(string targetFileName, GenericGenerator generator)
        {
            using (XmlTextReader reader = new XmlTextReader(mSourceFileName))
            {
                using (StreamWriter writer = new StreamWriter(targetFileName))
                {
                    Generate(reader, writer, generator);
                }
            }
        }

        public void Generate(XmlTextReader reader, TextWriter writer, GenericGenerator generator)
        {
            ProtocolData result = new ProtocolData();
            Parse(reader, result);
            generator.Write(result, writer);
        }


        // __ Impl _______________________________________________________


        private void Parse(XmlTextReader reader, ProtocolData result)
        {
            ProtocolObject currentObject = null;
            MessageData currentMsg = null;
            FieldData currentField = null;
            EnumData currentEnum = null;
            EnumEntry currentEntry = null;
            EnumEntryParameter currentParam = null;
            int currentEnumValue = 0;

            while (reader.Read())
            {
                if (reader.IsStartElement())
                {
                    switch (reader.Name)
                    {
                        case "include":
                            ProcessInclude(reader.ReadElementContentAsString(), result);
                            break;
                        case "version":
                            result.Version = reader.ReadElementContentAsInt();
                            break;
                        case "message":
                             // if (currentMsg != null) SortFields(currentMsg);   //<--- if condition deleted 
                            currentMsg = new MessageData();
                            currentObject = currentMsg;
                            currentMsg.Name = reader.GetAttribute("name");
                            currentMsg.Id = GetIntFromString(reader.GetAttribute("id"));
                            if (currentMsg.Id < 256)    // Msg id field is a byte, discard anything beyond 255
                                result.Messages.Add(currentMsg.Name, currentMsg);
                            break;
                        case "description": 
                            currentObject.Description = reader.ReadString();
                            break;
                        case "field":
                            currentField = new FieldData();
                            currentField.Name = reader.GetAttribute("name");
                            currentField.TypeString = reader.GetAttribute("type");
                            currentField.Type = GetFieldTypeFromString(currentField.TypeString);
                            currentField.NumElements = GetFieldTypeNumElements(currentField.TypeString);
                            currentField.Description = reader.ReadElementContentAsString();
                            UpdateEnumFields(result, currentField);
                            currentMsg.Fields.Add(currentField);
                            break;
                        case "enum":
                            currentEnum = GetEnumDataForName(result, reader.GetAttribute("name"));
                            currentObject = currentEnum;
                            result.Enumerations[currentEnum.Name] = currentEnum;
                            currentEnumValue = 0;
                            break;
                        case "entry":
                            currentEntry = new EnumEntry();
                            currentObject = currentEntry;
                            currentEntry.Name = reader.GetAttribute("name");
                            currentEntry.Value = GetIntFromString(reader.GetAttribute("value"), ++currentEnumValue);
                            currentEnumValue = currentEntry.Value;
                            currentEnum.Entries.Add(currentEntry);
                            break;
                        case "param":
                            currentParam = new EnumEntryParameter();
                            currentParam.Index = GetIntFromString(reader.GetAttribute("index"));
                            currentParam.Description = reader.ReadElementContentAsString();
                            currentEntry.Parameters.Add(currentParam);
                            break;
                    }
                }
                 else    //<----- Else condition added to control closing of the current node; *** BEGIN **** 
                { 
                    if (reader.Name == "message") 
                    { 
                        if (currentMsg != null) 
                        { 
                            SortFields(currentMsg); 
                        } 
                    } 

                } //<----- *** END *** 
            }
        }

        private void ProcessInclude(string includeFileName, ProtocolData result)
        {
            string basePath = Path.GetDirectoryName(mSourceFileName);
            string targetFileName = (Path.IsPathRooted(includeFileName)) ? 
                includeFileName : 
                Path.Combine(basePath, includeFileName);

            if (mProcessedIncludes.Contains(targetFileName)) return;

            mProcessedIncludes.Add(targetFileName);

            using (XmlTextReader reader = new XmlTextReader(targetFileName))
            {
                Parse(reader, result);
            }
        }

        private EnumData GetEnumDataForName(ProtocolData data, string name)
        {
            EnumData result;

            if (!data.Enumerations.TryGetValue(name, out result))
            {
                result = new EnumData();
                result.Name = name;
            }

            return result;
        }

        private static int GetIntFromString(string intString, int defaultValue = -1)
        {
            int result;
            if (int.TryParse(intString, out result)) return result;

            return defaultValue;
        }

        public static FieldDataType GetFieldTypeFromString(string t)
        {
            // Take the basic type, remove array qualifier if present

            string basicType = GetBasicFieldTypeFromString(t);

            switch (basicType)
            {
                case "float": return FieldDataType.FLOAT32;
                case "int8_t": return FieldDataType.INT8;
                case "uint8_t": return FieldDataType.UINT8;
                case "int16_t": return FieldDataType.INT16;
                case "uint16_t": return FieldDataType.UINT16;
                case "int32_t": return FieldDataType.INT32;
                case "uint32_t": return FieldDataType.UINT32;
                case "int64_t": return FieldDataType.INT64;
                case "uint64_t": return FieldDataType.UINT64;
                case "char": return FieldDataType.CHAR;
                case "double": return FieldDataType.DOUBLE;
                default:
                    Console.Error.WriteLine("Unknown type: " + t);
                    return FieldDataType.NONE;
            }
        }

        public static string GetBasicFieldTypeFromString(string t)
        {
            string[] tt = t.Split('[', ']');
            
            if (tt.Length == 0) 
                return "";

            string result = tt[0];

            if (result == "uint8_t_mavlink_version") 
                result = "uint8_t";

            return result;
        }

        private static int GetFieldTypeNumElements(string t)
        {
            string[] tt = t.Split('[', ']');
            if (tt.Length > 1) 
                return GetIntFromString(tt[1]);

            return 1;
        }

        private static void SortFields(MessageData obj)
        {
            // Sort by field size first, then by the order the fields already have. 

            List<FieldData> L8 = new List<FieldData>();
            List<FieldData> L4 = new List<FieldData>();
            List<FieldData> L2 = new List<FieldData>();
            List<FieldData> L1 = new List<FieldData>();

            foreach (FieldData f in obj.Fields)
            {
                switch (f.Type)
                {
                    case FieldDataType.CHAR:
                    case FieldDataType.INT8:
                    case FieldDataType.UINT8:
                        L1.Add(f);
                        break;
                    case FieldDataType.INT16:
                    case FieldDataType.UINT16:
                        L2.Add(f);
                        break;
                    case FieldDataType.INT64:
                    case FieldDataType.UINT64:
                    case FieldDataType.DOUBLE:
                        L8.Add(f);
                        break;
                    default: 
                        L4.Add(f);
                        break;
                }
            }

            List<FieldData> result = new List<FieldData>();
            result.AddRange(L8);
            result.AddRange(L4);
            result.AddRange(L2);
            result.AddRange(L1);

            obj.Fields = result;
        }

        private static void UpdateEnumFields(ProtocolData data, FieldData field)
        {
            foreach (string s in data.Enumerations.Keys)
            {
                if (field.Description.IndexOf(s) != -1 && field.Type != FieldDataType.FLOAT32)
                {
                    field.IsEnum = true;
                    field.EnumType = s;
                    return;
                }
            }
        }

        
        private string mSourceFileName;
        private List<string> mProcessedIncludes = new List<string>();
    }
}

