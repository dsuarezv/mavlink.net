using System;
using System.IO;
using System.Xml;
using System.Collections.Generic;

namespace UavObjectGenerator
{
    public class XmlParser
    {
        public XmlParser(string fileName)
        {
            mSourceFileName = fileName;
        }

        public void Generate(string targetFileName)
        {
            using (XmlTextReader reader = new XmlTextReader(mSourceFileName))
            {
                using (StreamWriter writer = new StreamWriter(targetFileName))
                {
                    Generate(reader, writer);
                }
            }
        }

        public static void Generate(XmlTextReader reader, TextWriter writer)
        {
            ObjectData data = GetObjectFromXml(reader);

            CSharpGenerator.Write(writer, data);
        }


        // __ Impl _______________________________________________________


        private static ObjectData GetObjectFromXml(XmlTextReader reader)
        {
            ObjectData currentObject = null;
            FieldData currentField = null;

            while (reader.Read())
            {
                if (reader.IsStartElement())
                {
                    switch (reader.Name)
                    {
                        case "object":
                            currentObject = new ObjectData();    
                            currentObject.Name = reader.GetAttribute("name");
                            currentObject.IsSettingsInt = GetIntFromBoolString(reader.GetAttribute("settings"));
                            currentObject.IsSingleInstInt = GetIntFromBoolString(reader.GetAttribute("singleinstance"));
                            break;
                        case "description": 
                            currentObject.Description = reader.ReadString();                            
                            break;
                        case "field":
                            currentField = new FieldData();
                            currentField.Name = reader.GetAttribute("name");
                            currentObject.FieldsIndexedByName.Add(currentField.Name, currentField);

                            if (IsClone(reader))
                            {
                                currentField.CloneFrom(currentObject.FieldsIndexedByName[reader.GetAttribute("cloneof")]);
                            }
                            else
                            {
                                currentField.TypeString = reader.GetAttribute("type");
                                currentField.Type = GetFieldTypeFromString(currentField.TypeString);
                                currentField.Elements = reader.GetAttribute("elements");
                                currentField.Units = reader.GetAttribute("units");
                                currentField.ParseElementNamesFromAttribute(reader.GetAttribute("elementnames"));
                                currentField.ParseOptionsFromAttribute(reader.GetAttribute("options"));
                                currentField.ParseDefaultValuesFromAttribute(reader.GetAttribute("defaultvalue"));
                            }
                            currentObject.Fields.Add(currentField);
                            break;
                        case "option": 
                            currentField.Options.Add(reader.ReadString());
                            break;
                        case "elementname": 
                            currentField.ElementNames.Add(reader.ReadString());
                            break;
                    }
                }
            }

            ExpandDefaultValues(currentObject);
            SortFields(currentObject);

            SummaryGenerator.RegisterObjectId(
                Hasher.CalculateId(currentObject), 
                string.Format("{0}.{1}", CSharpGenerator.Namespace, currentObject.Name));

            return currentObject;
        }

        private static void ExpandDefaultValues(ObjectData obj)
        {
            foreach (FieldData f in obj.Fields)
            {
                f.ExpandDefaultValue();
            }
        }

        private static bool GetBoolFromString(string boolString)
        {
            if (boolString == "true") return true;

            return false;
        }

        private static int GetIntFromBoolString(string boolString)
        {
            if (boolString == "true") return 1;
            if (boolString == "false") return 0;

            return -1;
        }

        private static FieldDataType GetFieldTypeFromString(string t)
        {
            // Needed for hash calculation;
            switch (t)
            {
                case "float": return FieldDataType.FLOAT32;
                case "int8": return FieldDataType.INT8;
                case "uint8": return FieldDataType.UINT8;
                case "int16": return FieldDataType.INT16;
                case "uint16": return FieldDataType.UINT16;
                case "enum": return FieldDataType.ENUM;
                case "int32": return FieldDataType.INT32;
                case "uint32": return FieldDataType.UINT32;
                default: return FieldDataType.INT32;
            }
        }

        private static void SortFields(ObjectData obj)
        {
            // Sort by field size first, then by the order the fields already have. 

            /*
            "int8" << "int16" << "int32" << "uint8" << "uint16" << "uint32" << "float" << "enum";
            int(1) << int(2)  << int(4)  << int(1)  << int(2)   << int(4)   << int(4)  << int(1);
            */

            List<FieldData> L4 = new List<FieldData>();
            List<FieldData> L2 = new List<FieldData>();
            List<FieldData> L1 = new List<FieldData>();

            foreach (FieldData f in obj.Fields)
            {
                switch (f.Type)
                {
                    case FieldDataType.ENUM:
                    case FieldDataType.INT8:
                    case FieldDataType.UINT8:
                        L1.Add(f);
                        break;
                    case FieldDataType.INT16:
                    case FieldDataType.UINT16:
                        L2.Add(f);
                        break;
                    default: 
                        L4.Add(f);
                        break;
                }
            }

            List<FieldData> result = new List<FieldData>();
            result.AddRange(L4);
            result.AddRange(L2);
            result.AddRange(L1);

            obj.Fields = result;
        }

        private static bool IsClone(XmlReader reader)
        {
            string cloneOf = reader.GetAttribute("cloneof");

            return (cloneOf != null && cloneOf != "");
        }
        
        private string mSourceFileName;
    }
}

