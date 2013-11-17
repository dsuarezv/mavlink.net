using System;
using System.IO;
using System.Xml;
using MavLinkObjectGenerator;

namespace MavLinkGen
{
    class MainClass
    {
        public static int Main(string[] args)
        {
            try
            {
                Configuration c = new Configuration(args);
                c.CheckValid();

                foreach (string file in c.Files)
                {
                    try
                    {
                        new XmlParser(file).Generate(c.OutputFile, c.GetGenerator());
                    }
                    catch (Exception ex)
                    {
                        Console.Error.WriteLine("Error in [{0}]: {1} ", file, ex.Message);
                    }
                }

                return 0;
            }
            catch (Exception ex)
            {
                Console.Error.WriteLine(ex.Message);
                return 1;
            }
        }
    }
}
