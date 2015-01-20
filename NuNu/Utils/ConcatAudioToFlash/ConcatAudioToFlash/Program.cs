using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;



namespace ConcatAudioToFlash
{
    class Program
    {
        static void printInfo()
        {
            System.Console.WriteLine("ConcatAudioToFlash [dirname] [sizeofflash]");
        }

        static void Main(string[] args)
        {
            uint offset = 0;
            byte[] outputBuffer = {};
           
            if (args.Count() == 0)
            {
                printInfo();
                return;
            }
            if (args.Count() == 1)
            {
                outputBuffer = new byte[1024 * 512];
            }

            if (args.Count() == 2)
            {
                outputBuffer = new byte[int.Parse(args[1])];
            }

            System.Console.WriteLine(args[0]);
            string[] files = Directory.GetFiles(args[0],"*.wav");
            for (int i = 0; i < files.Count(); i++)
            {
                FileStream fs = new FileStream(files[i], FileMode.Open);
                fs.Read(outputBuffer, (int)offset,(int)fs.Length);
                System.Console.WriteLine(string.Format("{0},{1},{2},",files[i],offset,fs.Length));
                offset += (uint)fs.Length;
                offset = (uint)((offset + 0x200 - 1) & ~0x1ff); //align to 512
                fs.Close();
            }
            FileStream fsw = new FileStream(args[0] + "\\FlashImage.bin", FileMode.Create);
            fsw.Write(outputBuffer, 0, (int)offset);
            fsw.Close();

        }
    }
}
