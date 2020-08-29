package Util;
import java.io.*;

public class FileOutPut {

    public static void writeMethod(String fileName, String string)
    {
        try
        {
            BufferedWriter out=new BufferedWriter(new FileWriter(fileName));
            out.write(string);
            out.close();
        } catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    public static void readMethod2(String fileName)
    {
        String line="";
        try
        {
            BufferedReader in=new BufferedReader(new FileReader(fileName));
            line=in.readLine();
            while (line!=null)
            {
                System.out.println(line);
                line=in.readLine();
            }
            in.close();
        } catch (IOException e)
        {
            e.printStackTrace();
        }
    }

}
