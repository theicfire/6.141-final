package Localization;

import java.io.BufferedReader;
import java.io.BufferedWriter;

public class IOPipe {

    public static BufferedReader inp;
    public static BufferedWriter out;

    public static void print(String s) {
    System.out.println(s);
    }

    public static String pipe(String msg) {
    String ret;

    try {
        out.write( msg + "\n" );
        out.flush();
        ret = inp.readLine();
        return ret;
    }
    catch (Exception err) {

    }
    return "";
    }

}
