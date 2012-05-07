package Localization;

import java.io.BufferedReader;
import java.io.BufferedWriter;

import javax.management.RuntimeErrorException;

public class IOPipe {

	public static BufferedReader inp;
	public static BufferedWriter out;

	public static void print(String s) {
		System.out.println(s);
	}

	public static String pipe(String msg) {
		String ret;

		try {
			out.write(msg);
			out.flush();
			ret = inp.readLine();
			return ret;
		} catch (Exception err) {
			throw new RuntimeException("error communicating via IOPipe");
		}
	}

}
