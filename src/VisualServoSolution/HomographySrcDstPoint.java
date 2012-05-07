package VisualServoSolution;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;

public class HomographySrcDstPoint implements java.io.Serializable {
	public double src_x;
	public double src_y;
	public double dst_x;
	public double dst_y;

	HomographySrcDstPoint(double src_x, double src_y, double dst_x, double dst_y) {
		this.src_x = src_x;
		this.src_y = src_y;
		this.dst_x = dst_x;
		this.dst_y = dst_y;
	}

	public static void saveHomographyData(String filename,
			ArrayList<HomographySrcDstPoint> data) {
		FileOutputStream fileOut;
		try {
			fileOut = new FileOutputStream(filename);
			ObjectOutputStream out = new ObjectOutputStream(fileOut);
			out.writeObject(data);
			out.close();
			fileOut.close();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	public static ArrayList<HomographySrcDstPoint> loadHomographyData(
			String filename) {
		ArrayList<HomographySrcDstPoint> r = new ArrayList<HomographySrcDstPoint>();
		try {
			FileInputStream fileIn = new FileInputStream(filename);
			ObjectInputStream in = new ObjectInputStream(fileIn);
			r = (ArrayList<HomographySrcDstPoint>) in.readObject();
			in.close();
			fileIn.close();
		} catch (ClassNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return r;
	}

	public static ArrayList<HomographySrcDstPoint> loadHomographyDataTextFile(
			String filename) {
		ArrayList<HomographySrcDstPoint> r = new ArrayList<HomographySrcDstPoint>();

		String line;
		FileReader fr = null;
		try {
			fr = new FileReader(filename);
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		BufferedReader br = new BufferedReader(fr);

		try {
			while ((line = br.readLine()) != null) {
				String[] theline = line.split("\t");
				double src_x = Double.parseDouble(theline[0]);
				double src_y = Double.parseDouble(theline[1]);
				double dst_x = Double.parseDouble(theline[2]);
				double dst_y = Double.parseDouble(theline[3]);
				HomographySrcDstPoint p = new HomographySrcDstPoint(src_x, src_y,
						dst_x, dst_y);
				r.add(p);
			}
		} catch (NumberFormatException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		return r;
	}

}
