package Localization;

import java.awt.geom.GeneralPath;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.logging.Log;

import Controller.Utility;
import Controller.Utility.Pose;
import Navigation.PolygonObstacle;
import VisualServoSolution.VisualLocalization;

public class ICP {

	public static Pose computeOffset(List<Point2D.Double> worldMapPoints,
			List<Point2D.Double> visionPoints, Log log) {

		String s;
		String cmd = "/home/rss-student/ICP/icp";

		String r = null;
		
		try {
			// print(System.getProperty("user.dir"));
			Process proc = Runtime.getRuntime().exec(cmd);

			IOPipe.inp = new BufferedReader(new InputStreamReader(
					proc.getInputStream()));
			IOPipe.out = new BufferedWriter(new OutputStreamWriter(
					proc.getOutputStream()));

			StringBuffer str = new StringBuffer();
			for (Point2D.Double p : worldMapPoints) {
				str.append(p.x + "\n" + p.y + "\n" + "0.0" + "\n" + "OK" + "\n");
			}
			str.append("0\n0\n0\ndone\n");

			for (Point2D.Double p : visionPoints) {
				str.append(p.x + "\n" + p.y + "\n" + "0.0" + "\n" + "OK" + "\n");
			}
			str.append("0\n0\n0\ndone\n");

			log.info("POINT CLOUD RESULT");
			r = IOPipe.pipe(str.toString());
			log.info(r);

			IOPipe.inp.close();
			IOPipe.out.close();
		}
		catch (Exception err) {
			err.printStackTrace();
		}
		if (r == null)
			return null;
		return (new Utility()).new Pose(Double.parseDouble(r.split(",")[0]), 
										Double.parseDouble(r.split(",")[1]),
										Double.parseDouble(r.split(",")[2]));
	}

	public static List<Point2D.Double> discretizeMap(PolygonObstacle[] obstacles) {
		List<Point2D.Double> allPoints = new ArrayList<Point2D.Double>();
		for (PolygonObstacle po : obstacles) {
			Point2D.Double previousP = null;
			for (Point2D.Double p : po.getVertices()) {
				if (previousP != null) {
					// discretize at 5mm
					allPoints.addAll(VisualLocalization.discretizeLineSegment(
							p, previousP, 0.005));
				}
				previousP = p;
			}
		}
		return allPoints;
	}

	// /*
	// * todo: is \theta(N), could be O(n) or better with KD trees
	// */
	// private static Point2D.Double closestPoint(Point2D.Double start,
	// List<Point2D.Double> allPoints) {
	// double min = Double.MAX_SIZE;
	// Point2D.Double closest = null;
	// for (Point2D.Double p : allPoints) {
	// if (Utility.magnitude(p, start) < min) {
	// closest = p;
	// }
	// }
	// return closest;
	// }

	// private static List<Point2D.Double> grindToFinePoints(
	// List<PolygonObstacle> worldMap) {
	// ArrayList<Point2D.Double> r = new ArrayList<Point2D.Double>();
	// for (PolygonObstacle mapPoly : worldMap) {
	// for (Line2D mapLine : mapPoly.getSegments()) {
	//
	// }
	// }
	// }

	public static Pose runIPC(int iterations, List<Line2D> visionSegments) {
		return null;
	}

}
