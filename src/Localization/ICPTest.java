package Localization;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.util.ArrayList;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import Challenge.GrandChallengeMap;
import Controller.Utility;
import Controller.Utility.ConfidencePose;
import Controller.Utility.Pose;
import Navigation.PolygonObstacle;

public class ICPTest {

	public static void main(String[] args) {
		// load the map
		GrandChallengeMap map = Utility.getChallengeMap();
		
		ArrayList<Point2D.Double> visionPoints = new ArrayList<Point2D.Double>();
		visionPoints.add(new Point2D.Double(0, 1.8));
		visionPoints.add(new Point2D.Double(1.4, 0));
		visionPoints.add(new Point2D.Double(1.4, 0.1));
		visionPoints.add(new Point2D.Double(1.4, 0.2));
		visionPoints.add(new Point2D.Double(1.4, 0.3));
		visionPoints.add(new Point2D.Double(1.4, 0.4));
		visionPoints.add(new Point2D.Double(1.4, 0.5));

		Pose start = (new Utility()).new Pose(0.2, 0, 0);
		
		PolygonObstacle po = new PolygonObstacle();
		po.addVertex(new Point2D.Double(5.0, 5.0));
		po.addVertex(new Point2D.Double(0.0, 5.0));
		po.addVertex(new Point2D.Double(5.0, 0.0));
		po.addVertex(new Point2D.Double(0.0, 0.0));
				
		ArrayList<PolygonObstacle> pos = new ArrayList<PolygonObstacle>();
		pos.add(po);
		
		// convert it to points
		Log log = LogFactory.getLog(ICPTest.class);
		ConfidencePose offset = ICP.computeOffset(start, ICP.discretizeMap(map.obstacles), visionPoints, log, "/home/rss-student/points.txt");
//		Pose offset = ICP.computeOffset(start, ICP.discretizeMap(pos), visionPoints, log, "/home/rss-student/points.txt");
		log.info("done done, output " + offset);
	}
}
