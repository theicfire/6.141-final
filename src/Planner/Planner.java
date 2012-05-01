package Planner;

import java.awt.geom.Point2D;
import java.util.ArrayList;

import Challenge.ConstructionGUI;
import Challenge.ConstructionObject;
import Challenge.Fiducial;
import Challenge.GrandChallengeMap;
import Navigation.PolygonMap;
import Navigation.PolygonObstacle;

public class Planner {

	public static int blockIndex = 0;
	private static ArrayList<Point2D.Double> tmpPoints;
	
	private static void initPoints() {
		if (tmpPoints == null) {
			tmpPoints = new ArrayList<Point2D.Double>();
			tmpPoints.add(new Point2D.Double(.5, 0));
			tmpPoints.add(new Point2D.Double(.5, .5));
			tmpPoints.add(new Point2D.Double(0, .5));
			tmpPoints.add(new Point2D.Double(0, 0));
		}
	}
	
	public static Point2D.Double getCurrentBlockPoint() {
		initPoints();
		return tmpPoints.get(blockIndex);
	}

	public static void nextBlock() {
		initPoints();
		blockIndex += 1;
		blockIndex = blockIndex % tmpPoints.size();
	}
	
	public static Point2D findClosestThing() {
		for (ConstructionObject c : GrandChallengeMap.constructionObjects) {
			c.getPosition();
			// do something here
		}
		for (PolygonObstacle c : GrandChallengeMap.obstacles) {
			// pass
		}
		for (Fiducial f : GrandChallengeMap.fiducials) {
			// pass
		}
		return null;
	}

}
