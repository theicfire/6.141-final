package Planner;

import java.awt.geom.Point2D;
import java.util.ArrayList;

import org.apache.commons.logging.Log;

import Challenge.ConstructionGUI;
import Challenge.ConstructionObject;
import Challenge.Fiducial;
import Challenge.GrandChallengeMap;
import Controller.Utility;
import Localization.Localizer;
import Navigation.PolygonMap;
import Navigation.PolygonObstacle;

public class Planner {

	public int blockIndex = 0;
	private ArrayList<Point2D.Double> tmpPoints;
	private ArrayList<ConstructionObject> blocks;
	private ConstructionObject currentBlock;
	private Localizer odom;

	public Planner(Localizer ourOdom, Log log) {
		initPoints();
		blocks = new ArrayList<ConstructionObject>();

		String mapFileName = "/home/rss-student/RSS-I-group/Challenge/src/challenge_2012.txt";
		GrandChallengeMap map = new GrandChallengeMap();
		try {
			map = GrandChallengeMap.parseFile(mapFileName);
		} catch (Exception e) {
			throw new RuntimeException(
					"DIE DIE DIE DIE DIE DIE couldn't load map");
		}

		for (ConstructionObject c : map.constructionObjects) {
			blocks.add(c);
			log.info("block at " + c.getPosition());
		}
		this.nextClosestBlock();
		log.info("current block: " + currentBlock.getPosition());

		for (PolygonObstacle c : map.obstacles) {
			// pass
		}
		for (Fiducial f : map.fiducials) {
			// pass
		}
	}

	private void initPoints() {
		tmpPoints = new ArrayList<Point2D.Double>();
		tmpPoints.add(new Point2D.Double(.5, 0));
		tmpPoints.add(new Point2D.Double(.5, .5));
		tmpPoints.add(new Point2D.Double(0, .5));
		tmpPoints.add(new Point2D.Double(0, 0));
	}

	// public Point2D.Double getCurrentBlockPoint() {
	// return tmpPoints.get(blockIndex);
	// }
	//
	// public void nextBlock() {
	// blockIndex += 1;
	// blockIndex = blockIndex % tmpPoints.size();
	// }

	public void nextClosestBlock() {
		Point2D.Double here = odom.getPosition();
		double min = Double.MAX_VALUE;
		ConstructionObject choice = null;
		for (ConstructionObject b : blocks) {
			if (Utility.getMagnitude(here, b.getPosition()) < min) {
				choice = b;
			}
		}
		currentBlock = choice;
	}

	public Point2D.Double getCurrentBlockPosition() {
		return currentBlock.getPosition();
	}

	public void markCurrentBlockDone() {
		blocks.remove(currentBlock);
	}

}
