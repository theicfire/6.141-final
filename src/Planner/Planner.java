package Planner;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Arrays;

import org.apache.commons.logging.Log;

import Challenge.ConstructionGUI;
import Challenge.ConstructionObject;
import Challenge.Fiducial;
import Challenge.GrandChallengeMap;
import Controller.Utility;
import Localization.ICP;
import Localization.Localizer;
import Navigation.NavigationMain;
import Navigation.PolygonMap;
import Navigation.PolygonObstacle;
import Navigation.VisibilityGraph;

public class Planner {

	public int blockIndex = 0;
	private int counter = 0;
	private ArrayList<Point2D.Double> tmpPoints;
	private ArrayList<ConstructionObject> blocks;
	private ConstructionObject currentBlock;
	private Localizer odom;
	private Log log;
	NavigationMain navigationMain;
	public int blocksStored = 0;
	private Point2D.Double visualServoStartPoint;

	public Planner(Localizer ourOdom, Log log, NavigationMain navigationMain) {
		this.log = log;
		this.navigationMain = navigationMain;
		initPoints();
		odom = ourOdom;
		blocks = new ArrayList<ConstructionObject>();

		GrandChallengeMap map = new GrandChallengeMap();
		map = Utility.getChallengeMap();

		for (ConstructionObject c : map.constructionObjects) {
			if (VisibilityGraph.getReachablePoints(navigationMain.cspace.getObstacles(), null,
					c.getPosition(), navigationMain.cWorldRect).size() > 0) {
				ConstructionObject ob = new ConstructionObject();
				ob.setPosition(new Point2D.Double(c.getPosition().getX(), c.getPosition().getY()));
				ob.setColor(c.getColor());
				ob.setSize(c.getSize());
				blocks.add(ob);
				log.info("block at " + c.getPosition());
			}
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
			double dist = Utility.getMagnitude(here, b.getPosition());
			if (dist < min) {
				choice = b;
				min = dist;
			}
		}
		if (choice == null) {
			throw new RuntimeException("no reachable blocks left :P");
		}
		currentBlock = choice;
//		log.info("Next block at index" + counter);
//		currentBlock = blocks.get(counter);
	}

	public Point2D.Double getCurrentBlockPosition() {
		log.info("asking for block position; returning" + currentBlock.getPosition());
		log.info("All blocks: " + blocks);
		for (ConstructionObject block : blocks) {
			log.info("a block is: " + block.getPosition());
		}
		return new Point2D.Double(currentBlock.getPosition().getX(), currentBlock.getPosition().getY());
	}

	public void markCurrentBlockDone() {
		blocks.remove(currentBlock);
//		counter = (counter + 1) % blocks.size();
	}

	public void setVisualServoStartPoint(Point2D.Double startPoint) {
		visualServoStartPoint = startPoint;	
	}
	
	public Point2D.Double getVisualServoStartPoint() {
		return visualServoStartPoint;	
	}

}
