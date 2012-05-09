package Navigation;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.commons.logging.Log;
import org.ros.message.lab5_msgs.ColorMsg;
import org.ros.message.lab5_msgs.GUIEraseMsg;
import org.ros.message.lab5_msgs.GUISegmentMsg;
import org.ros.message.lab6_msgs.GUIPolyMsg;
import org.ros.message.lab6_msgs.GUIRectMsg;
import org.ros.node.Node;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;

import Challenge.ConstructionGUI;
import Challenge.ConstructionObject;
import Challenge.GrandChallengeMap;
import Controller.Utility;

public class NavigationMain {

	private Publisher<Object> pointPub;
	private Publisher<Object> segPub;
	private Publisher<Object> erasePub;
	private Publisher<Object> rectPub;
	private Publisher<Object> polyPub;
	public CSpace cspace;
	public PolygonObstacle cWorldRect;
	private Node globalNode;
	private Log log;
	private VisibilityGraph vis;

	private GrandChallengeMap map;
//	

	private static ColorMsg COLOR_MSG_RED;
	private static ColorMsg COLOR_MSG_GREEN;
	private static ColorMsg COLOR_MSG_YELLOW;
	private static ColorMsg COLOR_MSG_PINK;
	
	public NavigationMain(Node node) {
		globalNode = node;
		log = node.getLog();
		log.info("~~~~~~~~~~starting");

		log.info("~~~~~~~~~~resolving map file name");
//		ParameterTree paramTree = node.newParameterTree();
//		log.info("~~~~~~~~~~creating paramTree");

		COLOR_MSG_RED = new ColorMsg();
		COLOR_MSG_RED.r = 255;
		COLOR_MSG_RED.g = 0;
		COLOR_MSG_RED.b = 0;
		COLOR_MSG_GREEN = new ColorMsg();
		COLOR_MSG_GREEN.r = 0;
		COLOR_MSG_GREEN.g = 255;
		COLOR_MSG_GREEN.b = 0;
		COLOR_MSG_YELLOW = new ColorMsg();
		COLOR_MSG_YELLOW.r = 255;
		COLOR_MSG_YELLOW.g = 255;
		COLOR_MSG_YELLOW.b = 0;
		COLOR_MSG_PINK = new ColorMsg();
		COLOR_MSG_PINK.r = 255;
		COLOR_MSG_PINK.g = 0;
		COLOR_MSG_PINK.b = 255;
		
		// String mapFileName =
		// paramTree.getString(node.resolveName("~/mapFileName"));

		// TODO: need to fix this hardcoded value
		// String mapFileName =
		// "/home/rss-student/RSS-I-group/lab6/src/global-nav-maze-2011-basic.map";
		// String mapFileName =
		// "/home/rss-student/RSS-I-group/lab6/src/global-nav-maze-2011-med.map";
		// String mapFileName =
		// "/home/rss-student/RSS-I-group/lab6/src/global-nav-maze.map";
		// String mapFileName =
		// "/home/rss-student/RSS-I-group/lab6/src/practice-maze-01.map";
		// String mapFileName =
		// "/home/rss-student/RSS-I-group/lab6/src/practice-maze-02.map";

		map = new GrandChallengeMap();
//		map.makeLogger(node);
		log.info("~~~~~~~~~~parsing map file");
		map = Utility.getChallengeMap();

		erasePub = node.newPublisher("gui/Erase", "lab5_msgs/GUIEraseMsg");
		pointPub = node.newPublisher("gui/Point", "lab5_msgs/GUIPointMsg");
		segPub = node.newPublisher("gui/Segment", "lab5_msgs/GUISegmentMsg");

		log.info("~~~~~~~~~~waiting for gui as a listener");
		rectPub = node.newPublisher("gui/Rect", "lab6_msgs/GUIRectMsg");
		// wait for rectPub to have the gui as a listener
		while (rectPub.getNumberOfSubscribers() <= 0) {
			// log.info("rectPub.getNumberOfSubscribers(): " +
			// rectPub.getNumberOfSubscribers());
			// rectPub = node.newPublisher("gui/Rect", "lab6_msgs/GUIRectMsg");
		}

		polyPub = node.newPublisher("gui/Poly", "lab6_msgs/GUIPolyMsg");
		while (polyPub.getNumberOfSubscribers() <= 0) {
			// log.info("polyPub.getNumberOfSubscribers(): " +
			// polyPub.getNumberOfSubscribers());
			// polyPub = node.newPublisher("gui/Poly", "lab6_msgs/GUIPolyMsg");
		}
		
		displayConfigSpace();

	}

	void displayMap() {

	}

	void testConvexHull() {

	}

	void displayConfigSpace() {

		log.info("~~~~~~~~~~call display config");

		erasePub.publish(new GUIEraseMsg());

//		GUIRectMsg rectMsg = new GUIRectMsg();
//		log.info("world rect is " + map.getWorldRect());
//		ConstructionGUI.fillRectMsg(rectMsg, map.getWorldRect(), null, false);
//		rectPub.publish(rectMsg);

		// plot the non config space stuff
		PolygonObstacle[] polygons = map.getPolygonObstacles(); // world
																	// space
																	// obstacles
		// plotObstacles(polyObstacles, "red");

		// get config space
		// Rectangle2D.Double rsRobotRect = new Rectangle2D.Double(
		// -robotSquareSideLength,-robotSquareSideLength,robotSquareSideLength,robotSquareSideLength);

		
//		cspace = new CSpace(polygons, robotSquareSideLength);
		// CSpace cspace = new CSpace(wsPolyObstacles, createOldSquareRobot(),
		// new Point2D.Double(0.0,0.0));
		// Rectangle2D.Double rsRobotRect =
		// this.createOverEstimatedSquareRobot();
		Rectangle2D.Double rsRobotRect = this.createCorrectSquareRobot();
		PolygonObstacle robotPoly = this.rectToPoly(rsRobotRect);
		cspace = new CSpace(polygons, robotPoly, new Point2D.Double(0.0,
				0.0));

		PolygonObstacle[] configObstacles = cspace.getObstacles();
		plotObstacles(configObstacles, COLOR_MSG_YELLOW);

		Rectangle2D.Double wsWorldRect = map.getWorldRect();
		PolygonObstacle polyRect = new PolygonObstacle();
		polyRect.addVertex(new Point2D.Double(wsWorldRect.x, wsWorldRect.y));
		polyRect.addVertex(new Point2D.Double(
				wsWorldRect.x + wsWorldRect.width, wsWorldRect.y));
		polyRect.addVertex(new Point2D.Double(
				wsWorldRect.x + wsWorldRect.width, wsWorldRect.y
						+ wsWorldRect.height));
		polyRect.addVertex(new Point2D.Double(wsWorldRect.x, wsWorldRect.y
				+ wsWorldRect.height));
		polyRect.close();
		
		
		// cWorldRect = CSpace.sumAndConvexHull(polyRect, robotPoly);
		cWorldRect = calcCSpaceWorldRect(wsWorldRect, rsRobotRect);
		
		// plot the cspace rect
		plotObstacle(cWorldRect, COLOR_MSG_PINK);
//		plotObstacles(polygons, COLOR_MSG_GREEN);

		getAndPlotVisGraph(map.robotStart);
		

//		List<Point2D.Double> shortestPath = DijkstraGood.getMyDijkstra(
//				visGraph, ROBOT_START, ROBOT_END, log);
		// List<Point2D.Double> shortestPath = new ArrayList<Point2D.Double>();
		// shortestPath.add(map.getRobotStart());
		// shortestPath.add(new Point2D.Double(.9, 0));
		// shortestPath.add(new Point2D.Double(0, 0));
		// shortestPath.add(new Point2D.Double(.9, 0));
		// shortestPath.add(new Point2D.Double(0, 0));
		// shortestPath.add(new Point2D.Double(.9, 0));
		// shortestPath.add(new Point2D.Double(0, 0));
	}

	double robotSquareSideLength = .1;

	PolygonObstacle createOldSquareRobot() {
		ArrayList<Point2D.Double> testPoints = new ArrayList<Point2D.Double>();
		testPoints.add(new Point2D.Double(0.0, 0.0));
		testPoints.add(new Point2D.Double(-robotSquareSideLength, 0.0));
		testPoints.add(new Point2D.Double(-robotSquareSideLength,
				-robotSquareSideLength));
		testPoints.add(new Point2D.Double(0.0, -robotSquareSideLength));
		return GeomUtils.convexHull(testPoints);
	}

	Rectangle2D.Double createOldSquareRobotRect() {
		return new Rectangle2D.Double(
				-robotSquareSideLength,
				-robotSquareSideLength,
				robotSquareSideLength,
				robotSquareSideLength);
		
	}

	double metersRobotSquareSideLength = 0.377825;
	// double metersRobotSquareSideLength = 0.45;
	double metersToFront = 0.0635;

	// PolygonObstacle createOverEstimatedSquareRobot() {
	// double s = metersRobotSquareSideLength;
	// double f = metersToFront;
	// double right = f + 0.5 * (Math.sqrt(2) * s - s);
	// double left = -(Math.sqrt(2) - right);
	// double top = s * Math.sqrt(2) / 2;
	// double bottom = - top;
	// ArrayList<Point2D.Double> testPoints = new ArrayList<Point2D.Double>();
	// testPoints.add(new Point2D.Double(left, bottom));
	// testPoints.add(new Point2D.Double(right, bottom));
	// testPoints.add(new Point2D.Double(right, top));
	// testPoints.add(new Point2D.Double(left, top));
	// return GeomUtils.convexHull(testPoints);
	// }

	Rectangle2D.Double createOverEstimatedSquareRobot() {
		double s = metersRobotSquareSideLength;
		double f = metersToFront;
		double right = f + 0.5 * s * (Math.sqrt(2) - 1);
		double left = -(s * Math.sqrt(2) - right);
		double top = s * Math.sqrt(2) / 2;
		double bottom = -top;

		double width = s * Math.sqrt(2);
		double height = width;

		log.info("lr" + left + " " + right + " " + top + " " + bottom + " ");
		log.info("wh" + width + " " + height);

		return new Rectangle2D.Double(left, bottom, width, height);
	}

//	double xMetersToFarCorner = 0.314325;
	

	Rectangle2D.Double createCorrectSquareRobot() {
//		double xD = xMetersToFarCorner;
//		double yD = yMetersToFarCorner;
//		double b = Math.sqrt(xD * xD + yD * yD) * 0.9;
//
//		double left = -b;
//		double bottom = -b;
//
//		double width = 2 * b;
//		double height = width;
		double height = .58;
		double yMetersToFarCorner = height / 2; // symetrical
		double xMetersToFarCorner = 0.33;
		double width = 0.57;
		return new Rectangle2D.Double(-xMetersToFarCorner, -yMetersToFarCorner, width, height);
	}

	PolygonObstacle rectToPoly(Rectangle2D.Double rect) {
		double left = rect.x;
		double bottom = rect.y;
		double right = left + rect.width;
		double top = bottom + rect.height;

		log.info("lr" + left + " " + right + " " + top + " " + bottom + " ");

		Point2D.Double botLeft = new Point2D.Double(left, bottom);
		Point2D.Double topLeft = new Point2D.Double(left, top);
		Point2D.Double botRight = new Point2D.Double(right, bottom);
		Point2D.Double topRight = new Point2D.Double(right, top);

		PolygonObstacle poly = new PolygonObstacle();
		poly.addVertex(botLeft);
		poly.addVertex(botRight);
		poly.addVertex(topRight);
		poly.addVertex(topLeft);
		poly.close();
		return poly;
	}

	PolygonObstacle calcCSpaceWorldRect(Rectangle2D.Double wsRect,
			Rectangle2D.Double rsGridAlignedRectangularRobot // robot space
																// grid-aligned
																// rectangular
																// robot with
																// ref point 0,0
	) {
		// aliasing for convenience
		Rectangle2D.Double robot = rsGridAlignedRectangularRobot;

		Point2D.Double robotRefPoint = new Point2D.Double(0.0, 0.0);

		Rectangle2D.Double reflectedRobot = new Rectangle2D.Double(
				robotRefPoint.x - (robot.x - robotRefPoint.x) - robot.width,
				robotRefPoint.y - (robot.y - robotRefPoint.y) - robot.height,
				robot.width, robot.height);

		Point2D.Double cSpaceBottomLeft = new Point2D.Double(wsRect.x
				+ reflectedRobot.x + reflectedRobot.width, wsRect.y
				+ reflectedRobot.y + reflectedRobot.height);

		Point2D.Double cSpaceBottomRight = new Point2D.Double(wsRect.x
				+ wsRect.width, cSpaceBottomLeft.y);

		Point2D.Double cSpaceTopLeft = new Point2D.Double(cSpaceBottomLeft.x,
				wsRect.y + wsRect.height);

		Point2D.Double cSpaceTopRight = new Point2D.Double(wsRect.x
				+ wsRect.width, wsRect.y + wsRect.height);

		PolygonObstacle result = new PolygonObstacle();
		result.addVertex(cSpaceBottomLeft);
		result.addVertex(cSpaceBottomRight);
		result.addVertex(cSpaceTopRight);
		result.addVertex(cSpaceTopLeft);
		result.close();

		return result;
	}

	public void plotObstacles(PolygonObstacle[] polys, ColorMsg color) {
		for (PolygonObstacle obstacle : polys) {
			plotObstacle(obstacle, color);
		}
	}

	public void plotObstacle(PolygonObstacle obstacle, ColorMsg color) {
		GUIPolyMsg polyMsg = new GUIPolyMsg();
		polyMsg = new GUIPolyMsg();
		ConstructionGUI.fillPolyMsg(polyMsg, obstacle, ConstructionGUI.makeRandomColor());
		polyMsg.c = color;
		polyPub.publish(polyMsg);
	}

	public void updateMap() {
		log.info("!!!!!update map!!");
		log.info("!!!!!update map!!");
		log.info("!!!!!update map!!");
		log.info("!!!!!update map!!");
		// Color color = new Color(0, 255, 0);
		// ConstructionGUI.addRect(10.0, 10.0, 10.0, 10.0, true, color);

		while (rectPub.getNumberOfSubscribers() == 0) {
			; // wait for rectPub to have the gui as a listener
		}

		GUIRectMsg gpm = new GUIRectMsg();
		gpm.x = 5;
		gpm.y = 5;
		gpm.height = 5;
		gpm.width = 5;
		gpm.filled = 1;
		ColorMsg colorMsg = new ColorMsg();
		colorMsg.r = 0;
		colorMsg.g = 0;
		colorMsg.b = 0;
		gpm.c = colorMsg;
		rectPub.publish(gpm);

	}
	
	public Map<Point2D.Double, List<Point2D.Double>> getAndPlotVisGraph(Point2D.Double start) {
		vis = new VisibilityGraph(start,
				map.robotGoal, map.getConstructionObjects(), cWorldRect, cspace, globalNode);

		// plot the visibility graph
		Map<Point2D.Double, List<Point2D.Double>> visGraph = vis.getGraph();

		Iterator<Entry<Point2D.Double, List<Point2D.Double>>> it = visGraph
				.entrySet().iterator();
		while (it.hasNext()) {
			Map.Entry<Point2D.Double, List<Point2D.Double>> pairs = it.next();
			Point2D.Double point1 = pairs.getKey();
			for (Point2D.Double point2 : pairs.getValue()) {

				// map the line on the graph
				GUISegmentMsg segMsg = new GUISegmentMsg();
				segMsg.startX = point1.x;
				segMsg.startY = point1.y;
				segMsg.endX = point2.x;
				segMsg.endY = point2.y;
				segPub.publish(segMsg);
			}
			// it.remove(); // avoids a ConcurrentModificationException
		}
		return visGraph;
	}
	


//	public Point2D.Double pickNewPoint() {
//		int n = (int) (Math.random() * visGraph.keySet().size());
//		return (Point2D.Double) visGraph.keySet().toArray()[n];
//	}
	
}
