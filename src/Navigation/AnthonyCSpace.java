//package Navigation;
//
//import java.awt.geom.Point2D;
//import java.awt.geom.Rectangle2D;
//import java.awt.geom.Point2D.Double;
//import java.util.ArrayList;
//import java.util.List;
//
//public class CSpace {
//	// java.util.List<PolygonObstacle> realObstacles, double robotRadius
//	private PolygonObstacle[] worldSpaceObstacles;
//	private double robotRadius;
//	private PolygonObstacle robot; // robot in robotSpace
//	private Point2D.Double robotReferencePoint;
////	Rectangle2D.Double worldRect;
//	private PolygonObstacle[] worldRectEdges;
//	
////	public CSpace(PolygonObstacle[] realObstacles, double robotRadius) {
////		this.worldSpaceObstacles = realObstacles;
////		this.robotRadius = robotRadius;
////		this.robot = null;
////		this.robotReferencePoint = null;
////	}
//
//	public CSpace(PolygonObstacle[] realObstacles,
//			PolygonObstacle robot,
//			Point2D.Double robotReferencePoint,
//			Rectangle2D.Double worldRect) {
//		this.worldSpaceObstacles = realObstacles;
//		this.robotRadius = 0;
//		this.robot = robot;
//
////		this.worldRect =
////			new Rectangle2D.Double(worldRect.x,worldRect.y,
////					worldRect.width,worldRect.height);
//
//		this.worldRectEdges = new PolygonObstacle[4];
//		createWorldRectEdges(worldRect);
//
//		this.robotReferencePoint = new Point2D.Double(robotReferencePoint.x,robotReferencePoint.y);
//	}
//	
//	
//	void createWorldRectEdges(Rectangle2D.Double worldRect) {
//		double x = worldRect.x;
//		double y = worldRect.y;
//		double width = worldRect.width;
//		double height = worldRect.height;
//
//		PolygonObstacle left = new PolygonObstacle();
//		PolygonObstacle right = new PolygonObstacle();
//		PolygonObstacle bot = new PolygonObstacle();
//		PolygonObstacle top = new PolygonObstacle();
//		left.addVertex(x,y);
//		left.addVertex(x,y+height);
//		right.addVertex(x+width,y);
//		right.addVertex(x+width,y+height);
//		bot.addVertex(x,y);
//		bot.addVertex(x+width,y);
//		top.addVertex(x,y+height);
//		top.addVertex(x+width,y+height);
//
//		worldRectEdges[0] = left;
//		worldRectEdges[1] = right;
//		worldRectEdges[2] = bot;
//		worldRectEdges[3] = top;
//	}
//	
//	public PolygonObstacle[] getObstacles() {
//		ArrayList<PolygonObstacle> ret =
//				new ArrayList<PolygonObstacle>();
//
//		PolygonObstacle reflected =
//			reflectPolygonAboutPoint(this.robot, robotReferencePoint);
//		for (PolygonObstacle wsObstacle : worldSpaceObstacles) {
//			ArrayList<Point2D.Double> sum =
//				minkowskiSum(reflected,wsObstacle);
//			ret.add(GeomUtils.convexHull(sum));
//		}
//
//		// now add the cspace imposed by the world rectangle
//		for (PolygonObstacle wsObstacle : worldRectEdges) {
//			ArrayList<Point2D.Double> sum =
//				minkowskiSum(reflected,wsObstacle);
//			ret.add(GeomUtils.convexHull(sum));
//		}
//
//		return ret.toArray(new PolygonObstacle[ret.size()]);
//	}
//
//	PolygonObstacle reflectPolygonAboutPoint(PolygonObstacle poly, Point2D.Double point) {
//		// reflect each of the points
//		ArrayList<Point2D.Double> tempList = new ArrayList<Point2D.Double>();
//		List<Point2D.Double> polyPoints = poly.getVertices();
//		int numPolyPoints = polyPoints.size();
//		for (int i = 0; i < numPolyPoints; ++i) {
//			Point2D.Double polyPoint = polyPoints.get(i);
//			double newX = point.x - (polyPoint.x - point.x);
//			double newY = point.y - (polyPoint.y - point.y);
//			tempList.add(new Point2D.Double(newX, newY));
//		}
//
//		// adding points backwards due to reflection
//		PolygonObstacle reflected = new PolygonObstacle();
//		for (int i = numPolyPoints-1; i >= 0; --i) {
//			Point2D.Double vertex = tempList.get(i);
//			reflected.addVertex(vertex);
//		}
//		reflected.close();
//
//		return reflected;
//	}
//
//	public ArrayList<Point2D.Double> minkowskiSum(PolygonObstacle polyA, PolygonObstacle polyB) {
//		ArrayList<Point2D.Double> newPoints = new ArrayList<Point2D.Double>();
//		List<Double> polyAPoints = polyA.getVertices();
//		List<Double> polyBPoints = polyB.getVertices();
//		int numPointsA = polyAPoints.size();
//		int numPointsB = polyBPoints.size();
//		for (int i = 0; i < numPointsA; ++i) {
//			Point2D.Double p1 = polyAPoints.get(i);
//			for (int j = 0; j < numPointsB; ++j) {
//				Point2D.Double p2 = polyBPoints.get(j);
//				newPoints.add(new Point2D.Double(p1.x+p2.x,p1.y+p2.y));
//			}
//		}
//		return newPoints;
//	}
//
//	/*
//	 * returns the polygon that looks like the robot
//	 */
//	public PolygonObstacle getRobotPoly() {
//		ArrayList<Point2D.Double> testPoints = new ArrayList<Point2D.Double>();
//		testPoints.add(new Point2D.Double(0.0, 0.0));
//		testPoints.add(new Point2D.Double(robotRadius, 0.0));
//		testPoints.add(new Point2D.Double(robotRadius, robotRadius));
//		testPoints.add(new Point2D.Double(0.0, robotRadius));
//		return GeomUtils.convexHull(testPoints);
//	}
//	
//	public static PolygonObstacle sumAndConvexHull(PolygonObstacle poly, PolygonObstacle robot) {
//		PolygonObstacle minkSum = new PolygonObstacle();
//		for (Point2D.Double point : poly.getVertices()) {
//			for (Point2D.Double point2 : robot.getVertices()) {
//				minkSum.addVertex(new Point2D.Double(point.x + point2.x, point.y + point2.y));
//			}
//		}
//		
//		PolygonObstacle conHull = GeomUtils.convexHull(minkSum.getVertices());
//		
//		// TODO make sure part of robot is at 0,0
//		return conHull;
//	}
//}
