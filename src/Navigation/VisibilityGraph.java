package Navigation;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.logging.Log;
import org.ros.node.Node;

public class VisibilityGraph {
	private Point2D.Double start;
	private Point2D.Double goal;
	private PolygonObstacle cworldRect;
	private CSpace cspace;
	private Log log;
	public VisibilityGraph(Point2D.Double start, Point2D.Double goal,
			PolygonObstacle cworldRect, CSpace cspace, Node node) {
		this.start = start;
		this.goal = goal;
		this.cworldRect = cworldRect;
		this.cspace = cspace;
		this.log = node.getLog();

	}
	
	public Map<Point2D.Double,List<Point2D.Double>> getGraph() {
		log.info("getting graph");
		ArrayList<PolygonObstacle> configObstacles = cspace.getObstacles();
		Map<Point2D.Double,List<Point2D.Double>> ret = new HashMap<Point2D.Double,List<Point2D.Double>>();
		// TODO include start and end
//		PolygonObstacle startPoly = new PolygonObstacle();
//		startPoly.addVertex(start);
//		configObstacles.add(startPoly);
//		
//		PolygonObstacle goalPoly = new PolygonObstacle();
//		startPoly.addVertex(goal);
//		configObstacles.add(goalPoly);
//		
		
		List<Point2D.Double> pointsVisibleToStartPoint = new ArrayList<Point2D.Double>();
		List<Point2D.Double> pointsVisibleToGoalPoint = new ArrayList<Point2D.Double>();
		for (PolygonObstacle poly2 : configObstacles) {			
				for (Point2D.Double point2 : poly2.getVertices()) {
					Line2D edgeStart = new Line2D.Double(start, point2);
					if (isEdgeInsideWorldRect(edgeStart) && !edgeIntersects(edgeStart, configObstacles)) {
						pointsVisibleToStartPoint.add(point2);
					}
				}
				
				if (poly2.contains(goal)) 
					continue;
				
				for (Point2D.Double point2 : poly2.getVertices()) {
					Line2D edgeGoal = new Line2D.Double(goal, point2);
					if (isEdgeInsideWorldRect(edgeGoal) && !edgeIntersects(edgeGoal, configObstacles)) {
						pointsVisibleToGoalPoint.add(point2);
					}
				}
		}
		ret.put(start, pointsVisibleToStartPoint);
		ret.put(goal, pointsVisibleToGoalPoint);

		// CONSTRUCT VISIBILITY GRAPH
		
		// iterate though each point
		for (PolygonObstacle poly1 : configObstacles) {
//			for (Point2D.Double point1 : poly1.getVertices()) {
			List<Double> polygonVertices = poly1.getVertices();
			for (int i = 0; i < poly1.getVertices().size(); i++) {
				Point2D.Double point1 = polygonVertices.get(i);
				List<Point2D.Double> pointsVisibleToPoint1 = new ArrayList<Point2D.Double>();

				// for each point, find all edges that connect this point to others
				for (PolygonObstacle poly2 : configObstacles) {
					
					if (poly2 != poly1) { // we don't want to make edges within the same polygon
						for (Point2D.Double point2 : poly2.getVertices()) {
							Line2D edge = new Line2D.Double(point1, point2);
							
							// make sure this edge does not intersect any polygon edges
							if (edgeOpenSpace(edge, configObstacles, poly1, poly2) && !edgeIntersects(edge, configObstacles)) {
								pointsVisibleToPoint1.add(point2);
							}
						}
					}
					
					// add the edges of the polygon, only if they should be added
					int before = (i-1) % (poly1.getVertices().size());
					int after = (i+1) % (poly1.getVertices().size());
					if (before < 0) { // java does negative modular arithmetic wierdly
						before += poly1.getVertices().size();
					}
					Line2D edge1 = new Line2D.Double(poly1.getVertices().get(before), poly1.getVertices().get(i));
					Line2D edge2 = new Line2D.Double(poly1.getVertices().get(after), poly1.getVertices().get(i));
					if (edgeOpenSpace(edge1, configObstacles, poly1, poly1)) {
						pointsVisibleToPoint1.add(poly1.getVertices().get(before));
					}
					if (edgeOpenSpace(edge2, configObstacles, poly1, poly1)) {
						pointsVisibleToPoint1.add(poly1.getVertices().get(after));
					}
					
				}

				ret.put(point1, pointsVisibleToPoint1);
			}
		}
		
		
		return ret;
	}
	
	/*
	 * See if the given edge is inside other polygons
	 * poly1 corresponds to edge.getP1(), poly2...
	 */
	public boolean edgeOpenSpace(Line2D edge, ArrayList<PolygonObstacle> configObstacles, PolygonObstacle poly1, PolygonObstacle poly2) {
		// neither of the points on the edges should not be in other polygons
		for (PolygonObstacle poly3 : configObstacles) {
			// if a different polygon has this point, forget about this point totally
			if (poly3 != poly1 && poly3.contains(edge.getP1())
					|| poly3 != poly2 && poly3.contains(edge.getP2())) {
				return false;
			}
		}
		
		// make sure the point is inside the rectangle
		return isEdgeInsideWorldRect(edge);
	}
	
	public boolean isEdgeInsideWorldRect(Line2D edge) {
		return cworldRect.contains(edge.getP1()) && cworldRect.contains(edge.getP2());
	}
	
	/*
	 * Tests if a line intersects any polygon
	 */
	public static boolean edgeIntersects(Line2D edge, ArrayList<PolygonObstacle> configObstacles) {
//		log.info("checking intersect");
		double epsilon = .01;
		for (PolygonObstacle poly3 : configObstacles) {
//			log.info("go through poly");
			List<Point2D.Double> verts = poly3.getVertices();
			
			Point2D.Double edgeStart;
			Point2D.Double edgeEnd = verts.get(verts.size()-1);
			int count = 0;

			for (Point2D.Double point3 : poly3.getVertices()) {
				edgeStart = point3;
				Line2D polyEdge = new Line2D.Double(edgeStart, edgeEnd);
				count += 1;
//					log.info("edge is " + polyEdge);
				// if the two lines do not share a point and intersect, this is a bad edge
				if (edge.getP1().distance(edgeStart) > epsilon
					&& edge.getP1().distance(edgeEnd) > epsilon
					&& edge.getP2().distance(edgeStart) > epsilon
					&& edge.getP2().distance(edgeEnd) > epsilon
					&& polyEdge.intersectsLine(edge)) {
//							log.info("intersection");
						return true;
					}
				edgeEnd = point3;
			}
//			log.info("count was" + count);

		}
		return false;
	}
	
	

}
