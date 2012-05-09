package VisualServoSolution;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Map.Entry;

import Controller.Utility;

public class PolygonNormalCalculator {

	// assumes that shapes are specified
	// in counter clockwise order
	public static ArrayList<Point2D.Double> calcOutwardNormals(
			ArrayList<Point2D.Double> shape) {

		int numVertices = shape.size();
		ArrayList<Point2D.Double> normals = new ArrayList<Point2D.Double>(
				numVertices);
		Point2D.Double v1 = new Point2D.Double();
		Point2D.Double v2 = new Point2D.Double();
		Point2D.Double p0 = shape.get(0);
		Point2D.Double p1 = shape.get(1);
		for (int i = 0; i < numVertices; ++i) {
			Point2D.Double p2 = shape.get((i + 2) % numVertices);
			v1.x = p0.x - p1.x;
			v1.y = p0.y - p1.y;
			v2.x = p2.x - p1.x;
			v2.y = p2.y - p1.y;

			double angle = Utility.signedAngleBetweenVectorsInStandardPosition(
					v1, v2);

			// dx = x2-x1 and dy=y2-y1,
			// then the normals are (-dy, dx) and (dy, -dx)
			Point2D.Double n = new Point2D.Double(v1.y, -v1.x);
			double dot = n.x * v2.x + n.y * v2.y;
			if (angle < Math.PI) {
				if (dot < 0) {
					n.y = -n.y;
					n.x = -n.x;
				}
			} else {
				if (dot > 0) {
					n.y = -n.y;
					n.x = -n.x;
				}
			}

			double invLen = 1.0 / Math.sqrt(n.x * n.x + n.y * n.y);
			n.x *= invLen;
			n.y *= invLen;

			normals.add(n);

			p0 = p1;
			p1 = p2;
		}

		return normals;
	}

	public static
	Utility.Pair<ArrayList<Double>, ArrayList<Double>>
	calcFacingEdges(
			Point2D.Double position,
			Hashtable<ArrayList<Point2D.Double>, ArrayList<Point2D.Double>> obstacleToNormals) {

		ArrayList<Point2D.Double> lineStartPoints = new ArrayList<Point2D.Double>();
		ArrayList<Point2D.Double> lineEndPoints = new ArrayList<Point2D.Double>();

		Point2D.Double pointToLineEndpoint = new Point2D.Double();

		for (Entry<ArrayList<Double>, ArrayList<Double>> e : obstacleToNormals
				.entrySet()) {
			ArrayList<Double> obstacle = e.getKey();
			ArrayList<Double> normals = e.getValue();

			Point2D.Double lineStartPoint = obstacle.get(0);
			int numNormals = normals.size();
			for (int i = 0; i < normals.size(); ++i) {
				Point2D.Double n = normals.get(i);
				pointToLineEndpoint.x = lineStartPoint.x - position.x;
				pointToLineEndpoint.y = lineStartPoint.y - position.y;
				double dot = n.x * pointToLineEndpoint.x + n.y
						* pointToLineEndpoint.y;
				Point2D.Double lineEndPoint = obstacle
						.get((i + 1) % numNormals);
				if (dot < 0) {
					lineStartPoints.add(lineStartPoint);
					lineEndPoints.add(lineEndPoint);
				}
				lineStartPoint = lineEndPoint;
			}
		}

		return new Utility.Pair<ArrayList<Point2D.Double>, ArrayList<Point2D.Double>>(
				lineStartPoints, lineEndPoints);
	}

}
