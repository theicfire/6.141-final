package Navigation;

import java.awt.geom.Point2D;
import java.util.PriorityQueue;

import java.util.Iterator;
import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.commons.logging.Log;

/* Copyright (c) 2012 the authors listed at the following URL, and/or
 the authors of referenced articles or incorporated external code:
 http://en.literateprograms.org/Dijkstra's_algorithm_(Java)?action=history&offset=20081113161332

 Permission is hereby granted, free of charge, to any person obtaining
 a copy of this software and associated documentation files (the
 "Software"), to deal in the Software without restriction, including
 without limitation the rights to use, copy, modify, merge, publish,
 distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so, subject to
 the following conditions:

 The above copyright notice and this permission notice shall be
 included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 Retrieved from: http://en.literateprograms.org/Dijkstra's_algorithm_(Java)?oldid=15444
 */

class Vertex implements Comparable<Vertex> {
	public final String name;
	public final Point2D.Double myPoint;
	public List<Edge> adjacencies = new ArrayList<Edge>();
	public double minDistance = Double.POSITIVE_INFINITY;
	public Vertex previous;

	public Vertex(Point2D.Double p) {
		name = p.toString();
		myPoint = p;
	}

	public String toString() {
		return name;
	}

	public int compareTo(Vertex other) {
		return Double.compare(minDistance, other.minDistance);
	}

}

class Edge {
	public final Vertex target;
	public final double weight;

	public Edge(Vertex argTarget, double argWeight) {
		target = argTarget;
		weight = argWeight;
	}
}

public class DijkstraGood {
	public static void computePaths(Vertex source) {
		source.minDistance = 0.;
		PriorityQueue<Vertex> vertexQueue = new PriorityQueue<Vertex>();
		vertexQueue.add(source);

		while (!vertexQueue.isEmpty()) {
			Vertex u = vertexQueue.poll();

			// Visit each edge exiting u
			for (Edge e : u.adjacencies) {
				Vertex v = e.target;
				double weight = e.weight;
				double distanceThroughU = u.minDistance + weight;
				if (distanceThroughU < v.minDistance) {
					vertexQueue.remove(v);

					v.minDistance = distanceThroughU;
					v.previous = u;
					vertexQueue.add(v);

				}

			}
		}
	}

	public static List<Vertex> getShortestPathTo(Vertex target) {
		List<Vertex> path = new ArrayList<Vertex>();
		for (Vertex vertex = target; vertex != null; vertex = vertex.previous)
			path.add(vertex);

		Collections.reverse(path);
		return path;
	}

	private static Vertex findVertex(List<Vertex> vertices, Point2D.Double p) {
		for (Vertex v : vertices) {
			if (v.myPoint.x == p.x && v.myPoint.y == p.y) {
				return v;
			}
		}
		throw new RuntimeException("no point found for vertex");
	}

	public static List<Point2D.Double> getMyDijkstra(
			Map<Point2D.Double, List<Point2D.Double>> adjacencyMatrix, 
			Point2D.Double start, Point2D.Double end, Log log) {
		
//		log.info("start: " + start.toString());
//		end = new Point2D.Double(1.3595287799835205, 2.2231287956237793);
//		log.info("END: " + end.toString());
		
		Iterator<Entry<Point2D.Double, List<Point2D.Double>>> it = adjacencyMatrix
				.entrySet().iterator();
		List<Vertex> vertices = new ArrayList<Vertex>();
		while (it.hasNext()) {
			Map.Entry<Point2D.Double, List<Point2D.Double>> pairs = it.next();
			Point2D.Double point1 = pairs.getKey();
			vertices.add(new Vertex(point1));
			
//			log.info("added new vertex for " + point1.toString());
		}

		Iterator<Entry<Point2D.Double, List<Point2D.Double>>> pit = adjacencyMatrix
				.entrySet().iterator();
		while (pit.hasNext()) {
			Map.Entry<Point2D.Double, List<Point2D.Double>> pairs = pit.next();
			Point2D.Double point1 = pairs.getKey();
			Vertex v1 = findVertex(vertices, point1);

			for (Point2D.Double point2 : pairs.getValue()) {
				double cost = Math.sqrt(Math.pow(Math.abs(point1.x - point2.x),
						2) + Math.pow(Math.abs(point1.y - point2.y), 2));

				Vertex v2 = findVertex(vertices, point2);
				v1.adjacencies.add(new Edge(v2, cost));
				v2.adjacencies.add(new Edge(v1, cost));
			}
		}

		DijkstraGood.computePaths(findVertex(vertices, start));
//		for (Vertex v : vertices) {
//			List<Edge> edges = v.adjacencies;
//			String s = v + " {";
//			for (Edge e : edges) {
//				s += e.target.toString() + ", ";
//			}
//			log.info(s);
			
//			log.info("Distance to " + v + ": " + v.minDistance);
//			List<Vertex> path = DijkstraGood.getShortestPathTo(v);
//			log.info("Path: " + path);
//		}
		Vertex endV = findVertex(vertices, end);
		List<Vertex> path = DijkstraGood.getShortestPathTo(endV);
		List<Point2D.Double> fin = new ArrayList<Point2D.Double>();
		for (Vertex v : path) {
			fin.add(v.myPoint);
			log.info("FINAL PATH TO " + v.myPoint);
		}
		return fin;
	}

	public static void main(String[] args) {

	}
}
