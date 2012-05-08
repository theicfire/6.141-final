package VisualServoSolution;

import java.awt.Point;
import java.awt.geom.Point2D;
import java.util.ArrayList;

import Controller.Utility;

import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_core.CvMat;


public class VisualLocalization {

	// discretizes between start and end
	// adds the start
	// does not add end
	public static ArrayList<Point2D.Double> discretizeLineSegment(Point2D.Double start,
			Point2D.Double end, double granularity) {
		ArrayList<Point2D.Double> result =
			new ArrayList<Point2D.Double>();

		double xDiff = end.x-start.x;
		double yDiff = end.y-start.y;

		Point2D.Double scaledDir =
			new Point2D.Double(xDiff,yDiff);
		double dist = Math.sqrt(xDiff*xDiff + yDiff*yDiff);
		double scale = granularity/dist;
		scaledDir.x *= scale;
		scaledDir.y *= scale;

		int numTimes = (int) (dist/granularity);
		for (int i = 0; i < numTimes; ++i) {
			double x = i*scaledDir.x + start.x;
			double y = i*scaledDir.y + start.y;
			result.add(new Point2D.Double(x,y));
		}

		int resultSize = result.size();
		if (resultSize == 0) {
			result.add(new Point2D.Double(start.x,start.y));
		} else {
			int idxLast = resultSize - 1;
			Point2D.Double last = result.get(idxLast);
			if (last.equals(end)) {
				result.remove(idxLast);
			}
		}
		return result;
	}
	
	// returns
	// one point per column
	// points are in homogeneous coordinates
	static CvMat getFloorPointsLocalSpaceFromPoints2D(
			ArrayList<Point2D.Double> srcPoints,
			CvMat pixelToFloorHomography) {
		int numPoints = srcPoints.size();
		CvMat interpolatedPoints = CvMat.create(3,numPoints,opencv_core.CV_32FC1);
		int i = 0;
		for (Point2D.Double pt: srcPoints) {
			interpolatedPoints.put(0,i,pt.x);
			interpolatedPoints.put(1,i,pt.y);
			interpolatedPoints.put(2,i,1.0);
			++i;
		}
		opencv_core.cvMatMul(pixelToFloorHomography,
				interpolatedPoints,interpolatedPoints);
		return interpolatedPoints;
	}

	// returns
	// one point per column
	// points are in homogeneous coordinates
	static CvMat getFloorPointsLocalSpaceFromPoints(
			ArrayList<Point> srcPoints,
			CvMat pixelToFloorHomography) {
		int numPoints = srcPoints.size();
		CvMat interpolatedPoints = CvMat.create(3,numPoints,opencv_core.CV_32FC1);
		int i = 0;
		for (Point pt: srcPoints) {
			interpolatedPoints.put(0,i,(double)pt.x);
			interpolatedPoints.put(1,i,(double)pt.y);
			interpolatedPoints.put(2,i,1.0);
			++i;
		}
		opencv_core.cvMatMul(pixelToFloorHomography,
				interpolatedPoints,interpolatedPoints);
		return interpolatedPoints;
	}

	static ArrayList<Point2D.Double> getInterpolatedPointsFromContour(
			ArrayList<Point> contour,
//			CvMat pixelToFloorHomography,
			double slopeThresh) {
//		CvMat ptfh = pixelToFloorHomography;
		
		ArrayList<Point2D.Double> interpolatedPoints =
			new ArrayList<Point2D.Double>();
		
		int numPoints = contour.size();
		Point start = contour.get(0);
		for (int i = 0; i < numPoints; ++i) {
			Point end = contour.get((i+1)%numPoints);
			
			// call the 

			start = end;
		}

		return interpolatedPoints;
	}
	
	static ArrayList<Point2D.Double> pointsFromLine(Point start, Point end, double slopeThresh) {
		ArrayList<Point2D.Double> interpolatedPoints =
				new ArrayList<Point2D.Double>();
		int xDiff = end.x-start.x;
		int yDiff = end.y-start.y;
		double slope = ((double)yDiff)/xDiff;
		if (!Double.isInfinite(slope)
				&& !Double.isNaN(slope)
				&& Math.abs(slope) < slopeThresh) {
			// for each pixel between start(inclusive) and end(exclusive)
			int increment = 1;
//			int numTimes = end.x-start.x; // 
			int numTimes = Math.abs(end.x - start.x);
//			if ( end.x < start.x) {
//				increment = -1;
//				numTimes = start.x-end.x;
//			}
//			
			int x = start.x;
			for (int j = 0; j < numTimes; ++j) {
				double y = start.y + j*((double)slope);
				interpolatedPoints.add(new Point2D.Double(x,y));
				x += increment;
			}
		}
		return interpolatedPoints;
	}
	
	// given a list of contours
	// one of these contours is the best
	// that is, it contains the most points below the horizontal threshold
	static ArrayList<Point> findBestContour(ArrayList<ArrayList<Point>> contours,
			int horizontalYThresh, double fractionThresh) {
		// pick the best contour
		// i.e. the one that has the most distance below a certain threshold
		double bestDistSum = 0;
		ArrayList<Point> bestContour = null;
		for (ArrayList<Point> c: contours) {
			double largestDist = 0;

			int numPoints = c.size();
			if (numPoints > 3) {
				double distSum = 0;
				int numBelowThresh = 0;

				Point pt = c.get(0);
				for (int i = 0; i < numPoints; ++i) {
					Point next = c.get((i+1)%numPoints);
					if (pt.y > horizontalYThresh) {
						++numBelowThresh;
						if (next.y > horizontalYThresh) {
							double diffX = next.x-pt.x;
							double diffY = next.y-pt.y;
							double dist = Math.sqrt(diffX*diffX+diffY*diffY);
							if (dist > largestDist) {
								largestDist = dist;
							}
							distSum += dist;
						}
					}
					pt = next;
				}
				// subtract the largestDistSqr because the sum right now is
				// equal to the square perimeter of a polygon with the same
				// points as the contour. we want the curve not the polygon
				distSum -= largestDist;
				double fractionBelowThresh =
					((double)numBelowThresh)/numPoints;
				if (fractionBelowThresh > fractionThresh) {
					if (distSum > bestDistSum) {
						bestDistSum = distSum;
						bestContour = c;
					}
				}
			}
		}
		
		return bestContour;
	}
	
	
	
	
	// computes the current World position
	// given a correspondence between two rays
//	public static Utility.Pose computeWorldPosition(
//			Point2D.Double worldRayOrigin,
//			Point2D.Double unitWorldRayDir,
//			Point2D.Double localRayOrigin,
//			Point2D.Double unitLocalRayDir) {
//
//		// solve (Ax=b) for x
//		CvMat A = CvMat.create(4, 4);
//		CvMat b = CvMat.create(4, 1);
//		CvMat x = CvMat.create(4, 1);
//		A.put(0,0,unitLocalRayDir.x);
//		A.put(0,1,-unitLocalRayDir.y);
//		A.put(0,2,0);
//		A.put(0,3,0);
//
//		A.put(1,0,unitLocalRayDir.y);
//		A.put(1,1,unitLocalRayDir.x);
//		A.put(1,2,0);
//		A.put(1,3,0);
//
//		A.put(2,0,localRayOrigin.x);
//		A.put(2,1,-localRayOrigin.y);
//		A.put(2,2,1);
//		A.put(2,3,0);
//
//		A.put(3,0,localRayOrigin.y);
//		A.put(3,1,localRayOrigin.x);
//		A.put(3,2,0);
//		A.put(3,3,1);
//
//		b.put(0,0,unitWorldRayDir.x);
//		b.put(1,0,unitWorldRayDir.y);
//		b.put(2,0,worldRayOrigin.x);
//		b.put(3,0,worldRayOrigin.y);
//
//		int result =
//			opencv_core.cvSolve(A,b,x,opencv_core.CV_LU);
//		if (result != 1) {
//			System.out.println("Error solving using Gaussian Elimination.");
//			result =
//				opencv_core.cvSolve(A,b,x,opencv_core.CV_SVD);
//			if (result != 1) {
//				System.out.println("Error solving Singular Value Decomposition.");
//				return null;
//			}
//		}
//
//		double poseX = x.get(2,0);
//		double poseY = x.get(3,0);
//		double poseTheta =
//			Math.atan2(x.get(1,0),x.get(0,0));
//		return new Utility.Pose(poseX,poseY,poseTheta);
//	}

}
