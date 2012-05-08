package VisualServoSolution;

import java.awt.Point;
import java.util.ArrayList;


import Controller.Utility;

import com.googlecode.javacpp.Loader;
import com.googlecode.javacpp.Pointer;
import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_imgproc;
import com.googlecode.javacv.cpp.opencv_core.CvContour;
import com.googlecode.javacv.cpp.opencv_core.CvMemStorage;
import com.googlecode.javacv.cpp.opencv_core.CvPoint;
import com.googlecode.javacv.cpp.opencv_core.CvSeq;
import com.googlecode.javacv.cpp.opencv_core.CvSize;
import com.googlecode.javacv.cpp.opencv_core.IplImage;

public class ImageAnalyzer {

	// blurring properties and edge detection
	static int lowThreshold = 200;
	static int ratio = 5;
	static int kernel_size = 5;

	// blob thresholds
	int MIN_BLOB_AREA = 256;
	int MAX_BLOB_AREA = 1024;
	
//	IplImage imgThreshold;
	IplImage imgHSV;
	IplImage imgDetectedEdges;
	
	CvMemStorage cvStorage;

	public ImageAnalyzer(int width, int height) {
		CvSize size = new CvSize();
		size.width(width);
		size.height(height);
		imgHSV = opencv_core.cvCreateImage(
				size,opencv_core.IPL_DEPTH_8U,3);
//		imgThreshold = opencv_core.cvCreateImage(
//				size,opencv_core.IPL_DEPTH_8U,1);
		imgDetectedEdges = opencv_core.cvCreateImage(
				size,opencv_core.IPL_DEPTH_8U,1);
		cvStorage = opencv_core.cvCreateMemStorage(0);
	}
	
//	static void getBlobs2(IplImage srcImgRGB,
//			IplImage imgHSV,
//			int hueLowerBound,
//			int hueUpperBound,
//			int satLowerBound,
//			int valLowerBound,
//			IplImage dstImgBlobMask) {
//		opencv_imgproc.cvCvtColor(srcImgRGB,imgHSV,
//				opencv_imgproc.CV_RGB2HSV);
//		
//		opencv_core.cvInRangeS(imgHSV,
//        		opencv_core.cvScalar(hueLowerBound, satLowerBound, valLowerBound, 0),
//        		opencv_core.cvScalar(hueUpperBound, 255, 255, 0),
//        		dstImgBlobMask);
//	}
	static void getBlobs(IplImage imgHSV,
			int hueLowerBound,
			int hueUpperBound,
			int satLowerBound,
			int valLowerBound,
			IplImage dstImgBlobMask) {
		opencv_core.cvInRangeS(imgHSV,
        		opencv_core.cvScalar(hueLowerBound, satLowerBound, valLowerBound, 0),
        		opencv_core.cvScalar(hueUpperBound, 255, 255, 0),
        		dstImgBlobMask);
	}
	
	// hueLowerBound: [0,255]
	// hueUpperBound: [0,255]
	ArrayList<Utility.Pair<
	ArrayList<Point>, Double>> detectBlobs(
			IplImage srcImgRGB,
			int hueLowerBound,
			int hueUpperBound,
			int minBlobArea,
			int maxBlobArea) {
		opencv_imgproc.cvCvtColor(srcImgRGB,imgHSV,
				opencv_imgproc.CV_RGB2HSV);
		opencv_core.cvInRangeS(imgHSV,
        		opencv_core.cvScalar(hueLowerBound, 128, 0, 0),
        		opencv_core.cvScalar(hueUpperBound, 255, 255, 0),
        		imgDetectedEdges);

        opencv_imgproc.cvCanny(imgDetectedEdges,imgDetectedEdges,
        		lowThreshold,lowThreshold*ratio,kernel_size);
		opencv_imgproc.cvDilate(imgDetectedEdges,
				imgDetectedEdges, null, 1);

		// find contours
		CvSeq first_contour = new CvSeq();
		CvSeq contour = first_contour;
		int header_size = Loader.sizeof(CvContour.class);
		opencv_imgproc.cvFindContours(imgDetectedEdges,
			cvStorage,first_contour,header_size,
			opencv_imgproc.CV_RETR_LIST,
//			opencv_imgproc.CV_RETR_TREE,
			opencv_imgproc.CV_CHAIN_APPROX_SIMPLE);

		ArrayList<Utility.Pair<ArrayList<Point>,Double>> shapes =
			new ArrayList<Utility.Pair<ArrayList<Point>,Double>>();
		while (contour != null && !contour.isNull()) {
			int numPoints = contour.total();
			if (numPoints >= 6) {
				double epsilon = 0.03;
				epsilon *= opencv_imgproc.cvContourPerimeter(contour);
				CvSeq seqPoints2 = contour;
				CvSeq seqPoints = opencv_imgproc.cvApproxPoly(seqPoints2,
						Loader.sizeof(CvContour.class),cvStorage,
						opencv_imgproc.CV_POLY_APPROX_DP, epsilon,
						1);
				int numSeqPts = seqPoints.total();
				if (numSeqPts >= 4) {
					double area = opencv_imgproc.cvContourArea(
							seqPoints, opencv_core.CV_WHOLE_SEQ, 0);
					if (area >= minBlobArea && area <= maxBlobArea) {
						ArrayList<Point> points = new ArrayList<Point>();
						for (int i = 0; i < numSeqPts; ++i) {
							Pointer ptrPoint = opencv_core.cvGetSeqElem(seqPoints,i);
							CvPoint point = new CvPoint(ptrPoint);
			                points.add(new Point(point.x(),point.y()));
						}
						Utility.Pair<ArrayList<Point>,Double> shapeAndAreaPair =
							new Utility.Pair<ArrayList<Point>,Double>(points, area);
						shapes.add(shapeAndAreaPair);
					}
				}
			}
			
			contour = contour.h_next();
		}

		opencv_core.cvClearMemStorage(cvStorage);
		return shapes;
	}
	
	static void detectRectangularPrisms() {
		
	}
	
	
	void interpolation(double x, double y) {
		
	}
	
}
