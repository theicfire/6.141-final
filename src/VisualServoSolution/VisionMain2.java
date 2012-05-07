package VisualServoSolution;

import java.awt.Point;
import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.message.lab5_msgs.GUIPointMsg;
import org.ros.message.sensor_msgs.Image;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import com.googlecode.javacpp.Loader;
import com.googlecode.javacpp.Pointer;
import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_core.CvContour;
import com.googlecode.javacv.cpp.opencv_core.CvMat;
import com.googlecode.javacv.cpp.opencv_core.CvMemStorage;
import com.googlecode.javacv.cpp.opencv_core.CvPoint;
import com.googlecode.javacv.cpp.opencv_core.CvSeq;
import com.googlecode.javacv.cpp.opencv_core.IplImage;
import com.googlecode.javacv.cpp.opencv_imgproc;

import Challenge.ConstructionObject;
import Challenge.GrandChallengeMap;
import Controller.Utility;
import Controller.Utility.ConfidencePose;
import Controller.Utility.Pose;
import Localization.ICP;
import Localization.Localizer;
import Navigation.PolygonObstacle;
import Navigation.VisibilityGraph;
import VisualServoSolution.VisionMain.InterpRawVid;

public class VisionMain2 implements NodeMain {

	Node globalNode;
	GrandChallengeMap map;
	Publisher<org.ros.message.sensor_msgs.Image> vidPub;
	Subscriber<org.ros.message.sensor_msgs.Image> rawVidSub;
	Publisher<Object> pointPub;
	Log log;
	Localizer odom;

	static IplImage imgHsv; // 3 channels
	static IplImage imgThreshold; // 1 channel
	static IplImage imgDetectedEdges; // 1 channel
	static IplImage imgContours; // 1 channel

	CvMemStorage cvStorage;
	CvMat pixelToFloorH;

	// edge detection parameters
	int edgeThresh = 1;
	static int lowThreshold = 200;
	static int ratio = 5;
	static int kernel_size = 5;

	@Override
	public void onStart(Node node) {
		log = node.getLog();
		this.odom = new Localizer(node, true);

		// TODO Auto-generated method stub
		vidPub = node.newPublisher("/rss/blobVideo2", "sensor_msgs/Image");

		rawVidSub = node.newSubscriber("rss/video2", "sensor_msgs/Image");
		rawVidSub.addMessageListener(new InterpRawVid());
		
		pointPub = node.newPublisher("gui/Point", "lab5_msgs/GUIPointMsg");
		
		map = Utility.getChallengeMap();
		
		// image processing initialization
		cvStorage = opencv_core.cvCreateMemStorage(0);

		// homography
		String homographyData = "/home/rss-student/RSS-I-group/Challenge/calibration2.hom";
		ArrayList<HomographySrcDstPoint> homoPoints = HomographySrcDstPoint
				.loadHomographyData(homographyData);
		pixelToFloorH = Utility.computeHomography(homoPoints);
	}

	public class InterpRawVid implements
			MessageListener<org.ros.message.sensor_msgs.Image> {

		boolean firstMessage = true;

		@Override
		public void onNewMessage(org.ros.message.sensor_msgs.Image src) {
			// ANTHONY CODE GOES HERE
			if (firstMessage) {
				// image processing initialization
				imgHsv = IplImage.create((int) src.width, (int) src.height, 1,
						3);
				// imgThreshold = opencv_core.cvCreateImage(
				// opencv_core.cvGetSize(srcImg), 8, 1);
				imgThreshold = IplImage.create((int) src.width,
						(int) src.height, 1, 1);
				imgDetectedEdges = IplImage.create((int) src.width,
						(int) src.height, 1, 1);
				imgContours = IplImage.create((int) src.width,
						(int) src.height, 1, 1);
				firstMessage = false;
			}
			processImage(src);
		}
	}

	void processImage(Image src) {
		// TODO Auto-generated method stub
		IplImage rgbIpl = Utility.rgbImgMsgToRgbIplImg(src, log);

		
		log.info("");
		log.info("");
		log.info("");
		log.info("");
		log.info("");
		log.info(rgbIpl);
		
		// compute HSV
		opencv_imgproc.cvCvtColor(rgbIpl, imgHsv, opencv_imgproc.CV_BGR2HSV);
		// compute Threshold
		
		double hueLower = 0;
		double hueUpper = 255;
		double satLower = 0;
		double satUpper = 40;
		double valLower = 160;
		double valUpper = 255;
		opencv_core.cvInRangeS(imgHsv,
				opencv_core.cvScalar(hueLower, satLower, valLower, 0),
				opencv_core.cvScalar(hueUpper, satUpper, valUpper, 0),
				imgThreshold);
		// compute Detected Edge
		opencv_imgproc.cvCanny(imgThreshold, imgDetectedEdges, lowThreshold,
				lowThreshold * ratio, kernel_size);
		// compute contours
		cvStorage = null;
		if (cvStorage == null) {
			cvStorage = opencv_core.cvCreateMemStorage(0);
		}
		CvSeq first_contour = new CvSeq();
		CvSeq contour = first_contour;
		int header_size = Loader.sizeof(CvContour.class);
		int result;
		int findCountoursresult = opencv_imgproc.cvFindContours(imgContours,
				cvStorage, first_contour, header_size,
				// src_gray,cvStorage,first_contour,header_size,
				opencv_imgproc.CV_RETR_LIST,
				// opencv_imgproc.CV_RETR_TREE,
				opencv_imgproc.CV_CHAIN_APPROX_SIMPLE);

		ArrayList<ArrayList<Point>> contourList = new ArrayList<ArrayList<Point>>();
		while (contour != null && !contour.isNull()) {
			ArrayList<Point> points = new ArrayList<Point>();
			for (int i = 0; i < contour.total(); ++i) {
				Pointer ptrPoint = opencv_core.cvGetSeqElem(contour, i);
				CvPoint point = new CvPoint(ptrPoint);
				points.add(new Point(point.x(), point.y()));
			}
			contourList.add(points);
			contour = contour.h_next();
		}

		ArrayList<Point> bestContour = VisualLocalization.findBestContour(
				contourList, 60, .75);
		ArrayList<Point2D.Double> interpolated = VisualLocalization
				.getInterpolatedPointsFromContour(bestContour, 1);
		CvMat res = VisualLocalization.getFloorPointsLocalSpace(interpolated,
				this.pixelToFloorH);

		Pose bestGuess = odom.getPositionPose();
		int numInterpolated = interpolated.size();
		ArrayList<Point2D.Double> visionPoints =
				new ArrayList<Point2D.Double>(numInterpolated);
		for (int i = 0; i < numInterpolated; ++i) {
			double x = res.get(0,i);
			double y = res.get(1,i);
			double invZ = 1.0/res.get(2,i);
			visionPoints.add(new Point2D.Double(x*invZ,y*invZ));
			// plot these points on the GUI

			double cos = Math.cos(bestGuess.getTheta());
			double sin = Math.sin(bestGuess.getTheta());
			GUIPointMsg pm = new GUIPointMsg();
			pm.x = x*cos - y*sin + bestGuess.getX();
			pm.y = sin*x + y* cos + bestGuess.getY();
			pointPub.publish(pm);
		}

		ConfidencePose offset = ICP.computeOffset(bestGuess, ICP.discretizeMap(map.obstacles), visionPoints, log, null);
		// TODO update via ICP happens here
		log.info("confidence " + offset.getConfidence() + " offset "
				+ offset.getX() + ", " + offset.getY() + ", theta "
				+ offset.getTheta());
	}

	@Override
	public void onShutdown(Node arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public void onShutdownComplete(Node arg0) {
		// TODO Auto-generated method stub

	}

	@Override
	public GraphName getDefaultNodeName() {
		// TODO Auto-generated method stub
		return null;
	}

}
