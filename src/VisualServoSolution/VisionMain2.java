package VisualServoSolution;

import java.awt.Point;
import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.message.lab5_msgs.GUIEraseMsg;
import org.ros.message.lab5_msgs.GUIPointMsg;
import org.ros.message.rss_msgs.VisionMsg;
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
	Publisher<org.ros.message.sensor_msgs.Image> vidPubThreshold;
	Publisher<org.ros.message.sensor_msgs.Image> vidPubHue;
	Subscriber<org.ros.message.sensor_msgs.Image> rawVidSub;
	private Publisher<Object> erasePub;
	private Publisher<Object> pointPub;
	Log log;
	Localizer odom;

	private ArrayList<Point2D.Double> visionPoints;
	final int TAKE_NUM_AVERAGES = 5;
	int averageCount;
	
	static IplImage imgHsv; // 3 channels
	static IplImage imgThreshold; // 1 channel
	static IplImage imgDetectedEdges; // 1 channel
	static IplImage imgContours; // 1 channel

	CvMemStorage cvStorage;
	private CvMat pixelToFloorH;

	// edge detection parameters
	int edgeThresh = 1;
	static int lowThreshold = 200;
	static int ratio = 5;
	static int kernel_size = 5;

	@Override
	public void onStart(Node node) {
		log = node.getLog();
		this.odom = new Localizer(node, false);
		map = Utility.getChallengeMap();

		erasePub = node.newPublisher("gui/Erase", "lab5_msgs/GUIEraseMsg");
		
		// TODO Auto-generated method stub
		vidPub = node.newPublisher("/rss/blobVideo2", "sensor_msgs/Image");
		vidPubThreshold = node.newPublisher("/rss/blobVideo2B",
				"sensor_msgs/Image");
		vidPubHue = node.newPublisher("/rss/blobVideo2C", "sensor_msgs/Image");

		pointPub = node.newPublisher("gui/Point", "lab5_msgs/GUIPointMsg");

		// image processing initialization
		cvStorage = opencv_core.cvCreateMemStorage(0);

		// homography
		String homographyData = "/home/rss-student/RSS-I-group/Challenge/calibration4.txt";
		ArrayList<HomographySrcDstPoint> homoPoints = HomographySrcDstPoint
				.loadHomographyDataTextFile(homographyData);

		double FEET_TO_METERS = 0.3048;
		for (int i = 0; i < homoPoints.size(); ++i) {
			homoPoints.get(i).dst_x *= FEET_TO_METERS;
			homoPoints.get(i).dst_y *= FEET_TO_METERS;
		}

		pixelToFloorH = Utility.computeHomography(homoPoints);

		Subscriber<org.ros.message.sensor_msgs.Image> rawVidSub = node
				.newSubscriber("rss/video2", "sensor_msgs/Image");
		rawVidSub.addMessageListener(new InterpRawVid());
		rawVidSub = node.newSubscriber("rss/video2", "sensor_msgs/Image");

	}

	public class InterpRawVid implements
			MessageListener<org.ros.message.sensor_msgs.Image> {

		boolean firstMessage = true;
		int counter = 0;

		@Override
		synchronized public void onNewMessage(
				org.ros.message.sensor_msgs.Image src) {
			// log.info("VisionMain2.java: gotmsg," + src.width + " " +
			// src.height);
			counter += 1;
			if (firstMessage) {
				log.info("VisionMain2.java: firstmsg," + src.width + " "
						+ src.height);
				// image processing initialization
				int depth = 8;
				imgHsv = IplImage.create((int) src.width, (int) src.height,
						depth, 3);
				// imgThreshold = opencv_core.cvCreateImage(
				// opencv_core.cvGetSize(srcImg), 8, 1);
				imgThreshold = IplImage.create((int) src.width,
						(int) src.height, depth, 1);
				imgDetectedEdges = IplImage.create((int) src.width,
						(int) src.height, depth, 1);
				imgContours = IplImage.create((int) src.width,
						(int) src.height, depth, 1);
				visionPoints = new ArrayList<Point2D.Double>();
				firstMessage = false;
			}
			if (counter % 2 == 0) {
				processImage(src);
			} 
//				erasePub.publish(new GUIEraseMsg());
		}
	}

	synchronized void processImage(Image src) {
		// TODO Auto-generated method stub
		IplImage rgbIpl = Utility.rgbImgMsgToRgbIplImg(src, log);

		// compute HSV
		opencv_imgproc.cvCvtColor(rgbIpl, imgHsv, opencv_imgproc.CV_BGR2HSV);
		opencv_imgproc.cvSmooth(imgHsv, imgHsv, opencv_imgproc.CV_GAUSSIAN, 3);

		opencv_core.cvSplit(imgHsv, imgThreshold, null, null, null);
		Image dest3;
		dest3 = Utility.monoIplImgToRgbImgMsg(imgThreshold, log);
		vidPubHue.publish(dest3);

		// compute Threshold

		double hueLower = 0;
		double hueUpper = 255;
		double satLower = 0;
		double satUpper = 65;// 65;// 40
		double valLower = 60; // 160
		double valUpper = 255;
		opencv_core.cvInRangeS(imgHsv,
				opencv_core.cvScalar(hueLower, satLower, valLower, 0),
				opencv_core.cvScalar(hueUpper, satUpper, valUpper, 0),
				imgThreshold);

		processThresheldImg(imgThreshold);

		Image dest2;
		dest2 = Utility.monoIplImgToRgbImgMsg(imgThreshold, log);
		vidPubThreshold.publish(dest2);

		// compute Detected Edge
		opencv_imgproc.cvCanny(imgThreshold, imgDetectedEdges, lowThreshold,
				lowThreshold * ratio, kernel_size);

		imgContours = imgDetectedEdges;

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
				contourList, 100, .75);

		double SLOPE_THRESH = 2.0;
		if (bestContour != null) {
			ArrayList<Point2D.Double> interpolated;
			// interpolated = VisualLocalization
			// .getInterpolatedPointsFromContour(bestContour, 1);
			interpolated = new ArrayList<Point2D.Double>();

			Pose bestGuess = odom.getPositionPose();

			Point prev = null;
			for (Point p : bestContour) {
				if (p.y < 120 || prev == null) {
					prev = p;
					continue;
				}

				// if too step, dont add point
				double absSlope = Math.abs(((double) (p.y - prev.y)) / (p.x - prev.x));
				if (absSlope > SLOPE_THRESH) {
					continue;
				}

				interpolated.add(new Point2D.Double(p.x, p.y));
				prev = p;
			}

			Image dest;
			// dest = Utility.monoIplImgToRgbImgMsg(
			// imgDetectedEdges, log);
			drawContourOnImage(rgbIpl, interpolated, 0, 0, 255);
			dest = Utility.rgbIplImgToRgbImgMsg(rgbIpl);
			vidPub.publish(dest);

			if (interpolated.size() > 0) {
				CvMat floorPoints = VisualLocalization
						.getFloorPointsLocalSpace(interpolated,
								this.pixelToFloorH);

				averageCount += 1;

				int numInterpolated = interpolated.size();
				double cos = Math.cos(bestGuess.getTheta());
				double sin = Math.sin(bestGuess.getTheta());

				for (int i = 0; i < numInterpolated; ++i) {
					double x = floorPoints.get(0, i);
					double y = floorPoints.get(1, i);
					double invZ = 1.0 / floorPoints.get(2, i);
					visionPoints.add(new Point2D.Double(x * invZ, y * invZ));
					// plot these points on the GUI

					// log.info("visinonmain2.java, " + bestGuess.getX() +
					// " " +
					// bestGuess.getY());
					// log.info("visinonmain2.java, " + x + " " + y);
//
//					GUIPointMsg pm = new GUIPointMsg();
//					// pm.x = x * invZ;// x*cos - y*sin + bestGuess.getX();
//					// pm.y = y * invZ;// sin*x + y* cos + bestGuess.getY();
//					pm.x = x * invZ * cos - y * invZ * sin + bestGuess.getX();
//					pm.y = x * invZ * sin + y * invZ * cos + bestGuess.getY();
//					pointPub.publish(pm);
				}

				// TODO hack to make sure we're not at 0, 0
				if (averageCount >= TAKE_NUM_AVERAGES
						&& odom.getPosition().x != 0
						&& odom.getPosition().y != 0) {
					ConfidencePose newLocation = ICP.computeCorrectedPosition(
							bestGuess, ICP.discretizeMap(map.obstacles),
							visionPoints, log, "");
					log.info("confidence " + newLocation.getConfidence()
							+ " new pose " + newLocation.getX() + ", "
							+ newLocation.getY() + ", theta "
							+ newLocation.getTheta());
					this.odom.updatePosition(newLocation);
					averageCount = 0;
					visionPoints = new ArrayList<Point2D.Double>();
				}
			}
		} else {
			log.info("VisionMain2.java: no best contour");
		}
	}

	private void processThresheldImg(IplImage imgThreshold) {
		// TODO Auto-generated method stub
		int width = imgThreshold.width();
		int height = imgThreshold.height();
		int widthStep = imgThreshold.widthStep();
		ByteBuffer canvasBP = imgThreshold.getByteBuffer();

		for (int x = width - 1; x >= 0; --x) {
			int y = height - 1;
			int index = y * widthStep + x;
			while (y >= 0 && canvasBP.get(index) != ((byte) 0)) {
				index = (--y) * widthStep + x;
			}
			while (y >= 0) {
				canvasBP.put(index, (byte) 0);
				index = (--y) * widthStep + x;
			}
		}
	}

	private static void drawContourOnImage(IplImage dstImg,
			ArrayList<Point2D.Double> contour, int b, int g, int r) {
		ByteBuffer canvasBP = dstImg.getByteBuffer();
		int widthStep = dstImg.widthStep();
		for (Point2D.Double pt : contour) {
			canvasBP.put((int) (pt.y * widthStep + pt.x * 3 + 0), (byte) b);
			canvasBP.put((int) (pt.y * widthStep + pt.x * 3 + 1), (byte) g);
			canvasBP.put((int) (pt.y * widthStep + pt.x * 3 + 2), (byte) r);
		}
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
