package VisualServoSolution;

import java.awt.Point;
import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Hashtable;
import java.util.List;
import java.util.Map.Entry;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.Random;

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
import Controller.Utility.Pair;
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
	final int TAKE_NUM_AVERAGES = 50;
	int averageCount;
	
	static IplImage imgHsv; // 3 channels
	static IplImage imgThreshold; // 1 channel
	static IplImage imgDetectedEdges; // 1 channel
	static IplImage imgContours; // 1 channel
	static IplImage rgbIpl; // 3 channels

	CvMemStorage cvStorage;
	private CvMat pixelToFloorH;

	ArrayList<HashSet<Point2D.Double>> pointCloudList;
	final Lock lockPointCloudList = new ReentrantLock();

	// exponentially weighted moving mean and variances
	// of the floor points
	// s(n) = sample(n)
	// mean(n) = (1-alpha) * mean(n-1) + alpha*s(n)
	// var(n) = (1-alpha) * (var(n-1) + alpha * (s(n)-mean(n-1))^2)
	double ewmaWeightX = 1.0/8;
	double ewmaWeightY = 1.0/8;
	double VAR_THRESHOLD_X = 0.1;//0.5;
	double VAR_THRESHOLD_Y = 0.1;//0.5;
	double ewmaFloorX[];
	double ewmaFloorY[];
	double ewmvFloorX[];
	double ewmvFloorY[];
	int pixelYWhereFloorMeetsWall[];
	boolean pixelYIsValid[];

	// y increases from top to bottom
	int HORIZONTAL_Y_THRESHOLD = 120; // pixels with y value less than this are not considered
	int SLOPE_THRESHOLD = 2;
	CvMat pixelPointsWhereFloorMeetsWall;  // remember to initialize the third coordinate to 1.0
	CvMat floorPoints;

	// edge detection parameters
	int edgeThresh = 1;
	static int lowThreshold = 200;
	static int ratio = 5;
	static int kernel_size = 5;

	private Hashtable<ArrayList<Point2D.Double>, ArrayList<Point2D.Double>> obstacleToNormals;

	// for ICP jittering
	Random rGen = new Random();
	
	@Override
	public void onStart(Node node) {
		log = node.getLog();
		
		map = Utility.getChallengeMap();
		this.odom = new Localizer(node, (new Utility()).new Pose(map.robotStart.x, map.robotStart.y, 0));
		
		obstacleToNormals =
				new Hashtable<ArrayList<Point2D.Double>,
				ArrayList<Point2D.Double>>();
		buildObstacleToNormalsMapping();

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

		pointCloudList = new ArrayList<HashSet<Point2D.Double>>();
//		IcpRunnable icpRunnable = new IcpRunnable(node);
//		Thread icpThread = new Thread(icpRunnable);
//		icpThread.run();
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
//			log.info("visionmain2.java: here");
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

				rgbIpl = IplImage.create((int) src.width,
						(int) src.height, opencv_core.IPL_DEPTH_8U, 3);
				
				
				visionPoints = new ArrayList<Point2D.Double>((int) src.width);
				visionPoints.ensureCapacity((int) src.width);
				
				// initialize matrices here
				ewmaFloorX = new double[(int) src.width];
				ewmaFloorY = new double[(int) src.width];
				ewmvFloorX = new double[(int) src.width];
				ewmvFloorY = new double[(int) src.width];
				
				// pick something high, so you start
				// in a state of disbelief
				double startingVariance = 100;
				for (int i = 0; i < ewmaFloorX.length; ++i) {
					ewmaFloorX[i] = 0;
					ewmaFloorY[i] = 0;
					ewmvFloorX[i] = 100;
					ewmvFloorY[i] = 100;
				}
				pixelYIsValid = new boolean[(int) src.width];
				pixelYWhereFloorMeetsWall = new int[(int) src.width];
				for (int i = 0; i < pixelYWhereFloorMeetsWall.length; ++i) {
					pixelYWhereFloorMeetsWall[i] = 0;
				}
				pixelPointsWhereFloorMeetsWall = CvMat.create(3,
						(int) src.width,opencv_core.CV_32FC1);
				 // remember to initialize the third coordinate to 1.0
				for (int i = 0; i < src.width; ++i) {
					pixelPointsWhereFloorMeetsWall.put(2,i,1.0);
				}
				floorPoints = CvMat.create(3,
						(int) src.width,opencv_core.CV_32FC1);

				firstMessage = false;
			}
			if (counter % 2 == 0) {
				processImage(src);
			} 
//				erasePub.publish(new GUIEraseMsg());
		}
	}

	synchronized void processImage(Image src) {
		Utility.copyFromRgbImgMsgToRgbIplImg(src,rgbIpl);
//		IplImage rgbIpl = Utility.rgbImgMsgToRgbIplImg(src, log);

		// compute HSV
		opencv_imgproc.cvCvtColor(rgbIpl, imgHsv, opencv_imgproc.CV_BGR2HSV);
		opencv_imgproc.cvSmooth(imgHsv, imgHsv, opencv_imgproc.CV_GAUSSIAN, 3);

//		opencv_core.cvSplit(imgHsv, imgThreshold, null, null, null);
//		Image dest3;
//		dest3 = Utility.monoIplImgToRgbImgMsg(imgThreshold, log);
//		vidPubHue.publish(dest3);

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
/*
		// compute Detected Edge
		opencv_imgproc.cvCanny(imgThreshold, imgDetectedEdges, lowThreshold,
				lowThreshold * ratio, kernel_size);

		imgContours = imgDetectedEdges;

		// compute contours
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
*/
		
		preparePixelPointsForHomographicTransform();
		doHomographicTransform();
		updateMeansAndVariances();
		
		Image dest;
		ArrayList<Double> validPoints = new ArrayList<Double>();
		for (int i = 0; i < pixelYWhereFloorMeetsWall.length; ++i) {
			
			if (this.pixelYIsValid[i]
					&& ewmvFloorX[i] < VAR_THRESHOLD_X
					&& ewmvFloorY[i] < VAR_THRESHOLD_Y) {
				validPoints.add(new Point2D.Double(
						i,pixelYWhereFloorMeetsWall[i]));
			}
		}
		// dest = Utility.monoIplImgToRgbImgMsg(
		// imgDetectedEdges, log);
		drawContourOnImage(rgbIpl, validPoints , 0, 0, 255);
		dest = Utility.rgbIplImgToRgbImgMsg(rgbIpl);
		vidPubHue.publish(dest);

		Pose bestGuess = odom.getPositionPose();
//		addJitterToPose(bestGuess);

		double cos = Math.cos(bestGuess.getTheta());
		double sin = Math.sin(bestGuess.getTheta());

		for (int i = 0; i < ewmaFloorX.length; ++i) {
			
			if (this.pixelYIsValid[i]
					&& ewmvFloorX[i] < VAR_THRESHOLD_X
					&& ewmvFloorY[i] < VAR_THRESHOLD_Y) {
				visionPoints.add(new Point2D.Double(
						ewmaFloorX[i],ewmaFloorY[i]));
				
				GUIPointMsg pm = new GUIPointMsg();
				// pm.x = x * invZ;// x*cos - y*sin + bestGuess.getX();
				// pm.y = y * invZ;// sin*x + y* cos + bestGuess.getY();
				double x = ewmaFloorX[i];
				double y = ewmaFloorY[i];
				pm.x = x * cos - y * sin + bestGuess.getX();
				pm.y = x * sin + y * cos + bestGuess.getY();
				//pointPub.publish(pm);
			}
		}
		
		if (visionPoints.size() > 0) {
			Utility.Pair<ArrayList<Point2D.Double>, ArrayList<Point2D.Double>> lineStartAndLineEnds = calcFacingEdges(new Point2D.Double(
					bestGuess.getX(), bestGuess.getY()));
			HashSet<Point2D.Double> pointCloud = discretizeLines(
					lineStartAndLineEnds.first, lineStartAndLineEnds.second);

			averageCount += 1;
			// TODO hack to make sure we're not at 0, 0
			if (averageCount >= TAKE_NUM_AVERAGES && odom.getPosition().x != 0 && odom.getPosition().y != 0) {			
//				lockPointCloudList.lock();
//				this.pointCloudList.add(visionPoints);
//				lockPointCloudList.unlock();
				
				ConfidencePose newLocation = ICP.computeCorrectedPosition(
						// bestGuess, ICP.discretizeMap(map.obstacles),
						bestGuess, pointCloud, visionPoints, log,
						"/home/rss-student/ICP/points.txt");
				log.info("confidence " + newLocation.getConfidence()
						+ " new pose " + newLocation.getX() + ", "
						+ newLocation.getY() + ", theta "
						+ newLocation.getTheta());
//				this.odom.updatePosition(newLocation);
				averageCount = 0;
				visionPoints.clear();// = new ArrayList<Point2D.Double>();
			}
		} else {
			log.info("VisionMain2.java: no contours");
		}


//		double SLOPE_THRESH = 2.0;
//		if (bestContour != null) {
//			ArrayList<Point2D.Double> interpolated;
//			// interpolated = VisualLocalization
//			// .getInterpolatedPointsFromContour(bestContour, 1);
//			interpolated = new ArrayList<Point2D.Double>();
//
//			Pose bestGuess = odom.getPositionPose();
//
//			Point prev = null;
//			for (Point p : bestContour) {
//				if (p.y < 120 || prev == null) {
//					prev = p;
//					continue;
//				}
//
//				// if too step, dont add point
//				double absSlope = Math.abs(((double) (p.y - prev.y)) / (p.x - prev.x));
//				if (absSlope > SLOPE_THRESH) {
//					continue;
//				}
//
//				interpolated.add(new Point2D.Double(p.x, p.y));
//				prev = p;
//			}
//
//			Image dest;
//			// dest = Utility.monoIplImgToRgbImgMsg(
//			// imgDetectedEdges, log);
//			drawContourOnImage(rgbIpl, interpolated, 0, 0, 255);
//			dest = Utility.rgbIplImgToRgbImgMsg(rgbIpl);
//			vidPub.publish(dest);
//
//			if (interpolated.size() > 0) {
//				CvMat floorPoints = VisualLocalization
//						.getFloorPointsLocalSpaceFromPoints2D(interpolated,
//								this.pixelToFloorH);
//
//				averageCount += 1;
//
//				int numInterpolated = interpolated.size();
//				double cos = Math.cos(bestGuess.getTheta());
//				double sin = Math.sin(bestGuess.getTheta());
//
//				for (int i = 0; i < numInterpolated; ++i) {
//					double x = floorPoints.get(0, i);
//					double y = floorPoints.get(1, i);
//					double invZ = 1.0 / floorPoints.get(2, i);
//					visionPoints.add(new Point2D.Double(x * invZ, y * invZ));
//					// plot these points on the GUI
//
//					// log.info("visinonmain2.java, " + bestGuess.getX() +
//					// " " +
//					// bestGuess.getY());
//					// log.info("visinonmain2.java, " + x + " " + y);
////
////					GUIPointMsg pm = new GUIPointMsg();
////					// pm.x = x * invZ;// x*cos - y*sin + bestGuess.getX();
////					// pm.y = y * invZ;// sin*x + y* cos + bestGuess.getY();
////					pm.x = x * invZ * cos - y * invZ * sin + bestGuess.getX();
////					pm.y = x * invZ * sin + y * invZ * cos + bestGuess.getY();
////					pointPub.publish(pm);
//				}
//				// TODO hack to make sure we're not at 0, 0
//				if (averageCount >= TAKE_NUM_AVERAGES
//						&& odom.getPosition().x != 0
//						&& odom.getPosition().y != 0) {
//					ConfidencePose newLocation = ICP.computeCorrectedPosition(
//							bestGuess, ICP.discretizeMap(map.obstacles),
//							visionPoints, log, "");
//					log.info("confidence " + newLocation.getConfidence()
//							+ " new pose " + newLocation.getX() + ", "
//							+ newLocation.getY() + ", theta "
//							+ newLocation.getTheta());
//					this.odom.updatePosition(newLocation);
//					averageCount = 0;
//					visionPoints = new ArrayList<Point2D.Double>();
//				}
//			}
//		} else {
//			log.info("VisionMain2.java: no best contour");
//		}
	}

	private void addJitterToPose(Pose bestGuess, double maxJitterDistance, double maxJitterTheta) {
		// rejection sampling is fast
		while (true) {
			// pick a point inside circle of radius 1 centered at (0,0)
			double dx = rGen.nextDouble() * 2 - 1;
			double dy = rGen.nextDouble() * 2 - 1;
			if (dx*dx+dy*dy <= 1.0) {
				bestGuess._x += dx*maxJitterDistance;
				bestGuess._y += dy*maxJitterDistance;
				break;
			}
		}
		
		double dTheta = rGen.nextDouble() * 2 - 1;
		bestGuess._theta += dTheta*maxJitterTheta;
	}

	private HashSet<Point2D.Double> discretizeLines(
			ArrayList<Point2D.Double> lineStarts,
			ArrayList<Point2D.Double> lineEnds) {
		HashSet<Point2D.Double> pointCloud =
				new HashSet<Point2D.Double>();
		int len = lineStarts.size();
		for (int i = 0; i < len; ++i) {
			Point2D.Double start = lineStarts.get(i);
			Point2D.Double end = lineEnds.get(i);
			ArrayList<Point2D.Double> discrLine = VisualLocalization.discretizeLineSegment(
					start, end, 0.005);
			pointCloud.addAll(discrLine);
			pointCloud.add(end);
		}

		return pointCloud;
	}

	void buildObstacleToNormalsMapping() {
		for (PolygonObstacle o: this.map.obstacles) {
			ArrayList<Double> obstacle = new ArrayList<Point2D.Double>();
			o.getVertices(obstacle);
			ArrayList<Double> normals = Utility.calcOutwardNormals(obstacle);
			this.obstacleToNormals.put(obstacle,normals);
		}
		
		// now add the world rect wall
		java.awt.geom.Rectangle2D.Double worldRect = this.map.getWorldRect();
		ArrayList<Double> world = new ArrayList<Double>();
		// add the world in CLOCKWISE order as opposed to CCW order (which is
		// what you do for objects) so that the normals are reversed
		world.add(new Double(worldRect.x, worldRect.y+worldRect.height));
		world.add(new Double(worldRect.x+worldRect.width, worldRect.y+worldRect.height));
		world.add(new Double(worldRect.x+worldRect.width, worldRect.y));
		world.add(new Double(worldRect.x, worldRect.y));
		ArrayList<Double> worldNormals = Utility.calcOutwardNormals(world);
		this.obstacleToNormals.put(world,worldNormals);
	}
	
	Utility.Pair<ArrayList<Double>, ArrayList<Double>> calcFacingEdges(
			Point2D.Double position) {

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
	
	void updateMeansAndVariances() {
		for (int i = 0; i < ewmaFloorX.length; ++i) {
			if (this.pixelYIsValid[i]) {
				double invZ = 1.0/this.floorPoints.get(2, i);
				double y = this.floorPoints.get(1, i)*invZ;
				double x = this.floorPoints.get(0, i)*invZ;
	
				double diffX = x - ewmaFloorX[i];
				double incrX = ewmaWeightX * diffX;
				ewmaFloorX[i] = ewmaFloorX[i] + incrX;
				ewmvFloorX[i] = (1-ewmaWeightX)*(ewmvFloorX[i] + diffX*incrX);
	
				double diffY = y - ewmaFloorY[i];
				double incrY = ewmaWeightY * diffY;
				ewmaFloorY[i] = ewmaFloorY[i] + incrY;
				ewmvFloorY[i] = (1-ewmaWeightY)*(ewmvFloorY[i] + diffY*incrY);
			}
		}
	}

	void doHomographicTransform() {
		opencv_core.cvMatMul(this.pixelToFloorH,
				this.pixelPointsWhereFloorMeetsWall,this.floorPoints);
	}

	// returns the number of pixel points where floor meets wall
	// that are valid in pixelPointsWhereFloorMeetsWall
	// if return N, the first N points of are
	// pixelPointsWhereFloorMeetsWall valid
	void preparePixelPointsForHomographicTransform() {
		
		CvMat ppwfmw = this.pixelPointsWhereFloorMeetsWall;
		int[] pywfmw = this.pixelYWhereFloorMeetsWall;

		//Math.max(Math.min(heightMinusOne,y+1),0);
		int height = imgThreshold.height();
		if (pywfmw[0] >= height || pywfmw[0] <= 0) {
			pywfmw[0] = 0;
			pixelYIsValid[0] = false;
		} else if ((pywfmw[0] > HORIZONTAL_Y_THRESHOLD)
				&& Math.abs(pywfmw[1] - pywfmw[0])
				< SLOPE_THRESHOLD) {
			ppwfmw.put(0,0,0);
			ppwfmw.put(1,0,pywfmw[0]);
			pixelYIsValid[0] = true;
		} else {
			pixelYIsValid[0] = false;
		}
		
		for (int i = 1; i < pywfmw.length - 1; ++i) {
			if (pywfmw[i] >= height || pywfmw[i] <= 0) {
				pywfmw[i] = 0;
				pixelYIsValid[i] = false;
			} else if ((pywfmw[i] > HORIZONTAL_Y_THRESHOLD)
					&& (Math.abs(pywfmw[i] - pywfmw[i - 1]) < SLOPE_THRESHOLD)
					&& (Math.abs(pywfmw[i + 1] - pywfmw[i]) < SLOPE_THRESHOLD)) {
				ppwfmw.put(0, i, i);
				ppwfmw.put(1, i, pywfmw[i]);
				pixelYIsValid[i] = true;
			} else {
				pixelYIsValid[i] = false;
			}
		}
		

		int lenMinusOne = pywfmw.length-1;
		if (pywfmw[lenMinusOne] >= height || pywfmw[lenMinusOne] <= 0) {
			pywfmw[lenMinusOne] = 0;
			pixelYIsValid[lenMinusOne] = false;
		} else if ((pywfmw[lenMinusOne] > HORIZONTAL_Y_THRESHOLD)
				&& Math.abs(
			pywfmw[lenMinusOne] - pywfmw[lenMinusOne-1])
				< SLOPE_THRESHOLD) {
			ppwfmw.put(0,lenMinusOne,lenMinusOne);
			ppwfmw.put(1,lenMinusOne,pywfmw[lenMinusOne]);
			pixelYIsValid[lenMinusOne] = true;
		} else {
			pixelYIsValid[lenMinusOne] = false;
		}

	}

	private void processThresheldImg(IplImage imgThreshold) {
		int width = imgThreshold.width();
		int height = imgThreshold.height();
		int widthStep = imgThreshold.widthStep();
		ByteBuffer canvasBP = imgThreshold.getByteBuffer();

		int heightMinusOne = height - 1;
		for (int x = 0; x < width; ++x) {
			int y = heightMinusOne;
			int index = y * widthStep + x;
			while (y >= 0 && canvasBP.get(index) != ((byte) 0)) {
				index -= widthStep;
				--y;
			}
			pixelYWhereFloorMeetsWall[x] = y+1;//Math.max(Math.min(heightMinusOne,y+1),0);
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
