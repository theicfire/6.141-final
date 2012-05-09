package VisualServoSolution;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashSet;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.EncoderMsg;
import org.ros.node.Node;
import org.ros.node.topic.Subscriber;

import Challenge.GrandChallengeMap;
import Controller.Utility;
import Controller.Utility.ConfidencePose;
import Controller.Utility.Pose;
import Localization.ICP;

import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_core.CvMat;


public class IcpRunnable implements Runnable {

	private Subscriber<EncoderMsg> encoderSub;

	boolean firstMsgReceived;
	
	long TICKS_BEFORE_LOCALIZE = 30000;

	long initialReadingLeft;
	long initialReadingRight;

	long previousReadingLeft;
	long previousReadingRight;
	
	long accumulatedLeft;
	long accumulatedRight;

	long lastAccumulateReadingLeft;
	long lastAccumulateReadingRight;

	CvMat worldToLocal;
	VisionMain2 parent;

	Log log;
	GrandChallengeMap map;
	
	public IcpRunnable(Node node, VisionMain2 parent) {
		
		this.log = node.getLog();
		this.map = Utility.getChallengeMap();

		this.parent = parent;
		accumulatedLeft = 0;
		accumulatedRight = 0;
		lastAccumulateReadingLeft = 0;
		lastAccumulateReadingRight = 0;

		worldToLocal = opencv_core.cvCreateMat(
				3,3,opencv_core.CV_32FC1);
		
//		subscribe to the encoder;
		encoderSub = node.newSubscriber(
			"/rss/Encoder", "rss_msgs/EncoderMsg");
		encoderSub.addMessageListener(
				new MessageListener<EncoderMsg>() {
			@Override
			public void onNewMessage(EncoderMsg msg) {
				processEncoderMsg(msg);
			}
		});

	}

	synchronized
	void processEncoderMsg(EncoderMsg msg) {
		
		// accumulate the absolute difference;
		if (!firstMsgReceived) {
			initialReadingLeft = msg.left;
			initialReadingRight = msg.right;
			previousReadingLeft = initialReadingLeft;
			previousReadingRight = initialReadingRight;
			firstMsgReceived = true;
			return;
		}

		this.accumulatedLeft += Math.abs(
				msg.left-this.previousReadingLeft);
		this.accumulatedRight += Math.abs(
				msg.right-this.previousReadingRight);

		this.previousReadingLeft = msg.left;
		this.previousReadingRight = msg.right;
	}
	
	@Override
	public void run() {

		// wait for the first message
		while (!firstMsgReceived) {
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		mainLoop();
	}

	void mainLoop() {
		
		ArrayList<Point2D.Double> visionPoints =
			new ArrayList<Point2D.Double>();
		
		while (true) {
			// race condition
			// thus we need to save first
			long savedLeft = accumulatedLeft;
			long savedRight = accumulatedRight;
			long leftDiff = savedLeft - lastAccumulateReadingLeft;
			long rightDiff = savedRight - lastAccumulateReadingRight;

			log.info("leftDiff" + leftDiff + " rightDiff " + rightDiff);
			
			if (leftDiff < TICKS_BEFORE_LOCALIZE
					&& rightDiff < TICKS_BEFORE_LOCALIZE) {
				//TODO: the number of seconds to
				// sleep should depend on the current reading
				try {
					Thread.sleep(20);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				continue;
			}
			
			// take the point clouds
			this.parent.lockPointCloudList.lock();
			ArrayList<HashSet<Point2D.Double>> pointClouds =
				this.parent.pointCloudList;
			this.parent.pointCloudList =
				new ArrayList<HashSet<Point2D.Double>>();
			this.parent.lockPointCloudList.unlock();

			// transform all the points into local space
			
			Pose bestGuess = parent.odom.getPositionPose();
			computeWorldToLocalMatrix(
					bestGuess._x,bestGuess._y,bestGuess._theta);

			int numCols = 0;
			for (HashSet<Point2D.Double> pointCloud: pointClouds) {
				numCols += pointCloud.size();
			}
			
			if (numCols <= 0) {
				log.info("warning: no columns");
				continue;
			}

			CvMat worldPoints = opencv_core.cvCreateMat(
					3,numCols,opencv_core.CV_32FC1);
			
			int numpoints = 0;
			for (HashSet<Point2D.Double> pointCloud: pointClouds) {
				for (Point2D.Double point:pointCloud) {
					worldPoints.put(0,numpoints,point.x);
					worldPoints.put(1,numpoints,point.y);
					worldPoints.put(2,numpoints,1.0);
					++numpoints;
				}
			}
			
			opencv_core.cvMatMul(worldToLocal,worldPoints,worldPoints);
			// no need to divide by third coordinate
			CvMat localPoints = worldPoints;

			for (int i = 0; i < numCols; ++i) {
				double x = localPoints.get(0,i);
				double y = localPoints.get(1,i);
				visionPoints.add(new Point2D.Double(x,y));
			}
			
			ConfidencePose newLocation = ICP.computeCorrectedPosition(
					 bestGuess, ICP.discretizeMap(map.obstacles),
//					bestGuess, pointCloud,
					visionPoints, log,
					"/home/rss-student/ICP/points.txt");
			
			log.info("confidence " + newLocation.getConfidence()
					+ " new pose " + newLocation.getX() + ", "
					+ newLocation.getY() + ", theta "
					+ newLocation.getTheta());
			
//			ConfidencePose newLocation = ICP.computeCorrectedPosition(
//					// bestGuess, ICP.discretizeMap(map.obstacles),
//					bestGuess, pointCloud, visionPoints, log,);
			
			// isaac needs to plug code in here
//			doubel conf = location.getConfidence();
//			
//			if ( conf > convertConfidence) {
//
//				newX = min(newLocation.getX(), )
//			}
			
//			ICP.getConfidence();
			
//			this.odom.updatePosition(newLocation);

			opencv_core.cvReleaseMat(worldPoints);
			
			lastAccumulateReadingLeft = savedLeft;
			lastAccumulateReadingRight = savedRight;
		}
	}


	void computeWorldToLocalMatrix(double translateX,
			double translateY, double theta) {
		double cos = Math.cos(theta);
		double sin = Math.sin(theta);
		this.worldToLocal.put(0,0,cos);
		this.worldToLocal.put(1,0,sin);
		this.worldToLocal.put(2,0,0);
		this.worldToLocal.put(0,1,-sin);
		this.worldToLocal.put(1,1,cos);
		this.worldToLocal.put(2,1,0);
		this.worldToLocal.put(0,2,translateX);
		this.worldToLocal.put(1,2,translateY);
		this.worldToLocal.put(2,2,1);

		opencv_core.cvInvert(worldToLocal,worldToLocal,opencv_core.CV_LU);
	}
	
}
