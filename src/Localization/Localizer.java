package Localization;


import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import Controller.Utility;
import Controller.Utility.Pose;

public class Localizer {

	Utility.Pose odometryOffset;
	private double[] totalTicks;
	private double[] odomXY;
	private double odomTheta;
	
	Subscriber<OdometryMsg> odomSub;
	Publisher<OdometryMsg> odomAdjustPub;
	
	public Localizer(Node node) {
		odometryOffset = (new Utility()).new Pose(0, 0, 0);
		this.totalTicks = new double[2];
		this.totalTicks[0] = this.totalTicks[1] = 0.0;
		this.odomXY = new double[2];
		this.odomXY[0] = this.odomXY[1] = 0.0;
		
		// updates from odometry
	    odomSub = node.newSubscriber("rss/odometry", "rss_msgs/OdometryMsg");
	    odomAdjustPub = node.newPublisher("rss/odometryAdjuster", "rss_msgs/OdometryMsg");
	    odomSub.addMessageListener(new MessageListener<OdometryMsg>() {
	      @Override
	      public void onNewMessage(OdometryMsg message) {
//			  log.info("got odom message" + message.x + " " + message.y);
	    	  odomXY[0] = message.x;
	    	  odomXY[1] = message.y;
	    	  odomTheta = message.theta;
	    	  odomAdjustPub.publish(getPositionPose().getOdomMsg());
	      }
	    });
	}

	// update this by getting the difference between your current position and getPosition()
	public void updatePosition(Utility.Pose correctLocation) {
		this.odometryOffset.setX(odomXY[0] - correctLocation.getX());
		this.odometryOffset.setY(odomXY[1] - correctLocation.getY());
		this.odometryOffset.setTheta(odomTheta - correctLocation.getTheta());
	}
	
//	public double getTicksLeft() {
//		return totalTicks[0];
//	}
//
//	public double getTicksRight() {
//		return totalTicks[1];
//	}
	
	public Point2D.Double getPosition() {
		return new Point2D.Double(this.getPositionPose().getX(), this.getPositionPose().getY());
	}
	
	public Utility.Pose getPositionPose() {
		return (new Utility()).new Pose(odomXY[0] - this.odometryOffset.getX(),
					odomXY[1] - this.odometryOffset.getY(),
					odomTheta - this.odometryOffset.getTheta());
	}

	public double getTheta() {
		return this.getPositionPose().getTheta();
	}

	public double getX() {
		return this.getPositionPose().getX();
	}
	
	public double getY() {
		return this.getPositionPose().getY();
	}

}