package Localization;


import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import Controller.Utility;
import Controller.Utility.Pose;

public class Localizer {

//	Utility.Pose odometryOffset;
	Utility.Pose lastRawPose;
	Utility.Pose lastRealPose;
	Utility.Pose curRawPose;
	//private double[] totalTicks;
//	private double[] odomXY;
//	private double odomTheta;
	private Log log;
	Subscriber<OdometryMsg> odomSub;
	Publisher<OdometryMsg> odomAdjustPub;
	private boolean init = true;
	private boolean master;
	/**
	 * For testing
	 */
	public Localizer() {
		lastRawPose = (new Utility()).new Pose(0, 0, 0);
		lastRealPose = (new Utility()).new Pose(0, 0, 0);
		curRawPose = (new Utility()).new Pose(0, 0, 0);
	}
	/**
	 * 
	 * @param node
	 * @param master only one master localizer; synchronizes all the others
	 */
	public Localizer(Node node, boolean isMaster) {
		log = node.getLog();
		lastRawPose = (new Utility()).new Pose(0, 0, 0);
		lastRealPose = (new Utility()).new Pose(0, 0, 0);
		curRawPose = (new Utility()).new Pose(0, 0, 0);
		master = isMaster;
		//this.totalTicks = new double[2];
		//this.totalTicks[0] = this.totalTicks[1] = 0.0;
		//this.odomXY = new double[2];
		//this.odomXY[0] = this.odomXY[1] = 0.0;

		// the master adjusts the position and relays the information to everyone else
		if (master) {
			odomSub = node.newSubscriber("rss/odometryToAdjust",
					"rss_msgs/OdometryMsg");
			odomAdjustPub = node.newPublisher("rss/odometry",
					"rss_msgs/OdometryMsg");
			odomSub.addMessageListener(new MessageListener<OdometryMsg>() {
				@Override
				public void onNewMessage(OdometryMsg message) {
					curRawPose = (new Utility()).new Pose(message.x, message.y,
							message.theta);
					if (init) {
						updatePosition(0.0, 0.0, 0.0);
						init = false;
					}
					OdometryMsg newOdomMsg = getPositionPose().getOdomMsg();
					odomAdjustPub.publish(newOdomMsg);
				}
			});
		} else {
			odomSub = node.newSubscriber("rss/odometry", "rss_msgs/OdometryMsg");
			odomSub.addMessageListener(new MessageListener<OdometryMsg>() {
				@Override
				public void onNewMessage(OdometryMsg message) {
					// log.info("got odom message" + message.x + " " +
					// message.y);
					curRawPose = (new Utility()).new Pose(message.x, message.y,
							message.theta);
				}
			});
		}
	}

	public void updatePosition(double x, double y, double theta) {
		updatePosition((new Utility()).new Pose(x, y, theta));
	}
	
	// update this by getting the difference between your current position and getPosition()
	public void updatePosition(Utility.Pose correctLocation) {
		if (! master) {
			throw new RuntimeException("non master node attempted to update Position");
		}
		log.info("updating position to" + correctLocation);
//		log.info("Update position to " + correctLocation);
		lastRawPose = (new Utility()).new Pose(curRawPose.getX(), curRawPose.getY(), curRawPose.getTheta());
		lastRealPose = correctLocation;
//		this.odometryOffset.setX(odomXY[0] - correctLocation.getX());
//		this.odometryOffset.setY(odomXY[1] - correctLocation.getY());
//		this.odometryOffset.setTheta(odomTheta - correctLocation.getTheta());
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
		// trig to go from lastRaw->raw, lastReal->real
		double distance = Utility.getMagnitude(lastRawPose.getPoint(), curRawPose.getPoint());
        // TODO angle should be in 3d or something
        double angle = Utility.getAngle(lastRawPose.getPoint(), curRawPose.getPoint());
        angle += lastRealPose.getTheta() - lastRawPose.getTheta();
		return (new Utility()).new Pose(lastRealPose.getX() + distance * Math.cos(angle),
                    lastRealPose.getY() + distance * Math.sin(angle),
					lastRealPose.getTheta() + (curRawPose.getTheta() - lastRawPose.getTheta()));
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
	
	public static void main(String[] args) {
		System.out.println("hello");
		Localizer loc = new Localizer();
		loc.updatePosition(1, 0, Math.PI / 2.0);
		loc.curRawPose = (new Utility()).new Pose(1, .5, 0);
		System.out.println(loc.getPositionPose());
//		loc.updatePosition(1, 1, 0);
//		System.out.println(loc.getPositionPose());
//		loc.curRawPose = (new Utility()).new Pose(2, 2, 0);
//		System.out.println(loc.getPositionPose());
	}

}