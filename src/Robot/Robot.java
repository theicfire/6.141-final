package Robot;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.util.ArrayList;
import java.util.LinkedList;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.BreakBeamMsg;
import org.ros.message.rss_msgs.BumpMsg;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.message.rss_msgs.VisionMsg;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.Node;

import Controller.Utility;
import Door.RosDoorDriver;
import Grasping.Arm;
import Grasping.RosArmDriver;
import Localization.Localizer;
import Navigation.NavigationMain;
import Planner.Planner;
import VisualServoSolution.VisionMsgWrapper;

public class Robot {
	Arm arm;
	RosArmDriver armDriver;
	RosDoorDriver doorDriver;
	// Wheels wheels;
	// BreakBeamSensor handBreakBeam;
	// SonarSensor sideSonars[4];
	
	private RobotState robotState;
	public Planner planner;
	NavigationMain navigationMain;
	public Localizer odom;
	VisionMsgWrapper vision;
	private boolean doneMoving;
	private boolean leftBump;
	private boolean rightBump;

	private Subscriber<VisionMsg> visionSub;
	private Subscriber<BreakBeamMsg> doneMovingSub;
	private Subscriber<BreakBeamMsg> breakBeamSub;
	private Subscriber<BumpMsg> bumpSensorsSub;
	private boolean beamBroken;

	private Publisher<OdometryMsg> waypointCommandPub;
	private Publisher<OdometryMsg> angleCommandPub;
	private Publisher<MotionMsg> movePub;
	private Publisher<BreakBeamMsg> stopPub;
	public Log log;
	
	private LinkedList<Point2D.Double> navQueue;

	public Robot(Node node) {
		this.log = node.getLog();
		this.navigationMain = new NavigationMain(node);
		this.arm = new Arm();
		this.armDriver = new RosArmDriver(node);
		this.doorDriver = new RosDoorDriver(node);
		
		// TODO DELLLLLETEEEEE
		doorDriver.sendDoorPWM();
		log.info("MOVING DOOR NOW");
		
		navQueue = new LinkedList<Point2D.Double>();
		
		this.visionSub = node.newSubscriber("rss/VisionMain", "rss_msgs/VisionMsg");

		this.doneMovingSub = node.newSubscriber("rss/waypointcomplete", "rss_msgs/BreakBeamMsg");
		this.breakBeamSub = node.newSubscriber("rss/BreakBeam", "rss_msgs/BreakBeamMsg");
		this.bumpSensorsSub = node.newSubscriber("rss/BumpSensors", "rss_msgs/BumpMsg");
		this.waypointCommandPub = node.newPublisher("rss/waypointcommand","rss_msgs/OdometryMsg");
		this.angleCommandPub = node.newPublisher("rss/anglecommand", "rss_msgs/OdometryMsg");

		this.stopPub = node.newPublisher("rss/stopcommand",
				"rss_msgs/BreakBeamMsg");
		this.movePub = node.newPublisher("rss/waypointMovecommand", "rss_msgs/MotionMsg");

		log.info("Waiting for movePub");
		
		log.info("Done waiting for movePub");
		log.info("Done waiting for waypointCommand");
		while (waypointCommandPub.getNumberOfSubscribers() == 0) {
			// block
		}
		log.info("Done waiting for waypointCommand");
		log.info("~~~~DONE ALL WAITING IN ROBOT~~~~");
		
		this.odom = new Localizer(node, null);
		this.planner = new Planner(odom, log, navigationMain);
		this.vision = new VisionMsgWrapper();
		this.visionSub.addMessageListener(new VisionMessageListener());
		this.doneMovingSub.addMessageListener(new DoneMovingListener());
		this.breakBeamSub.addMessageListener(new BreakBeamListener());
		this.bumpSensorsSub.addMessageListener(new BumpSensorsListener());
	}

	public class VisionMessageListener implements
			MessageListener<org.ros.message.rss_msgs.VisionMsg> {
		public void onNewMessage(org.ros.message.rss_msgs.VisionMsg vm) {
			vision = new VisionMsgWrapper(vm);
		}
	}

	public class DoneMovingListener implements
			MessageListener<org.ros.message.rss_msgs.BreakBeamMsg> {
		public void onNewMessage(org.ros.message.rss_msgs.BreakBeamMsg bb) {
			log.info("Done moving message received");
			driveToNextLocation(bb.beamBroken);
			planner.markCurrentBlockDone();
		}
	}
	
	public class BreakBeamListener implements
			MessageListener<org.ros.message.rss_msgs.BreakBeamMsg> {
		public void onNewMessage(org.ros.message.rss_msgs.BreakBeamMsg bb) {
			beamBroken = bb.beamBroken; // probably always true
		}
	}
	
	public class BumpSensorsListener implements
			MessageListener<org.ros.message.rss_msgs.BumpMsg> {
		public void onNewMessage(org.ros.message.rss_msgs.BumpMsg bm) {
			leftBump = bm.left;
			rightBump = bm.right;
		}
	}

	public void setStateObject(RobotState newRobotState) {
		log.info("Setting new state" + newRobotState.getClass());
		robotState = newRobotState;
	}

	public RobotState getRobotState() {
		return robotState;
	}

	public boolean doneMoving() {
		return doneMoving;
	}

	public void stopMoving() {
		log.info("Stop all movement!!!");
		BreakBeamMsg stop = new BreakBeamMsg();
		stop.beamBroken = true;
		stopPub.publish(stop);
		navQueue.clear();
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void driveToLocation(Point2D.Double loc) {
		log.info("go to location" + loc);
		OdometryMsg dest = new OdometryMsg();
		dest.x = loc.x;
		dest.y = loc.y;
		waypointCommandPub.publish(dest);
		doneMoving = false;
	}
	
	public void driveToLocations(ArrayList<Point2D.Double> locs) {
		log.info("ROBOT driveToLocations " + locs);
		for (Point2D.Double loc : locs) {
			navQueue.addLast(loc);
		}
		driveToNextLocation(true);
	}
	
	private void driveToNextLocation(boolean didStep) {
		if (navQueue.size() > 0) {
			// keep going to the next location in the queue until you are done with all the location points
			Point2D.Double nextLoc = navQueue.removeFirst();
			log.info("DRIVING TO NEXT LOC: " + nextLoc);
			driveToLocation(nextLoc);
		} else {
			// we are done with everything
			doneMoving = didStep; // probably always true
		}
	}

	public void rotateToLocation(Point2D.Double loc) {
		log.info("setting angle based on point2d" + loc);
		OdometryMsg dest = new OdometryMsg();
		dest.x = loc.x;
		dest.y = loc.y;		
		angleCommandPub.publish(dest);
		doneMoving = false;
	}

	public void rotateToTheta(double theta) {
		log.info("rotating by " + theta);
		OdometryMsg dest = new OdometryMsg();
		dest.x = 0;
		dest.y = 0;
		dest.theta = theta;
		angleCommandPub.publish(dest);
		doneMoving = false;
	}
	
	public void sendMotorMessage(double translationalVelocity, double rotationalVelocity) {
		MotionMsg motionMsg = new MotionMsg();
		motionMsg.rotationalVelocity = rotationalVelocity;
		motionMsg.translationalVelocity = translationalVelocity;
		movePub.publish(motionMsg);
	}
	
	public Double getCurrentLocation() {
		return this.odom.getPosition();
	}

	// TODO move these somewhere else? I don't really like this...
	public void driveForward(double atAngle) {
		OdometryMsg om = new OdometryMsg();
		om.type = "forward";
		om.theta = atAngle;
		this.waypointCommandPub.publish(om);
	}
	
	public void driveBackward(double atAngle) {
		OdometryMsg om = new OdometryMsg();
		om.type = "backward";
		om.theta = atAngle;
		this.waypointCommandPub.publish(om);
	}
	
	public boolean isBeamBroken() {
		return beamBroken;
	}
	
	public boolean isLeftBump() {
		return leftBump;
	}
	
	public boolean isRightBump() {
		return rightBump;
	}

	// NO MORE METHODS HERE!!!! NO MORE METHODS HERE!!!!
	// instead of doing robot.resetArm()
	// put the methods where they logically belong in the hierarchy
	// eg., robot.arm.reset()
	// instead of writing robot.getBlockDistance()
	// use robot.vision.getBlockDistance()
}
