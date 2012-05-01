package WaypointDriver;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.util.List;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.BreakBeamMsg;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import Controller.AngleController;
import Controller.PositionController;
import Controller.PositionController.VelocityPair;
import Controller.Utility;
import Localization.Localizer;

/**
 * Moves the robot to a location (actually drives the robot)
 * 
 * @author rss-student
 * 
 */
public class RosWaypointDriver implements NodeMain {
	// parameters
	final double PROPORTIONAL_GAIN = 1.5;
	final double INTEGRAL_GAIN = 9999.0;
	final double MAX_ROTATIONAL_VELOCITY = .3;
	final double epsilon2 = .005; // in m
	final long ENOUGH_TIME = 400;

	private Node globalNode;
	private Log log;
	private Publisher<MotionMsg> movePub;
	private Localizer odom;
	long lastTime = 0;
	
	boolean forceStop = false; // comes from a message; stops movement

	Publisher<BreakBeamMsg> pubComplete; 
	Subscriber<OdometryMsg> destSub;
	Subscriber<OdometryMsg> angleSub;
	Subscriber<BreakBeamMsg> stopSub;
	Subscriber<MotionMsg> moveSub;

	@Override
	public void onStart(Node node) {
		// TODO Auto-generated method stub
		StartRosWaypointDriver(node);

		movePub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
		while (movePub.getNumberOfSubscribers() == 0) {
			// block
		}
		pubComplete = node.newPublisher("rss/waypointcomplete", "rss_msgs/BreakBeamMsg");
		
		destSub = node.newSubscriber("rss/waypointcommand", "rss_msgs/OdometryMsg");
		destSub.addMessageListener(new DestCommandMessageListener());
		
		moveSub = node.newSubscriber("rss/waypointMovecommand", "rss_msgs/MotionMsg");
		moveSub.addMessageListener(new MoveCommandMessageListener());
		
		angleSub = node.newSubscriber("rss/anglecommand", "rss_msgs/OdometryMsg");
		angleSub.addMessageListener(new AngleCommandMessageListener());
		
		stopSub = node.newSubscriber("rss/stopcommand", "rss_msgs/BreakBeamMsg");
		stopSub.addMessageListener(new StopListener());
	}

	public class DestCommandMessageListener implements
			MessageListener<org.ros.message.rss_msgs.OdometryMsg> {
		public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg om) {
			forceStop = false;
			if (om.type.startsWith("forward")) {
				log.info("MOVE FORWARD BY THETA: " + om.theta);
				sendMotorMessage(0.15, om.theta);
			} else if (om.type.startsWith("backward")) {
				log.info("MOVE BACKWARD BY THETA: " + om.theta);
				sendMotorMessage(-0.15, om.theta);
			}
			else {
				driveToPoint(new Point2D.Double(om.x, om.y));
			}
		}
	}
	
	public class MoveCommandMessageListener implements
			MessageListener<org.ros.message.rss_msgs.MotionMsg> {
		public void onNewMessage(org.ros.message.rss_msgs.MotionMsg motionMsg) {
			forceStop = true;
			movePub.publish(motionMsg);
		}
	}

	public class AngleCommandMessageListener implements
			MessageListener<org.ros.message.rss_msgs.OdometryMsg> {
		public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg om) {
			rotateToPoint(new Point2D.Double(om.x, om.y));
		}
	}

	public class StopListener implements
			MessageListener<org.ros.message.rss_msgs.BreakBeamMsg> {
		public void onNewMessage(org.ros.message.rss_msgs.BreakBeamMsg bb) {
			// stop driving
			forceStop = true;
			stopMoving();
		}
	}
	
	public void StartRosWaypointDriver(Node node) {
		this.globalNode = node;
		this.log = node.getLog();
		
		odom = new Localizer(globalNode, false);
	}

	void robotMoveAlongPath(List<Point2D.Double> verts) {
		log.info("MOVE ALONG PATH");

		for (Point2D.Double vert : verts) {
			driveToPoint(vert);
		}

		this.stopMoving();
		this.stopMoving();

		log.info("NOW WE ARE DONE");
	}

	private void sendMotorMessage(double translationalVelocity,
			double rotationalVelocity) {
		MotionMsg stopMsg = new MotionMsg();
		stopMsg.rotationalVelocity = rotationalVelocity;
		stopMsg.translationalVelocity = translationalVelocity;
		movePub.publish(stopMsg);
	}

	private void sendMotorMessageIfEnoughTimeHasPassedSinceLastCall(
			double translationalVelocity, double rotationalVelocity) {
		boolean enoughTimePassed = System.currentTimeMillis() - lastTime > ENOUGH_TIME;
		if (enoughTimePassed) {
			MotionMsg stopMsg = new MotionMsg();
			stopMsg.rotationalVelocity = rotationalVelocity;
			stopMsg.translationalVelocity = translationalVelocity;
			movePub.publish(stopMsg);
		}
	}

	/**
	 * Blocking method that rotates the robot to point to a specified point
	 * @param vert
	 * TODO use the commented code below this which has a controller implemented
	 */
	public void rotateToPoint(Double vert) {
		// AngleController ac = new AngleController(odom);
		// ac.setGain(0.5);
		// ac.setDesiredOutput(Math.atan2(vert.y - odom.odomXY[1], vert.x
		// - odom.odomXY[0]));

		double error = Math.atan2(vert.y - odom.getY(), vert.x
				- odom.getX())
				- odom.getTheta();
		if (error > Math.PI) {
			do {
				error -= 2 * Math.PI;
			} while (error > Math.PI);
		} else if (error < -Math.PI) {
			do {
				error += 2 * Math.PI;
			} while (error < -Math.PI);
		}

		double rv = MAX_ROTATIONAL_VELOCITY;
		if (error < 0)
			rv *= -1;
		this.sendMotorMessage(0, rv);
		try {
			Thread.sleep((long) (Math.abs(error * 1000 / rv)));
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

	this.stopMoving();
	}
//	public void rotateToPoint(double theta) {		
//		double desired = theta + odom.getAngle();		
//		desired = Utility.inRangeNegPiToPi(desired);
//		
//		AngleController ac = new AngleController(0.5, desired, odom);
//
//		double rv = MAX_ROTATIONAL_VELOCITY;
//		
//		while (Math.abs(ac.difference()) > 0.01) {
////			log.info("desired angle " + desired + " actual " + Utility.inRangeNegPiToPi(odom.getAngle()) + 
////					"actual actual " + ac.getFeedbackOutput());
////			log.info("difference was " + ac.difference());
//			double acOutput = ac.controlStep();
////			double output = Math.min(ac.getFeedbackOutput(), rv);
////			if (ac.getFeedbackOutput() < 0)
////				output = Math.max(ac.getFeedbackOutput(), -rv);
//			this.sendMotorMessage(0, acOutput);
//			try {
//				Thread.sleep(20);
////				Thread.sleep((long) (Math.abs(acOutput * 1000 / rv)));
//			} catch (InterruptedException e1) {
//				// TODO Auto-generated catch block
//				e1.printStackTrace();
//			}
//		}
//			
//		this.stopMoving();
//	}
	/**
	 * Blocking method which drives the robot to a target point using
	 * proportional control.
	 * 
	 * @param vert
	 *            The destination point
	 */
	public void driveToPoint(Double vert) {
		// TODO Auto-generated method stub
		log.info("GO FIND " + vert);

		Point2D.Double start = odom.getPosition();
		rotateToPoint(vert);
		
		PositionController p = new PositionController(PROPORTIONAL_GAIN,
				INTEGRAL_GAIN, new Point2D.Double(start.x, start.y),
				new Point2D.Double(vert.x, vert.y), odom, log);
		while (!forceStop) {
			

			boolean enoughTimeHasPassed = System.currentTimeMillis() - lastTime > ENOUGH_TIME;
			if (enoughTimeHasPassed) {
				VelocityPair step = p.controlStep();
				log.info("Step is " + step);
				this.sendMotorMessage(step.getTranslationalVelocity(),
						step.getRotationalVelocity());
				lastTime = System.currentTimeMillis();
			}

			double distanceToDst = Math.sqrt(Math.pow(odom.getX() - vert.x,
					2) + Math.pow(odom.getY() - vert.y, 2));

			if (distanceToDst < epsilon2) {
				break;
			}
		}
		
		this.sendMotorMessage(0.0, 0.0);

		log.info("DONE FINDING " + vert);
		
		// Say that you are done to the world
		BreakBeamMsg doneMsg = new BreakBeamMsg();
		doneMsg.beamBroken = true;
		pubComplete.publish(doneMsg);
		
		start = vert;
	}

	public void stopMoving() {
		// TODO Auto-generated method stub
		this.sendMotorMessage(0, 0.0);
	}

	public boolean doneMovement() {
		// TODO Auto-generated method stub
		return false;
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
