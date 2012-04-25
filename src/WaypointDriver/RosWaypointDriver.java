package WaypointDriver;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.util.List;

import org.apache.commons.logging.Log;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

import Controller.AngleController;
import Controller.PositionController;
import Controller.PositionController.VelocityPair;

/**
 * Moves the robot to a location (actually drives the robot)
 * 
 * @author rss-student
 * 
 */
public class RosWaypointDriver {
	// parameters
	final double PROPORTIONAL_GAIN = 1.5;
	final double INTEGRAL_GAIN = 9999.0;
	final double MAX_ROTATIONAL_VELOCITY = .3;
	final double epsilon2 = .005; // in m
	final long ENOUGH_TIME = 400;

	private Node globalNode;
	private Log log;
	private Publisher<MotionMsg> movePub;
	private Odometer odom;
	long lastTime = 0;

	public RosWaypointDriver(Node node) {
		this.globalNode = node;
		this.log = node.getLog();
		movePub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
		odom = new Odometer(globalNode);
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

		// AngleController ac = new AngleController(odom);
		// ac.setGain(0.5);
		// ac.setDesiredOutput(Math.atan2(vert.y - odom.odomXY[1], vert.x
		// - odom.odomXY[0]));

		double error = Math.atan2(vert.y - odom.odomXY[1], vert.x
				- odom.odomXY[0])
				- odom.odomTheta;
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
		PositionController p = new PositionController(PROPORTIONAL_GAIN,
				INTEGRAL_GAIN, new Point2D.Double(start.x, start.y),
				new Point2D.Double(vert.x, vert.y), odom, log);
		while (true) {
			VelocityPair step = p.controlStep();

			boolean enoughTimeHasPassed = System.currentTimeMillis() - lastTime > ENOUGH_TIME;
			if (enoughTimeHasPassed) {
				this.sendMotorMessage(step.getTranslationalVelocity(),
						step.getRotationalVelocity());
				lastTime = System.currentTimeMillis();
			}

			double distanceToDst = Math.sqrt(Math.pow(odom.odomXY[0] - vert.x,
					2) + Math.pow(odom.odomXY[1] - vert.y, 2));

			if (distanceToDst < epsilon2) {
				break;
			}
		}
		this.sendMotorMessage(0.0, 0.0);

		log.info("DONE FINDING " + vert);
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

}
