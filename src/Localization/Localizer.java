package Localization;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import Challenge.GrandChallengeMap;
import Controller.Utility;
import Controller.Utility.Pose;

public class Localizer {

	final double MAX_ODOMETRY_DIFF_MAGNITUDE = 0.05;
	final double MAX_ODOMETRY_DIFF_THETA = 5.0 * Math.PI / 180.0;
        final double startTime;

	// Utility.Pose odometryOffset;
	private Utility.Pose lastRawPose;
	private Utility.Pose lastRealPose;
	private Utility.Pose curRawPose;
	// private double[] totalTicks;
	// private double[] odomXY;
	// private double odomTheta;
	private Log log;
	Subscriber<OdometryMsg> odomSub;
	Publisher<OdometryMsg> odomAdjustPub;
	private final Pose startPose;
	private boolean init = true;
	private boolean master;

	// /**
	// * For testing
	// */
	// public Localizer() {
	// lastRawPose = (new Utility()).new Pose(0, 0, 0);
	// lastRealPose = (new Utility()).new Pose(0, 0, 0);
	// curRawPose = (new Utility()).new Pose(0, 0, 0);
	// }

	/**
	 * 
	 * @param node
	 * @param startPose
	 *            Only the master node provides a start pose. Slave nodes
	 *            provide null.
	 */
	public Localizer(Node node, final Pose startPose) {
	        startTime = System.currentTimeMillis();
		log = node.getLog();
		this.startPose = startPose;
		if (startPose != null) {
			lastRawPose = startPose;
			lastRealPose = startPose;
			curRawPose = startPose;
		} else {
			// TODO HARDCODED POSITION!!!!
			lastRawPose = (new Utility()).new Pose(0.6, 0.6, 0);
			lastRealPose = (new Utility()).new Pose(0.6, 0.6, 0);
			curRawPose = (new Utility()).new Pose(0.6, 0.6, 0);
		}
		master = (startPose != null);
		// this.totalTicks = new double[2];
		// this.totalTicks[0] = this.totalTicks[1] = 0.0;
		// this.odomXY = new double[2];
		// this.odomXY[0] = this.odomXY[1] = 0.0;

		// the master adjusts the position and relays the information to
		// everyone else
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
						updatePosition(startPose);
						init = false;
					}
					OdometryMsg newOdomMsg = getPositionPose().getOdomMsg();
					odomAdjustPub.publish(newOdomMsg);
				}
			});
		} else {
			odomSub = node
					.newSubscriber("rss/odometry", "rss_msgs/OdometryMsg");
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

	public Localizer(Node node) {
		this(node, null);
	}

	// public void updatePosition(double x, double y, double theta) {
	// updatePosition((new Utility()).new Pose(x, y, theta));
	// }

	public void updatePosition(Utility.ConfidencePose correctLocation) {
		if (!master) {
			throw new RuntimeException(
					"non-master node attempted to update Position");
		}

		// make the curve of accepting more dramatic
		// double confidence = Math.pow(correctLocation.getConfidence(), 2);
		double confidence = correctLocation.getConfidence();
		if (confidence < 1) {
			return;
		}

		// make the lastRealPose a linear combination of lastRawPose and
		// correctLocation, depending on confidence
		Pose newCorrect = (new Utility()).new Pose(correctLocation.getX()
				* confidence + curRawPose.getX() * (1 - confidence),
				correctLocation.getY() * confidence + curRawPose.getY()
						* (1 - confidence), correctLocation.getTheta()
						* confidence + curRawPose.getTheta() * (1 - confidence));

		// curRawPose should be the raw odometry point
		// ensure that the new position is within odometry error of the old
		// position

		double secondsSinceStart = (Math.abs(System.currentTimeMillis() - startTime) / 1000.0);
		// increase multiplier one every sixty seconds
		double odomErrorTimeMultiplier = 1.0 * secondsSinceStart / 60.0;

		if (Utility.getMagnitude(curRawPose.getPoint(), newCorrect.getPoint()) < MAX_ODOMETRY_DIFF_MAGNITUDE * odomErrorTimeMultiplier
				&& Utility.inRangeNegPiToPi(curRawPose.getTheta()
						- newCorrect.getTheta()) < MAX_ODOMETRY_DIFF_THETA * odomErrorTimeMultiplier) {
			log.info("new pose " + newCorrect + " is within odom error (*" + odomErrorTimeMultiplier + ") of +"
					+ curRawPose);
			updatePosition(newCorrect);
		} else {
			// outer edge... that is MAX_MAGNITUDE along the curRawPose ->
			// newCorrect vector
			// TODO
		}

	}

	// update this by getting the difference between your current position and
	// getPosition()
	private void updatePosition(Utility.Pose correctLocation) {
		log.info("updating position to" + correctLocation);
		// log.info("Update position to " + correctLocation);
		lastRawPose = (new Utility()).new Pose(curRawPose.getX(),
				curRawPose.getY(), curRawPose.getTheta());
		lastRealPose = correctLocation;
		// this.odometryOffset.setX(odomXY[0] - correctLocation.getX());
		// this.odometryOffset.setY(odomXY[1] - correctLocation.getY());
		// this.odometryOffset.setTheta(odomTheta - correctLocation.getTheta());
	}

	// public double getTicksLeft() {
	// return totalTicks[0];
	// }
	//
	// public double getTicksRight() {
	// return totalTicks[1];
	// }

	public Point2D.Double getPosition() {
		return new Point2D.Double(this.getPositionPose().getX(), this
				.getPositionPose().getY());
	}

	public Utility.Pose getPositionPose() {
		// trig to go from lastRaw->raw, lastReal->real
		double distance = Utility.getMagnitude(lastRawPose.getPoint(),
				curRawPose.getPoint());
		// TODO angle should be in 3d or something
		double angle = Utility.getAngle(lastRawPose.getPoint(),
				curRawPose.getPoint());
		angle += lastRealPose.getTheta() - lastRawPose.getTheta();
		return (new Utility()).new Pose(lastRealPose.getX() + distance
				* Math.cos(angle), lastRealPose.getY() + distance
				* Math.sin(angle), lastRealPose.getTheta()
				+ (curRawPose.getTheta() - lastRawPose.getTheta()));
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

	// public static void main(String[] args) {
	// System.out.println("hello");
	// Localizer loc = new Localizer();
	// loc.updatePosition(1, 0, Math.PI / 2.0);
	// loc.curRawPose = (new Utility()).new Pose(1, .5, 0);
	// System.out.println(loc.getPositionPose());
	// loc.updatePosition(1, 1, 0);
	// System.out.println(loc.getPositionPose());
	// loc.curRawPose = (new Utility()).new Pose(2, 2, 0);
	// System.out.println(loc.getPositionPose());
	// }

}
