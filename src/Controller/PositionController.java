package Controller;

import java.awt.geom.Point2D;
import org.apache.commons.logging.Log;

import Localization.Localizer;

public class PositionController extends
ProportionalIntegralFeedbackController
<
PositionController.PointAnglePair, // input: desired position, and angle while traveling
PositionController.VelocityPair, // output: velocities
PositionController.PointAnglePair, // feedback: actual position and angle while traveling
PositionController.Error,PositionController.VelocityPair, Double, // proportional: in, out, gain
PositionController.VelocityPair,Double // integral: in, out, gain
> {

	// currently this is just a proportional controller
	// do we really need an integral component?

	private static final double MAX_SPEED = .4;
	private static final double MIN_SPEED = .1;
	private static final double MAX_ROTATIONAL_SPEED = .2;

	Localizer odom;
	private Error accumulatedError;
	private Log log;

	public PositionController(Double propGain, Double integralGain,
			Point2D.Double start, Point2D.Double goal,
			Localizer odom, Log log) {
		super(propGain, integralGain);
		this.odom = odom;
		this.accumulatedError = new Error(0.0,0.0,0.0);
		this.desired = new PointAnglePair(goal, 0);
		this.log = log;
	}

	@Override
	public PointAnglePair getFeedbackOutput() {
//		log.info("location is" + " " + this.odom.getPosition() + " " + this.odom.getAngle());
		return new PointAnglePair(this.odom.getPosition(), this.odom.getTheta());
	}

	@Override
	Error difference(PointAnglePair actualPosition) {
		Error error = new Error(
				this.desired.point.x - actualPosition.point.x,
				this.desired.point.y - actualPosition.point.y,
				this.desired.angle - actualPosition.angle);
		error.errorTheta = bringIntoRangeNegPiToPi(error.errorTheta);
//		log.info("feedback says" + " " + error.errorX + " " + error.errorY);
		return error;
	}

	@Override
	void accumulateError(Error integralIn) {
//		this.accumulatedError.errorX += integralIn.errorX;
//		this.accumulatedError.errorY += integralIn.errorY;
//		this.accumulatedError.errorTheta += integralIn.errorTheta;
	}

	@Override
	VelocityPair gainIntPath(Error integralIn) {
		return new VelocityPair(0.0,0.0);
	}

	public static double bringIntoRangeNegPiToPi(double radians) {
		if (radians > Math.PI) {
			do {
				radians -= 2*Math.PI;
			} while (radians > Math.PI);
		} else if (radians < -Math.PI) {
			do {
				radians += 2*Math.PI;
			} while (radians < -Math.PI);
		}
		return radians;
	}
	
	@Override
	VelocityPair gainPropPath(Error error) {
		double translationalVelocity = 0.0;
		double rotationalVelocity = 0.0;
		double angleToGoal = Math.atan2(error.errorY,error.errorX);
		double errorAngleToGoal = bringIntoRangeNegPiToPi(angleToGoal - this.odom.getTheta());
		
		if (errorAngleToGoal < Math.PI/3 && errorAngleToGoal > -Math.PI/3) {
				// we can drive forward
				double distanceToGoal =
					Math.sqrt(error.errorX*error.errorX+error.errorY*error.errorY);
				double maxTranslationalSpeed = distanceToGoal * this.propGain;
				if (maxTranslationalSpeed > MAX_SPEED) {
					maxTranslationalSpeed = MAX_SPEED;
				}
				if (maxTranslationalSpeed < MIN_SPEED) {
					maxTranslationalSpeed = MIN_SPEED;
				}
//				rotationalVelocity = maxTranslationalSpeed * Math.sin(errorAngleToGoal);
//				translationalVelocity = maxTranslationalSpeed * Math.cos(errorAngleToGoal);
				translationalVelocity = maxTranslationalSpeed;
		}
		rotationalVelocity = errorAngleToGoal * this.propGain;
		if (rotationalVelocity > MAX_ROTATIONAL_SPEED) {
			rotationalVelocity = MAX_ROTATIONAL_SPEED;
		}
		
		VelocityPair result = new VelocityPair(translationalVelocity,rotationalVelocity);
		return result;
	}

	@Override
	VelocityPair sum(VelocityPair propOut, VelocityPair integralOut) {
//		return new VelocityPair(
//				propOut.translationalVelocity + integralOut.translationalVelocity,
//				propOut.rotationalVelocity + integralOut.rotationalVelocity);
		return new VelocityPair(
				propOut.getTranslationalVelocity(),
				propOut.getRotationalVelocity());
	}

	class PointAnglePair {
		Point2D.Double point;
		double angle; // radians
		PointAnglePair(Point2D.Double point, double angle) {
			this.point = new Point2D.Double(point.x,point.y);
			this.angle = angle;
			
		}
	}
	
	public class VelocityPair {
		private double translationalVelocity;
		private double rotationalVelocity;
		VelocityPair(double translationalVelocity, double rotationalVelocity) {
			this.setTranslationalVelocity(translationalVelocity);
			this.setRotationalVelocity(rotationalVelocity);
		}
		public double getTranslationalVelocity() {
			return translationalVelocity;
		}
		public void setTranslationalVelocity(double translationalVelocity) {
			this.translationalVelocity = translationalVelocity;
		}
		public double getRotationalVelocity() {
			return rotationalVelocity;
		}
		public void setRotationalVelocity(double rotationalVelocity) {
			this.rotationalVelocity = rotationalVelocity;
		}
		@Override
		public String toString() {
			return "VelocityPair [translationalVelocity="
					+ translationalVelocity + ", rotationalVelocity="
					+ rotationalVelocity + "]";
		}
		
	}
	
	// holder
	class Error {
		double errorX;
		double errorY;
		double errorTheta; // radians
		Error(double errorX, double errorY, double errorTheta) {
			this.errorX = errorX;
			this.errorY = errorY;
			this.errorTheta = errorTheta;
		}
	}

}