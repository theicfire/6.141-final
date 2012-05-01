package Controller;

import org.ros.message.rosgraph_msgs.Log;

import WaypointDriver.Odometer;

public class AngleController extends ProportionalController {

	Odometer odom;
	
	public AngleController(double gain, double desired, Odometer odom) {
		super(gain, desired);
		this.odom = odom;
	}

	@Override
	public double getFeedbackOutput() {
		return this.odom.getAngle();
	}

	@Override
	public double difference() {
		double error = this.getDesiredOutput() - getFeedbackOutput();
		return Utility.inRangeNegPiToPi(error);
	}
	
	@Override
	public double controlStep() {
		return Utility.inRangeNegPiToPi(super.controlStep());
	}

}
