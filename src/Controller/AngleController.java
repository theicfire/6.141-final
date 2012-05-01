package Controller;

import org.ros.message.rosgraph_msgs.Log;

import Localization.Localizer;

public class AngleController extends ProportionalController {

	Localizer odom;
	
	public AngleController(double gain, double desired, Localizer odom) {
		super(gain, desired);
		this.odom = odom;
	}

	@Override
	public double getFeedbackOutput() {
		return this.odom.getTheta();
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
