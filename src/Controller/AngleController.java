package Controller;

import WaypointDriver.Odometer;

public class AngleController extends ProportionalController {

	Odometer odom;
	
	public AngleController(Odometer odom) {
		this.odom = odom;
	}

	@Override
	public double getFeedbackOutput() {
		return this.odom.getAngle();
	}

	@Override
	public double difference() {
		double error = this.getDesiredOutput() - getFeedbackOutput();

		if (error > Math.PI) {
			do {
				error -= 2*Math.PI;
			} while (error > Math.PI);
		} else if (error < -Math.PI) {
			do {
				error += 2*Math.PI;
			} while (error < -Math.PI);
		}

		return error;
	}

}
