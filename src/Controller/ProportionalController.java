package Controller;

public abstract class ProportionalController {

	private final double gain;
	private final double desiredOutput;

	public ProportionalController(double gain, double desired) {
		this.gain = gain;
		this.desiredOutput = desired;
	}
	
	public abstract double difference();
	
	public abstract double getFeedbackOutput();

	public double controlStep() {
		double error = difference();//this.desiredOutput - getFeedbackOutput();
		double result = getGain() * error;
	    return result;
	}

	public double getDesiredOutput() {
		return desiredOutput;
	}

//	public void setDesiredOutput(double desiredOutput) {
//		this.desiredOutput = desiredOutput;
//	}

	public double getGain() {
		return gain;
	}

//	public void setGain(double gain) {
//		this.gain = gain;
//	}

}