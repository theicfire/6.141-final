package Controller;

public abstract class ProportionalIntegralFeedbackController<TInput, TOutput, TFeedback, TDiff, TPropOut, TPropGain,
// TIntIn,
TIntOut, TIntGain> {

	TPropGain propGain;
	TIntGain integralGain;

	public TInput desired;

	ProportionalIntegralFeedbackController(TPropGain propGain,
			TIntGain integralGain) {
		this.propGain = propGain;
		this.integralGain = integralGain;
	}

	public abstract TFeedback getFeedbackOutput();

	abstract TDiff difference(final TFeedback feedback);

	// abstract TIntIn differenceForIntPath(TFeedback feedback);

	// proportional "box"
	abstract TPropOut gainPropPath(TDiff propIn);

	// integral "box"
	abstract TIntOut gainIntPath(TDiff integralIn);

	// computed for the integral
	abstract void accumulateError(TDiff integralIn);

	abstract TOutput sum(TPropOut propOut, TIntOut integralOut);

	public TOutput controlStep() {
		TFeedback feedback = this.getFeedbackOutput();
		// maybe the error; diff between feedback and commanded input (desired)
		TDiff diff = this.difference(feedback);
		// gain function that takes in diff and outputs the output
		TPropOut gainedProp = this.gainPropPath(diff);
		// accumulateError(diff);
		// uses an integral and some input to get an output
		TIntOut gainedInt = this.gainIntPath(diff);
		return sum(gainedProp, gainedInt);
	}

}