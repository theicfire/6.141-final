package Grasping;

/**
 * MIN:
 * 	elbow: all the way up
 * 	shoulder: all the way down
 * 	grip: closed
 * @author rss-student
 *
 */
public class ArmJoint extends InverseKinematics.Joint {
	final int MAX_PWM;
	final int MIN_PWM;
	final int PWM_theta0;
	final int PWM_thetaPIBy2;
	private int desiredPWM; // desired; not necessarily the current pwm
	private int curPWM;
	
	
	public ArmJoint(double length, double startTheta, int min_pwm, int max_pwm, int pwm_theta0, int pwm_thetaPIBy2) {
		super(length, startTheta);
		MAX_PWM = max_pwm;
		MIN_PWM = min_pwm;
		PWM_theta0 = pwm_theta0;
		PWM_thetaPIBy2 = pwm_thetaPIBy2;		
		desiredPWM = MIN_PWM;
		curPWM = MIN_PWM;
	}
		
	public double pwmToRadians(int pwm) {
		int diff = pwm - PWM_theta0;
		return (Math.PI/2.0) * diff / (PWM_thetaPIBy2-PWM_theta0);
	}

	public boolean setTheta(double theta) {
		// TODO put somewhere else...
//		if (Math.abs(desiredPWM - curPWM) > 200) {
//			throw new RuntimeException("Too much movement in one turn - " + Math.abs(desiredPWM - curPWM));
//		}
//		
		double outputPWM = (PWM_thetaPIBy2 - PWM_theta0) * (theta / (Math.PI / 2.0)) + MIN_PWM;
		
		if (outputPWM > MAX_PWM) {
			throw new RuntimeException("non-physical angle setting for joint");
		} else if (outputPWM < MIN_PWM) {
			throw new RuntimeException("non-physical angle setting for joint");
		}
		desiredPWM = (int)outputPWM;
		return true;
	}
	
	public int getPWM() {
		if (desiredPWM > MAX_PWM) {
			return MAX_PWM;
		} else if (desiredPWM < MIN_PWM) {
			return MIN_PWM;
		}
		return desiredPWM;
	}
	
	public void setCurPWM(int pwm) {
		if (pwm > MAX_PWM) {
			throw new RuntimeException("exceeded MAX_PWM " + pwm + " > " + MAX_PWM);
		} else if (pwm < MIN_PWM) {
			throw new RuntimeException("below MIN_PWM " + pwm + " < " + MIN_PWM);
		}
		this.curPWM = pwm;
	}
	
	public int getCurPWM() {
		return curPWM;
	}

	public int getDesiredPWM() {
		return desiredPWM;
	}

	public void setDesiredPWM(int desiredPWM) {
		this.desiredPWM = desiredPWM;
	}
}
