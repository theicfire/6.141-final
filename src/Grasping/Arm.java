package Grasping;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.logging.Log;

import Grasping.ArmJoint;

public class Arm implements ArmGymnastics {

	private static final int ARM_PWM_INCREMENT = 10;
	private List<ArmJoint> joints;
	private ArmJoint shoulder;
	private ArmJoint elbow;
	private ArmJoint wrist;

//	private double gripperMaxAngle;
//	private double gripperMinAngle;
	
	public Arm() {
		final double SHOULDER_LENGTH = .25;
		final double ELBOW_LENGTH = .18;
		shoulder = new ArmJoint(SHOULDER_LENGTH, 0, 550, 2000, 1200, 1975);
		// last two numbers (pwm_theta0, pwm_thetaPiBy2) are wrong for elbow
		elbow = new ArmJoint(ELBOW_LENGTH, 0, 1080, 1280, 1250, 2100);
		wrist = new ArmJoint(1.0, 0, 520, 910, 520, 910);
	
		joints = new ArrayList<ArmJoint>();
		joints.add(elbow);
		joints.add(wrist);
		joints.add(shoulder);
		joints.add(new ArmJoint(0, 0, 0, 0, 0, 0));
		joints.add(new ArmJoint(0, 0, 0, 0, 0, 0));
		joints.add(new ArmJoint(0, 0, 0, 0, 0, 0));
	}
	
	public ArmJoint getShoulder() {
		return shoulder;
	}

	public ArmJoint getElbow() {
		return elbow;
	}
	
	public ArmJoint getWrist() {
		return wrist;
	}
	
	public List<ArmJoint> getJoints() {
		return joints;
	}
	
	public long[] getPwms() {
		long[] ret = new long[joints.size()];
		for (int i = 0; i < joints.size(); i++) {
			ret[i] = joints.get(i).getCurPWM();
		}
		return ret;
	}
	
	@Override
	public void closeGripper() {
		wrist.setDesiredPWM(wrist.MIN_PWM);
	}
	
	@Override
	public void closeOnBlock() {
		wrist.setDesiredPWM(wrist.MIN_PWM);
	}
	
	@Override
	public void openGripper() {
		wrist.setDesiredPWM((int)(wrist.MAX_PWM * 0.9));
	}

	@Override
	public void lowerArm() {
		elbow.setDesiredPWM(elbow.MAX_PWM);
		shoulder.setDesiredPWM(shoulder.MIN_PWM);
	}

	@Override
	public void raiseArm() {
		elbow.setDesiredPWM(elbow.MIN_PWM);
		shoulder.setDesiredPWM(shoulder.MAX_PWM);
	}

	@Override
	public void setGripAngle(double radians) {
		wrist.setTheta(radians);
	}

	/*
	 * Returns true if we are done stepping; otherwise false
	 */
	@Override
	public boolean step() {
//		lastAngle = (lastAngle + 1) % 360;
//		joints.getWrist().setTheta(lastAngle / 180 * Math.PI);
//		joints.getShoulder().setTheta(lastAngle / 180 * Math.PI);
//		joints.getElbow().setTheta(lastAngle / 180 * Math.PI);
		
		// for all joints, get closer to desired..
		int countDone = 0;
		// Order of movement: shoulder > elbow > wrist
		if (discreteMoveJoint(getShoulder()) &&
				discreteMoveJoint(getElbow()) &&
				discreteMoveJoint(getWrist())) {
			return true;
		}
		return false;		
	}	
	
	/*
	 * Return true if done; otherwise set new pwm and return false
	 */
	public boolean discreteMoveJoint(ArmJoint joint) {
		int delta = ARM_PWM_INCREMENT;
		int diffPWM = joint.getDesiredPWM() - joint.getCurPWM();
//		log.info("desired " + joint.getDesiredPWM() + " current " + joint.getCurPWM());
//		log.info("diff is " + diffPWM);
		if (Math.abs(diffPWM) >= delta) {
			// move by delta, signed to +/- depending on diffPWM
			joint.setCurPWM(joint.getCurPWM() + delta * (Math.abs(diffPWM) / diffPWM));			
		} else if (diffPWM != 0) {
			// move by the difference
			joint.setCurPWM(joint.getDesiredPWM());
		} else {
			// don't move; you are done!
			// stop calling step when all 3 joints have hit this
			return true;
		}
		return false;
	}	
	
}
