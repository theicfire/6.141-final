package Grasping;

import java.awt.geom.Point2D;
import java.util.List;

public class InverseKinematics {

	public static class Joint {
		public final double length;
		public double MAX_THETA = Double.MAX_VALUE;
		public double MIN_THETA = Double.MIN_VALUE;
		private double theta;

		public Joint(double length, double theta) {
			this.theta = theta;
			this.length = length;
		}
		
		public Joint(double length, double theta, double min_theta, double max_theta) {
			this(length, theta);
			this.MAX_THETA = max_theta;
			this.MIN_THETA = min_theta;
		}
		
		double getTheta() {
			return theta;
		}
		
		/**
		 * blocks until movement is complete
		 * @param newTheta
		 * @return
		 */
		public boolean setTheta(double newTheta) {
			if (theta < MAX_THETA && newTheta > MIN_THETA) {
				theta = newTheta;
			} else {
				throw new RuntimeException("non-physical angle setting for joint");
			}
			return true;
		}
	}
	
	/*
	 * @precondition joint is in joints
	 * @returns The XY of the pivot of the specified joint, computed 
	 * by examining the list of joints
	 */
	public static Point2D.Double getJointXY(Joint joint, List<Joint> joints) {
		Point2D.Double pos = new Point2D.Double(0, 0);
		double accTheta = 0;
		for (Joint j : joints) {
			if (j == joint) {
				return pos;
			}
			
			accTheta += j.theta;
			pos.x += j.length * Math.cos(accTheta);
			pos.y += j.length * Math.sin(accTheta);
		}
		return pos;
	}
	
	public static double sq(double val) {
		return val * val;
	}
	
	/*
	 * from http://www.google.com/url?sa=t&rct=j&q=inverse%20kinematics%20two%20joints&source=web&cd=3&ved=0CDoQFjAC&url=http%3A%2F%2Fwww.eng.utah.edu%2F~cs5310%2Fchapter5.pdf&ei=ATuDT_auK8KTtwed6_yjBg&usg=AFQjCNGlEBRwC1WYfmdu5b1aNv_Hj5cpjg&cad=rja
	 */
	public static void Move2JointsToPoint(Joint joint1, Joint joint2, Point2D.Double p) {
		double top = sq(joint1.length + joint2.length) - (sq(p.x) + sq(p.y));
		double bottom = (sq(p.x) + sq(p.y)) - sq(joint1.length - joint2.length);
		double theta2 = 2 * Math.atan(Math.sqrt(top / bottom));
		
		double phi = Math.atan2(p.y, p.x);
		double psi = Math.atan2(joint2.length * Math.sin(theta2), joint1.length + joint2.length * Math.cos(theta2));
		
		joint1.theta = (phi - psi);
		joint2.theta = theta2;
	}
	
	/**
	 * modifies joint1, joint2, joint3
	 * @param joint1
	 * @param joint2
	 * @param joint3
	 * @param p
	 */
	public static void Move3JointsToPoint(Joint joint1, Joint joint2,
			Joint joint3, Point2D.Double p) {
		double theta1 = Math.atan2(p.y, p.x);
		int z = 0;
		double c3 = (sq(p.x) + sq(p.y) + sq(z - joint1.length) - sq(joint2.length) - sq(joint3.length));
		c3 /= 2 * joint2.length * joint3.length;
		
		double s3 = Math.sqrt(1 - sq(c3));
		
		double theta3 = Math.atan2(s3, c3);
		
		double theta2 = Math.atan2(z - joint1.length, Math.sqrt(sq(p.x) + sq(p.y))); 
		theta2 -= Math.atan2(joint3.length * s3, joint2.length + joint3.length * c3);
		
		joint1.theta = theta1;
		joint2.theta = theta2;
		joint3.theta = theta3;
	}
	
	 public static void main (String args[]) {
		System.out.println("hello");
		
		InverseKinematics.Joint k1 = new InverseKinematics.Joint(3.0, 0, 0, 2 * Math.PI);
		InverseKinematics.Joint k2 = new InverseKinematics.Joint(2.0, 0, 0, 2 * Math.PI);
		InverseKinematics.Move2JointsToPoint(k1, k2, new Point2D.Double(4.0, 1.0));
		
		InverseKinematics.Joint j1 = new InverseKinematics.Joint(3.0, 0, 0, 2 * Math.PI);
		InverseKinematics.Joint j2 = new InverseKinematics.Joint(2.0, 0, 0, 2 * Math.PI);
		InverseKinematics.Joint j3 = new InverseKinematics.Joint(1.0, 0, 0, 2 * Math.PI);
		InverseKinematics.Move3JointsToPoint(j1, j2, j3, new Point2D.Double(6.0, 0.0));
		
//		joints.add(new Joint(1, 0, 0, 5));
	}
	
}