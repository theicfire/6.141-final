package Controller;

import java.awt.geom.Point2D;

public class Utility {
	public static double getMagnitude(Point2D.Double a, Point2D.Double b) {
		return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
	}

	public static void sleepForASecond() {
		try {
			Thread.sleep(20);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public class Pose {
		private double _x, _y, _theta;

		public Pose(double x, double y, double theta) {
			setX(x);
			setY(y);
			setTheta(theta);
		}

		public double getY() {
			return _y;
		}

		public void setY(double _y) {
			this._y = _y;
		}

		public double getTheta() {
			return _theta;
		}

		public void setTheta(double _theta) {
			this._theta = _theta;
		}

		public double getX() {
			return _x;
		}

		public void setX(double _x) {
			this._x = _x;
		}
	}

}