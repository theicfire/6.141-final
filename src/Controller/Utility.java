package Controller;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;

import org.ros.message.rss_msgs.OdometryMsg;

public class Utility {
	public static double getMagnitude(Point2D.Double a, Point2D.Double b) {
		return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
	}
	
	public static double getAngle(Point2D.Double a, Point2D.Double b) {
		return Math.atan2(b.y - a.y, b.x - a.x);
	}

	public static void sleepFor20ms() {
		try {
			Thread.sleep(20);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public static void sleepFor250ms() {
		try {
			Thread.sleep(250);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static void sleepFor5Seconds() {
		try {
			Thread.sleep(5000);
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

		public Pose(Double point, double theta) {
			setX(point.x);
			setY(point.y);
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
		
		public Double getPoint() {
			return new Double(this._x, this._y);
		}

		public OdometryMsg getOdomMsg() {
			OdometryMsg om = new OdometryMsg();
			om.x = getX();
			om.y = getY();
			om.theta = getTheta();
			return om;
		}

		@Override
		public String toString() {
			return "Pose [_x=" + _x + ", _y=" + _y + ", _theta=" + _theta + "]";
		}
		
		
	}

	public static double inRangeNegPiToPi(double radians) {
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
	
	public class Pair<E1,E2> {
		  public final E1 first;
		  public final E2 second;

		  public Pair(E1 first, E2 second) {
		    this.first = first;
		    this.second = second;
		  }
	}

}