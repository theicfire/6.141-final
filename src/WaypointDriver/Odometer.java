package WaypointDriver;


import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.OdometryMsg;
import org.ros.node.Node;
import org.ros.node.topic.Subscriber;

public class Odometer {

	protected double[] totalTicks;

	public double[] odomXY;
	public double odomTheta;
	
	Subscriber<OdometryMsg> odomSub;
	
	public Odometer(Node node) {
		this.totalTicks = new double[2];
		this.totalTicks[0] = this.totalTicks[1] = 0.0;
		this.odomXY = new double[2];
		this.odomXY[0] = this.odomXY[1] = 0.0;
	    odomSub = node.newSubscriber("rss/odometry", "rss_msgs/OdometryMsg");
	    odomSub.addMessageListener(new MessageListener<OdometryMsg>() {
	      @Override
	      public void onNewMessage(OdometryMsg message) {
//			  log.info("got odom message" + message.x + " " + message.y);
	    	  odomXY[0] = message.x;
	    	  odomXY[1] = message.y;
	    	  odomTheta = message.theta;
	      }
	    });
	}

	public double getTicksLeft() {
		return totalTicks[0];
	}

	public double getTicksRight() {
		return totalTicks[1];
	}

	public Point2D.Double getPosition() {
		return new Point2D.Double(this.odomXY[0],this.odomXY[1]);
	}

	public double getAngle() {
		return odomTheta;
	}


}