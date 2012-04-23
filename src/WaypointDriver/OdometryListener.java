package WaypointDriver;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.OdometryMsg;

public class OdometryListener implements MessageListener<OdometryMsg> {

	private RosWaypointDriver driver;

	public OdometryListener(RosWaypointDriver driver) {
		// TODO Auto-generated constructor stub
		driver = driver;
	}

	@Override
	public void onNewMessage(OdometryMsg arg0) {
		// TODO Auto-generated method stub
		driver.handle(arg0);
	}

}
