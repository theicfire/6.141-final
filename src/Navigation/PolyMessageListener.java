package Navigation;

import org.ros.message.MessageListener;
import org.ros.message.lab6_msgs.GUIPolyMsg;

import java.awt.Color;
import java.awt.geom.*;
import java.util.ArrayList;
import java.util.List;

public class PolyMessageListener implements MessageListener<GUIPolyMsg> {

	private MapGUI gui;

	public PolyMessageListener(MapGUI mapGUI) {
		gui = mapGUI;
	}

	@Override
	public synchronized void onNewMessage(GUIPolyMsg msg) {
		List<Point2D.Double> vertices = new ArrayList<Point2D.Double>();
		for (int i = 0; i < msg.numVertices; i++){
			Point2D.Double p = new Point2D.Double(msg.x[i], msg.y[i]);
			vertices.add(p);
		}
		boolean closed = msg.closed == 1;
		boolean filled = msg.filled == 1;
		Color c = new Color((int)msg.c.r, (int)msg.c.g, (int)msg.c.b); //gui.makeRandomColor();
		gui.addPoly(vertices, closed, filled, c);
	}

}
