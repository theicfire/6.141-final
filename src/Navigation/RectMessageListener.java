package Navigation;

import java.awt.Color;

import org.ros.message.MessageListener;
import org.ros.message.lab5_msgs.ColorMsg;
import org.ros.message.lab6_msgs.GUIRectMsg;

public class RectMessageListener implements MessageListener<GUIRectMsg> {

	private MapGUI gui;

	public RectMessageListener(MapGUI mapGUI) {
		this.gui = mapGUI;
	}

	@Override
	public void onNewMessage(GUIRectMsg message) {
		boolean filled = message.filled == 1;
		Color color = getColorFromMsg(message.c);
		gui.addRect(message.x, message.y, message.width, message.height,
	              filled, color);
		
	}

	public Color getColorFromMsg(ColorMsg c) {
		Color color;
		if (c== null){
			color = gui.rectColor;
		} else {
			color = new Color(c.r, c.g, c.b);
		}
		return color;
	}

}
