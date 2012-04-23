package Navigation;

import org.ros.message.MessageListener;
import org.ros.message.lab5_msgs.GUIEraseMsg;

import LocalNavigation.SonarGUI;

public class EraseMessageListener extends LocalNavigation.EraseMessageListener
		implements MessageListener<GUIEraseMsg> {

	MapGUI gui;
	
	public EraseMessageListener(MapGUI mapGUI) {
		super(mapGUI);
		this.gui = mapGUI;
	}

	@Override
	public void onNewMessage(GUIEraseMsg arg0) {
		gui.eraseRects();
		gui.erasePolys();
		super.onNewMessage(arg0);
	}

}
