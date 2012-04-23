package Challenge;

import java.awt.*;
import java.awt.geom.*;
import javax.swing.*;
import java.util.*;

import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.namespace.GraphName;
import org.ros.node.topic.Subscriber;
import org.ros.message.Challenge_msgs.GUIEllipseMessage;
import org.ros.message.Challenge_msgs.GUIStringMessage;
import org.ros.message.MessageListener;

import Navigation.MapGUI;


/**
 * <p>Extends <code>GlobalNavigation.MapGUI</code> to display
 * construction-related data (first read the doc for that class).</p>
 *
 * <p>New methods (and corresponding ROS messages) have been added to draw
 * ellipses and strings.</p>
 *   
 * @author vona
 **/
public class ConstructionGUI extends Navigation.MapGUI implements NodeMain {

  /**
   * <p>The application name.</p>
   **/
  public static final String APPNAME = "ConstructionGUI";

  /**
   * <p>Bitfield constant for {@link GUIEraseMessage}.</p>
   **/
  public static final int ERASE_ELLIPSES = 1<<8;

  /**
   * <p>Bitfield constant for {@link GUIEraseMessage}.</p>
   **/
  public static final int ERASE_STRINGS = 1<<9;

  /**
   * <p>Line width of the lines making up a {@link ConstructionGUI.Ellipse}
   * in pixels.</p>
   **/
  public static final float ELLIPSE_LINE_WIDTH = 1.0f;

  /**
   * <p>Default color for {@link ConstructionGUI.Ellipse}s.</p>
   **/
  public static final Color DEFAULT_ELLIPSE_COLOR = Color.RED;

  /**
   * <p>Default color for {@link ConstructionGUI.GUIString}s.</p>
   **/
  public static final Color DEFAULT_STRING_COLOR = Color.BLACK;

  /**
   * <p>Whether to paint the ellipses.</p>
   **/
  protected boolean ellipsesEnabled = true;

  /**
   * <p>Whether to paint the strings.</p>
   **/
  protected boolean stringsEnabled = true;

  /**
   * <p>The current {@link ConstructionGUI.Ellipse} color.</p>
   **/
  protected Color ellipseColor = dupColor(DEFAULT_ELLIPSE_COLOR);

  /**
   * <p>The current {@link ConstructionGUI.GUIString} color.</p>
   **/
  protected Color stringColor = dupColor(DEFAULT_STRING_COLOR);

  /**
   * <p>A visual Ellipse.</p>
   **/
  protected class Ellipse extends Glyph {

    /**
     * <p>Ellipse color.</p>
     **/
    Color color;

    /**
     * <p>Whether to draw filled.</p>
     **/
    boolean filled;

    /**
     * <p>The ellipse in world coords.</p>
     **/
    Ellipse2D.Double e = new Ellipse2D.Double();

    /**
     * <p>Create a new ellipse.</p>
     **/
    Ellipse(Ellipse2D.Double e, boolean filled, Color color) {
      this.color = dupColor(color);
      this.filled = filled;
      this.e.setFrame(e.getX(), e.getY(), e.getWidth(), e.getHeight());
    }

    /**
     * <p>Create a new ellipse.</p>
     **/
    Ellipse(double x, double y, double width, double height,
            boolean filled, Color color) {

      this.color = dupColor(color);
      this.filled = filled;

      this.e.x = x-width/2;
      this.e.y = y-height/2;
      this.e.width = width;
      this.e.height = height;
    }

    /**
     * <p>Paints the ellipse.</p>
     *
     * <p>Assumes line width is already set.</p>
     *
     * @param g2d the graphics context
     **/
    public void paint(Graphics2D g2d) {

      g2d.setColor(color);

      if (filled)
        g2d.fill(e);

      g2d.draw(e);
    }
  }

  /**
   * <p>A visual string.</p>
   **/
  protected class GUIString extends Glyph {

    /**
     * <p>String color.</p>
     **/
    Color color;

    /**
     * <p>The string.</p>
     **/
    String string;

    /**
     * <p>Left x coord in world.</p>
     **/
    double x;

    /**
     * <p>Baseline y coord in world.</p>
     **/
    double y;

    /**
     * <p>The current x view coordinate.</p>
     **/
    private double xView;

    /**
     * <p>The current y view coordinate.</p>
     **/
    private double yView;

    /**
     * <p>Scale the x and y world coordinates to view coordinates.</p>
     *
     * @param sx the view x scale
     * @param sy the view y scale
     **/
    void scalePoint(double sx, double sy) {
      xView = x*sx;
      yView = y*sy;
    }

    /**
     * <p>Create a new GUIString.</p>
     **/
    GUIString(String string, double x, double y, Color color) {
      this.color = dupColor(color);
      this.string = string;
      this.x = x;
      this.y = y;
    }

    /**
     * <p>Paints the string.</p>
     *
     * @param g2d the graphics context
     **/
    public void paint(Graphics2D g2d) {
      g2d.setColor(color);
      g2d.drawString(string, (float) xView, (float) yView);
    }
  }

  /**
   * <p>All the {@link ConstructionGUI.Ellipse}s.</p>
   **/
  protected java.util.List<Ellipse> ellipses =
  Collections.synchronizedList(new ArrayList<Ellipse>());

  /**
   * <p>All the {@link ConstructionGUI.GUIString}s.</p>
   **/
  protected java.util.List<GUIString> strings =
  Collections.synchronizedList(new ArrayList<GUIString>());

  /**
   * <p>Consruct a new ConstructionGUI.</p>
   *
   * <p>See <code>GlobalNavigation.MapGUI(int, double, double)</code>.</p>
   **/
  public ConstructionGUI(int poseSaveInterval, double maxTV, double maxRV) {
    super(poseSaveInterval, maxTV, maxRV);
  }

  /**
   * <p>See <code>GlobalNavigation.MapGUI(int)</code>.</p>
   **/
  public ConstructionGUI(int poseSaveInterval) {
    super(poseSaveInterval);
  }

  /**
   * <p>See <code>GlobalNavigation.MapGUI()</code>.</p>
   **/
  public ConstructionGUI() {
    super();
  }

  /**
   * {@inheritDoc}
   *
   * <p>Default impl returns {@link #APPNAME}.</p>
   *
   * @return the title for the GUI frame
   **/
  public String getAppName() {
    return APPNAME;
  }

  /**
   * <p>Add an ellipse for display.</p>
   * 
   * @param x the llc x coord in world frame (m)
   * @param y the llc y coord in world frame (m)
   * @param width the width in world frame (m)
   * @param height the height in world frame (m)
   * @param filled whether to fill the ellipse
   * @param color the ellipse color or null to use current
   **/
  public void addEllipse(double x, double y, double width, double height,
                         boolean filled, Color color) {

    synchronized (ellipses) {

      if (color != null)
        ellipseColor = dupColor(color);

      ellipses.add(new Ellipse(x, y, width, height, filled, ellipseColor));
    }

    repaint();
  }

  /**
   * <p>Add an ellipse for display.</p>
   *
   * @param e the ellipse
   * @param filled whether to fill the ellipse
   * @param color the ellipse color or null to use current
   **/
  public void addEllipse(Ellipse2D.Double e, boolean filled, Color color) {
    synchronized (ellipses) {

      if (color != null)
        ellipseColor = dupColor(color);

      ellipses.add(new Ellipse(e, filled, ellipseColor));
    }

    repaint();
  }

  /**
   * <p>Add a string for display.</p>
   *
   * @param string the string
   * @param x the left x coord
   * @param y the baseline y coord
   * @param color the string color or null to use current
   **/
  public void addString(String string, double x, double y, Color color) {
    synchronized (strings) {

      if (color != null)
        stringColor = dupColor(color);

      strings.add(new GUIString(string, x, y, stringColor));
    }

    repaint();
  }

  /**
   * <p>Erase all previously plotted ellipses.</p>
   **/
  public void eraseEllipses() {
    ellipses.clear();
    repaint();
  }

  /**
   * <p>Erase all previously strings.</p>
   **/
  public void eraseStrings() {
    strings.clear();
    repaint();
  }

  /**
   * {@inheritDoc}
   *
   * <p>This impl {@link #paintEllipses}, {@link #paintStrings}, iff each is
   * enabled.</p>
   **/
  protected void paintInWorldOverGridUnderPosesHook(Graphics2D g2d) {

    super.paintInWorldOverGridUnderPosesHook(g2d);

    if (ellipsesEnabled)
      paintEllipses(g2d);
    
    if (stringsEnabled)
      paintStrings(g2d);
  }

  /**
   * <p>Paint all {@link #ellipses}.</p>
   *
   * @param g2d the graphics context
   **/
  protected void paintEllipses(Graphics2D g2d) {

    //avoid NPE on init
    if (ellipses == null)
      return;

    setLineWidth(g2d, ELLIPSE_LINE_WIDTH);

    synchronized (ellipses) {
	for (Iterator it = ellipses.iterator(); it.hasNext(); )
	    ((Ellipse) it.next()).paint(g2d);
    }
  }

  /**
   * <p>Paint all {@link #strings}.</p>
   *
   * @param g2d the graphics context
   **/
  protected void paintStrings(Graphics2D g2d) {

    //avoid NPE on init
    if (strings == null)
      return;

    //undo world scale

    AffineTransform transformWas = g2d.getTransform();

    double sx = transformWas.getScaleX();
    double sy = transformWas.getScaleY();

    g2d.scale(1.0/sx, 1.0/sy);

    synchronized (strings) {
      for (Iterator it = strings.iterator(); it.hasNext(); ) {
	  GUIString string = (GUIString) it.next();
	  string.scalePoint(sx, sy);
	  string.paint(g2d);
      }
    }

    //redo world scale

    g2d.setTransform(transformWas);
  }

    private Subscriber<org.ros.message.lab5_msgs.GUIEraseMsg> guiEraseSub2;
    private Subscriber<org.ros.message.Challenge_msgs.GUIEllipseMessage> guiEllipseSub;
    private Subscriber<org.ros.message.Challenge_msgs.GUIStringMessage> guiStringSub;

    public void onStart(Node node) {
	super.onStart(node);

	guiEraseSub2 = node.newSubscriber("gui/Erase", "lab5_msgs/GUIEraseMsg");
	guiEraseSub2.addMessageListener(new MessageListener<org.ros.message.lab5_msgs.GUIEraseMsg>() {
		@Override public void onNewMessage(org.ros.message.lab5_msgs.GUIEraseMsg message) {
		    eraseEllipses();
		    eraseStrings();
		}
	    }
	    );

	guiEllipseSub = node.newSubscriber("gui/Ellipse", "Challenge_msgs/GUIEllipseMessage");
	guiEllipseSub.addMessageListener(new MessageListener<org.ros.message.Challenge_msgs.GUIEllipseMessage>() {
		@Override public void onNewMessage(org.ros.message.Challenge_msgs.GUIEllipseMessage message) {
		    addEllipse((double)message.x, (double)message.y, (double)message.width, (double)message.height,
			       message.filled==1, new Color((int)message.c.r, (int)message.c.g, (int)message.c.b));
		}
	    }
	    );

	guiStringSub = node.newSubscriber("gui/String", "Challenge_msgs/GUIStringMessage");
	guiStringSub.addMessageListener(new MessageListener<org.ros.message.Challenge_msgs.GUIStringMessage>() {
		@Override public void onNewMessage(org.ros.message.Challenge_msgs.GUIStringMessage message) {
		    addString(message.text, message.x, message.y, 
			      new Color((int)message.c.r, (int)message.c.g, (int)message.c.b));
		}
	    }
	    );
    }
    
}
