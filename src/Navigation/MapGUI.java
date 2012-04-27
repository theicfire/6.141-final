package Navigation;

import java.awt.*;
import java.awt.geom.*;
import javax.swing.*;

import org.ros.message.lab5_msgs.ColorMsg;
import org.ros.message.lab6_msgs.GUIPolyMsg;
import org.ros.message.lab6_msgs.GUIRectMsg;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.namespace.GraphName;
import org.ros.node.topic.Subscriber;

import java.util.*;
import LocalNavigation.SonarGUI;


/**
 * <p>Extends <code>LocalNavigation.SonarGUI</code> to display map-related
 * data (first read the doc for that class).</p>
 *
 * <p>New methods (and corresponding ROS messages) have been added to draw
 * rectangles and polygons.</p>
 *   
 * @author vona
 **/
public class MapGUI extends SonarGUI implements NodeMain{

  /**
   * <p>The application name.</p>
   **/
  public static final String APPNAME = "MapGUI";

  /**
   * <p>Bitfield constant for {@link GUIEraseMessage}.</p>
   **/
  public static final int ERASE_RECTS = 1<<6;

  /**
   * <p>Bitfield constant for {@link GUIEraseMessage}.</p>
   **/
  public static final int ERASE_POLYS = 1<<7;

  /**
   * <p>Line width of the lines making up a {@link MapGUI.Rect} in
   * pixels.</p>
   **/
  public static final float RECT_LINE_WIDTH = 1.5f;

  /**
   * <p>Line width of the lines making up a {@link MapGUI.Poly} in
   * pixels.</p>
   **/
  public static final float POLY_LINE_WIDTH = 1.5f;

  /**
   * <p>Default color for {@link MapGUI.Rect}s.</p>
   **/
  public static final Color DEFAULT_RECT_COLOR = Color.RED;

  /**
   * <p>Default color for {@link MapGUI.Poly}s.</p>
   **/
  public static final Color DEFAULT_POLY_COLOR = Color.RED;

  /**
   * <p>Whether to paint the rects.</p>
   **/
  protected boolean rectsEnabled = true;

  /**
   * <p>Whether to paint the polys.</p>
   **/
  protected boolean polysEnabled = true;

  /**
   * <p>The current {@link MapGUI.Rect} color.</p>
   **/
  protected Color rectColor = dupColor(DEFAULT_RECT_COLOR);

  /**
   * <p>The current {@link MapGUI.Poly} color.</p>
   **/
  protected Color polyColor = dupColor(DEFAULT_POLY_COLOR);

  /**
   * <p>A visual rectangle.</p>
   **/
  protected class Rect extends Glyph {

    /**
     * <p>Rect color.</p>
     **/
    Color color;

    /**
     * <p>Whether to draw filled.</p>
     **/
    boolean filled;

    /**
     * <p>The rect in world coords.</p>
     **/
    Rectangle2D.Double r = new Rectangle2D.Double();

    /**
     * <p>Create a new rect.</p>
     **/
    Rect(Rectangle2D.Double r, boolean filled, Color color) {
      this.color = dupColor(color);
      this.filled = filled;
      this.r.setRect(r);
    }

    /**
     * <p>Create a new rect.</p>
     **/
    Rect(double x, double y, double width, double height,
         boolean filled, Color color) {

      this.color = dupColor(color);
      this.filled = filled;

      this.r.x = x;
      this.r.y = y;
      this.r.width = width;
      this.r.height = height;
    }

      public int hashCode() {
	  return 0;
      }

      @Override public boolean equals(Object other) {
	  Rect another = (Rect)other;
	  return ( color.equals(another.color) && 
		   filled == another.filled && 
		   r.equals(another.r) );
      }

    /**
     * <p>Paints the rect.</p>
     *
     * <p>Assumes line width is already set.</p>
     *
     * @param g2d the graphics context
     **/
    public void paint(Graphics2D g2d) {

      g2d.setColor(color);

      if (filled)
        g2d.fill(r);

      g2d.draw(r);
    }
  }

  /**
   * <p>A visual polygon.</p>
   **/
  protected class Poly extends Glyph {

    /**
     * <p>Poly color.</p>
     **/
    Color color;

    /**
     * <p>Whether to close the poly.</p>
     **/
    boolean closed;

    /**
     * <p>Whether to draw filled.</p>
     **/
    boolean filled;

    /**
     * <p>The poly in world coords.</p>
     **/
    GeneralPath path = new GeneralPath();

    /**
     * <p>Create a new poly.</p>
     **/
    Poly(java.util.List<Point2D.Double> vertices,
         boolean closed, boolean filled, Color color) {

      this.color = dupColor(color);
      this.closed = closed;
      this.filled = filled;

      boolean first = true;
      for (Point2D.Double vertex : vertices) {

        float x = (float) (vertex.getX());
        float y = (float) (vertex.getY());

        if (first) {
          path.moveTo(x, y);
          first = false;
        } else {
          path.lineTo(x, y);
        }
      }

      if (closed)
        path.closePath();
    }

      public int hashCode() {
	  return 0;
      }

      @Override public boolean equals(Object other) {
	  Poly another = (Poly)other;
	  PathIterator pi1 = path.getPathIterator(null);
	  PathIterator pi2 = another.path.getPathIterator(null);
	  while ( !pi1.isDone() && !pi2.isDone() ) {
	      double[] c1 = new double[6];
	      int t1 = pi1.currentSegment(c1);
	      double[] c2 = new double[6];
	      int t2 = pi2.currentSegment(c2);
	      if ( t1 != t2 ) return false;
	      if ( c1[0] != c2[0] || c1[1] != c2[1] ) return false;
	      pi1.next();
	      pi2.next();
	  }
	  if ( !(pi1.isDone() && pi2.isDone()) ) return false;

	  return ( color.equals(another.color) && 
		   closed == another.closed && 
		   filled == another.filled );
      }

    /**
     * <p>Paints the poly.</p>
     *
     * <p>Assumes line width is already set.</p>
     *
     * @param g2d the graphics context
     **/
    public void paint(Graphics2D g2d) {

      g2d.setColor(color);

      if (filled)
        g2d.fill(path);

      g2d.draw(path);
    }
  }

  /**
   * <p>All the {@link MapGUI.Rect}s.</p>
   **/
  protected java.util.Set<Rect> rects =
  Collections.synchronizedSet(new HashSet<Rect>());

  /**
   * <p>All the {@link MapGUI.Poly}s.</p>
   **/
  protected java.util.Set<Poly> polys =
  Collections.synchronizedSet(new HashSet<Poly>());

  /**
   * <p>Consruct a new MapGUI.</p>
   *
   * <p>See <code>LocalNavigation.SonarGUI(int, double, double)</code>.</p>
   **/
  public MapGUI(int poseSaveInterval, double maxTV, double maxRV) {
    super(poseSaveInterval, maxTV, maxRV);
  }

  /**
   * <p>See <code>LocalNavigation.SonarGUI(int)</code>.</p>
   **/
  public MapGUI(int poseSaveInterval) {
    super(poseSaveInterval);
  }

  /**
   * <p>See <code>LocalNavigation.SonarGUI()</code>.</p>
   **/
  public MapGUI() {
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
   * <p>Add a rect for display.</p>
   * 
   * @param x the llc x coord in world frame (m)
   * @param y the llc y coord in world frame (m)
   * @param width the width in world frame (m)
   * @param height the height in world frame (m)
   * @param filled whether to fill the rect
   * @param color the rect color or null to use current
   **/
  public void addRect(double x, double y, double width, double height,
                      boolean filled, Color color) {

    synchronized (rects) {

      if (color != null)
        rectColor = dupColor(color);

      rects.add(new Rect(x, y, width, height, filled, rectColor));
    }

    repaint();
  }

  /**
   * <p>Add a rect for display.</p>
   *
   * @param r the rect
   * @param filled whether to fill the rect
   * @param color the rect color or null to use current
   **/
  public void addRect(Rectangle2D.Double r, boolean filled, Color color) {
    synchronized (rects) {

      if (color != null)
        rectColor = dupColor(color);

      rects.add(new Rect(r, filled, rectColor));
    }

    repaint();
  }

  /**
   * <p>Add a polygon for display.</p>
   *
   * @param vertices the vertices in ccw order
   * @param closed whether to close the poly
   * @param filled whether to fill the poly
   * @param color the poly color or null to use current
   **/
  public void addPoly(java.util.List<Point2D.Double> vertices,
                      boolean closed, boolean filled, Color color) {
    synchronized (polys) {

      if (color != null)
        polyColor = dupColor(color);

      if ( polys.add(new Poly(vertices, closed, filled, polyColor)) ) {

	  //System.err.println("added poly with " + vertices.size() + " verts");
	  for (Point2D.Double vertex : vertices)
	      System.err.println("  " + vertex);
      }
    }

    repaint();
  }

  /**
   * <p>Erase all previously plotted rects.</p>
   **/
  public void eraseRects() {
    rects.clear();
    repaint();
  }

  /**
   * <p>Erase all previously polys.</p>
   **/
  public void erasePolys() {
    polys.clear();
    repaint();
  }

  /**
   * {@inheritDoc}
   *
   * <p>This impl {@link #paintRects}, {@link #paintPolys}, iff each is
   * enabled.</p>
   **/
  protected void paintInWorldOverGridUnderPosesHook(Graphics2D g2d) {

    super.paintInWorldOverGridUnderPosesHook(g2d);

    if (rectsEnabled)
      paintRects(g2d);
    
    if (polysEnabled)
      paintPolys(g2d);
  }

  /**
   * <p>Paint all {@link #rects}.</p>
   *
   * @param g2d the graphics context
   **/
  protected void paintRects(Graphics2D g2d) {

    //avoid NPE on init
    if (rects == null)
      return;

    setLineWidth(g2d, RECT_LINE_WIDTH);

    synchronized (rects) {
      for (Iterator it = rects.iterator(); it.hasNext(); )
        ((Rect) it.next()).paint(g2d);
    }
  }

  /**
   * <p>Paint all {@link #polys}.</p>
   *
   * @param g2d the graphics context
   **/
  protected void paintPolys(Graphics2D g2d) {

    //avoid NPE on init
    if (polys == null)
      return;

    setLineWidth(g2d, POLY_LINE_WIDTH);

    synchronized (polys) {

      for (Iterator it = polys.iterator(); it.hasNext(); )
        ((Poly) it.next()).paint(g2d);
    }
  }

    private Subscriber<org.ros.message.lab6_msgs.GUIRectMsg> guiRectSub;
    private Subscriber<org.ros.message.lab6_msgs.GUIPolyMsg> guiPolySub;
    private Subscriber<org.ros.message.lab5_msgs.GUIEraseMsg> guiEraseSub;

    /**
     * Hook called by ROS to start the gui
     **/
    public void onStart(Node node) {
	guiRectSub = node.newSubscriber("gui/Rect", "lab6_msgs/GUIRectMsg");
	guiRectSub.addMessageListener(new RectMessageListener(this));
	guiPolySub = node.newSubscriber("gui/Poly", "lab6_msgs/GUIPolyMsg");
	guiPolySub.addMessageListener(new PolyMessageListener(this));
	guiEraseSub = node.newSubscriber("gui/Erase", "lab5_msgs/GUIEraseMsg");
	guiEraseSub.addMessageListener(new EraseMessageListener(this));
	super.onStart(node);
    }
    
  
  /**
   * {@inheritDoc}
   *
   * <p>This impl tests the map graphics.</p>
   **/
  public void testGraphicsHook() throws InterruptedException {

    super.testGraphicsHook();

    //TBD
  }
  

	public static void fillRectMsg(GUIRectMsg gpm,
			java.awt.geom.Rectangle2D.Double worldRect, Object object, boolean b) {
		gpm.x = (float) worldRect.x;
      gpm.y = (float) worldRect.y;
      gpm.height = (float) worldRect.height;
      gpm.width = (float) worldRect.width;
      gpm.filled = 0;
      ColorMsg colorMsg = new ColorMsg();
      colorMsg.r = 0;
      colorMsg.g = 0;
      colorMsg.b = 0;
      gpm.c = colorMsg;
	}

	public static void fillPolyMsg(GUIPolyMsg polyMsg,
			PolygonObstacle obstacle, Color makeRandomColor) {
		float[] xCoords = new float[obstacle.getVertices().size()];
		float[] yCoords = new float[obstacle.getVertices().size()];
		for (int i = 0; i < obstacle.getVertices().size(); i++) {
			java.awt.geom.Point2D.Double vert = obstacle.getVertices().get(i);
			xCoords[i] = (float) (vert.x);
			yCoords[i] = (float) (vert.y);
		}
		polyMsg.x = xCoords;
		polyMsg.y = yCoords;
		polyMsg.filled = 0;
		polyMsg.numVertices = obstacle.getVertices().size();
		polyMsg.closed = 1;
		ColorMsg colorMsg = new ColorMsg();
      colorMsg.r = 0;
      colorMsg.g = 0;
      colorMsg.b = 0;
      polyMsg.c = colorMsg;
		
	}
  
}
