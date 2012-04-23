package Navigation;

import java.awt.geom.*;
import java.io.*;
import java.util.*;
import java.text.*;

import org.ros.message.lab5_msgs.GUIEraseMsg;
import org.ros.message.lab6_msgs.GUIPolyMsg;
import org.ros.message.lab6_msgs.GUIRectMsg;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.namespace.GraphName;
import org.apache.commons.logging.Log;


import VisualServo.*;

/**
 * <p>The 2D {@link #worldRect}, {@link PolygonObstacle} {@link #obstacles},
 * {@link #robotStart}, and {@link #robotGoal} that
 * make-up the environment in which the robot will navigate.</p>
 *
 * <p>You can either make an instance of this class and use it from within your
 * own program (typical usage), or you can run this code as an indepedent
 * process (see {@link #main}).  The latter mode allows you to display the
 * contents of a map file, but does not give programmatic access to those
 * contents, and thus is useful mostly for displaying/debugging map
 * files.</p>.
 *
 * <p>The format of the map file is as follows (all items are signed doubles in
 * ASCII text format, all units are meters, and all separators are whitespace):
 * <pre>
 * robotStartX robotStartY
 * robotGoalX robotGoalY
 * worldX worldY worldWidth worldHeight
 * obstacle0X0 obstacle0Y0 obstacle0X1 obstacle0Y1 ...
 * ...
 * </pre>
 * The first three lines initialize the corresponding instance fields {@link #robotStart}, {@link #robotGoal}, and {@link
 * #worldRect}, respectively.  Each obstacle line gives the coordinates of the
 * obstacle's vertices in CCW order.  There may be zero obstacles.</p>
 *
 * @author Marty Vona
 * @author Aisha Walcott
 * 
 * @author Kenny Donahue (minor edits 03/11/11)
 * @author Dylan Hadfield-Menell (port to ROS 01/12)
 **/
public class PolygonMap implements NodeMain{

	public Log log;
	/**
	 * <p>Name to use when run as an application.</p>
	 **/
	public static final String APPNAME = "PolygonMap";

	/**
	 * <p>The start point for the robot origin, read in from the map file
	 * (m).</p>
	 **/
	protected Point2D.Double robotStart = new Point2D.Double();

	/**
	 * <p>The goal point for the robot origin, read in from the map file (m).</p>
	 **/
	protected Point2D.Double robotGoal = new Point2D.Double();

	/**
	 * <p>The location and size of the world boundary, read in from the map file
	 * (m).</p>
	 **/
	protected Rectangle2D.Double worldRect = new Rectangle2D.Double();

	/**
	 * <p>The obstacles (does not include the world boundary).</p>
	 **/
	protected LinkedList<PolygonObstacle> obstacles =
			new LinkedList<PolygonObstacle>();

	private Publisher<Object> erasePub;

	private Publisher<Object> rectPub;

	private Publisher<Object> polyPub;

	private String mapFile = "/home/rss-staff/ros/rss/solutions/lab6/src/global-nav-maze-2011-basic.map";

	/**
	 * <p>Create a new map, parsing <code>mapFile</code>.</p>
	 *
	 * @param mapFile the map file to parse, or null if none
	 **/
	public PolygonMap(File mapFile) throws IOException, ParseException {
		if (mapFile != null)
			parse(mapFile);
	}

	/**
	 * <p>Covers {@link #PolygonMap(File)}.</p>
	 **/
	public PolygonMap(String mapFile) throws IOException, ParseException {
		this((mapFile != null) ? new File(mapFile) : null);
	}

	/**
	 * <p>Create a new un-initialized polygon map.</p>
	 *
	 * <p>You may populate it using the accessors below.</p>
	 **/
	public PolygonMap() {
	}

	/**
	 * <p>Parse a <code>double</code>.</p> 
	 *
	 * @param br the double is expected to be on the next line of this reader
	 * @param name the name of the double
	 * @param lineNumber the line number of the double
	 *
	 * @return the parsed double
	 *
	 * @exception IOException if there was an I/O error
	 * @exception ParseException if there was a format error
	 * @exception NumberFormatException if there was a format error
	 **/
	protected double parseDouble(BufferedReader br, String name, int lineNumber) 
			throws IOException, ParseException, NumberFormatException {

		String line = br.readLine();
		if (line == null)
			throw new ParseException(name + " (line " + lineNumber + ")",
					lineNumber);

		return Double.parseDouble(line);
	}

	/**
	 * <p>Parse a <code>Point2D.Double</code>.</p> 
	 *
	 * @param point the returned point
	 * @param br the point is expected to be on the next line of this reader
	 * @param name the name of the point
	 * @param lineNumber the line number of the point
	 *
	 * @exception IOException if there was an I/O error
	 * @exception ParseException if there was a format error
	 * @exception NumberFormatException if there was a format error
	 **/
	protected void parsePoint(Point2D.Double point,
			BufferedReader br, String name, int lineNumber) 
					throws IOException, ParseException, NumberFormatException {

		String line = br.readLine();
		String[] tok = (line != null) ? line.split("\\s+") : null;

		if ((tok == null) || (tok.length < 2)){
			throw new ParseException(name + " (line " + lineNumber + ")",
					lineNumber);
		}

		point.x = Double.parseDouble(tok[0]);
		point.y = Double.parseDouble(tok[1]);
	}

	/**
	 * <p>Parse a <code>Rectangle2D.Double</code>.</p> 
	 *
	 * @param rect the returned rectangle
	 * @param br the rect is expected to be on the next line of this reader
	 * @param name the name of the rect
	 * @param lineNumber the line number of the rect
	 *
	 * @exception IOException if there was an I/O error
	 * @exception ParseException if there was a format error
	 * @exception NumberFormatException if there was a format error
	 **/
	protected void parseRect(Rectangle2D.Double rect,
			BufferedReader br, String name, int lineNumber) 
					throws IOException, ParseException, NumberFormatException {

		String line = br.readLine();
		String[] tok = (line != null) ? line.split("\\s+") : null;

		if ((tok == null) || (tok.length < 4))
			throw new ParseException(name + " (line " + lineNumber + ")",
					lineNumber);

		rect.x = Double.parseDouble(tok[0]);
		rect.y = Double.parseDouble(tok[1]);
		rect.width = Double.parseDouble(tok[2]);
		rect.height = Double.parseDouble(tok[3]);
	}

	/**
	 * <p>Parse a {@link PolygonObstacle}.</p>
	 *
	 * @param br the polygon is expected to be on the next line of this reader
	 * @param name the name of the polygon
	 * @param lineNumber the line number of the polygon
	 *
	 * @return a new polygon containing the vertices on the line, or null if
	 * there was no line
	 *
	 * @exception IOException if there was an I/O error
	 * @exception ParseException if there was a format error
	 * @exception NumberFormatException if there was a format error
	 **/
	protected PolygonObstacle parseObs(BufferedReader br,
			String name, int lineNumber) 
					throws IOException, ParseException, NumberFormatException {

		String line = br.readLine();

		if (line == null)
			return null;

		String[] tok = line.trim().split("\\s+");

		if (tok.length == 0)
			return null;

		//    System.err.println(
		//      "line " + lineNumber + " (" + tok.length + " tokens): ");
		//    for (int i = 0; i < tok.length; i++)
		//      System.err.println("  " + tok[i]);

		if (tok.length%2 != 0)
			throw new ParseException(name + " (line " + lineNumber + ")",
					lineNumber);

		PolygonObstacle poly = new PolygonObstacle();

		for (int i = 0; i < tok.length/2; i++)
			poly.addVertex(Double.parseDouble(tok[2*i]),
					Double.parseDouble(tok[2*i+1]));

		poly.close();

		return poly;
	}

	/**
	 * <p>Hook called from {@link #parse} after the first four lines are parsed
	 * to allow subclasses to parse extra lines before the obstacles.</p>
	 *
	 * <p>Default impl just returns false.</p>
	 *
	 * @param br the next line of this reader is the next to be parsed
	 * @param lineNumber the line number of the next line of br
	 *
	 * @return true if the line was parsed and more are expected, false if no
	 * more extra lines are expected
	 *
	 * @exception IOException if there was an I/O error
	 * @exception ParseException if there was a format error
	 * @exception NumberFormatException if there was a format error
	 **/
	protected boolean parseExtra(BufferedReader br, int lineNumber)
			throws IOException, ParseException, NumberFormatException {
		return false;
	}

	/**
	 * <p>Parse <code>mapFile</code>.</p>
	 *
	 * <p>Format is specified in the class header doc for {@link PolygonMap}.</p>
	 *
	 * @param mapFile the map file, not null
	 **/
	protected void parse(File mapFile) throws IOException, ParseException {
		log.info("PARSING MAP FILE");
	    int lineNumber = 1;
	    try {
		
		BufferedReader br = new BufferedReader(new FileReader(mapFile));
		
		parsePoint(robotStart, br, "robot start", lineNumber++);
		parsePoint(robotGoal, br, "robot goal", lineNumber++);
		parseRect(worldRect, br, "world rect", lineNumber++);
		
		while (parseExtra(br, lineNumber++))
		    ;
		
		for (int obstacleNumber = 0; ; obstacleNumber++) {
		    
		    PolygonObstacle poly = parseObs(br, "obstacle " + obstacleNumber,
						    lineNumber++);
		    if (poly != null) {
			poly.color = MapGUI.makeRandomColor();
			obstacles.add(poly);
		    }
		    else
			break;
		}
		
	    } catch (NumberFormatException e) {
		throw new ParseException("malformed number on line " + lineNumber,
					 lineNumber);
	    }
	}

	/**
	 * <p>Get {@link #robotStart}.</p>
	 *
	 * @return a reference to <code>robotStart</code> (you may modify it)
	 **/
	public Point2D.Double getRobotStart() {
		return robotStart;
	}

	/**
	 * <p>Get {@link #robotGoal}.</p>
	 *
	 * @return a reference to <code>robotGoal</code> (you may modify it)
	 **/
	public Point2D.Double getRobotGoal() {
		return robotGoal;
	}

	/**
	 * <p>Get {@link #worldRect}.</p>
	 *
	 * @return a reference to <code>worldRect</code> (you may modify it)
	 **/
	public Rectangle2D.Double getWorldRect() {
		return worldRect;
	}

	/**
	 * <p>Get {@link #obstacles}.</p>
	 *
	 * @return a reference to <code>obstacles</code> (you may modify it)
	 **/
	public List<PolygonObstacle> getObstacles() {
		return obstacles;
	}

	/**
	 * <p>Return a human-readable string representation of this map.</p>
	 *
	 * @return a human-readable string representation of this map
	 **/
	public String toString() {

		StringBuffer sb = new StringBuffer();

		sb.append("\nrobot start: ");
		sb.append(Double.toString(robotStart.x));
		sb.append(", ");
		sb.append(Double.toString(robotStart.y));

		sb.append("\nrobot goal: ");
		sb.append(Double.toString(robotGoal.x));
		sb.append(", ");
		sb.append(Double.toString(robotGoal.y));

		sb.append("\nworld rect: x=");
		sb.append(Double.toString(worldRect.x));
		sb.append(" y=");
		sb.append(Double.toString(worldRect.y));
		sb.append(" width=");
		sb.append(Double.toString(worldRect.width));
		sb.append(" height=");
		sb.append(Double.toString(worldRect.height));

		sb.append("\n" + obstacles.size() + " obstacles:");
		for (PolygonObstacle obstacle : obstacles) {
			sb.append("\n ");
			obstacle.toStringBuffer(sb);
		}

		return sb.toString();
	}


	/**
	 * <p>Parses the specified map file, prints it out, and displays it in the
	 * GUI, if any.</p>
	 *
	 * <p>Usage: {@link #getAppName} &lt;mapfile&gt; [centralhost]</p>
	 **/
	protected void instanceMain(String mapFile) {

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		try {

			parse(new File(mapFile));

			System.out.println(toString());

			erasePub.publish(new GUIEraseMsg ());
			
			GUIRectMsg rectMsg = new GUIRectMsg();
			MapGUI.fillRectMsg(rectMsg, getWorldRect(), null, false);
			rectPub.publish(rectMsg);
			GUIPolyMsg polyMsg = new GUIPolyMsg();
			for (PolygonObstacle obstacle : getObstacles()){
				polyMsg = new GUIPolyMsg();
				MapGUI.fillPolyMsg(polyMsg, obstacle, MapGUI.makeRandomColor());
				polyPub.publish(polyMsg);
			}

			driverDisplayHook();

		} catch (IOException e) {
			System.err.println("I/O error: " + e);
		} catch (ParseException e) {
			System.err.println("Parse error: " + e);
		}
	}

	/**
	 * <p>Get the application name.</p>
	 *
	 * @return the application name
	 **/
	protected String getAppName() {
		return APPNAME;
	}

	/**
	 * <p>Hook called after painting map objects in {@link #instanceMain} to
	 * allow subclasses to display extra stuff.</p>
	 *
	 * <p>Default impl does nothing.</p>
	 **/
	protected void driverDisplayHook() {
	}
	
	/**
	 * Shutdown hook for ROS when called as stand-alone node
	 */
	@Override
	public void onShutdown(Node node) {
		// TODO Auto-generated method stub

	}

	/**
	 * Entry hook for ROS when called as stand-alone node
	 */
	@Override
	public void onStart(Node node) {
		log = node.getLog();
		log.info("poly starting!!");
		erasePub = node.newPublisher("gui/Erase", "lab5_msgs/GUIEraseMsg");
		rectPub = node.newPublisher("gui/Rect", "lab6_msgs/GUIRectMsg");
		polyPub = node.newPublisher("gui/Poly", "lab6_msgs/GUIPolyMsg");
		this.instanceMain(mapFile);
	}
	
	public void makeLogger(Node node) {
		log = node.getLog();
	}


    @Override public void onShutdownComplete(Node node) {
    }

    @Override public GraphName getDefaultNodeName() {
	return new GraphName("rss/polygonmap");
    }
}
