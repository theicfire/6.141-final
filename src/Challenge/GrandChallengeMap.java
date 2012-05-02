package Challenge;

import java.util.*;
import java.awt.Color;
import java.awt.geom.Rectangle2D;
import java.awt.Dimension;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.text.ParseException;
import java.awt.geom.Point2D;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.namespace.GraphName;
import org.ros.message.lab5_msgs.GUIEraseMsg;
import org.ros.message.lab5_msgs.GUIPointMsg;
import org.ros.message.lab6_msgs.GUIPolyMsg;
import org.ros.message.lab6_msgs.GUIRectMsg;
import org.ros.message.Challenge_msgs.GUIEllipseMessage;
import org.ros.message.Challenge_msgs.GUIStringMessage;
import Navigation.PolygonObstacle;
import org.ros.node.parameter.ParameterTree;

public class GrandChallengeMap implements NodeMain {
    
	/**
	   * <p>Toggles on/off debugging messages printed by {@link DEBUG}</p>
	   **/
	private static final boolean DEBUGGING_ON = false;
	
	/**
	   * <p>Debugging utility.  Can toggle on/off debugging messages by setting {@link #DEBUGGING_ON}</p>
	   *
	   * @param msg the String to be displayed
	   **/
    private static void DEBUG(String msg){
    	if(DEBUGGING_ON){
    		System.out.println(msg);
    	}
    }
	
    /**
     * <p> array of ConstructionObjects parsed from the file</p>
     */
    public ConstructionObject[] constructionObjects;
    /**
     * <p> array of fiducials parsed from the file</p>
     */
    public Fiducial[] fiducials;
    /**
     * <p> array of obstacles parsed from the file</p>
     */
    public PolygonObstacle[] obstacles;
    /**
     * <p> bounding box rectangle of the world.</p>
     */
    private Rectangle2D.Double worldRect;

    public Point2D.Double robotStart;
    public Point2D.Double robotGoal;

    //These are a few things that are used a lot in the file parsing, so they have
    //Their own variables defined.  Most strings associated with file parsing live in
    //the corresponding parse method
    //NOTE: These strings must be only one character long
    public static final String COMMENT = "#";
    public static final String SECTION_START = "{";
    public static final String SECTION_END = "}";
    
    /**
     * <p>Generates a GrandChallengeMap from a file.  Only real way to make a GCM.</p>
     * <p>File is expected to be of form map{ bottom_left{...} top_right{...} fiducials{...} construction_objects{...} obstacles{...}}
     *
     * @param fileName the name of the file 
     **/
    public static GrandChallengeMap parseFile(String fileName) throws IOException, ParseException{
    	DEBUG("In GrandChallenge");
    	GrandChallengeMap m = new GrandChallengeMap();
    	BufferedReader br = new BufferedReader(new FileReader(fileName));
    	
    	parseToken(br,"map");
    	parseToken(br,SECTION_START);
    	
    	DEBUG("Starting map-parse loop");
    	Point2D.Double bottom_left = null;
    	Point2D.Double top_right = null;
    	while(true){
    		String token = parseToken(br);
    		if(token.equals(SECTION_END)){
    			break;
    		}
    		else if(token.equals("fiducials")){
    			m.fiducials = parseFiducials(br);
    		}
    		else if(token.equals("construction_objects")){
    			m.constructionObjects = parseConstructionObjects(br);
    		}
    		else if(token.equals("obstacles")){
    			m.obstacles = parsePolygonObstacles(br);
    		}
    		else if(token.equals("bottom_left")){
    			bottom_left = parsePoint2D(br);
    		}
    		else if(token.equals("top_right")){
    			top_right = parsePoint2D(br);
    		} 
		else if(token.equals("robot_start")){
    			m.robotStart = parsePoint2D(br);
    		}
    		else if(token.equals("robot_goal")){
    			m.robotGoal = parsePoint2D(br);
    		}
    		else{
    			throw new ParseException("Unknown section token '"+token+"' in file.",0);
    		}
    	}
    	if(bottom_left!=null && top_right!=null){
    		m.worldRect = new Rectangle2D.Double(bottom_left.getX(),
						     bottom_left.getY(),
						     top_right.getX()-bottom_left.getX(),
						     top_right.getY()-bottom_left.getY());
    	}
	else{
	    throw new ParseException("Could not define the world rectangle.  Missing either bottom_left or top_right.",0);
	}
    	//Don't forget to close the file!
    	br.close();
    	DEBUG("Done parsing map");
    	return m;
    }
    
    /**
     * <p>Parses the fiducials section of the map file.</p>
     * <p>Expect section to be of format { num_fiducials x 0 {} 1 {} 2 {} ... } where each subsection is a fiducial</p>
     * 
     * @param br BufferedReader, lets us know where we are in the file
     * @return List<Fiducial> list of fiducials in the map file
     * @throws IOException
     * @throws ParseException
     */
    private static Fiducial[] parseFiducials(BufferedReader br) throws IOException, ParseException{
    	DEBUG("Parsing Fiducial List");
    	
    	parseToken(br,SECTION_START);
    	parseToken(br,"num_fiducials");
    	int numFiducials = parseInt(br);
    	Fiducial[] fiducials = new Fiducial[numFiducials];
    	
    	for(int i=0;i<numFiducials;i++){
    		int index = parseInt(br);
    		if(index<0 || index>=numFiducials){
    			throw new ParseException("Fiducial Index out of range: Expected [0-"+(numFiducials-1)+"].  Got '"+index+"'.",0);
    		}
    		fiducials[index] = parseFiducial(br);
    	}
    	parseToken(br,SECTION_END);

    	DEBUG("Done parsing Fiducial List");
    	return fiducials;
    }
    
    /**
     * <p>Parses a single fiducial from the map file.</p>
     * <p>Expects section to be of format { position{...} top_color{...} bottom_color{...} top_radius x1 bottom_radius x2}</p> 
     * @param br BufferedReader, lets us know where we are in the file
     * @return Fiducial current fiducial we wanted to parse
     * @throws IOException
     * @throws ParseException
     */
    private static Fiducial parseFiducial(BufferedReader br) throws IOException, ParseException{
    	DEBUG("Parsing Fiducial");
    	parseToken(br,SECTION_START);
    	Fiducial fiducial = new Fiducial();
    	while(true){
    		String token = parseToken(br);
    		if(token.equals("position")){
    			fiducial.setPosition(parsePoint2D(br));
    		}
    		else if(token.equals("top_color")){
    			fiducial.setTopColor(parseColor(br));
    		}
    		else if(token.equals("bottom_color")){
    			fiducial.setBottomColor(parseColor(br));
    		}
    		else if(token.equals("top_radius")){
    			fiducial.setTopSize(parseDouble(br));
    		}
    		else if(token.equals("bottom_radius")){
    			fiducial.setBottomSize(parseDouble(br));
    		}
    		else if(token.equals(SECTION_END)){
    			break;
    		}
    		else{
    			throw new ParseException("Unknown token in Fiducial: '"+token+"'",0);
    		}
    	}
    	DEBUG("Done parsing Fiducial");
    	return fiducial;
    }
    
    /**
     * <p>Parses the ConstructionObjects section of the map file</p>
     * <p>Expects section to be of format { num_constructionObjects x 0 {} 1 {} 2 {} ...} where each subsection is a block</p>
     * @param br BufferedReader, lets us know where we are in the file
     * @return List<ConstructionObjects> list of ConstructionObjects in the map file
     * @throws IOException
     * @throws ParseException
     */
    private static ConstructionObject[] parseConstructionObjects(BufferedReader br) throws IOException, ParseException{
    	DEBUG("Parsing ConstructionObject List");
    	
    	parseToken(br,SECTION_START);
    	parseToken(br,"num_construction_objects");
    	int numConstructionObjects = parseInt(br);
    	ConstructionObject[] constructionObjects = new ConstructionObject[numConstructionObjects];
    	for (int i=0;i<numConstructionObjects;i++){
    		int index = parseInt(br);
    		if(index<0 || index>=numConstructionObjects){
    			throw new ParseException("ConstructionObject Index out of range: Expected [0-"+(numConstructionObjects-1)+"].  Got '"+index+"'.",0);
    		}
    		constructionObjects[index] = parseConstructionObject(br);
    	}
    	parseToken(br,SECTION_END);
    	DEBUG("Done parsing ConstructionObject List");
    	return constructionObjects;
    }
    
    /**
     * <p>Parses a single ConstructionObject from the map file</p>
     * <p>Expects section to be of format {position{...} color{...} size x}</p>
     * @param br BufferedReader, lets us know where we are in the file
     * @return ConstructionObject current block we wanted to parse
     * @throws IOException
     * @throws ParseException
     */
    private static ConstructionObject parseConstructionObject(BufferedReader br) throws IOException, ParseException{
    	DEBUG("Parsing ConstructionObject");
    	
    	parseToken(br,SECTION_START);
    	ConstructionObject block = new ConstructionObject();
    	while(true){
    		String token = parseToken(br);
    		if(token.equals("position")){
    			block.setPosition(parsePoint2D(br));
    		}
    		else if(token.equals("color")){
    			block.setColor(parseColor(br));
    		}
    		else if(token.equals("size")){
    			block.setSize(parseInt(br));
    		}
    		else if(token.equals(SECTION_END)){
    			break;
    		}
    		else{
    			throw new ParseException("Unknown token in ConstructionObject: '"+token+"'",0);
    		}
    	}
    	DEBUG("Done parsing ConstructionObject");
    	return block;
    }
    
    /**
     * <p>Parses the PolygonObstacle Section of the map file</p>
     * <p>Expects section to be of format { num_obstacles x 0 {} 1 {} 2 {} ...} where each subsection is a PolygonObstacle</p>
     * @param br BufferedReader, lets us know where we are in the file
     * @return List<PolygonObstacle> list of obstacles from the map file
     * @throws IOException
     * @throws ParseException
     */
    private static PolygonObstacle[] parsePolygonObstacles(BufferedReader br) throws IOException, ParseException{
    	DEBUG("Parsing Obstacle List");
    	parseToken(br,SECTION_START);
    	parseToken(br,"num_obstacles");
    	int numObstacles = parseInt(br);
    	PolygonObstacle[] obstacles = new PolygonObstacle[numObstacles];
    	for(int i=0;i<numObstacles;i++){
    		int index = parseInt(br);
    		if(index<0 || index>=numObstacles){
    			throw new ParseException("Obstacle Index out of range: Expected [0-"+(numObstacles-1)+"].  Got '"+index+"'.",0);
    		}
    		obstacles[index] = parsePolygonObstacle(br);
    	}
    	DEBUG("Done parsing Obstacle List");
    	return obstacles;
    }
    
    /**
     * <p>Parses a single PolygonObstacle from the map file</p>
     * <p>Expects section to be of format { num_points x {} {} {} ... } where each subsection is a PolygonObstacle and the number of PolygonObstacles matches num_points</p>
     * @param br BufferedReader, lets us know where we are in the file
     * @return PolygonObstacle we were trying to parse
     * @throws IOException
     * @throws ParseException
     */
    private static PolygonObstacle parsePolygonObstacle(BufferedReader br) throws IOException, ParseException{
    	DEBUG("Parsing Obstacle");
    	
    	parseToken(br,SECTION_START);
    	parseToken(br,"num_points");
    	int numPoints = parseInt(br);
    	if(numPoints<3){
    		throw new ParseException("Cannot have fewer than 3 points in obstacles.  This one only has "+numPoints+" points.",0);
    	}
    	
    	PolygonObstacle po = new PolygonObstacle();
    	
    	for (int i=0;i<numPoints;i++){
    		parseInt(br);//Don't need to store the index.
    		Point2D.Double v = parsePoint2D(br);
    		po.addVertex(v.getX(), v.getY());
    	}
    	parseToken(br,SECTION_END);
    	
    	po.close();
    	DEBUG("Done parsing Obstacle");
    	return po;
    }
    
    /**
     * <p>Parses length=n vector from the map file</p>
     * <p>Expects vector to be of format { x y z ... } </p>
     * <p>Should use {@link GrandChallengeMap.parsePoint2D} if applicable instead</p>
     * @param br
     * @return double[n] vector we wanted parsed
     * @throws IOException
     * @throws ParseException
     */
    private static double[] parseVectorNd(BufferedReader br, int n) throws IOException, ParseException {
    	if(n<1){
    		throw new ParseException("Tried to parse a Vector of length "+n+".",0);
    	}
    	double[] result = new double[n];
    	parseToken(br,SECTION_START);
    	for(int i=0;i<n;i++){
    		result[i] = parseDouble(br);
    	}
    	parseToken(br,SECTION_END);
    	return result;
    }
    
    /**
     * <p>Parses length=2 vector from the map file</p>
     * <p>Expects vector to be of format { x y } </p>
     * @param br
     * @return double[2] vector we wanted parsed
     * @throws IOException
     * @throws ParseException
     */
    private static Point2D.Double parsePoint2D(BufferedReader br) throws IOException, ParseException {
    	DEBUG("Parsing vector { x y }");
    	double[] vec = parseVectorNd(br,2);
	return new Point2D.Double(vec[0],vec[1]);
    }
    
    /**
     * <p>Parses a double from the map file</p>
     * <p> basically just a wrapper around the string->double conversion.</p>
     * @param br BufferedReader, lets us know where we are in the file
     * @return the double
     * @throws IOException
     * @throws ParseException
     */
    private static double parseDouble(BufferedReader br) throws IOException, ParseException {
    	DEBUG("Parsing Double");
    	String token = parseToken(br);
    	
    	try{
    		return Double.parseDouble(token);
    	}catch(NumberFormatException nfe){
    		throw new ParseException("Expected Double.  Got '"+token+"'.",0);
    	}
    }

    /**
     * <p>Parses a color-string from the map file</p>
     * @param br BufferedReader, lets us know where we are in the file
     * @return the corresponding color
     * @throws IOException
     * @throws ParseException
     */
    private static Color parseColor(BufferedReader br) throws IOException, ParseException {
	DEBUG("Parsing Color");
	String token = parseToken(br).toLowerCase();

	if(token.equals("red")){
	    return Color.RED;
	}
	else if(token.equals("blue")){
	    return Color.BLUE;
	}
	else if(token.equals("yellow")){
	    return Color.YELLOW;
	}
	else if(token.equals("orange")){
	    return Color.ORANGE;
	}
	else if(token.equals("green")){
	    return Color.GREEN;
	}
	throw new ParseException("Unable to parse color with token '"+token+"'.",0);
    }
    
    /**
     * <p>Parses an int from the map file</p>
     * <p> basically just a wrapper around the string->int conversion.</p>
     * @param br BufferedReader, lets us know where we are in the file
     * @return the int
     * @throws IOException
     * @throws ParseException
     */
    private static int parseInt(BufferedReader br) throws IOException, ParseException {
    	DEBUG("Parsing Int");
    	String token = parseToken(br);
    	try{
    		return Integer.parseInt(token);
    	}catch(NumberFormatException nfe){
    		throw new ParseException("Expected Integer.  Got '"+token+"'.",0);
    	}
    }
    
    /**
     * <p>Parses a token from the file.  Tokens are continuous non-whitespace characters</p>
     * <p>Ignores text that falls after {@link COMMENT} symbol</p>
     * @param br
     * @return
     * @throws IOException
     * @throws ParseException
     */
    private static String parseToken(BufferedReader br) throws IOException, ParseException {
    	DEBUG("Parsing Token");
    	String result = "";
    	boolean leadingWhitespace = true;
    	while(true){
    		int c = br.read();
    		String s = Character.toString((char)c);
    		if(s.equals(COMMENT)){
    			br.readLine();
    			continue;
    		}
    		else if(Character.isWhitespace(c)){
    			if(!leadingWhitespace){
    				break;
    			}
    		}
    		else{
    			leadingWhitespace = false;
    			result+=s;
    		}
    	}
    	DEBUG("Done parsing Token: "+result);
    	return result;
    }
    
    private static String parseToken(BufferedReader br, String expected) throws IOException, ParseException {
    	String token = parseToken(br);
    	if(!token.equals(expected)){
    		throw new ParseException("Expected token '"+expected+"'.  Got '"+token+"' instead.",0);
    	}
    	return token;
    }

    protected Publisher<Object> ellipsePub;
    protected Publisher<Object> stringPub;
    protected Publisher<Object> pointPub;
    protected Publisher<Object> rectPub;
    private Publisher<Object> erasePub;
    private Publisher<Object> polyPub;
    
    protected void publishEllipse(double x, double y, double w, double h, Color color) {
	GUIEllipseMessage ellipseMsg = new GUIEllipseMessage();
	ellipseMsg.x = (float)x;
	ellipseMsg.y = (float)y;
	ellipseMsg.width = (float)w;
	ellipseMsg.height = (float)h;
	ellipseMsg.filled = 1;
	ellipseMsg.c.r = color.getRed();
	ellipseMsg.c.g = color.getGreen();
	ellipseMsg.c.b = color.getBlue();
	ellipsePub.publish(ellipseMsg);
    }

    protected void publishString(double x, double y, String s) {
	GUIStringMessage msg = new GUIStringMessage();
	msg.x = (float)x;
	msg.y = (float)y;
	msg.text = s;
	stringPub.publish(msg);
    }

    protected void publishPoint(double x, double y, int shape, Color color) {
	GUIPointMsg msg = new GUIPointMsg();
	msg.x = x;
	msg.y = y;
	msg.shape = shape;
	msg.color.r = color.getRed();
	msg.color.g = color.getGreen();
	msg.color.b = color.getBlue();
	pointPub.publish(msg);
    }

    protected void publishRect(Rectangle2D.Double r, boolean filled, Color c) {
	GUIRectMsg msg = new GUIRectMsg();
	msg.filled = filled ? 1 : 0;
	if ( c == null ) {
	    c = Color.GREEN;
	}
	msg.c.r = c.getRed();
	msg.c.b = c.getBlue();
	msg.c.g = c.getGreen();
	msg.height = (float) r.height;
	msg.width = (float) r.width;
	msg.x = (float) r.x;
	msg.y = (float) r.y;
	rectPub.publish(msg);
    }

    protected void publishPoly(PolygonObstacle obstacle, Color c, boolean filled, boolean closed) {
	List<Point2D.Double> vertices = obstacle.getVertices();
	GUIPolyMsg msg = new GUIPolyMsg();
	msg.numVertices = vertices.size();
	float[] x = new float[msg.numVertices];
	float[] y = new float[msg.numVertices];
	for (int i = 0; i < msg.numVertices; i++){
	    x[i] = (float) vertices.get(i).x;
	    y[i] = (float) vertices.get(i).y;
	}
	msg.c.r = c.getRed();
	msg.c.b = c.getBlue();
	msg.c.g = c.getGreen();
	msg.x = x;
	msg.y = y;
	msg.closed = closed?1:0;
	msg.filled = filled?1:0;
	polyPub.publish(msg);
    }

    /**
     * Main for debugging
     * @param args
     */
    @Override
    public void onStart(Node node) {
	stringPub = node.newPublisher("gui/String", "Challenge_msgs/GUIStringMessage");
	ellipsePub = node.newPublisher("/gui/Ellipse", "Challenge_msgs/GUIEllipseMessage");
	rectPub = node.newPublisher("gui/Rect", "lab6_msgs/GUIRectMsg");
	pointPub = node.newPublisher("gui/Point", "lab5_msgs/GUIPointMsg");
	erasePub = node.newPublisher("gui/Erase", "lab5_msgs/GUIEraseMsg");
	polyPub = node.newPublisher("gui/Poly", "lab6_msgs/GUIPolyMsg");

	ParameterTree paramTree = node.newParameterTree();
//	paramTree.getString(node.resolveName("~/mapFileName"));
	// lol
	String mapFileName = "/home/rss-student/RSS-I-group/Challenge/src/challenge_2012.txt";
	System.out.println("filename = " + mapFileName);
	//String filename = "/home/jbrooksh/my_sandbox/6.141/spring2012/priv/labs/Challenge/src/construction_map_2011.txt";

    	try{
	        Thread.sleep(2000);

	        // don't erase
		//erasePub.publish(new GUIEraseMsg());
		Thread.sleep(1000);

    		GrandChallengeMap gcm = GrandChallengeMap.parseFile(mapFileName);
		publishRect(gcm.worldRect, false, Color.BLACK);
		for(int i=0; i < gcm.constructionObjects.length; i++) {
		    ConstructionObject b = gcm.constructionObjects[i];
    			Point2D.Double pos = b.getPosition();
    			System.out.println("ConstructionObject at: "+pos.getX()+", "+pos.getY());
			Rectangle2D.Double r = new Rectangle2D.Double();
			if ( b.getSize() == 2 ) {
			    r = new Rectangle2D.Double(b.getPosition().x, b.getPosition().y, 0.05, 0.05);
			} else { 
			    r = new Rectangle2D.Double(b.getPosition().x, b.getPosition().y, 0.05, 0.1);
			}
			publishRect(r, true, b.getColor());
			publishString(b.getPosition().x, b.getPosition().y, Integer.toString(i));
    		}
    		for(Fiducial f : gcm.fiducials){
    			Point2D.Double pos = f.getPosition();
    			System.out.println("Fiducial at ("+f.getTopColor()+"/"+f.getBottomColor()+")at: "+pos.getX()+", "+pos.getY());
			publishEllipse(f.getPosition().x+0.1, f.getPosition().y+0.1, f.getBottomSize()*4.0, 
				       f.getBottomSize()*4.0, f.getBottomColor());
			publishEllipse(f.getPosition().x, f.getPosition().y, f.getTopSize()*4.0, 
				       f.getTopSize()*4.0, f.getTopColor());
			
    		}
		for (PolygonObstacle obstacle : gcm.obstacles){
		    publishPoly(obstacle, Color.BLUE, false, true);
		}
    		System.out.println("WorldRect: "+gcm.worldRect);
		publishPoint(gcm.robotStart.x, gcm.robotStart.y, 0, Color.RED);
		publishPoint(gcm.robotGoal.x, gcm.robotGoal.y, 0, Color.CYAN);
		publishString(gcm.robotStart.x, gcm.robotStart.y, "S");
		publishString(gcm.robotGoal.x, gcm.robotGoal.y, "G");
    	}catch(Exception e){
    		System.err.println("FAILURE!!!!");
    		e.printStackTrace();
    	}
    }

	/**
	 * Shutdown hook for ROS when called as stand-alone node
	 */
	@Override
	public void onShutdown(Node node) {
		// TODO Auto-generated method stub

	}

    @Override public void onShutdownComplete(Node node) {
    }

    @Override public GraphName getDefaultNodeName() {
	return new GraphName("rss/challengemap");
    }
    
    /**
     * Access method for {@link fiducials}
     * @return the array of fiducials that were parsed from the file
     */
    public Fiducial[] getFiducials(){
    	return this.fiducials;
    }
    
    /**
     * Access method for {@link this.constructionObjects}
     * @return the array of constructionObjects that were parsed from the file
     */
    public ConstructionObject[] getConstructionObjects(){
    	return this.constructionObjects;
    }

    /**
     * Access method for {@link obstacles}
     * @return the array of polygonObstacles that were parsed from the file
     */
    public PolygonObstacle[] getPolygonObstacles(){
    	return this.obstacles;
    }

    /**
     * Access method for {@link worldRect}
     * @return the rectangle of the world, as parsed from the file
     */
    public Rectangle2D.Double getWorldRect(){
    	return this.worldRect;
    }
}
