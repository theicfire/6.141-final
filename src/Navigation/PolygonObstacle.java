package Navigation;

import java.awt.geom.*;
import java.util.*;
import java.awt.Color;

/**
 * <p>Represents a polygonal obstacle.<p>
 *
 * <p>Obstacles are initially empty (i.e. no vertices).  Vertices are then
 * added with any of the <code>addVertex()</code> methods, in CCW order.
 * {@link #close} is then called to finish the polygon.  From then on, the
 * polygon is immutable.</p>
 *
 * @author Marty Vona
 **/
public class PolygonObstacle {

  /**
   * <p>Underlying implementation of the geometry.</p>
   **/
  protected GeneralPath path = new GeneralPath();

  /**
   * <p>Whether this polygon has been started.</p>
   **/
  protected boolean started = false;
  
  /**
   * <p>Whether this polygon has been closed.</p>
   **/
  public boolean closed = false;

  public Color color = null;

  /**
   * <p>Covers {@link #addVertex(double, double)}.</p>
   **/
  public void addVertex(Point2D vertex) {
    addVertex(vertex.getX(), vertex.getY());
  }

  /**
   * <p>Covers {@link #addVertex(float, float)}.</p>
   **/
  public void addVertex(double x, double y) {
    addVertex((float) x, (float) y);
  }

  /**
   * <p>Add a vertex to this polygon.</p>
   *
   * <p>Vertices must be added in CCW order.</p>
   *
   * @exception IllegalStateException iff {@link #closed}
   **/
  public void addVertex(float x, float y) {

    if (closed)
      throw new IllegalStateException("already closed");

    if (!started)
      path.moveTo(x, y);
    else
      path.lineTo(x, y);

    started = true;
  }

  public GeneralPath getPath() {
	return path;
  }

/**
   * <p>Close this polygon.</p>
   *
   * @exception IllegalStateException iff not {@link #started} or already
   * {@link #closed}
   **/
  public void close() {

    if (!started || closed)
      throw new IllegalStateException("already closed");
    
    path.closePath();

    closed = true;
  }

  /**
   * <p>Check whether this obstacle polygon contains a point.</p>
   *
   * @param x the point x coord
   * @param y the point y coord
   **/
  public boolean contains(double x, double y) {
    return path.contains(x, y);
  }

  /**
   * <p>Check whether this obstacle polygon contains a point.</p>
   *
   * @param p the point
   **/
  public boolean contains(Point2D p) {
    return path.contains(p);
  }

  /**
   * <p>Check whether a rectangle intersects this obstacle polygon.</p>
   *
   * @param x the rectangle llc x coord
   * @param y the rectangle llc y coord
   * @param w the rectangle width
   * @param h the rectangle height
   **/
  public boolean intersects(double x, double y, double w, double h) {
    return path.intersects(x, y, w, h);
  }

  /**
   * <p>Check whether a rectangle intersects this obstacle polygon.</p>
   *
   * @param r the rectangle
   **/
  public boolean intersects(Rectangle2D r) {
    return path.intersects(r);
  }

  /**
   * <p>Covers {@link #getVertices(List)}, always makes a new list.</p>
   **/
  public List<Point2D.Double> getVertices() {
    return getVertices(null);
  }

  /**
   * <p>Get the vertices of this polygon in the order in which they were
   * added.</p>
   *
   * @param vertices if non-null the vertices are added to this list (else a
   * fresh one is consed)
   *
   * @return the vertices
   **/
  public List<Point2D.Double> getVertices(List<Point2D.Double> vertices) {

    if (vertices == null)
      vertices = new LinkedList<Point2D.Double>();

    double[] coords = new double[6];
    for (PathIterator it = path.getPathIterator(null);
         !it.isDone();
         it.next()) {
      if (it.currentSegment(coords) != PathIterator.SEG_CLOSE)
        vertices.add(new Point2D.Double(coords[0], coords[1]));
    }

    return vertices;
  }
  
  public List<Line2D> getSegments() {
	  Point2D.Double prevPoint = null;
	  ArrayList<Line2D> r = new ArrayList<Line2D>();
	  for (Point2D.Double p : this.getVertices()) {
		  if (prevPoint != null) {
			  r.add(new Line2D.Double(prevPoint, p));
		  }
		  prevPoint = p;
	  }
	  return r;
  }

  /**
   * <p>Covers {@link #toStringBuffer}, internally conses a StringBuffer.</p>
   **/
  public String toString() {
    StringBuffer sb = new StringBuffer();
    toStringBuffer(sb);
    return sb.toString();
  }
  
  /**
   * <p>Append a human-readable string representation of this obstacle to a
   * StringBuffer.</p>
   *
   * @param sb the StringBuffer
   **/
  public void toStringBuffer(StringBuffer sb) {
    double[] coords = new double[6];
    for (PathIterator it = path.getPathIterator(null);
         !it.isDone();
         it.next()) {
      if (it.currentSegment(coords) != PathIterator.SEG_CLOSE) {
        sb.append("(");
        sb.append(Double.toString(coords[0]));
        sb.append(", ");
        sb.append(Double.toString(coords[1]));
        sb.append(")");
        if (!it.isDone())
          sb.append(" ");
      }
    }
  }
}

