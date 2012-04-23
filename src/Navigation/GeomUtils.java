package Navigation;

import java.awt.geom.*;
import java.util.*;

/**
 * <p>Geometric utilities.</p>
 *
 * @author Marty Vona
 * @author Aisha Walcott
 **/
public class GeomUtils {

  /**
   * <p>Compare two points for sorting in x-increasing (then y-increasing)
   * order.</p>
   **/
  public static final Comparator<Point2D.Double> POINT_COMPARATOR_LR =
    new Comparator<Point2D.Double>() {

    public int compare(Point2D.Double p0, Point2D.Double p1) {
      
      if ((p0.x > p1.x) || ((p0.x == p1.x) && (p0.y > p1.y)))
        return 1;
      
      if ((p0.x < p1.x) || ((p0.x == p1.x) && (p0.y < p1.y)))
        return -1;
      
      return 0;
    }
  };

  /**
   * <p>Compute the convex hull of a set of points.</p>
   *
   * <p>Follows de Berg, van Kreveld, Overmars, Schwarzkopf p. 6.</p>
   *
   * @param points the set of points, not null.  Will be sorted by {@link
   * #POINT_COMPARATOR_LR} and will have duplicates removed.
   *
   * @return the convex hull of <code>points</code>, which degenerates to a
   * line segment or a point or an empty polygon if there are less than three
   * distinct points.
   **/
  public static PolygonObstacle convexHull(List<Point2D.Double> points) {
   
    PolygonObstacle hull = new PolygonObstacle();

    if ((points == null) || points.isEmpty())
      return hull;

    int n = points.size();

    if (n < 3) {
      for (Point2D.Double point : points)
        hull.addVertex(point);
      hull.close();
      return hull;
    }

    //sort the points from left to right

    Collections.sort(points, POINT_COMPARATOR_LR);

    //remove duplicates

    ListIterator<Point2D.Double> it = points.listIterator();

    Point2D.Double prev = it.next();

    while (it.hasNext()) {
      Point2D.Double next = it.next();
      if (prev.equals(next))
        it.remove();
      else
        prev = next;
    }

    //make upper hull in CW order

    LinkedList<Point2D.Double> halfHull = new LinkedList<Point2D.Double>();

    it = points.listIterator();

    //add the first two sorted points
    halfHull.add(it.next());
    halfHull.add(it.next());

    while (it.hasNext()) {

      //add a new sorted point
      halfHull.add(it.next());

      //fixup
      while (halfHull.size() > 2) {
        
        if (rightTurn(halfHull.get(halfHull.size() - 3),
                      halfHull.get(halfHull.size() - 2),
                      halfHull.get(halfHull.size() - 1)))
          break; //right turn, halfHull is ok
        
        //left turn; delete the middle of the last three points from halfHull
        //and continue checking
        halfHull.remove(halfHull.size() - 2);
      }
    }

    //store the upper hull in CCW order
    for (it = halfHull.listIterator(halfHull.size()); it.hasPrevious(); )
      hull.addVertex(it.previous());
       
    //make lower hull in CW order

    halfHull.clear();

    it = points.listIterator(points.size());

    //add the first two sorted points
    halfHull.add(it.previous());
    halfHull.add(it.previous());
    
    while (it.hasPrevious()) {
      
      //add a new sorted point
      halfHull.add(it.previous());

      //fixup
      while (halfHull.size() > 2) {
        
        if (rightTurn(halfHull.get(halfHull.size() - 3),
                      halfHull.get(halfHull.size() - 2),
                      halfHull.get(halfHull.size() - 1)))
          break; //right turn, halfHull is ok
        
        //left turn; delete the middle of the last three points from halfHull
        //and continue checking
        halfHull.remove(halfHull.size() - 2);
      }
    }

    //remove the first and the last point from lower hull to avoid duplication
    //where the upper and lower hulls meet
    halfHull.removeFirst();
    halfHull.removeLast();

    //store the lower hull in CCW order
    for (it = halfHull.listIterator(halfHull.size()); it.hasPrevious(); )
      hull.addVertex(it.previous());
    
    hull.close();
    
    return hull;
  }

  /**
   * <p>Check whether three ordered points make a right turn.</p>
   *
   * <p>This is equivalent to asking if <code>p1</code> lies to the left of the
   * oriented line from <code>p0</code> to <code>p2</code>, which is equivalent
   * to asking if the z component of the cross product of the vector from
   * <code>p0</code> to <code>p2</code> with the vector from <code>p0</code> to
   * <code>p1</code> is positive.</p>
   *
   * @param p0 the first point
   * @param p1 the second point
   * @param p2 the third point
   *
   * @return true iff the ordered sequence <code>p0</code>, <code>p1</code>,
   * <code>p2</code> makes a right turn
   **/
  public static boolean rightTurn(Point2D.Double p0,
                                  Point2D.Double p1,
                                  Point2D.Double p2) {
    //vector from p0 to p2
    double p02x = p2.x - p0.x;
    double p02y = p2.y - p0.y;

    //vector from p0 to p1
    double p01x = p1.x - p0.x;
    double p01y = p1.y - p0.y;

    //return true iff the z component of the cross product p02 x p01 is
    //positive
    return ((p02x*p01y-p02y*p01x) > 0.0);
  }
}
