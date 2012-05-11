package Navigation;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.Collection;
import java.util.List;


public class LinePolySetIntersector {

	
	// if the obstacle is NOT in CSpace
	// returns null
	Point2D.Double projectFromObstacleToValidSpace(Point2D.Double pointInValidSpace, Point2D.Double obstacleThatMightBeInCSpace,
			Collection<PolygonObstacle> cSpaceObstacles) {

		Point2D.Double rayStart = new Point2D.Double(
				pointInValidSpace.x,pointInValidSpace.y);
		
		Point2D.Double rayDirection = new Point2D.Double(
				obstacleThatMightBeInCSpace.x - pointInValidSpace.x,
				obstacleThatMightBeInCSpace.y - pointInValidSpace.y);
		
		Line2D.Double line = new Line2D.Double();
		
		Point2D.Double result = new Point2D.Double();
		
		boolean found = false;
		
		double closestScale = Double.MAX_VALUE;
		
		for (PolygonObstacle o: cSpaceObstacles) {
			List<Line2D> segments = o.getSegments();
			
			for (Line2D s : segments) {
				line.x1 = s.getX1();
				line.y1 = s.getY1();
				line.x2 = s.getX2();
				line.y2 = s.getY2();

				double scale =
						RayToLineSegmentIntersectionGetScale(rayStart, rayDirection, line);

				if (scale > 0 && scale <= 1.0 && scale < closestScale) {
					result.x = rayStart.x + scale*rayDirection.x;
					result.y = rayStart.y + scale*rayDirection.y;
					found = true;
				}
			}
		}

		if (found) {
			return result;
		}
		return null;
	}
	
	
    // Given ray: Ray(origin,direction), this function returns -1 if
    // the ray does not intersect the line segment L
    // else it returns k, such that (origin + k*direction) is a point on L
    public static double RayToLineSegmentIntersectionGetScale(
    		Point2D.Double rayStart, Point2D.Double rayDirection, Line2D.Double line)
    {
    	double r, s, d;
    	double x3 = line.x1;
    	double y3 = line.y1;
    	double x4 = line.x2;
    	double y4 = line.y2;
    	
    	double x1 = rayStart.x;
    	double y1 = rayStart.y;
    	double x2 = rayStart.x + rayDirection.x;
    	double y2 = rayStart.y + rayDirection.y;
        //Make sure the lines aren't parallel
    	if ((y2 - y1) / (x2 - x1) != (y4 - y3) / (x4 - x3))
    	{
    		d = (((x2 - x1) * (y4 - y3)) - (y2 - y1) * (x4 - x3));
    		if (d != 0)
    		{
    			r = (((y1 - y3) * (x4 - x3)) - (x1 - x3) * (y4 - y3)) / d;
                s = (((y1 - y3) * (x2 - x1)) - (x1 - x3) * (y2 - y1)) / d;
                if (r >= 0)
                {
                    if (s >= 0 && s <= 1)
                    {
                    	return r;
                    }
                }
            }
        }
        return -1;
    }
	
}
