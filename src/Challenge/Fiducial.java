package Challenge;

import java.awt.Color;
import java.awt.geom.*;

public class Fiducial{
    private Color top;
    private Color bottom;
    private double topSize;
    private double bottomSize;
    private Point2D.Double position;

    
    public Fiducial(){
    	topSize = -1;
    	bottomSize = -1;
    }
    
    public Fiducial(Color top, Color bottom, double topSize, double bottomSize, Point2D.Double position){
    	this.top = top;
    	this.bottom = bottom;
    	this.topSize = topSize;
    	this.bottomSize = bottomSize;
    	this.position = position;
    }

    public Color getTopColor(){
    	return this.top;
    }

    public Color getBottomColor(){
    	return this.bottom;
    }

    public double getTopSize(){
    	return this.topSize;
    }
    
    public double getBottomSize(){
    	return this.bottomSize;
    }

    public Point2D.Double getPosition(){
    	return this.position;
    }
    
    public void setTopColor(Color tc){
    	this.top = tc;
    }
    
    public void setBottomColor(Color bc){
    	this.bottom = bc;
    }
    
    public void setTopSize(double size){
    	this.topSize = size;
    }
    
    public void setBottomSize(double size){
    	this.bottomSize = size;
    }
    
    public void setPosition(Point2D.Double pos){
    	this.position = pos;
    }
}
