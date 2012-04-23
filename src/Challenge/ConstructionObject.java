package Challenge;

import java.awt.Color;
import java.awt.geom.*;

public class ConstructionObject{
	private Color color;
	private int size;
	private Point2D.Double position;
	
	//The volume of a ConstructionObject would be METERS_PER_CUBE*METERS_PER_CUBE*(size*METERS_PER_CUBE)
	public static final double METERS_PER_CUBE = 0.05;

	public ConstructionObject(){
		this.color = null;
		this.size = -1;//Unknown size
		this.position = null;
	}

	public void setColor(Color c){
		this.color = c;
	}

	public void setSize(int size){
		this.size = size;
	}

	public void setPosition(Point2D.Double pos){
		this.position = pos;
	}

	public boolean isKnownColor(){
		return (color!=null);
	}
	    
	public boolean isKnownSize(){
		return (size>0);
	}

	public boolean isKnownPosition(){
		return (position!=null);
	}

	public Color getColor(){
	    return this.color;
	}
	    
	public int getSize(){
		return this.size;
	}

	public Point2D.Double getPosition(){
		return this.position;
	}
}
