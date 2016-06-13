package roadgraph;

import geography.GeographicPoint;

public class MapPriorityPoint implements Comparable<MapPriorityPoint> {
	private GeographicPoint point;
	private double priority;
	
	public MapPriorityPoint(GeographicPoint point, double d) {
		// TODO Auto-generated constructor stub
		this.point = point;
		this.priority = d;
	}
	public GeographicPoint getPoint() {
		return point;
	}
	public void setPoint(GeographicPoint point) {
		this.point = point;
	}
	public double getPriority() {
		return priority;
	}
	public void setPriority(int priority) {
		this.priority = priority;
	}
	
	@Override
	public int compareTo(MapPriorityPoint pt){
	    return Double.compare(priority, pt.priority);
	}

}

