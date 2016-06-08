package roadgraph;

import java.util.List;

import geography.GeographicPoint;

public class MapNode {
	private GeographicPoint location;
	private List<MapEdge> edges;
	/**
	 * @param location
	 * @param edges
	 */
	public MapNode(GeographicPoint location, List<MapEdge> edges) {
		this.location = location;
		this.edges = edges;
	}
	/**
	 * @return the location
	 */
	public GeographicPoint getLocation() {
		return location;
	}
	/**
	 * @param location the location to set
	 */
	public void setLocation(GeographicPoint location) {
		this.location = location;
	}
	/**
	 * @return the edges
	 */
	public List<MapEdge> getEdges() {
		return edges;
	}
	/**
	 * @param edges the edges to set
	 */
	public void setEdges(List<MapEdge> edges) {
		this.edges = edges;
	}
	
	
}
