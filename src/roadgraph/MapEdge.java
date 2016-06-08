package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	private GeographicPoint start;
	private GeographicPoint end;
	private String streetName;
	private String streetType;

	/**
	 * @param start
	 * @param end
	 * @param streetName
	 * @param streetType
	 */
	public MapEdge(GeographicPoint start, GeographicPoint end, String streetName, String streetType) {
		this.start = start;
		this.end = end;
		this.streetName = streetName;
		this.streetType = streetType;
	}

	/**
	 * @return the start
	 */
	public GeographicPoint getStart() {
		return start;
	}

	/**
	 * @param start the start to set
	 */
	public void setStart(GeographicPoint start) {
		this.start = start;
	}

	/**
	 * @return the end
	 */
	public GeographicPoint getEnd() {
		return end;
	}

	/**
	 * @param end the end to set
	 */
	public void setEnd(GeographicPoint end) {
		this.end = end;
	}

	/**
	 * @return the streetName
	 */
	public String getStreetName() {
		return streetName;
	}

	/**
	 * @param streetName the streetName to set
	 */
	public void setStreetName(String streetName) {
		this.streetName = streetName;
	}

	/**
	 * @return the distance
	 */
	public String getStreetType() {
		return streetType;
	}

	/**
	 * @param distance the distance to set
	 */
	public void setStreetType(String streetType) {
		this.streetType = streetType;
	}
	
	
		
}
