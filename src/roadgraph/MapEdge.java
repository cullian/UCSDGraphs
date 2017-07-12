/**
 * @author William Cullian
 * 
 * A class which represents a map edge(road) connecting two
 * nodes(intersetions) in the MapGraph Object.  These are 
 * directional edges, so a two way road will require two
 * edges.  The Class contains the from node's geographic 
 * location as well as the to node's location.  Along with
 * additional information such as road name, type and length
 * in kilometers.
 *
 */
package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	private GeographicPoint from;
	private GeographicPoint to;
	private String roadName;
	private String roadType;
	private double roadLength;
	
	/**
	 * Constructor
	 * 
	 * @param from - Geographic location of from node
	 * @param to - Geographic location of to node
	 * @param roadName - road name
	 * @param roadType - road type
	 * @param roadLength - length in kilometers
	 */
	public MapEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double roadLength) {
		this.from = from;
		this.to = to;
		this.roadName = roadName;
		this.roadType = roadType;
		this.roadLength = roadLength;
	}

	/** 
	 * toString method
	 * @return a string representing the map edge
	 */
	@Override
	public String toString() {
		return "MapEdge [from=" + from + ", to=" + to + ", roadName=" + roadName + ", roadType=" + roadType
				+ ", roadLength=" + roadLength + "]";
	}

	/**
	 * Getter for from node location
	 * @return the from
	 */
	public GeographicPoint getFrom() {
		return new GeographicPoint(from.getX(), from.getY());
	}

	/**
	 * Getter for to node location
	 * @return the to
	 */
	public GeographicPoint getTo() {
		return new GeographicPoint(to.getX(), to.getY());
	}

	/**
	 * Getter for road name
	 * @return the roadName
	 */
	public String getRoadName() {
		return new String(roadName);
	}

	/**
	 * Getter for road type
	 * @return the roadType
	 */
	public String getRoadType() {
		return new String(roadType);
	}

	/**
	 * Getter for road length
	 * @return the roadLength
	 */
	public double getRoadLength() {
		return roadLength;
	}
	
}
