/**
 * @author William Cullian
 * 
 * A class which represents a map node(intersetion) in a 
 * MapGraph object connected to other nodes by edges(roads).
 * The MapNode contains the geographic location and a
 * list of edges(roads) that connect it to other 
 * nodes(intersetions).
 *
 */
package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class MapNode {

	private GeographicPoint location;
	private List<MapEdge> edges;
	
	/**
	 * Constructor sets location with empty edge list
	 * @param loc
	 */
	
	public MapNode(GeographicPoint loc) {
		this.location = loc;
		edges = new ArrayList<MapEdge> ();
		
	}

	/**
	 * Add an edge to the edges list if it does not 
	 * already exist
	 * @param add edges to list
	 */
	public void addEdge(MapEdge edge) {
		if (!this.edges.contains(edge)) {
			edges.add(edge);
		}
	}

	/**
	 * Getter for list of edges
	 * @return the edges
	 */
	public List<MapEdge> getEdges() {
		return new ArrayList<MapEdge>(edges);
	}

	/**
	 * @return the location
	 */
	public GeographicPoint getLocation() {
		return new GeographicPoint(location.getX(), location.getY());
	}

	/* (non-Javadoc)
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return "MapNode [location=" + location + ", edges=" + edges + "]";
	}

	
}
