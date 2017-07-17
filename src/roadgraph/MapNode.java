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

	private GeographicPoint location;		// location of intersection
	private List<MapEdge> roads;			// list of streets leaving this intersection
	// distanceToStart used for shortest path algorithms
	private double distanceToStart;
	// predictedDistanceToGoal used for A* algorithm
	private double predictedDistanceToGoal;
	
	/**
	 * Constructor sets location with empty edge list
	 * @param loc
	 */
	
	public MapNode(GeographicPoint loc) {
		this.location = loc;
		roads = new ArrayList<MapEdge> ();
		this.distanceToStart = Double.POSITIVE_INFINITY;
		this.predictedDistanceToGoal = Double.POSITIVE_INFINITY;
		
	}

	/**
	 * Add an edge to the edges list if it does not 
	 * already exist
	 * @param add edges to list
	 */
	public void addEdge(MapEdge edge) {
		if (!this.roads.contains(edge)) {
			roads.add(edge);
		}
	}

	/**
	 * @return the distanceToStart
	 */
	public double getDistanceToStart() {
		return distanceToStart;
	}

	/**
	 * @param distanceToStart the distanceToStart to set
	 */
	public void setDistanceToStart(double distanceToStart) {
		this.distanceToStart = distanceToStart;
	}

	/**
	 * @param distanceToStart the distanceToStart to reset
	 */
	public void reSetDistanceToStart() {
		this.distanceToStart = Double.POSITIVE_INFINITY;
	}

	/** Predicted distance from start to goal
	 * @return the predictedDistanceToGoal
	 */
	public double getPredictedDistance() {
		return predictedDistanceToGoal + distanceToStart;
	}


	/**
	 * @return the predictedDistanceToGoal
	 */
	public double getPredictedDistanceToGoal() {
		return predictedDistanceToGoal;
	}

	/**
	 * @param predictedDistanceToGoal the predictedDistanceToGoal to set
	 */
	public void reSetPredictedDistanceToGoal() {
		this.predictedDistanceToGoal = Double.POSITIVE_INFINITY;
	}

	/**
	 * @param predictedDistanceToGoal the predictedDistanceToGoal to set
	 */
	public void setPredictedDistanceToGoal(double predictedDistanceToGoal) {
		this.predictedDistanceToGoal = predictedDistanceToGoal;
	}

	/**
	 * Getter for list of edges
	 * @return the edges
	 */
	public List<MapEdge> getEdges() {
		return new ArrayList<MapEdge>(roads);
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
		return "MapNode [location=" + location + ", edges=" + roads + "]";
	}

	
}
