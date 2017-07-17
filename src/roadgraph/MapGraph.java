/**
 * @author UCSD MOOC development team and William Cullian
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections connected by edges
 * which are roads.  The Class maintains a hashmap of 
 * geographic points mapped to a list of their edges.
 * Also a count of the number of vertices(nodes) and edges
 * is maintained.
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

public class MapGraph {
	private Map<GeographicPoint, MapNode> intersections;
	private int numVertices;
	private int numEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		intersections = new HashMap<GeographicPoint, MapNode>();
		numVertices = 0;
		numEdges = 0;
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{		
		return intersections.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// if location is not null and not in vertices list
		if (location != null && !intersections.containsKey(location)) {
			// put it in the list, count it and return true
			intersections.put(location, new MapNode(location));
			numVertices++;
			return true;
		}
		// else false
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		// if any argument is null or length < 0 or points are not already nodes
		// throw IllegalArgumentException
		if (from == null || to == null || roadName == null || roadType == null || length < 0.0 || 
				!intersections.containsKey(from) || !intersections.containsKey(to)) {
			throw new IllegalArgumentException("Illegal Agrument");
		}
		else {
			// create new edge, add it to edge list and count it
			MapEdge newEdge = new MapEdge(from, to, roadName, roadType, length);
			intersections.get(from).addEdge(newEdge);
			numEdges++;
		}
		
	}
	
	
	/** 
	 * Prints all the vertices and their respective edge lists
	 * for debugging purposes
	 */
	public void printGraph() {
		for (MapNode node : intersections.values()) {
			System.out.println(node);
		}
	}

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if (!validNode(start) || !validNode(goal) || nodeSearched == null) {
			return null;
		}
		// map of parent nodes for each node searched(used to create path)
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		// queue of locations to be searched
		Queue<GeographicPoint> queue = new LinkedList<GeographicPoint>();
		// list of locations already visited
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		// enqueue start location
		queue.add(start);
		// add start to visited list
		visited.add(start);
		// while there are still more places to search
		while (!queue.isEmpty()) {
			// dequeue current node from front of queue
			GeographicPoint curr = queue.remove();
			// Hook for visualization.  See writeup.
			nodeSearched.accept(curr);
			// if we found goal, return path
			if (curr.equals(goal)) {
				return constructPath(start, goal, parentMap);
			}
			// for each of current nodes neighbors not in visited list
			for (MapEdge neighborEdge : intersections.get(curr).getEdges()) {
				GeographicPoint neighbor = neighborEdge.getTo();
				if (visited.contains(neighbor)) {
					continue;
				}
				// add neighbor to visited set
				visited.add(neighbor);
				// add current node as neighbor's parent in parentMap
				parentMap.put(neighbor, curr);
				// enqueue neighbor onto the queue
				queue.add(neighbor);
			}
		}
		// if we get here then there is no path
		return null;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// check for valid input
		if (!validNode(start) || !validNode(goal) || nodeSearched == null) {
			return null;
		}
		// Initialize variables
		// map of parent nodes for each node searched(used to create path)
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		// queue of locations to be searched sorted by distance to start using anonymous lambda
		PriorityQueue<GeographicPoint> pQueue =	new PriorityQueue<GeographicPoint>((a,b) -> 
				(intersections.get(a).getDistanceToStart() < intersections.get(b).getDistanceToStart()) ? -1
						:(intersections.get(a).getDistanceToStart() > intersections.get(b).getDistanceToStart() ? 1 : 0));
		// list of locations already visited
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		// reset all distances to infinity
		resetDistances();
		// enqueue start location with distance 0
		intersections.get(start).setDistanceToStart(0.0);
		pQueue.add(start);
		int count = 0;
		
		// while there are still more places to search
		while (!pQueue.isEmpty()) {
			// dequeue current node from front of queue
			GeographicPoint current = pQueue.remove();
			count++;
			System.out.print("DIJKSTRA visiting[NODE at location (" + current +") intersects streets: ");
			// Hook for visualization.  See writeup.
			nodeSearched.accept(current);

			// if current is not visited
			if (!visited.contains(current)) {
				// add current to visited set
				visited.add(current);
				// if we found goal, return path
				if (current.equals(goal)) {
					System.out.print("]\n");
					System.out.println("Dijkstra Node Count: " + count);
					return constructPath(start, goal, parentMap);
				}
				// for each of current nodes neighbors not in visited list
				for (MapEdge edgeToNeighbor : intersections.get(current).getEdges()) {
					GeographicPoint neighbor = edgeToNeighbor.getTo();
					if (visited.contains(neighbor)) {
						continue;
					}
					System.out.print(edgeToNeighbor.getRoadName() + ", ");
					// distance to start from current
					double actualDistToStart = intersections.get(current).getDistanceToStart();
					// add road length to distance
					actualDistToStart += edgeToNeighbor.getRoadLength();
					
					// if path from neighbor to start is shorter than neighbors current distance to start
					if (actualDistToStart < intersections.get(neighbor).getDistanceToStart()) {
						// update neighbors distance
						intersections.get(neighbor).setDistanceToStart(actualDistToStart);
						// update current node as neighbor's parent in parentMap
						parentMap.put(neighbor, current);
						// enqueue neighbor and distance onto the queue
						pQueue.add(neighbor);
					}
				}
				System.out.print("]\n");
				System.out.println("Actual = " + intersections.get(current).getDistanceToStart());
				System.out.println();
			}
		}
		// if we get here then there is no path
		System.out.println("Dijkstra Node Count(No Path): " + count);
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// check for valid input
		if (!validNode(start) || !validNode(goal) || nodeSearched == null) {
			return null;
		}
		// Initialize variables
		// map of parent nodes for each node searched(used to create path)
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
		// queue of locations to be searched sorted by predicted distance to goal using anonymous lambda
		PriorityQueue<GeographicPoint> pQueue =	new PriorityQueue<GeographicPoint>((a,b) -> 
				(intersections.get(a).getPredictedDistance() < intersections.get(b).getPredictedDistance()) ? -1
						:(intersections.get(a).getPredictedDistance() > intersections.get(b).getPredictedDistance() ? 1 : 0));
		// list of locations already visited
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		// reset all distances to infinity
		resetDistances();
		// enqueue start location with distance 0
		intersections.get(start).setDistanceToStart(0.0);
		intersections.get(start).setPredictedDistanceToGoal(0.0);
		pQueue.add(start);
		int count = 0;
		
		// while there are still more places to search
		while (!pQueue.isEmpty()) {
			// dequeue current node from front of queue
			GeographicPoint current = pQueue.remove();
			count++;
			System.out.print("A * visiting[NODE at location (" + current +") intersects streets: ");
			// Hook for visualization.  See writeup.
			nodeSearched.accept(current);

			// if current is not visited
			if (!visited.contains(current)) {
				// add current to visited set
				visited.add(current);
				// if we found goal, return path
				if (current.equals(goal)) {
					System.out.print("]\n");
					System.out.println("A * Node Count: " + count);
					return constructPath(start, goal, parentMap);
				}
				// for each of current nodes neighbors not in visited list
				for (MapEdge edgeToNeighbor : intersections.get(current).getEdges()) {
					GeographicPoint neighbor = edgeToNeighbor.getTo();
					if (visited.contains(neighbor)) {
						continue;
					}
					System.out.print(edgeToNeighbor.getRoadName() + ", ");
					// distance to start from current
					double actualDistToStart = intersections.get(current).getDistanceToStart();
					// add road length to distance
					actualDistToStart += edgeToNeighbor.getRoadLength();
					// add Heuristic Estimated Cost
					double heuristicEstCost = neighbor.distance(goal);
					double predictedDistToGoal = actualDistToStart + heuristicEstCost;
			
					// if path from neighbor to start is shorter than neighbors current distance to start
					if (predictedDistToGoal < intersections.get(neighbor).getDistanceToStart() + intersections.get(neighbor).getPredictedDistanceToGoal()) {
						// update neighbors distance
						intersections.get(neighbor).setDistanceToStart(actualDistToStart);
						// update neighbors Heuristic
						intersections.get(neighbor).setPredictedDistanceToGoal(heuristicEstCost);;
						// update current node as neighbor's parent in parentMap
						parentMap.put(neighbor, current);
						// enqueue neighbor and distance onto the queue
						pQueue.add(neighbor);
					}
				}
				System.out.print("]\n");
				System.out.println("Actual = " + intersections.get(current).getDistanceToStart());
				System.out.println("Predicted = " + intersections.get(current).getPredictedDistanceToGoal());
				System.out.println();
			}
		}
		// if we get here then there is no path
		System.out.println("A * Node Count(No Path): " + count);
		return null;
	}

	
	
	/**
	 * A helper method to construct the path from the parentMap
	 * @param start
	 * @param goal
	 * @param parentMap
	 * @return A linked list of geographic points of the path from
	 * start to goal
	 */
	private List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal,
			HashMap<GeographicPoint, GeographicPoint> parentMap) {
		GeographicPoint curr = goal;
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		while (!curr.equals(start)) {
			path.addFirst(curr);
			// get curr's parent
			curr = parentMap.get(curr);
		}
		// don't forget start
		path.addFirst(curr);
		
		return path;
	}

	/**
	 * A helper method to check if the node location is valid
	 * @param node location of node to check
	 * @return true if node is valid
	 */
	
	private boolean validNode(GeographicPoint node) {
		if (node == null) {
			throw new NullPointerException("Cannot find route from or to null node");
		}
		if (intersections.containsKey(node)) {
			return true;
		}
		return false;
	}

	/** Reset the ditance to start variables in all the intersections
	 * 
	 */
	private void resetDistances() {
		for (MapNode i : intersections.values()) {
			i.reSetDistanceToStart();
			i.reSetPredictedDistanceToGoal();
		}
	}

	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		firstMap.printGraph();
		// You can use this method for testing.  
		GeographicPoint start = new GeographicPoint(1.0, 1.0);
		GeographicPoint goal = new GeographicPoint(8.0, -1.0);
		// Test bfs using dummy consumer object
		List<GeographicPoint> path = firstMap.bfs(start, goal);
		System.out.println(path);
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		System.out.println(testroute);
		System.out.println(testroute2);
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		System.out.println(testroute);
		System.out.println(testroute2);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		System.out.println(testroute);
		System.out.println(testroute2);
		
		
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start1 = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start1,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start1,end);
		System.out.println(route);
		System.out.println(route2);

		
		
	}
	
}
