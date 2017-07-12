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
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

public class MapGraph {
	private Map<GeographicPoint, MapNode> vertices;
	private int numVertices;
	private int numEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		vertices = new HashMap<GeographicPoint, MapNode>();
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
		return vertices.keySet();
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
		if (location != null && !vertices.containsKey(location)) {
			// put it in the list, count it and return true
			vertices.put(location, new MapNode(location));
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
				!vertices.containsKey(from) || !vertices.containsKey(to)) {
			throw new IllegalArgumentException("Illegal Agrument");
		}
		else {
			// create new edge, add it to edge list and count it
			MapEdge newEdge = new MapEdge(from, to, roadName, roadType, length);
			vertices.get(from).addEdge(newEdge);
			numEdges++;
		}
		
	}
	
	
	/** 
	 * Prints all the vertices and their respective edge lists
	 * for debugging purposes
	 */
	public void printGraph() {
		for (MapNode node : vertices.values()) {
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
		// Initialize queue, visited and parentMap
		// queue of locations to be searched
		Queue<GeographicPoint> queue = new LinkedList<GeographicPoint>();
		// list of locations already visited
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		// map of parent nodes for each node searched(used to create path)
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
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
			for (MapEdge neighborEdge : vertices.get(curr).getEdges()) {
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

	/**
	 * A helper method to construct the path from the parentMap
	 * @param start
	 * @param goal
	 * @param parentMap
	 * @return An array list of geographic points of the path from
	 * start to goal
	 */
	private List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal,
			HashMap<GeographicPoint, GeographicPoint> parentMap) {
		GeographicPoint curr = goal;
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		while (!curr.equals(start)) {
			path.add(curr);
			// get curr's parent
			curr = parentMap.get(curr);
		}
		path.add(curr);
//		int pLen = path.size();
//		for (int i = 0; i < pLen/2; i++) {
//			GeographicPoint swap = path.get(i);
//			path.set(i, path.get(pLen - i - 1));
//			path.set(pLen - i - 1, swap);
//		}
		Collections.reverse(path);
		
		return path;
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
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
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
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
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
