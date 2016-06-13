/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	private HashMap<GeographicPoint, MapNode> nodeMap;
	
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		nodeMap = new HashMap<GeographicPoint, MapNode>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return nodeMap.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		Set<GeographicPoint> vertices = new HashSet<GeographicPoint>();
		for(GeographicPoint key : nodeMap.keySet()){
			vertices.add(key);
		}
		return vertices;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
		List<MapEdge> edges = new ArrayList<MapEdge>();
		for(GeographicPoint v : nodeMap.keySet()){
			MapNode temp = nodeMap.get(v);
			edges.addAll(temp.getEdges());
		}
		return edges.size();
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
		// TODO: Implement this method in WEEK 2
		List<MapEdge> edges = new ArrayList<MapEdge>(); 
		MapNode value = new MapNode(location, edges);
		if(nodeMap.get(location)!=null || location!=null){
			nodeMap.put(location, value);
			return true;
		} else {  
			return false;
		}
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
		//TODO: Implement this method in WEEK 2
		if(!nodeMap.containsKey(from)|| !nodeMap.containsKey(to) || from==null || to ==null
				|| roadName==null || roadType==null || length < 0 ){
			throw new IllegalArgumentException();
		}
		MapEdge edge = new MapEdge(from, to, roadName, roadType);
		nodeMap.get(from).getEdges().add(edge);
	}
	
	/**
	 * Get the neighbors of a specified vertex
	 * @return The list of neighbors of a vertex.
	 * @param vertex the name of a vertex
	 */
	public List<GeographicPoint> getNeighbors(GeographicPoint vertex)
	{
		List<GeographicPoint> neighbors = new ArrayList<GeographicPoint>();
		List<MapEdge> edges = nodeMap.get(vertex).getEdges();
		for(MapEdge e : edges){
			neighbors.add(e.getEnd());
		}
		return neighbors;
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
		// TODO: Implement this method in WEEK 2
		//initialization
		List<GeographicPoint> result = new ArrayList<GeographicPoint>(); 
		Queue<GeographicPoint> queue = new LinkedList<GeographicPoint>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<GeographicPoint, GeographicPoint>();
		
		//algorithm follows
		queue.add(start);
		visited.add(start);
		while(!queue.isEmpty()){
			GeographicPoint curr = queue.remove();
			// Hook for visualization.  See writeup.
			nodeSearched.accept(curr.getLocation());
			
			if(curr.equals(goal)){
				result = returnParent(start, goal, result, parent);
			}
			for(GeographicPoint neighbor : getNeighbors(curr)){   
				if(!visited.contains(neighbor)){
					visited.add(neighbor);
					parent.put(neighbor, curr);
					queue.add(neighbor);
				}
			}
		}

		return result;
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
		// TODO: Implement this method in WEEK 3
		
		//initialization
		PriorityQueue<MapPriorityPoint> pQueue = new PriorityQueue<MapPriorityPoint>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<GeographicPoint, GeographicPoint>();
		HashMap<GeographicPoint, Double> distances = new HashMap<GeographicPoint,Double>();
		List<GeographicPoint> result = new ArrayList<GeographicPoint>();
		for(GeographicPoint p : getVertices()){
			distances.put(p, Double.MAX_VALUE);
		}
		
		//algorithm follows
		pQueue.offer(new MapPriorityPoint(start, 0));
		distances.put(start, 0.0);
		while(!pQueue.isEmpty()){
			GeographicPoint curr = pQueue.poll().getPoint();
			// Hook for visualization.  See writeup.
			nodeSearched.accept(curr.getLocation());
			if(!visited.contains(curr)){
				visited.add(curr);
				if(curr.equals(goal)){
					//return parent; we are done
					result = returnParent(start, goal, result, parent);
				}
				for(GeographicPoint neighbor : getNeighbors(curr)){   
					if(!visited.contains(neighbor)){
						//if(path through curr to neighbor is shorter)
						if(distances.get(curr) + curr.distance(neighbor)< distances.get(neighbor)){
							double updated = distances.get(curr)+curr.distance(neighbor);
							distances.put(neighbor, updated); //update distance
							parent.put(neighbor, curr);
							pQueue.offer(new MapPriorityPoint(neighbor, updated));
						}
					}
				}
			}
		}
		return result;
	}

	private List<GeographicPoint> returnParent(GeographicPoint start, GeographicPoint goal, List<GeographicPoint> result, HashMap<GeographicPoint, GeographicPoint> parent) {
		while(!goal.equals(start)){
			result.add(goal);
			goal = parent.get(goal);
		}
		result.add(start);
		Collections.reverse(result);
		return result;
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
		// TODO: Implement this method in WEEK 3
		
		//initialization
		PriorityQueue<MapPriorityPoint> pQueue = new PriorityQueue<MapPriorityPoint>();
		HashSet<GeographicPoint> visited = new HashSet<GeographicPoint>();
		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<GeographicPoint, GeographicPoint>();
		HashMap<GeographicPoint, Double> distances = new HashMap<GeographicPoint,Double>();
		List<GeographicPoint> result = new ArrayList<GeographicPoint>();
		for(GeographicPoint p : getVertices()){
			distances.put(p, Double.MAX_VALUE);
		}

		//algorithm follows
		pQueue.offer(new MapPriorityPoint(start,start.distance(goal)));
		distances.put(start, 0.0);
		while(!pQueue.isEmpty()){
			GeographicPoint curr = pQueue.poll().getPoint();
			// Hook for visualization.  See writeup.
			nodeSearched.accept(curr.getLocation());
			if(!visited.contains(curr)){
				visited.add(curr);
				//System.out.println(" A star visited: " + curr);
				if(curr.equals(goal)){
					//return parent; we are done
					result = returnParent(start, goal, result, parent);
					break;
				}
				for(GeographicPoint neighbor : getNeighbors(curr)){   
					if(!visited.contains(neighbor)){
						//if(path through curr to neighbor is shorter)
						if(distances.get(curr) + curr.distance(neighbor)< distances.get(neighbor)){
							double updated = distances.get(curr)+curr.distance(neighbor);
							distances.put(neighbor, updated); //update distance
							parent.put(neighbor, curr);
							pQueue.offer(new MapPriorityPoint(neighbor, updated+neighbor.distance(goal)));
						}
					}
				}
			}
		}
		return result;
	}


	public static void main(String[] args)
	{
//		System.out.print("Making a new map...");
//		MapGraph theMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/maps/san_diego.map", theMap);
//		System.out.println("DONE.");
//		
//		//testing
//		System.out.println("Vertices: "+ theMap.getNumVertices());
//		//System.out.println(" Get Vertices: "+ theMap.getVertices());
//		System.out.println("Edges: " + theMap.getNumEdges());
		
		// You can use this method for testing.  
		
	    //Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		//GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");


		//GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		//GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		GeographicPoint start = new GeographicPoint(7,3);
		GeographicPoint end = new GeographicPoint(4,-1);
		//System.out.println("Heuristic distance " + start.distance(end));
		System.out.println(theMap.bfs(start,end));

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		System.out.println(route);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		System.out.println(route2);

		
		
	}
	
}
