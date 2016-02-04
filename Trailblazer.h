/******************************************************************************
 * File: Trailblazer.h
 *
 * Exports functions that use Dijkstra's algorithm, A* search, and Kruskal's
 * algorithm as specified in the assignment handout.
 */

#ifndef Trailblazer_Included
#define Trailblazer_Included

#include "TrailblazerTypes.h"
#include "set.h"
#include "grid.h"
#include "TrailblazerPQueue.h"
#include "hashmap.h"
#include "vector.h"


Map<Loc, LocInfoCell>initLocInfoMap(Grid<double>& world);

enum Direction{
    north, northEast,east,southEast,south ,southWest,west,northWest
};
Loc getAdjacentLoc(Loc curr, Direction dir,Grid<double>& world);

Vector<Loc> reversePath(Vector<Loc> path);

/* Function: shortestPath
 * 
 * Finds the shortest path between the locations given by start and end in the
 * specified world.	 The cost of moving from one edge to the next is specified
 * by the given cost function.	The resulting path is then returned as a
 * Vector<Loc> containing the locations to visit in the order in which they
 * would be visited.	If no path is found, this function should report an
 * error.
 *
 * In Part Two of this assignment, you will need to add an additional parameter
 * to this function that represents the heuristic to use while performing the
 * search.  Make sure to update both this function prototype and the
 * implementation inside of Trailblazer.cpp.
 */
Vector<Loc>
shortestPath(Loc start,
             Loc end,
             Grid<double>& world,
             double costFn(Loc from, Loc to, Grid<double>& world),
             double heuristic(Loc start,Loc end,Grid<double>&world ));

enum Direction_M{
    northM,eastM,southM ,westM
};

//Grid<Loc> initWorld(int numRows,int numCols,Grid<Loc> &world);
TrailblazerPQueue<Edge> createEdges(Grid<double>& world,Set<Edge>&existedEdge);
Loc getAdjacentLoc_Maze(Loc curr, Direction_M dir,Grid<double>& world);
int initBothMap(Map<Loc,int> &clusterNumberMap,Map<int,Set<Loc> >&clusterMap,Grid<double>& world);
void mergeCluster(Loc start,Loc end,HashMap<Loc,Set<Loc> > &clusterMap);


/* Function: createMaze
 * 
 * Creates a maze of the specified dimensions using a randomized version of
 * Kruskal's algorithm, then returns a set of all of the edges in the maze.
 *
 * As specified in the assignment handout, the edges you should return here
 * represent the connections between locations in the graph that are passable.
 * Our provided starter code will then use these edges to build up a Grid
 * representation of the maze.
 */
Set<Edge> createMaze(int numRows, int numCols);
HashMap<Loc, Set<Loc> > setUpMap(HashMap<Loc, Set<Loc> >& clusterMap,Grid<double>&world);
Vector<Edge> createEdgesV(Grid<double>& world,Set<Edge>&enqueuedEdges);

#endif
