/******************************************************************************
 * File: Trailblazer.cpp
 *
 * Implementation of the graph algorithms that comprise the Trailblazer
 * assignment.
 */

#include "Trailblazer.h"
#include "TrailblazerGraphics.h"
#include "TrailblazerTypes.h"
#include "TrailblazerPQueue.h"
#include"map.h"
#include "set.h"
#include "TrailblazerCosts.h"
#include <iostream>
#include"hashmap.h"
#include "vector.h"
#include "random.h"
using namespace std;




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
 * search.  Make sure to update both this implementation prototype and the
 * function prototype in Trailblazer.h.
 */
Vector<Loc>
shortestPath(Loc start,
             Loc end,
             Grid<double>& world,
             double costFn(Loc from, Loc to, Grid<double>& world),
             double heuristic(Loc start,Loc end,Grid<double>&world )) {
	// TODO: Fill this in!
    Vector<Loc> path ;
    TrailblazerPQueue<Loc> unprocessedLoc;
    Map<Loc, LocInfoCell> locInfoMap= initLocInfoMap(world);
    /* locInfoMap creates association between Loc and its data storage unit LocInfoCell, which records loc's candidateDistance  , state , and location of its parent cell.  initLocInfo assign a blankc LocInfoCell to every Loc in the world */
    
    locInfoMap[start].state=enqueued;
    locInfoMap[start].candidateDistance=0;
    colorCell(world, start, YELLOW);
    unprocessedLoc.enqueue(start, locInfoMap[start].candidateDistance+heuristic(start,end,world));
    
    while(!unprocessedLoc.isEmpty()){
        Loc curr;
        curr=unprocessedLoc.dequeueMin();
        locInfoMap[curr].state=expanded;
        colorCell(world, curr, GREEN);
        if(curr==end){
            // fill path vector and return
            path.add(end);
            Loc next=makeLoc(locInfoMap[end].parentRow, locInfoMap[end].parentCol);
            
            while(true){
                if(next==start) break;
                path.add(next);
                next=makeLoc(locInfoMap[next].parentRow, locInfoMap[next].parentCol);
                
            }
            path.add(start);
            path=reversePath(path);
            return path;
            
        }
        
        Loc neighbour;
        Loc outOfBound=makeLoc(-1, -1);
        
        for(Direction dir=north;dir<=northWest;dir++){
            neighbour=getAdjacentLoc(curr, dir, world);
            if(neighbour!=outOfBound&& locInfoMap[neighbour].state==unvisited){
                locInfoMap[neighbour].state=enqueued;
                colorCell(world, neighbour, YELLOW);
                locInfoMap[neighbour].candidateDistance=locInfoMap[curr].candidateDistance+
                costFn(curr,neighbour,world);
                locInfoMap[neighbour].parentRow=curr.row;
                locInfoMap[neighbour].parentCol=curr.col;
                unprocessedLoc.enqueue(neighbour,locInfoMap[neighbour].candidateDistance
                +heuristic(neighbour,end,world));
                
            }
            if(neighbour!=outOfBound&& locInfoMap[neighbour].state==enqueued&&
               locInfoMap[neighbour].candidateDistance>locInfoMap[curr].candidateDistance
               +costFn(curr,neighbour,world)){
                locInfoMap[neighbour].candidateDistance=locInfoMap[curr].candidateDistance
                +costFn(curr,neighbour,world);
                locInfoMap[neighbour].parentRow=curr.row;
                locInfoMap[neighbour].parentCol=curr.col;
                unprocessedLoc.decreaseKey(neighbour, locInfoMap[neighbour].candidateDistance
                +heuristic(neighbour,end,world));
            }
        }
        
    }

}




Loc getAdjacentLoc(Loc curr, Direction dir,Grid<double>& world){
    
    if(dir==north&&world.inBounds(curr.row-1, curr.col)){
        Loc result=makeLoc(curr.row-1,curr.col);
        return result;
    }
    
    if(dir==northEast&&world.inBounds(curr.row-1, curr.col+1)){
        Loc result=makeLoc(curr.row-1,curr.col+1);
        return result;
    }
    if(dir==east&&world.inBounds(curr.row, curr.col+1)){
        Loc result=makeLoc(curr.row,curr.col+1);
        return result;
    }
    if(dir==southEast&&world.inBounds(curr.row+1, curr.col+1)){
        Loc result=makeLoc(curr.row+1,curr.col+1);
        return result;
    }
    if(dir==south&&world.inBounds(curr.row+1, curr.col)){
        Loc result=makeLoc(curr.row+1,curr.col);
        return result;
    }
    if(dir==southWest&&world.inBounds(curr.row+1, curr.col-1)){
        Loc result=makeLoc(curr.row+1,curr.col-1);
        return result;
    }
    if(dir==west&&world.inBounds(curr.row, curr.col-1)){
        Loc result=makeLoc(curr.row,curr.col-1);
        return result;
    }
    if(dir==northWest&&world.inBounds(curr.row-1, curr.col-1)){
        Loc result=makeLoc(curr.row-1,curr.col-1);
        return result;
    }
    else {
        Loc outOfBound= makeLoc(-1, -1);
        return outOfBound;
    }
    
}

Map<Loc, LocInfoCell>initLocInfoMap(Grid<double>& world){
    Map<Loc, LocInfoCell> result;
    LocInfoCell unvistedCell;
    for(int i=0;i<world.numRows();i++){
        for(int j=0;j<world.numCols();j++){
            result.put(makeLoc(i, j), unvistedCell);
        }
    }
    return result;
    
}

Vector<Loc> reversePath(Vector<Loc> path){
    
    
    Vector<Loc> result;
    
    for (int i=path.size()-1;i>=0;i--){
        result.add(path[i]);
    }
    
    return result;
}


