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


Set<Edge> createMaze(int numRows, int numCols) {
    Set<Edge> minSpanningTree;
    Set<Edge> enqueuedEdges;
    Grid<double>world(numRows,numCols);
    //TrailblazerPQueue< Edge > edges=createEdges(world,enqueuedEdges);
    Vector<Edge> edges=createEdgesV(world, enqueuedEdges);
    HashMap<Loc, Set<Loc> > clusterMap;
    clusterMap=setUpMap(clusterMap,world);
    int nCluster=clusterMap.size();
    Edge currEdge;
    int ranIndex;
    
    

    while(!edges.isEmpty()){
        
        ranIndex=randomInteger(0, edges.size()-1);
        
        currEdge=edges[ranIndex];
        edges.remove(ranIndex);
        
        
        
        if (clusterMap.containsKey(currEdge.start) && clusterMap.containsKey(currEdge.end)  /*clusterMap[currEdge.start]!=clusterMap[currEdge.end]*/){
            //merge
            
            mergeCluster(currEdge.start, currEdge.end, clusterMap);
            
           
           
            minSpanningTree.add(currEdge);
        
        }
    }
   
    return minSpanningTree;
    
}

HashMap<Loc, Set<Loc> > setUpMap(HashMap<Loc, Set<Loc> >& clusterMap,Grid<double>&world){
   
    for(int i=0;i<world.numRows();i++){
        for(int j=0;j<world.numCols();j++){
            Set<Loc> currCluster;
            currCluster.add(makeLoc(i, j));
            
        
            
            clusterMap.put(makeLoc(i, j),currCluster);
            
        }
    }
    
    return clusterMap;
}


Vector<Edge> createEdgesV(Grid<double>& world,Set<Edge>&enqueuedEdges){
    Vector<Edge>result;
    Edge currEdge;
    Edge reverseCurrEdge;
    Loc currLoc;
    Loc neighbour;
    Loc outOfBound=makeLoc(-1, -1);
    for(int i=0;i<world.numRows();i++){
        for(int j=0;j<world.numCols();j++){
            currLoc=makeLoc(i, j);
            for(Direction_M dir=northM;dir<=westM;dir++){
                neighbour=getAdjacentLoc_Maze(currLoc, dir, world);
                if(neighbour!=outOfBound){
                    currEdge=makeEdge(currLoc, neighbour);
                    reverseCurrEdge=makeEdge(neighbour, currLoc);
                    
                    
                    if((!enqueuedEdges.contains(currEdge))&&(!enqueuedEdges.contains(reverseCurrEdge))){
                        result.add(currEdge);
                        
                        
                        enqueuedEdges.add(makeEdge(currLoc, neighbour));
                        enqueuedEdges.add(makeEdge(neighbour, currLoc));
                    }
                }
            }
        }
    }

        return result;
}


/*
Set<Edge> createMaze(int numRows, int numCols) {
	
    
    // TODO: Fill this in!
    Set<Edge> minimalSpanningTree;
    Set<Edge>existedEdge;
	Grid<double> world(numRows,numCols);
    TrailblazerPQueue< Edge > edges=createEdges(world,existedEdge);
    
    
    
    Map<Loc,int > clusterNumberMap;
    // Maps Loc to Its cluster number .
    Map<int,Set<Loc> > clusterMap;
    // Map cluster Number to the Actual Cluster;
    initBothMap(clusterNumberMap,clusterMap,world);
    //Assign each Loc a Cluster number. and Create a Cluster containing a single Loc in cluster
    //Map
    int map1Size=clusterMap.size();
    int map2Size=clusterNumberMap.size();
    Edge currEdge;
    
    int nCluster= numRows*numCols;
    while (nCluster>1){
        currEdge=edges.dequeueMin();
        
        int startClusterNumber=clusterNumberMap[currEdge.start];
        int endClusterNumber=clusterNumberMap[currEdge.end];
        if(startClusterNumber!=endClusterNumber){
             nCluster=mergeCluster(currEdge.start,currEdge.end,clusterNumberMap,clusterMap);
            int clusterNStart=clusterNumberMap[currEdge.start];
            int clusterNEnd=clusterNumberMap[currEdge.end];
            
            minimalSpanningTree.add(currEdge);
        }
    }
    int treeSize=minimalSpanningTree.size();
    return minimalSpanningTree;
 
}

*/

TrailblazerPQueue<Edge> createEdges(Grid<double>& world,Set<Edge>&enqueuedEdges){
    
    // potential problems: doesn't return the right number of Edges
   
    TrailblazerPQueue<Edge> result;
    Edge currEdge;
    Edge reverseCurrEdge;
    Loc currLoc;
    Loc neighbour;
    Loc outOfBound=makeLoc(-1, -1);
    for(int i=0;i<world.numRows();i++){
        for(int j=0;j<world.numCols();j++){
            currLoc=makeLoc(i, j);
            for(Direction_M dir=northM;dir<=westM;dir++){
                neighbour=getAdjacentLoc_Maze(currLoc, dir, world);
                if(neighbour!=outOfBound){
                    currEdge=makeEdge(currLoc, neighbour);
                    reverseCurrEdge=makeEdge(neighbour, currLoc);
                    
                                        
                    if((!enqueuedEdges.contains(currEdge))&&(!enqueuedEdges.contains(reverseCurrEdge))){
                        result.enqueue(currEdge, mazeCost(currLoc, neighbour, world));
                      
        
                        enqueuedEdges.add(makeEdge(currLoc, neighbour));
                        enqueuedEdges.add(makeEdge(neighbour, currLoc));

                    }
                }
            }
        }
    }

    return result;
    
}



Loc getAdjacentLoc_Maze(Loc curr, Direction_M dir,Grid<double>& world){
    
    if(dir==northM&&world.inBounds(curr.row-1, curr.col)){
        Loc result=makeLoc(curr.row-1,curr.col);
        return result;
    }
    
    
    if(dir==eastM&&world.inBounds(curr.row, curr.col+1)){
        Loc result=makeLoc(curr.row,curr.col+1);
        return result;
    }
    
    if(dir==southM&&world.inBounds(curr.row+1, curr.col)){
        Loc result=makeLoc(curr.row+1,curr.col);
        return result;
    }
    if(dir==westM&&world.inBounds(curr.row, curr.col-1)){
        Loc result=makeLoc(curr.row,curr.col-1);
        return result;
    }
    else {
        Loc outOfBound= makeLoc(-1, -1);
        return outOfBound;
    }
    

    
    
    
}

int initBothMap(Map<Loc,int > &clusterNumberMap,Map<int,Set<Loc> >&clusterMap,Grid<double>& world){
    Loc currLoc;
    int clusterNum=1;
    
    for(int i=0;i<world.numRows();i++){
        for(int j=0;j<world.numCols();j++){
            currLoc=makeLoc(i, j);
            clusterNumberMap.put(currLoc, clusterNum);
            Set<Loc> currCluster;
            currCluster.add(currLoc);
            clusterMap[clusterNum]=currCluster;
            clusterNum++;
            
            
        }
    }
    return clusterNumberMap.size();
}
// Optimizing this function !!!  pointer   map cluster to point 

void mergeCluster(Loc start,Loc end,HashMap<Loc,Set<Loc> > &clusterMap){
    Set<Loc> mergedCluster=clusterMap[start]+clusterMap[end];
 
    
    
    clusterMap[start]=mergedCluster;
    clusterMap.remove(end);
    
    
    /*
    for (Loc item :mergedCluster){
        clusterMap[item]=mergedCluster;
    }*/
     
    
    
    
}






    
    //  POINTER BASED IMPLEMENTATION
    
    
    
    /*
     Set<Edge> createMaze(int numRows, int numCols) {
     
     
     // TODO: Fill this in!
     Set<Edge> minimalSpanningTree;
     Grid<double> world(numRows,numCols);
     TrailblazerPQueue< Edge > edges=createEdges(world);
     Map<Loc,Set<Loc>* > clusterMap;
     int numCluster=initClusterMap(clusterMap,world);
     // Maps Loc to Pointer which points to the cluster that the loc belongs to
     Edge currEdge;
     
     
     while (numCluster>1){
     currEdge=edges.dequeueMin();
     if(clusterMap[currEdge.start]!=clusterMap[currEdge.end]){
     
     mergeCluster(clusterMap[currEdge.start],clusterMap[currEdge.end],numCluster);
     
     
     
     minimalSpanningTree.add(currEdge);
     }
     }
     
     return minimalSpanningTree;
     
     }
     
     
     TrailblazerPQueue<Edge> createEdges(Grid<double>& world){
     
     
     TrailblazerPQueue<Edge> result;
     Edge currEdge;
     Loc currLoc;
     Loc neighbour;
     Loc outOfBound=makeLoc(-1, -1);
     for(int i=0;i<world.numRows();i++){
     for(int j=0;j<world.numCols();j++){
     currLoc=makeLoc(i, j);
     for(Direction_M dir=northM;dir<=westM;dir++){
     neighbour=getAdjacentLoc_Maze(currLoc, dir, world);
     if(neighbour!=outOfBound){
     currEdge=makeEdge(currLoc, neighbour);
     result.enqueue(currEdge, mazeCost(currLoc, neighbour, world));
     }
     }
     }
     }
     
     return result;
     
     }
     Loc getAdjacentLoc_Maze(Loc curr, Direction_M dir,Grid<double>& world){
     
     if(dir==northM&&world.inBounds(curr.row-1, curr.col)){
     Loc result=makeLoc(curr.row-1,curr.col);
     return result;
     }
     
     
     if(dir==eastM&&world.inBounds(curr.row, curr.col+1)){
     Loc result=makeLoc(curr.row,curr.col+1);
     return result;
     }
     
     if(dir==southM&&world.inBounds(curr.row+1, curr.col)){
     Loc result=makeLoc(curr.row+1,curr.col);
     return result;
     }
     if(dir==westM&&world.inBounds(curr.row, curr.col-1)){
     Loc result=makeLoc(curr.row,curr.col-1);
     return result;
     }
     else {
     Loc outOfBound= makeLoc(-1, -1);
     return outOfBound;
     }
     
     
     
     
     
     }
     
     int initClusterMap(Map<Loc,Set<Loc>* > &clusterMap,Grid<double>& world){
     Loc currLoc;
     
     for(int i=0;i<world.numRows();i++){
     for(int j=0;j<world.numCols();j++){
     currLoc=makeLoc(i, j);
     Set<Loc>* clusterPointer;
     clusterPointer= new Set<Loc>;
     Set<Loc> cluster;
     cluster.add(currLoc);
     *clusterPointer=cluster;
     clusterMap.put(currLoc, clusterPointer);
     
     
     
     }
     }
     return clusterMap.size();
     }
     
     void mergeCluster(Set<Loc >* &cluster1,Set<Loc >* &cluster2,int &numCluster){
     
     Set<Loc> newCluster;
     newCluster= *cluster1+*cluster2;
     
     Set<Loc>* newClusterPointer= new Set<Loc>;
     *newClusterPointer=newCluster;
     
     cluster1=newClusterPointer;
     cluster2=newClusterPointer;
     
     //  Free memory later
     
     numCluster--;
     }
     
     */



