#include "trailblazer.h"
#include "queue.h"
#include "hashmap.h"
#include "pqueue.h"
#include "hashset.h"

using namespace std;
bool depthFirstSearchHelper(BasicGraph& graph, Vertex* start, Vertex*& end,Vector<Vertex*>& path);
Vector<Vertex*> reconstructPath(Vertex* start, Vertex* end, HashMap<Vertex*,Vertex*> vCouple);
HashMap<Vertex*, double> initializeCost(BasicGraph& graph, Vertex*& start);

// This algorithm colors finds a path (any path) from one point to another on a graph, if the path exists
bool depthFirstSearchHelper(BasicGraph& graph, Vertex* start, Vertex*& end,Vector<Vertex*>& path) {
    if(start == end) {
        path.add(end);
        start->setColor(GREEN);
        return true;
    }
    if (start->getColor() != UNCOLORED) return false;
    start->setColor(YELLOW);
    path.add(start);
    start->setColor(GREEN);
    for( Edge* e : start->edges){
        if (depthFirstSearchHelper(graph, e->finish, end, path)) return true;
    }
    start->setColor(GRAY);
    path.removeBack();
    return false;
}

// This algorithm finds a path (any path) using DFS, if the path exists
Vector<Vertex*> depthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Vector<Vertex*> path;
    bool path_found = depthFirstSearchHelper(graph, start,  end, path);
    if(!path_found){
        throw("No path found");
    }
    return path;
}

// This function backtracks the path BFS path once you've reached the end vertex
Vector<Vertex*> reconstructPath(Vertex* start, Vertex* end, HashMap<Vertex*,Vertex*> vCouple) {
    Vertex* previous;
    Vertex* next = end;
    Vector<Vertex*> path;
    path.add(end);
    while(!vCouple.isEmpty() && previous != start){
        previous = vCouple[next];
        path.add(previous);
        next = previous;
    }
    path.reverse();
    return path;
}


// This algorithm finds a path (optimal distance path) using BFS, if a path exists
Vector<Vertex*> breadthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Queue<Vertex*> chosen;      // a queue to track the visied vertices
    start->setColor(YELLOW);
    chosen.add(start);
    HashMap<Vertex*,Vertex*> vCouple;
    while(!chosen.isEmpty()){
        Vertex* v = chosen.dequeue();
        v->setColor(GREEN);
        if (v == end) break;
        for( Edge* edge : v->edges){
            if (edge->finish->getColor() == UNCOLORED){
                chosen.enqueue(edge->finish);
                edge->finish->setColor(YELLOW);
                vCouple[edge->finish] = edge->start;
            }
        }
    }
    return reconstructPath(start, end, vCouple);
}

// Tis function iitializes all vertices to have an initial cost infinity, and for start, initial cost zero
HashMap<Vertex*, double> initializeCost(BasicGraph& graph, Vertex*& start){
    HashMap<Vertex*, double> cost;
    for(Vertex* v : graph.getVertexSet()){
        cost[v] = POSITIVE_INFINITY;
    }
    cost[start] = 0;
    return  cost;
}


// This algorithm finds a path (optimal cost path) using Dijkstra's algorithm, if a path exists
Vector<Vertex*> dijkstrasAlgorithm(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    PriorityQueue<Vertex*> chosen;
    HashMap<Vertex*, double> cost = initializeCost(graph, start);
    chosen.enqueue(start, cost[start]);
    start->setColor(YELLOW);
    HashMap<Vertex*,Vertex*> vCouple;

    while(!chosen.isEmpty()){
        Vertex* v = chosen.dequeue();
        v->setColor(GREEN);
        if (v == end) break;
        for( Edge* edge : v->edges){
            double newCost = cost[v] + edge->cost;
            if (cost[edge->finish] > newCost){
                cost[edge->finish] = newCost;
                vCouple[edge->finish] = edge->start;
                if(edge->finish->getColor() == UNCOLORED){
                    chosen.enqueue(edge->finish, newCost);
                    edge->finish->setColor(YELLOW);
                }else{
                    chosen.changePriority(edge->finish, newCost);
                }
            }
        }
    }
    return reconstructPath(start, end, vCouple);
}

// This algorithm finds a path (optimal heuristic cost path) using Dijkstra's algorithm, if a path exists
Vector<Vertex*> aStar(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    PriorityQueue<Vertex*> chosen;
    HashMap<Vertex*, double> cost = initializeCost(graph, start);
    chosen.enqueue(start, cost[start] + heuristicFunction(start, end));
    start->setColor(YELLOW);
    HashMap<Vertex*,Vertex*> vCouple;

    while(!chosen.isEmpty()){
        Vertex* v = chosen.dequeue();
        v->setColor(GREEN);
        if (v == end) break;
        for( Edge* edge : v->edges){
            double newCost = cost[v] + edge->cost ;
            if (cost[edge->finish] > newCost){
                cost[edge->finish] = newCost;
                vCouple[edge->finish] = edge->start;
                if(edge->finish->getColor() == UNCOLORED){
                    chosen.enqueue(edge->finish, newCost + heuristicFunction(edge->finish, end));
                    edge->finish->setColor(YELLOW);
                }else{
                    chosen.changePriority(edge->finish, newCost + heuristicFunction(edge->finish, end));
                }
            }
        }

    }
    return reconstructPath(start, end, vCouple);
}

PriorityQueue<Edge*> getallEdges(BasicGraph& graph){
    PriorityQueue<Edge*> allEdges;
    for(Edge* edge : graph.getEdgeSet()){
        allEdges.enqueue(edge, edge->cost);
    }
    return allEdges;
}

//This function creates a new cluster of vertices
void createNewCluster(Vector<HashSet<Vertex*>>& clusters, Edge* edge, Set<Edge*>& mst) {
    HashSet<Vertex*> newCluster;
    newCluster.add(edge->start);
    newCluster.add(edge->finish);
    clusters.add(newCluster);
    mst.add(edge);
}

//This function implements kruskal's algorithm 
Set<Edge*> kruskal(BasicGraph& graph) {
    PriorityQueue<Edge*> allEdges = getallEdges(graph);
    Set<Edge*> mst;
    Vector<HashSet<Vertex*>> clusters;
    bool startFound = false;
    bool endFound = false;
    int start = -1;
    int end = -1;
    createNewCluster(clusters, allEdges.dequeue(), mst);
    while (!allEdges.isEmpty()){
        Edge* edge = allEdges.dequeue();

        for (int i = 0; i < clusters.size(); i++) {
            if (clusters[i].contains(edge->start)) {
                startFound = true;
                start = i;
            }
            if (clusters[i].contains(edge->end)) {
                endFound = true;
                end = i;
            }
            if (endFound && startFound) break;
        }
        if (endFound && startFound) {
            if (start != end) {
                clusters[start] += clusters[end];
                clusters.remove(end);
                mst.add(edge);
            }
        } else if (startFound) {
            clusters[start].add(edge->end);
            mst.add(edge);
        } else if (endFound) {
            clusters[end].add(edge->start);
            mst.add(edge);
        } else {
            createNewCluster(clusters, edge, mst);
        }
        startFound = false;
        endFound = false;
    }
    return mst;
}
