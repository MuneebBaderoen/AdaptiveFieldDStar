//Performs the D* Lite algorithm on a given mesh
//Daanyaal du Toit
//Created: 7 July 2013
//Last Modified: 23 September 2013

#ifndef PATHFINDER_H_INCLUDED
#define PATHFINDER_H_INCLUDED

#include "boost/heap/d_ary_heap.hpp"
#include "UpdateHelpers.h"
#include "NodeManager.h"
#include "WeightedGraph.h"
#include <iostream>
#include <iomanip>

namespace ADStar{

template<class N, class E>
class Pathfinder{

    public:
    //typedefs

        typedef typename boost::graph_traits<WeightedGraph>::out_edge_iterator oe_iterator;
        typedef typename boost::graph_traits<WeightedGraph>::in_edge_iterator ie_iterator;

        //Constructor
        Pathfinder(E & g): manager(g){}

    private:
        //typedefs
        typedef typename boost::heap::d_ary_heap<N, boost::heap::mutable_<true>
                                , boost::heap::arity<4>
                                , boost::heap::compare<std::greater<N> >
                                > Dary_queue;
        typedef typename Dary_queue::handle_type q_handle_type;
        typedef typename NodeManager<N, E>::node_handle node_handle;

        //Pathfinder members
        NodeManager<N, E> manager;
        std::vector<node_handle> path;
        //E & graph;
        node_handle last, start, end;
        float km, cost;
        std::map<node_handle, q_handle_type> on_queue;          //maps from node indices to nodes on the heap
        Dary_queue p_queue;

    public:

//---------------------------------------------------------------------------------
//Main pathfinding method
//---------------------------------------------------------------------------------

    void findPath(node_handle s, node_handle e){

        cost = 0;

        //initialise pathfinder
        start = s;
        end = e;

        last = start;
        path.clear();
        on_queue.clear();
        p_queue.clear();
        manager.setRHS(end, 0);
        manager.setG(end, manager.heuristic(start, end));

        on_queue[end] = p_queue.push(N(end, manager.heuristic(start, end), 0));

        //perform initial path computation
        computePath();

    }

//---------------------------------------------------------------------------------
//Move along the least expensive path
//---------------------------------------------------------------------------------

    void move(){

        if(start != end){

        //exit if no path
        if(manager.getRHS(start) == INFINITY){

            std::cout << "No path found." << std::endl;
            return;

        }

        /*if(start == 34003)
            std::cout << "Key of 34002 : " << graph[34002].key << std::endl;*/

        //calc min traversal to one of cur's successors and assign to start
        node_handle min_succ = manager.getMinSucc(start);

        /*std::cout << "Min Succ = " << min_succ << " g = " << manager.getG(min_succ);
        char a;
        std::cin >> a;*/

        //move to succesor
        path.push_back(start);
        cost += manager.getCost(start, min_succ);
        //std::cout << "Cost from " << start << " to " << min_succ << " = " << min_cost - graph[min_succ].key.first << std::endl;
        //std::cout << "Accum Cost = " << cost << std::endl;
        start = min_succ;
        //std::cout << "Moving to " << start << std::endl;

        if(start == end)
            path.push_back(start);

        }

        //std::cout << "Move complete" << std::endl;

    }

//---------------------------------------------------------------------------------
//Handle changes polled by the NodeUpdator
//---------------------------------------------------------------------------------

    void handleChanges(const std::vector<EdgeChange> & c){

        using namespace boost;

        if(!c.empty()){

            //std::cout << "Change detected" << std::endl;

            manager.km += manager.heuristic(last,start);
            last = start;
            //for all changed (directed) edges
            for(std::vector<EdgeChange>::const_iterator it = c.begin(); it != c.end(); ++it){

                //std::cout << "Processing start = " << it->start << ", end = " << it->end << std::endl;
                //std::cout << "Edge exists " << edge(it->start, it->end, graph).second << std::endl;

                float old_weight = manager.getCost(it->start, it->end);

                //std::cout << "Old weight: " << get(edge_weight, graph, e) << std::endl;

                //update edge cost
                manager.setCost(it->start, it->end, it->weight);

                //std::cout << "Change edge weight from " << old_weight << " to " << get(edge_weight, graph, e) << std::endl;

                //std::cout << "start.rhs " << std::setprecision(10) << graph[it->start].key.second << std::endl;
                //std::cout << "end.g + old " << graph[it->end].key.first + old_weight << std::endl;

                //std::cout << "end.rhs " << graph[it->end].key.second << std::endl;
                //std::cout << "start.g + old " << graph[it->start].key.first + old_weight << std::endl;

                if(it->start != end){

                    if(old_weight > it->weight){//std::cout << "Weight lowered." << std::endl;
                        //graph[it->start].key.second = std::min(graph[it->start].key.second, it->weight + graph[it->end].key.first);
                        manager.setRHS(it->start, std::min(manager.getRHS(it->start), it->weight + manager.getG(it->end)));}
                    else if(manager.getRHS(it->start) == (old_weight + manager.getG(it->end))){//std::cout << "Accurate lookahead" << std::endl;
                        //graph[it->start].key.second = minSucCost(it->start);
                        manager.setRHS(it->start, manager.getCost(it->start, manager.getMinSucc(it->start)) + manager.getG(manager.getMinSucc(it->start)));}

                }
                /*if(it->end != end){

                    if(old_weight > it->weight){std::cout << "Weight lowered." << std::endl;
                        graph[it->end].key.second = std::min(graph[it->end].key.second, it->weight + graph[it->start].key.first);}
                    else if(std::fabs(graph[it->end].key.second - (old_weight + graph[it->start].key.first)) == 0){std::cout << "Accurate lookahead" << std::endl;
                        graph[it->end].key.second = minSucCost(it->end);}

                }*/

                //std::cout << "Start key: " << graph[it->start].key << std::endl;

                //update node
                updateNode(it->start);
                //updateNodeD(it->end);

            }

            //std::cout << "Computing path" << std::endl;
            computePath();

        }

    }

//---------------------------------------------------------------------------------
//Get path and cost of path
//---------------------------------------------------------------------------------

    void printPath(){

        std::cout << "Starting Path:" << std::endl << "================================================" << std::endl;

        node_handle cur = start;
        node_handle tempCur;
        float cost = start->key.second;
        std::vector<CGAL::Point_3<K> > path;
        path.push_back(cur->point());
        bool interpol = false;
        PathPoly_3::Halfedge_handle intEdge;

        while(cur != end){

            if(interpol){

                std::cout << "Processing interpolated point " << tempCur->point() << std::endl;
                PathPoly_3::Halfedge_around_vertex_circulator it = tempCur->vertex_begin();
                costPair lowCost(INFINITY, intPair());
                PathPoly_3::Facet_handle lowTri;

                do{

                    if(it->facet() != PathPoly_3::Facet_handle() && lowCost.first > manager.getCost(tempCur, it->facet())){
                        std::cout << "Processing triangle with weight " << it->facet()->weight << std::endl;
                        lowCost = manager.getCostPair(tempCur, it->facet());
                        lowTri = it->facet();
                        }
                    ++it;

                }
                while(it != tempCur->vertex_begin());

                std::cout << std::setprecision(6) << "Next point: " << lowCost.second.first.point() << std::endl;

                nextPair next = manager.getNextVert(lowTri, lowCost);
                interpol = next.first;
                path.push_back(lowCost.second.first.point());
                if(!interpol){

                    //Find point on original mesh
                    PathPoly_3::Facet_handle f = intEdge->facet();
                    PathPoly_3::Halfedge_around_facet_circulator fit1 = f->facet_begin();

                    do{

                        if(isEqual(*(fit1->vertex()), *(next.second.first))){

                            cur = fit1->vertex();
                            break;

                        }
                        ++fit1;

                    }
                    while(fit1 != f->facet_begin());

                    if(intEdge->opposite()->facet() != PathPoly_3::Facet_handle()){

                    f = intEdge->opposite()->facet();
                    PathPoly_3::Halfedge_around_facet_circulator fit2 = f->facet_begin();

                    do{

                        if(isEqual(*(fit2->vertex()), *(next.second.first))){

                            cur = fit2->vertex();
                            break;

                        }
                        ++fit2;

                    }
                    while(fit2 != f->facet_begin());

                }

                }
                else{
                    tempCur = next.second.first;
                    intEdge = next.second.second;
                    }


            }
            else{

                std::cout << "Processing mesh point " << cur->point() << std::endl;

                PathPoly_3::Halfedge_around_vertex_circulator it = cur->vertex_begin();
                costPair lowCost(INFINITY, intPair());
                PathPoly_3::Facet_handle lowTri;

                //get triangle with lowest cost func
                do{

                    if(it->facet() != PathPoly_3::Facet_handle() && lowCost.first > manager.getCost(cur, it->facet())){
                        std::cout << "Processing triangle with weight " << it->facet()->weight << std::endl;
                        lowCost = manager.getCostPair(cur, it->facet());
                        lowTri = it->facet();
                        }
                    ++it;

                }
                while(it != cur->vertex_begin());

                std::cout << std::setprecision(6) << "Next point: " << lowCost.second.first.point() << std::endl;

                nextPair next = manager.getNextVert(lowTri, lowCost);
                interpol = next.first;
                path.push_back(lowCost.second.first.point());
                if(!interpol)
                    cur = next.second.first;
                else{
                    tempCur = next.second.first;
                    intEdge = next.second.second;
                    }

            }

        }

        std::cout << "Shortest Path:" << std::endl;
        for(typename std::vector<CGAL::Point_3<K> >::iterator it = path.begin(); it != path.end(); ++it)
            std::cout << "(" << *it << ") ";
        std::cout << std::endl;

        std::cout << "Path cost: " << cost << std::endl;

    }

    inline bool atEnd(){return start == end;}

    private:

//---------------------------------------------------------------------------------
//Change the key of graph node
//---------------------------------------------------------------------------------

    void updateNode(node_handle index){

        if(manager.getG(index) != manager.getRHS(index)){  //if key is inconsistent
            if(on_queue.find(index) != on_queue.end()){         //if already on queue
            //update node priority

            p_queue.update(on_queue[index], N(index, manager.computeKey(start, index)));
            index->key = manager.computeKey(start, index);
            //std::cout << "Updating key for node " << index << " to " << graph[index].key << std::endl;

            }
            else{
            //insert node onto queue
            on_queue[index] = p_queue.push(N(index, manager.computeKey(start, index)));
            index->key = manager.computeKey(start, index);
            std::cout << "Inserting node " << index->point() << " with key " << manager.computeKey(start, index) << std::endl;
            }

        }
        else if(on_queue.find(index) != on_queue.end()){        //if key consistent and on queue
            //remove node from queue
            p_queue.erase(on_queue[index]);
            on_queue.erase(index);
            //std::cout << "Removing node " << index << std::endl;

        }

    }

//---------------------------------------------------------------------------------
//Finds a path from the goal to the start
//---------------------------------------------------------------------------------

    void computePath(){

    while(!p_queue.empty() && (node_key(p_queue.top().getG(), p_queue.top().getRHS()) < manager.computeKey(start, start) || manager.getRHS(start) > manager.getG(start))){

        std::cout << "Processing Node " << p_queue.top().index->point() << std::endl;
        //compare old and new keys for changes
        node_handle top_index = p_queue.top().index;
        node_key key_old(p_queue.top().getG(), p_queue.top().getRHS());
        node_key key_new = manager.computeKey(start, top_index);

        std::cout << "Old key of " << top_index->point() << " : " << key_old << std::endl;
        std::cout << "New key of " << top_index->point() << " : " << key_new << std::endl;

        //char a;
        //std::cin >> a;

        //if key has changed
        if(key_old < key_new){

            //update the current node on the queue
            std::cout << "Updating node " << top_index->point() << " with key " << key_new << std::endl;
            p_queue.update(on_queue[top_index], N(top_index, key_new));

            }
        //if the graph's key has different values
        else if(manager.getG(top_index) > manager.getRHS(top_index)){

            std::cout << "Adjusting inconsistent key " << key_old << " of " << top_index->point() << std::endl;
            //update node and remove from queue
            manager.setG(top_index, manager.getRHS(top_index));
            p_queue.pop();
            on_queue.erase(top_index);

            //std::cin >> a;

            std::vector<node_handle> toProcess = manager.getPreds(top_index, false);
            //std::cout << toProcess.size() << std::endl;
            //std::cin >> a;
            for(typename std::vector<node_handle>::iterator it = toProcess.begin(); it != toProcess.end(); ++it){

                //std::cout << top_index->point() << " != " << (*it)->point() << " : ";
                //std::cout << (*it != end) << std::endl;

                /*std::cout << "Setting RHS of " << (*it)->point();
                std::cout << " to min(" << manager.getRHS(*it) << ", ";
                std::cout << manager.getG(top_index) << " + " << manager.getCost(*it, top_index) << ")" << std::endl;*/

                if(*it != end){

                    PathPoly_3::Halfedge_around_vertex_circulator vit = (*it)->vertex_begin();

                    do{

                        if(vit->facet() != PathPoly_3::Facet_handle()){

                            std::cout << "Processing " << (*it)->point() << " on triangle with weight " << vit->facet()->weight << std::endl;
                            manager.setRHS(*it, std::min(manager.getRHS(*it), manager.getCost(*it, vit->facet())));

                            }

                        ++vit;

                    }
                    while(vit != (*it)->vertex_begin());

                    std::cout << "RHS of " << (*it)->point() << " set to " << manager.getRHS(*it) << std::endl;

                }


                //std::cin >> a;

                updateNode(*it);

                }

        }
        else{

        //float old_g = manager.getG(top_index);
        manager.setG(top_index, INFINITY);

        //std::cout << "Key of " << top.index << " : " << graph[top.index].key << std::endl;

        std::vector<node_handle> toProcess = manager.getPreds(top_index, true);
        for(typename std::vector<node_handle>::iterator it = toProcess.begin(); it != toProcess.end(); ++it){

            if(*it != end){

            PathPoly_3::Halfedge_around_vertex_circulator vit = (*it)->vertex_begin();

            do{

                if(vit->facet() != PathPoly_3::Facet_handle()){

                    std::cout << "Processing " << (*it)->point() << " on triangle with weight " << vit->facet()->weight << std::endl;
                    manager.setRHS(*it, std::min(manager.getRHS(*it), manager.getCost(*it, vit->facet())));

                }

            }
            while(vit != (*it)->vertex_begin());

            }

            updateNode(*it);

        }

        }

    }

    }

};

}

#endif // PATHFINDER_H_INCLUDED
