//Performs the D* Lite algorithm on a given mesh
//Daanyaal du Toit
//Created: 7 July 2013
//Last Modified: 18 August 2013

#include "Pathfinder.h"

namespace ADStar{

//---------------------------------------------------------------------------------
//Main pathfinding method
//---------------------------------------------------------------------------------

    void Pathfinder::findPath(int s, int e, WeightedGraph & g){

        float cost = 0;

        graph = g;

        //initialise pathfinder
        changed = false;
        start = s;
        end = e;

        int last = start;
        path.clear();
        on_queue.clear();
        p_queue.clear();
        km = 0;
        graph[end].key.second = 0;
        graph[end].key.first = heuristic(start, end);

        Dary_queue::handle_type h = p_queue.push(Node(end, heuristic(start, end), 0));
        on_queue[end] = h;

        //perform initial path computation
        computePath();

        while(start != end){

            //exit if no path
            if(graph[start].key.second == INFINITY){

                std::cout << "No path found." << std::endl;
                return;

            }

            //calc min traversal to one of cur's successors and assign to start
            std::pair<oe_iterator, oe_iterator> suc_range = boost::out_edges(start, graph);

            float min_cost = INFINITY;
            int min_succ;

            for(oe_iterator it = suc_range.first; it != suc_range.second; ++it){

            int cur_index = boost::target(*it, graph);
            //std::cout << "Looking at " << cur_index << ", successor of " << start << std::endl;
            //std::cout << "Cost = " << graph[cur_index].key.first + boost::get(boost::edge_weight, graph, *it) << std::endl;
            if(graph[cur_index].key.first + boost::get(boost::edge_weight, graph, *it) < min_cost){

                min_cost = graph[cur_index].key.first + boost::get(boost::edge_weight, graph, *it);
                min_succ = cur_index;

                }

            }

            //std::cout << "Min cost: " << min_cost << std::endl;
            //std::cout << "Min suc cost method: " << minSucCost(start) << std::endl;
            //std::cout << "Min succ: " << min_succ << std::endl;

            //move to succesor
            path.push_back(start);
            cost += min_cost - graph[min_succ].key.first;
            start = min_succ;

            //scan for changes
            if(changed){

                km += heuristic(last,start);
                last = start;
                //for all changed edges
                    //update edge cost
                    //update node
                //compute path
                computePath();

            }

        }

        path.push_back(start);
        std::cout << "Shortest Path:" << std::endl;
        for(std::vector<int>::iterator it = path.begin(); it != path.end(); ++it)
            std::cout << *it << " ";
        std::cout << std::endl;

        std::cout << "Path cost: " << cost << std::endl;

    }

//---------------------------------------------------------------------------------
//Finds a path from the goal to the start
//---------------------------------------------------------------------------------

    void Pathfinder::computePath(){

    while(!p_queue.empty() && (p_queue.top().key < computeKey(start) || graph[start].key.second > graph[start].key.first)){

        //compare old and new keys for changes
        const Node top = p_queue.top();
        node_key key_old = top.key;
        node_key key_new = computeKey(top.index);

        //std::cout << "Old key of " << top.index << " : " << key_old << std::endl;
        //std::cout << "New key of " << top.index << " : " << key_new << std::endl;

        //if key has changed
        if(key_old < key_new){

            //update the current node on the queue
            //std::cout << "Updating node " << top.index << " with key " << key_new.first << ", " << key_new.second << std::endl;
            p_queue.update(on_queue[top.index], Node(top.index, key_new));

            }
        //if the graph's key has different values
        else if(graph[top.index].key.first > graph[top.index].key.second){

            //std::cout << "Adjusting inconsistent key of " << top.index << std::endl;
            //update node and remove from queue
            graph[top.index].key.first = graph[top.index].key.second;
            p_queue.pop();
            on_queue.erase(top.index);

            std::pair<ie_iterator, ie_iterator> pred_range = boost::in_edges(top.index, graph);

            //for all predecesors
            for(ie_iterator it = pred_range.first; it != pred_range.second; ++it){

                int cur_index = boost::source(*it, graph);
                if(cur_index != end){           //if that node is not goal

                    graph[cur_index].key.second = std::min(graph[cur_index].key.second, boost::get(boost::edge_weight, graph, *it)  + top.key.first); //rhs = min g and cost of any successor
                    //std::cout << "Set key of " << cur_index << " to " << graph[cur_index].key << std::endl;

                }

            //update node
            updateNode(cur_index);
            }

        }
        else{

        float old_g = graph[top.index].key.first;
        graph[top.index].key.first = INFINITY;

        std::cout << "Key of " << top.index << " : " << graph[top.index].key << std::endl;

        std::pair<ie_iterator, ie_iterator> pred_range = boost::in_edges(top.index, graph);

        //for all predecesors and cur node
        for(ie_iterator it = pred_range.first; it != pred_range.second; ++it){

            int cur_index = boost::source(*it, graph);
            //if that node rhs = cost to that node from cur + g_cur
            if(graph[cur_index].key.second == boost::get(boost::edge_weight, graph, *it) + old_g)
                if(cur_index != end){           //if that node is not goal

                    graph[cur_index].key.second = minSucCost(cur_index); //rhs = min g and cost of any successor

                }

            //update node
            updateNode(cur_index);
            }

        if(graph[top.index].key.second == old_g)
            if(top.index != end)           //if that node is not goal
                graph[top.index].key.second = minSucCost(top.index); //rhs = min g and cost of any successor

        updateNode(top.index);

        }

        //std::cout << "Queue: " <<std::endl;
        //for(Dary_queue::iterator it = p_queue.begin(); it != p_queue.end(); ++it)
        //        std::cout << it->index << " ";
        //std::cout << std::endl;

        //std::cout << "Top of queue: " << p_queue.top().index << std::endl;

             //if not at start node or has nodes in queue
        //std::cout << "Queue empty: " << p_queue.empty() << std::endl;
        //std::cout << "Top key less than start: " << (p_queue.top().key)<< " < " << computeKey(start) << " : " << (p_queue.top().key < computeKey(start)) << std::endl;
        //std::cout << "Inconsistent start key: " << (graph[start].key.second > graph[start].key.first) << std::endl;

    }

    }

//---------------------------------------------------------------------------------
//Estimate distance from start to end
//---------------------------------------------------------------------------------

    float Pathfinder::heuristic(int s_index, int e_index){

        return sqrt(pow(graph[e_index].x - graph[s_index].y, 2) + pow(graph[e_index].y - graph[s_index].y, 2)
                    + pow(graph[e_index].z - graph[s_index].z, 2));

    }

//---------------------------------------------------------------------------------
//Compute priority key of node
//---------------------------------------------------------------------------------

    Pathfinder::node_key Pathfinder::computeKey(int index){

    return node_key(std::min(graph[index].key.first, graph[index].key.second) + heuristic(start, index) + km
                    , std::min(graph[index].key.first, graph[index].key.second));

    }

//---------------------------------------------------------------------------------
//Change the key of graph node
//---------------------------------------------------------------------------------

    void Pathfinder::updateNode(int index){

        if(graph[index].key.first != graph[index].key.second){  //if key is inconsistent
            if(on_queue.find(index) != on_queue.end()){         //if already on queue
            //update node priority

            graph[index].key = computeKey(index);
            p_queue.update(on_queue[index], Node(index, computeKey(index)));
            //std::cout << "Updating key for node " << index << " to " << graph[index].key << std::endl;

            }
            else{
            //insert node onto queue
            graph[index].key = computeKey(index);
            Dary_queue::handle_type h = p_queue.push(Node(index, computeKey(index)));
            on_queue[index] = h;
            //std::cout << "Inserting node " << index << " with key " << graph[index].key << std::endl;
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
//Calculate the min cost to a successor node
//---------------------------------------------------------------------------------

    float Pathfinder::minSucCost(int index){

        std::pair<oe_iterator, oe_iterator> suc_range = boost::out_edges(index, graph);
        float min_cost = INFINITY;

        for(oe_iterator it = suc_range.first; it != suc_range.second; ++it){

            int suc_index = boost::target(*it, graph);
            if(min_cost > boost::get(boost::edge_weight, graph, *it) + graph[suc_index].key.first)
                min_cost = boost::get(boost::edge_weight, graph, *it) + graph[suc_index].key.first;

        }

        return min_cost;

    }

}
