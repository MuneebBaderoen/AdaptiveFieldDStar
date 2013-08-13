
#include "Pathfinder.h"

namespace ADStar{

    void Pathfinder::findPath(int s, int e, WeightedGraph & g){

        graph = g;

        //initialise pathfinder
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

            std::cout << "start: " << start << std::endl;
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
            std::cout << "Looking at " << cur_index << ", successor of " << start << std::endl;
            if(graph[cur_index].key.first + boost::get(boost::edge_weight, graph, *it) < min_cost){

                min_cost = graph[cur_index].key.first + boost::get(boost::edge_weight, graph, *it);
                min_succ = cur_index;

                }

            }

            //move to succesor
            path.push_back(start);
            start = min_succ;

            //scan for changes

            //if changes
                //adjust km by heuristic
                //assign last to cur
                //for all changed edges
                    //update edge cost
                    //update node
                //compute path

        }

        path.push_back(start);
        std::cout << "Shortest Path:" << std::endl;
        for(std::vector<int>::iterator it = path.begin(); it != path.end(); ++it)
            std::cout << *it << " ";
        std::cout << std::endl;

    }

    //Finds a path from the goal to the start
    void Pathfinder::computePath(){

    //if not at start node
    while(p_queue.top().key < computeKey(start) || graph[start].key.second > graph[start].key.first){

        //compare old and new keys for changes
        const Node top = p_queue.top();
        node_key key_old = top.key;
        node_key key_new = computeKey(top.index);

        //if key has changed
        if(key_old < key_new){

            //update the current node on the queue

            }
        //if the graph's key has different values
        else if(graph[top.index].key.first > graph[top.index].key.second){

            //update node and remove from queue
            graph[top.index].key.first = graph[top.index].key.second;
            p_queue.pop();
            on_queue.erase(top.index);

            std::cout << "Node " << top.index << std::endl;
            std::cout << "Has in degree " << boost::in_degree(top.index, graph) << std::endl;
            std::cout << "On Queue: ";
            for(Dary_queue::iterator it = p_queue.begin(); it != p_queue.end(); ++it)
                std::cout << it->index << " ";

            std::cout << std::endl;

            std::pair<ie_iterator, ie_iterator> pred_range = boost::in_edges(top.index, graph);

            //for all predecesors
            for(ie_iterator it = pred_range.first; it != pred_range.second; ++it){

                int cur_index = boost::source(*it, graph);
                if(cur_index != end){           //if that node is not goal

                    graph[cur_index].key.second = std::min(graph[cur_index].key.second, boost::get(boost::edge_weight, graph, *it)  + top.key.first); //rhs = min g and cost of any successor

                }

            //update node
            updateNode(cur_index);
            }

        }
        else{

        float old_g = graph[top.index].key.first;
        graph[top.index].key.first = INFINITY;

        std::pair<ie_iterator, ie_iterator> pred_range = boost::in_edges(top.index, graph);

        //for all predecesors and cur node
        if(graph[top.index].key.second == old_g)
            if(top.index != end)           //if that node is not goal
                graph[top.index].key.second = minSucCost(top.index); //rhs = min g and cost of any successor

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
        }

    }

    }

    float Pathfinder::heuristic(int s_index, int e_index){

        return 1;

    }

    Pathfinder::node_key Pathfinder::computeKey(int index){

    return node_key(std::min(graph[index].key.first, graph[index].key.second) + heuristic(start, index) + km
                    , std::min(graph[index].key.first, graph[index].key.second));

    }

    //change the key of graph node
    void Pathfinder::updateNode(int index){

        if(graph[index].key.first != graph[index].key.second && on_queue.find(index) != on_queue.end()){}
            //update node priority
        else if(graph[index].key.first != graph[index].key.second && on_queue.find(index) == on_queue.end()){
            //insert node onto queue
            graph[index].key = computeKey(index);
            Dary_queue::handle_type h = p_queue.push(Node(index, computeKey(index)));
            on_queue[index] = h;
            }
        else if(graph[index].key.first == graph[index].key.second && on_queue.find(index) != on_queue.end()){
            //remove node from queue
            p_queue.erase(on_queue[index]);
            on_queue.erase(index);

        }


    }

    /*float Pathfinder::minPredCost(int index){

        std::pair<ie_iterator, ie_iterator> pred_range = boost::in_edges(index, graph);
        float min_cost = INFINITY;

        for(ie_iterator it = pred_range.first; it != pred_range.second; ++it){

            int pred_index = boost::target(*it, graph);
            if(min_cost > boost::get(boost::edge_weight, graph, *it) + graph[index].key.first)
                min_cost = boost::get(boost::edge_weight, graph, *it) + graph[index].key.first;

        }

        return min_cost;

    }*/

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
