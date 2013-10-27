//Abstract node representation away from pathfinder logic
//Daanyaal du Toit
//Created: 4 September 2013
//Last modified: 23 September 2013

#ifndef NODEMANAGER_H_INCLUDED
#define NODEMANAGER_H_INCLUDED

#include "WeightedGraph.h"
#include "Helpers.h"
#include <cmath>
#include <vector>

namespace ADStar{

typedef std::pair<PathPoly_3::Vertex, PathPoly_3::Halfedge_handle> intPair;
typedef std::pair<PathPoly_3::Vertex_handle, PathPoly_3::Halfedge_handle> intHanPair;
typedef std::pair<float, intPair> costPair;
typedef std::pair<bool, intHanPair> nextPair;

//Node for use of the priority queue
//Contains handle for vertex and key
template <typename H>
    struct Node{

            typedef H handle_type;
            H index;                      //graph handle for the current vertex
            node_key key;                   //the g and rhs value of the vertex

            Node(const H & i, node_key k): index(i), key(k){}
            Node(const H & i, float g, float rhs): index(i), key(g, rhs){}  //initial node constructor

            float getG(){return key.first;}
            float getRHS(){return key.second;}

            bool operator < (const Node & n) const{

                return key < n.key;

            }

            bool operator > (const Node & n) const{

                return n.key < key;

            }

        };

template<>
    struct Node<PathPoly_3::Vertex_handle>{

        typedef PathPoly_3::Vertex_handle handle_type;
        handle_type index;

        Node(const handle_type & i, node_key k): index(i){}
        Node(const handle_type & i, float g, float rhs): index(i){}

        float getG() const{return index->key.first;}
        float getRHS() const{return index->key.second;}

        bool operator < (const Node & n) const{

                return index->key < n.index->key;

            }

        bool operator > (const Node & n) const{

                return n < *this;

        }

    };

//Template for node manager to allow abstraction
template <typename N, typename E>
class NodeManager{

    public:
        typedef typename N::handle_type node_handle;
        float km;
        NodeManager(E & e): envir(e){}
        E & envir;
        void setG(node_handle h, float value);
        float getG(node_handle h) const;
        void setRHS(node_handle h, float value);
        float getRHS(node_handle h) const;
        void setCost(node_handle s, node_handle e, float value);
        float getCost(node_handle s, node_handle e) const;
        node_handle getMinSucc(node_handle h) const;
        std::vector<node_handle> getPreds(node_handle h, bool incl);
        float heuristic(node_handle s_index, node_handle e_index) const;
        node_key computeKey(node_handle start, node_handle h) const;

};

//Specialised for D* Lite on weighted graphs
template<>
class NodeManager<Node<int>, WeightedGraph>{

    public:
    typedef int node_handle;
    float km;
    private:
    typedef typename boost::graph_traits<WeightedGraph>::out_edge_iterator oe_iterator;
    typedef typename boost::graph_traits<WeightedGraph>::in_edge_iterator ie_iterator;

    WeightedGraph & envir;

    public:
    NodeManager(WeightedGraph & e): envir(e){}
    void setG(int h, float value){envir[h].key.first = value;}
    float getG(int h) const{return envir[h].key.first;}
    void setRHS(int h, float value){envir[h].key.second = value;}
    float getRHS(int h) const {return envir[h].key.second;}

//---------------------------------------------------------------------------------
//Get the cost of traversal between two vertices
//---------------------------------------------------------------------------------

    float getCost(int s, int e) const{

        if(s == e)
            return 0;
        else if(boost::edge(s, e, envir).second)     //if edge exists
            return boost::get(boost::edge_weight, envir, boost::edge(s, e, envir).first);

        return INFINITY;

    }

//---------------------------------------------------------------------------------
//Set cost of traversal between two vertices
//---------------------------------------------------------------------------------

    void setCost(int s, int e, float value){

        boost::put(boost::edge_weight, envir, boost::edge(s, e, envir).first, value);

    }

//---------------------------------------------------------------------------------
//Return the least costly successor of a vertex
//---------------------------------------------------------------------------------

    int getMinSucc(int index) const{

        std::pair<oe_iterator, oe_iterator> suc_range = boost::out_edges(index, envir);
        float min_cost = INFINITY;
        int min_succ = 0;

        //iterate over all outgoing edges and replace current index if cost is lower
        for(oe_iterator it = suc_range.first; it != suc_range.second; ++it){

            int suc_index = boost::target(*it, envir);
            if(min_cost > boost::get(boost::edge_weight, envir, *it) + envir[suc_index].key.first){
                min_cost = boost::get(boost::edge_weight, envir, *it) + envir[suc_index].key.first;
                min_succ = suc_index;

                }

        }

        return min_succ;

    }

//---------------------------------------------------------------------------------
//Estimates the distance between two vertices (Euclidean heuristic)
//---------------------------------------------------------------------------------

    float heuristic(int s_index, int e_index) const{

        return sqrtf(pow(envir[e_index].x - envir[s_index].x, 2) + pow(envir[e_index].y - envir[s_index].y, 2)
                     + pow(envir[e_index].z - envir[s_index].z, 2));

    }

//---------------------------------------------------------------------------------
//Compute priority key of node
//---------------------------------------------------------------------------------

    node_key computeKey(int start, int index) const{

    return node_key(std::min(envir[index].key.first, envir[index].key.second) + heuristic(start, index) + km
                    , std::min(envir[index].key.first, envir[index].key.second));

    }

//---------------------------------------------------------------------------------
//Return a list of a vertex's predecessors
//Bool toggles self-inclusion
//---------------------------------------------------------------------------------

    std::vector<int> getPreds(int h, bool incl){

        std::vector<int> toProcess;
        std::pair<ie_iterator, ie_iterator> pred_range = boost::in_edges(h, envir);

            //for all predecesors
            for(ie_iterator it = pred_range.first; it != pred_range.second; ++it){

                int cur_index = boost::source(*it, envir);
                toProcess.push_back(cur_index);

            }

        if(incl)        //process self if needed
            toProcess.push_back(h);

        return toProcess;

    }

};

//Specialised to Field D* on weighted meshes
template<>
class NodeManager<Node<PathPoly_3::Vertex_handle>, PathPoly_3 >{

    public:
    typedef PathPoly_3::Vertex_handle node_handle;
    float km;
    private:
    PathPoly_3 & envir;
    float minWeight = INFINITY;

    public:

    NodeManager(PathPoly_3 & e): envir(e){

        for(PathPoly_3::Facet_iterator it = e.facets_begin(); it != e.facets_end(); ++it)
            if(minWeight > it->weight)
                minWeight = it->weight;

    }
    PathPoly_3 & getEnvir(){return envir;}
    void setG(node_handle h, float value){h->key.first = value;}
    float getG(node_handle h){return h->key.first;}
    void setRHS(node_handle h, float value){h->key.second = value;}
    float getRHS(node_handle h){return h->key.second;}

    float getMinWeight(){return minWeight;}

//---------------------------------------------------------------------------------
//Optimise the x variable in the cost function
//---------------------------------------------------------------------------------

    float minimiseX(float lambda, const Vec3 & v1, const Vec3 & v2, float mu) const{

        //std::cout << "Mu: " << mu << std::endl;
        //std::cout << "Lambda: " << lambda << std::endl;

        float root = INFINITY;

        float a = v1.squared_length();
        float b = v2.squared_length();
        float c = v1*v2;

        if(mu == 0){std::cout << "Mu = 0" << std::endl;
            root = -c/b;}
        else if(pow(mu, 2) < b*pow(lambda, 2)){std::cout << "Inequality satisfied" << std::endl;
            root = -c/b + (mu*sqrt((pow(mu, 2) - b*pow(lambda, 2)) * (pow(c, 2) - a*b)))/(b * (pow(mu, 2) - b*pow(lambda, 2)));}

        std::cout << "Root: " << root << std::endl;

        //no critical points found
        if(root == INFINITY){
            if(mu < -lambda*sqrt(v2.squared_length()))  //G'(X) always negative
                root = 1;
            else
                root = 0;
        }

        if(root > 1)
            root = 1;
        else if(root < 0)
            root = 0;

        return root;


    }

//---------------------------------------------------------------------------------
//Computes the cost ofmoving through a triangle starting at a vertex
//---------------------------------------------------------------------------------

    float costFuncIndirect(float tri_weight, Vec3 & u1, Vec3 & u2, float opp_tri_weight, float G1) const{

        //std::cout << "Calculating indirect cost" << std::endl;
        float minX = minimiseX(tri_weight, u1, -u2, opp_tri_weight*sqrt(u2.squared_length()));
        //std::cout << "MinX = " << minX << std::endl;
        return tri_weight*sqrt((u1 - minX*u2).squared_length()) + opp_tri_weight*sqrt((minX*u2).squared_length()) + G1;

    }

//---------------------------------------------------------------------------------
//Computes the cost of moving along the edge of a triangle
//---------------------------------------------------------------------------------

    std::pair<float, float> costFuncDirect(float tri_weight, Vec3 & u1, Vec3 & u3, float G1, float G2) const{

        std::cout << "Calculating direct cost" << std::endl;
        std::cout << "G1: " << G1 << ", G2: " << G2 << std::endl;
        std::cout << "U1: " << u1 << std::endl;
        std::cout << "U3: " << u3 << std::endl;
        //std::cout << "Weight = " << tri_weight << std::endl;
        //std::cout << "Lin comb U1, U3 = " << sqrt((u1 + minX*u3).squared_length()) << std::endl;
        std::cout << "G1 = " << G1 << std::endl;
        std::cout << "G2 = " << G2 << std::endl;
        if(G2 == INFINITY){

            //std::cout << "Caught inf G2" << std::endl;
            return std::pair<float, float>(0, tri_weight*sqrt(u1.squared_length()) + G1);
            }
        else if(G1 == INFINITY){

            //std::cout << "Caught inf G1" << std::endl;
            return std::pair<float, float>(1, tri_weight*sqrt((u1 + u3).squared_length()) + G2);
            }
        else{
            float minX = minimiseX(tri_weight, u1, u3, G2 - G1);
            std::cout << "MinX for direct path = " << minX << std::endl;
            return std::pair<float, float>(minX, tri_weight*sqrt((u1 + minX*u3).squared_length()) + minX*G2 + (1 - minX)*G1);
            }

    }

//---------------------------------------------------------------------------------
//Get the cost of traversal between two vertices
//---------------------------------------------------------------------------------

    float getCost(node_handle s, PathPoly_3::Facet_handle f) const{return getCostPair(s, f).first;}

    costPair getCostPair(node_handle s, PathPoly_3::Facet_handle f, bool pathExt = false) const{

        node_handle A = s;
        node_handle B1, B2;
        float TWeight = f->weight;
        float T1Weight = INFINITY;

        PathPoly_3::Halfedge_around_facet_circulator it = f->facet_begin();
        do{

        //std::cout << "Checking point " << it->vertex()->point() << " on triangle shared by " << s->point() << std::endl;

            if(it->vertex() == s){

                PathPoly_3::Facet_handle T1 = it->opposite()->facet();
                if(T1 != PathPoly_3::Facet_handle())
                    T1Weight = T1->weight;
                B1 = (++it)->vertex();
                B2 = (++it)->vertex();
                break;

            }

            ++it;

        }
        while(it != f->facet_begin());

        //std::cout << "B1: " << B1->point() << std::endl;
        //std::cout << "B2: " << B2->point() << std::endl;

        Vec3 U1(A->point(), B1->point());
        Vec3 U2(A->point(), B2->point());
        Vec3 U3(B1->point(), B2->point());

        float B1K = B1->key.first;
        //std::cout << B2->point() << " is a new point: " << B2->newPoint << std::endl;
        //std::cout << "Extracting path: " << pathExt << std::endl;

        if(B2->newPoint && pathExt){

        B1K -= B2->key.first/4;
        std::cout << "B1 augmented from " << B1->key.first << " to " << B1K << std::endl;
        //char a;
        //std::cin >> a;

        }


        std::pair<float, float> dirCost = costFuncDirect(TWeight, U1, U3, B1K, B2->key.first);
        float cost = std::min(dirCost.second, costFuncIndirect(TWeight, U1, U2, T1Weight, B1->key.first));

        std::cout << "Cost = min(" << dirCost.second << ", " << costFuncIndirect(TWeight, U1, U2, T1Weight, B1->key.first) << ")" << std::endl;

        if(cost == dirCost.second){
            //std::cout << "Min cost of " << s->point() << " uses direct cost." << std::endl;
            std::cout << "With X = " << dirCost.first << std::endl;
            CGAL::Point_3<K> minPoint(dirCost.first*(B2->point().x()) + (1 - dirCost.first)*(B1->point().x())
                                      , dirCost.first*(B2->point().y()) + (1 - dirCost.first)*(B1->point().y())
                                    , dirCost.first*(B2->point().z()) + (1 - dirCost.first)*(B1->point().z()));
            PathPoly_3::Vertex intPoint(minPoint);
            return costPair(cost, intPair(intPoint, it));

            }

        return costPair(cost, intPair(PathPoly_3::Vertex(CGAL::Point_3<K>(INFINITY, INFINITY, INFINITY)), PathPoly_3::Halfedge_handle()));

    }

//---------------------------------------------------------------------------------
//Set cost of facet to new value
//---------------------------------------------------------------------------------

    void setCost(node_handle s, node_handle e, float value){

        /*for(PathPoly_3::Vertex::Halfedge_around_vertex_circulator it = ++(s->vertex_begin()); it != s->vertex_begin(); ++it)
            if((PathPoly_3::Halfedge_handle(it))->opposite()->vertex() == e){
                it->weight = value;
                return;

            }*/

    }
//---------------------------------------------------------------------------------
//Estimates the distance between two vertices (Euclidean heuristic)
//---------------------------------------------------------------------------------

    float heuristic(node_handle s_index, node_handle e_index) const{

        return minWeight * sqrt(pow(e_index->point().x() - s_index->point().x(), 2) + pow(e_index->point().y() - s_index->point().y(), 2)
                     + pow(e_index->point().z() - s_index->point().z(), 2));

    }

//---------------------------------------------------------------------------------
//Compute priority key of node (leaves out km)
//---------------------------------------------------------------------------------

    node_key computeKey(node_handle start, node_handle index) const{

    std::cout << index->key.first << " " << index->key.second << std::endl;
    std::cout << heuristic(start, index) << std::endl;

    return node_key(std::min(index->key.first, index->key.second) + heuristic(start, index)
                    , std::min(index->key.first, index->key.second));

    }

//---------------------------------------------------------------------------------
//Return a list of a vertex's predecessors
//Bool toggles self-inclusion
//---------------------------------------------------------------------------------

    std::vector<node_handle> getPreds(node_handle h, bool incl){

        std::vector<node_handle> toProcess;

        if(incl)        //process self if needed
            toProcess.push_back(h);

        //for all predecesors
        PathPoly_3::Halfedge_around_vertex_circulator he = h->vertex_begin();

        do{

            toProcess.push_back(he->opposite()->vertex());
            ++he;

        }
        while(he != h->vertex_begin());

        return toProcess;

    }

//---------------------------------------------------------------------------------
//
//---------------------------------------------------------------------------------

    PathPoly_3 tempCells;

    nextPair getNextVert(PathPoly_3::Facet_handle T, costPair pair){

        std::cout << "Finding vert for pair: " << pair.first << " " << pair.second.first.point() << std::endl;

        PathPoly_3::Halfedge_around_facet_circulator circ = T->facet_begin();

        //if result is interpolated point
        if(!isEqual(*(circ->vertex()), pair.second.first) && !isEqual(*((++circ)->vertex()), pair.second.first)
                        && !isEqual(*((++circ)->vertex()), pair.second.first)){
                    //std::cout << "Direct cost: " << pair.first << std::endl;
                    //std::cout << "Move to " << pair.second.first.point() << std::endl;

                    //construct temp cells
                    PathPoly_3::Facet_handle T1 = pair.second.second->facet();
                    PathPoly_3::Facet_handle T2 = pair.second.second->opposite()->facet();

                    Build_temp temp(pair.second.first, pair.second.second, T1, T2);
                    tempCells.delegate(temp);

                    //std::cout << "Int point in temp cells has key = " << tempCells.vertices_begin()->key << std::endl;

                    intHanPair nextPoint(tempCells.vertices_begin(), pair.second.second);
                    return nextPair(true, nextPoint);

            }
        else{

                PathPoly_3::Halfedge_around_facet_circulator fit = T->facet_begin();

                do{

                    if(isEqual(*(fit->vertex()), pair.second.first)){

                        intHanPair nextPoint(fit->vertex(), PathPoly_3::Halfedge_handle());
                        return nextPair(false, nextPoint);

                    }

                    ++fit;

                }
                while(fit != T->facet_begin());

        }

        return nextPair();

    }

};


}

#endif // NODEMANAGER_H_INCLUDED
