//Class for registering graph changes in the pathfinder
//Daanyaal du Toit
//Created: 27 August 2013
//Last Modified: 27 August 2013

#ifndef NODEUPDATOR_H_INCLUDED
#define NODEUPDATOR_H_INCLUDED

#include "UpdateHelpers.h"

namespace ADStar{

    class NodeUpdator{

        public:
            const std::vector<EdgeChange> & getChanges(){return changes;};
            void changeEdge(int start, int end, float weight){changes.push_back(EdgeChange(start, end, weight));}
            void clear(){changes.clear();}
        private:
            std::vector<EdgeChange> changes;


    };

}

#endif // NODEUPDATOR_H_INCLUDED
