//Gui for path viewing
//Daanyaal du Toit
//Created: 23 September 2013
//Last modified: 23 September 2013

#ifndef INTERFACE_H_INCLUDED
#define INTERFACE_H_INCLUDED

#include "NodeManager.h"

namespace ADStar{

    class Interface{

        public:
            Interface(){



            }
        private:
            PathPoly_3 & mesh;
            std::vector<CGAL::Point_3<K> > path;

    };

}

#endif // INTERFACE_H_INCLUDED
