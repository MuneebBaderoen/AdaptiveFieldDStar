#ifndef UPDATEHELPERS_H_INCLUDED
#define UPDATEHELPERS_H_INCLUDED

namespace ADStar{

    struct EdgeChange{

        int start, end;
        float weight;
        EdgeChange(int s, int e, float w): start(s), end(e), weight(w){}

    };

}

#endif // UPDATEHELPERS_H_INCLUDED
