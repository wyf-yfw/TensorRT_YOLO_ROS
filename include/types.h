#ifndef TYPES_H
#define TYPES_H

#include <string>


struct Detection
{
    // x1, y1, x2, y2
    float bbox[4];
    float conf;
    int classId;
    //三维坐标x,y,z
    float coordinate[3];
};

#endif  // TYPES_H
