#ifndef INFILL_CONCENTRIC_ARC_INFILL_H
#define INFILL_CONCENTRIC_ARC_INFILL_H

#include "../utils/AABB.h"
#include "utils/IntPoint.h"

namespace cura
{

class ConcentricArcInfill
{
public:
    ConcentricArcInfill();

    ~ConcentricArcInfill();

    static void generateConcentricArcInfill(result_lines, line_distance, fill_angle);
}
