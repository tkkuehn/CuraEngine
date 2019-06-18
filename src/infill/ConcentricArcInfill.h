#ifndef CONCENTRIC_ARC_INFILL_H
#define CONCENTRIC_ARC_INFILL_H

#include "../utils/Coord_t.h"
#include "../utils/AABB.h"
#include "../utils/polygon.h"

namespace cura
{

class ConcentricArcInfill
{
public:

    static void generateConcentricArcInfill(const Polygons& in_outline, coord_t outline_offset, Polygons& result_lines, const int line_distance, const Point infill_origin, const coord_t infill_line_width);

private:

    static void addArcInfill(const Polygons& outline, Polygons& result, const AABB boundary, std::vector<std::vector<double>>& cut_list, const int line_distance, const Point infill_origin, const coord_t infill_line_width);
};

}

#endif

