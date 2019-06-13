//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <algorithm> //For std::sort.
#include <functional>
#include <unordered_set>

#include "infill.h"
#include "sliceDataStorage.h"
#include "infill/ImageBasedDensityProvider.h"
#include "infill/GyroidInfill.h"
#include "infill/NoZigZagConnectorProcessor.h"
#include "infill/SierpinskiFill.h"
#include "infill/SierpinskiFillProvider.h"
#include "infill/SubDivCube.h"
#include "infill/UniformDensityProvider.h"
#include "utils/logoutput.h"
#include "utils/PolygonConnector.h"
#include "utils/polygonUtils.h"
#include "utils/UnionFind.h"

/*!
 * Function which returns the scanline_idx for a given x coordinate
 *
 * For negative \p x this is different from simple division.
 *
 * \warning \p line_width is assumed to be positive
 *
 * \param x the point to get the scansegment index for
 * \param line_width the width of the scan segments
 */
static inline int computeScanSegmentIdx(int x, int line_width)
{
    if (x < 0)
    {
        return (x + 1) / line_width - 1;
        // - 1 because -1 belongs to scansegment -1
        // + 1 because -line_width belongs to scansegment -1
    }
    return x / line_width;
}

namespace cura {

void Infill::generate(Polygons& result_polygons, Polygons& result_lines, const SierpinskiFillProvider* cross_fill_provider, const SliceMeshStorage* mesh)
{
    coord_t outline_offset_raw = outline_offset;
    outline_offset -= wall_line_count * infill_line_width; // account for extra walls

    if (infill_multiplier > 1)
    {
        bool zig_zaggify_real = zig_zaggify;
        if (infill_multiplier % 2 == 0)
        {
            zig_zaggify = false; // generate the basic infill pattern without going via the borders
        }
        Polygons generated_result_polygons;
        Polygons generated_result_lines;
        _generate(generated_result_polygons, generated_result_lines, cross_fill_provider, mesh);
        zig_zaggify = zig_zaggify_real;
        multiplyInfill(generated_result_polygons, generated_result_lines);
        result_polygons.add(generated_result_polygons);
        result_lines.add(generated_result_lines);
    }
    else
    {
        //_generate may clear() the generated_result_lines, but this is an output variable that may contain data before we start.
        //So make sure we provide it with a Polygons that is safe to clear and only add stuff to result_lines.
        Polygons generated_result_polygons;
        Polygons generated_result_lines;
        _generate(generated_result_polygons, generated_result_lines, cross_fill_provider, mesh);
        result_polygons.add(generated_result_polygons);
        result_lines.add(generated_result_lines);
    }

    // generate walls around infill pattern
    for (size_t wall_idx = 0; wall_idx < wall_line_count; wall_idx++)
    {
        const coord_t distance_from_outline_to_wall = outline_offset_raw - infill_line_width / 2 - wall_idx * infill_line_width;
        result_polygons.add(in_outline.offset(distance_from_outline_to_wall));
    }

    if (connect_polygons)
    {
        // remove too small polygons
        coord_t snap_distance = infill_line_width * 2; // polygons with a span of max 1 * nozzle_size are too small
        auto it = std::remove_if(result_polygons.begin(), result_polygons.end(), [snap_distance](PolygonRef poly) { return poly.shorterThan(snap_distance); });
        result_polygons.erase(it, result_polygons.end());

        PolygonConnector connector(infill_line_width, infill_line_width * 3 / 2);
        connector.add(result_polygons);
        result_polygons = connector.connect();
    }
}

void Infill::_generate(Polygons& result_polygons, Polygons& result_lines, const SierpinskiFillProvider* cross_fill_provider, const SliceMeshStorage* mesh)
{
    if (in_outline.empty()) return;
    if (line_distance == 0) return;

    if (zig_zaggify && (pattern == EFillMethod::LINES || pattern == EFillMethod::TRIANGLES || pattern == EFillMethod::GRID || pattern == EFillMethod::CUBIC || pattern == EFillMethod::TETRAHEDRAL || pattern == EFillMethod::QUARTER_CUBIC || pattern == EFillMethod::TRIHEXAGON))
    {
        outline_offset -= infill_line_width / 2; // the infill line zig zag connections must lie next to the border, not on it
    }

    switch(pattern)
    {
    case EFillMethod::GRID:
        generateGridInfill(result_lines);
        break;
    case EFillMethod::LINES:
        generateLineInfill(result_lines, line_distance, fill_angle, 0);
        break;
    case EFillMethod::CUBIC:
        generateCubicInfill(result_lines);
        break;
    case EFillMethod::TETRAHEDRAL:
        generateTetrahedralInfill(result_lines);
        break;
    case EFillMethod::QUARTER_CUBIC:
        generateQuarterCubicInfill(result_lines);
        break;
    case EFillMethod::TRIANGLES:
        generateTriangleInfill(result_lines);
        break;
    case EFillMethod::TRIHEXAGON:
        generateTrihexagonInfill(result_lines);
        break;
    case EFillMethod::CONCENTRIC:
        generateConcentricInfill(result_polygons, line_distance);
        break;
    case EFillMethod::ZIG_ZAG:
        generateZigZagInfill(result_lines, line_distance, fill_angle);
        break;
    case EFillMethod::CUBICSUBDIV:
        if (!mesh)
        {
            logError("Cannot generate Cubic Subdivision infill without a mesh!\n");
            break;
        }
        generateCubicSubDivInfill(result_lines, *mesh);
        break;
    case EFillMethod::CROSS:
    case EFillMethod::CROSS_3D:
        if (!cross_fill_provider)
        {
            logError("Cannot generate Cross infill without a cross fill provider!\n");
            break;
        }
        generateCrossInfill(*cross_fill_provider, result_polygons, result_lines);
        break;
    case EFillMethod::GYROID:
        generateGyroidInfill(result_lines);
        break;
    case EFillMethod::CONCENTRIC_ARC:
        generateConcentricArcInfill(in_outline, outline_offset, result_lines, line_distance);
        break;
    default:
        logError("Fill pattern has unknown value.\n");
        break;
    }

    //TODO: The connected lines algorithm is only available for linear-based infill, for now.
    //We skip ZigZag, Cross and Cross3D because they have their own algorithms. Eventually we want to replace all that with the new algorithm.
    //Cubic Subdivision ends lines in the center of the infill so it won't be effective.
    if (zig_zaggify && (pattern == EFillMethod::LINES || pattern == EFillMethod::TRIANGLES || pattern == EFillMethod::GRID || pattern == EFillMethod::CUBIC || pattern == EFillMethod::TETRAHEDRAL || pattern == EFillMethod::QUARTER_CUBIC || pattern == EFillMethod::TRIHEXAGON))
    {
        //The list should be empty because it will be again filled completely. Otherwise might have double lines.
        result_lines.clear();
        
        connectLines(result_lines);
    }
    crossings_on_line.clear();
}

void Infill::multiplyInfill(Polygons& result_polygons, Polygons& result_lines)
{
    if (pattern == EFillMethod::CONCENTRIC)
    {
        result_polygons = result_polygons.processEvenOdd(); // make into areas
    }

    bool odd_multiplier = infill_multiplier % 2 == 1;
    coord_t offset = (odd_multiplier)? infill_line_width : infill_line_width / 2;

    if (zig_zaggify && !odd_multiplier)
    {
        outline_offset -= infill_line_width / 2; // the infill line zig zag connections must lie next to the border, not on it
    }

    const Polygons outline = in_outline.offset(outline_offset);

    Polygons result;
    Polygons first_offset;
    { // calculate [first_offset]
        const Polygons first_offset_lines = result_lines.offsetPolyLine(offset); // make lines on both sides of the input lines
        const Polygons first_offset_polygons_inward = result_polygons.offset(-offset); // make lines on the inside of the input polygons
        const Polygons first_offset_polygons_outward = result_polygons.offset(offset); // make lines on the other side of the input polygons
        const Polygons first_offset_polygons = first_offset_polygons_outward.difference(first_offset_polygons_inward);
        first_offset = first_offset_lines.unionPolygons(first_offset_polygons); // usually we only have either lines or polygons, but this code also handles an infill pattern which generates both
        if (zig_zaggify)
        {
            first_offset = outline.difference(first_offset);
        }
    }
    result.add(first_offset);
    Polygons reference_polygons = first_offset;
    for (size_t infill_line = 1; infill_line < infill_multiplier / 2; infill_line++) // 2 because we are making lines on both sides at the same time
    {
        Polygons extra_offset = reference_polygons.offset(-infill_line_width);
        result.add(extra_offset);
        reference_polygons = std::move(extra_offset);
    }

    if (zig_zaggify)
    {
        result = result.intersection(outline);
    }

    if (!odd_multiplier)
    {
        result_polygons.clear();
        result_lines.clear();
    }
    result_polygons.add(result);
    if (!zig_zaggify)
    {
        for (PolygonRef poly : result_polygons)
        { // make polygons into polylines
            if (poly.empty())
            {
                continue;
            }
            poly.add(poly[0]);
        }
        Polygons polylines = outline.intersectionPolyLines(result_polygons);
        for (PolygonRef polyline : polylines)
        {
            Point last_point = no_point;
            for (Point point : polyline)
            {
                Polygon line;
                if (last_point != no_point)
                {
                    line.add(last_point);
                    line.add(point);
                    result_lines.add(line);
                }
                last_point = point;
            }
        }
        result_polygons.clear(); // the output should only contain polylines
    }
}

void Infill::generateGyroidInfill(Polygons& result_lines)
{
    GyroidInfill::generateTotalGyroidInfill(result_lines, zig_zaggify, outline_offset + infill_overlap, infill_line_width, line_distance, in_outline, z);
}

void Infill::generateConcentricInfill(Polygons& result, int inset_value)
{
    Polygons first_concentric_wall = in_outline.offset(outline_offset + infill_overlap - line_distance + infill_line_width / 2); // - infill_line_width / 2 cause generateConcentricInfill expects [outline] to be the outer most polygon instead of the outer outline

    if (perimeter_gaps)
    {
        const Polygons inner = first_concentric_wall.offset(infill_line_width / 2 + perimeter_gaps_extra_offset);
        const Polygons gaps_here = in_outline.difference(inner);
        perimeter_gaps->add(gaps_here);
    }
    generateConcentricInfill(first_concentric_wall, result, inset_value);
}

void Infill::generateConcentricInfill(Polygons& first_concentric_wall, Polygons& result, int inset_value)
{
    result.add(first_concentric_wall);
    Polygons* prev_inset = &first_concentric_wall;
    Polygons next_inset;
    Polygons new_inset;  // This intermediate inset variable is needed because prev_inset is referencing
    while (prev_inset->size() > 0)
    {
        new_inset = prev_inset->offset(-inset_value);
        result.add(new_inset);
        if (perimeter_gaps)
        {
            const Polygons outer = prev_inset->offset(-infill_line_width / 2 - perimeter_gaps_extra_offset);
            const Polygons inner = new_inset.offset(infill_line_width / 2);
            const Polygons gaps_here = outer.difference(inner);
            perimeter_gaps->add(gaps_here);
        }
        // This operation helps to prevent the variable "prev_inset" changes whenever next_inset changes
        next_inset = new_inset;
        prev_inset = &next_inset;
    }
    std::reverse(std::begin(result), std::end(result));
}

void Infill::generateGridInfill(Polygons& result)
{
    generateLineInfill(result, line_distance, fill_angle, 0);
    generateLineInfill(result, line_distance, fill_angle + 90, 0);
}

void Infill::generateCubicInfill(Polygons& result)
{
    const coord_t shift = one_over_sqrt_2 * z;
    generateLineInfill(result, line_distance, fill_angle, shift);
    generateLineInfill(result, line_distance, fill_angle + 120, shift);
    generateLineInfill(result, line_distance, fill_angle + 240, shift);
}

void Infill::generateTetrahedralInfill(Polygons& result)
{
    generateHalfTetrahedralInfill(0.0, 0, result);
    generateHalfTetrahedralInfill(0.0, 90, result);
}

void Infill::generateQuarterCubicInfill(Polygons& result)
{
    generateHalfTetrahedralInfill(0.0, 0, result);
    generateHalfTetrahedralInfill(0.5, 90, result);
}

void Infill::generateHalfTetrahedralInfill(float pattern_z_shift, int angle_shift, Polygons& result)
{
    const coord_t period = line_distance * 2;
    coord_t shift = coord_t(one_over_sqrt_2 * (z + pattern_z_shift * period * 2)) % period;
    shift = std::min(shift, period - shift); // symmetry due to the fact that we are applying the shift in both directions
    shift = std::min(shift, period / 2 - infill_line_width / 2); // don't put lines too close to each other
    shift = std::max(shift, infill_line_width / 2); // don't put lines too close to each other
    generateLineInfill(result, period, fill_angle + angle_shift, shift);
    generateLineInfill(result, period, fill_angle + angle_shift, -shift);
}

void Infill::generateTriangleInfill(Polygons& result)
{
    generateLineInfill(result, line_distance, fill_angle, 0);
    generateLineInfill(result, line_distance, fill_angle + 60, 0);
    generateLineInfill(result, line_distance, fill_angle + 120, 0);
}

void Infill::generateTrihexagonInfill(Polygons& result)
{
    generateLineInfill(result, line_distance, fill_angle, 0);
    generateLineInfill(result, line_distance, fill_angle + 60, 0);
    generateLineInfill(result, line_distance, fill_angle + 120, line_distance / 2);
}

void Infill::generateCubicSubDivInfill(Polygons& result, const SliceMeshStorage& mesh)
{
    Polygons uncropped;
    mesh.base_subdiv_cube->generateSubdivisionLines(z, uncropped);
    addLineSegmentsInfill(result, uncropped);
}

void Infill::generateCrossInfill(const SierpinskiFillProvider& cross_fill_provider, Polygons& result_polygons, Polygons& result_lines)
{
    outline_offset += infill_overlap;
    if (zig_zaggify)
    {
        outline_offset += -infill_line_width / 2;
    }
    Polygons outline = in_outline.offset(outline_offset);

    Polygon cross_pattern_polygon = cross_fill_provider.generate(pattern, z, infill_line_width, pocket_size);

    if (cross_pattern_polygon.empty())
    {
        return;
    }

    if (zig_zaggify)
    {
        Polygons cross_pattern_polygons;
        cross_pattern_polygons.add(cross_pattern_polygon);
        result_polygons.add(outline.intersection(cross_pattern_polygons));
    }
    else
    {
        // make the polyline closed in order to handle cross_pattern_polygon as a polyline, rather than a closed polygon
        cross_pattern_polygon.add(cross_pattern_polygon[0]);

        Polygons cross_pattern_polygons;
        cross_pattern_polygons.add(cross_pattern_polygon);
        Polygons poly_lines = outline.intersectionPolyLines(cross_pattern_polygons);
        
        for (PolygonRef poly_line : poly_lines)
        {
            for (unsigned int point_idx = 1; point_idx < poly_line.size(); point_idx++)
            {
                result_lines.addLine(poly_line[point_idx - 1], poly_line[point_idx]);
            }
        }
    }
}

void Infill::addLineSegmentsInfill(Polygons& result, Polygons& input)
{
    ClipperLib::PolyTree interior_segments_tree = in_outline.offset(infill_overlap).lineSegmentIntersection(input);
    ClipperLib::Paths interior_segments;
    ClipperLib::OpenPathsFromPolyTree(interior_segments_tree, interior_segments);
    for (size_t idx = 0; idx < interior_segments.size(); idx++)
    {
        result.addLine(interior_segments[idx][0], interior_segments[idx][1]);
    }
}

void Infill::addLineInfill(Polygons& result, const PointMatrix& rotation_matrix, const int scanline_min_idx, const int line_distance, const AABB boundary, std::vector<std::vector<coord_t>>& cut_list, coord_t shift)
{
    auto compare_coord_t = [](const void* a, const void* b)
    {
        coord_t n = (*(coord_t*)a) - (*(coord_t*)b);
        if (n < 0)
        {
            return -1;
        }
        if (n > 0)
        {
            return 1;
        }
        return 0;
    };

    unsigned int scanline_idx = 0;
    for(coord_t x = scanline_min_idx * line_distance + shift; x < boundary.max.X; x += line_distance)
    {
        if (scanline_idx >= cut_list.size())
        {
            break;
        }
        std::vector<coord_t>& crossings = cut_list[scanline_idx];
        qsort(crossings.data(), crossings.size(), sizeof(coord_t), compare_coord_t);
        for(unsigned int crossing_idx = 0; crossing_idx + 1 < crossings.size(); crossing_idx += 2)
        {
            if (crossings[crossing_idx + 1] - crossings[crossing_idx] < infill_line_width / 5)
            { // segment is too short to create infill
                continue;
            }
            //We have to create our own lines when they are not created by the method connectLines.
            if (!zig_zaggify || pattern == EFillMethod::ZIG_ZAG || pattern == EFillMethod::LINES)
            {
                result.addLine(rotation_matrix.unapply(Point(x, crossings[crossing_idx])), rotation_matrix.unapply(Point(x, crossings[crossing_idx + 1])));
            }
        }
        scanline_idx += 1;
    }
}

coord_t Infill::getShiftOffsetFromInfillOriginAndRotation(const double& infill_rotation)
{
    if (infill_origin.X != 0 || infill_origin.Y != 0)
    {
        const double rotation_rads = infill_rotation * M_PI / 180;
        return infill_origin.X * std::cos(rotation_rads) - infill_origin.Y * std::sin(rotation_rads);
    }
    return 0;
}

void Infill::generateLineInfill(Polygons& result, int line_distance, const double& infill_rotation, coord_t shift)
{
    shift += getShiftOffsetFromInfillOriginAndRotation(infill_rotation);
    PointMatrix rotation_matrix(infill_rotation);
    NoZigZagConnectorProcessor lines_processor(rotation_matrix, result);
    bool connected_zigzags = false;
    generateLinearBasedInfill(outline_offset, result, line_distance, rotation_matrix, lines_processor, connected_zigzags, shift);
}


void Infill::generateZigZagInfill(Polygons& result, const coord_t line_distance, const double& infill_rotation)
{
    const coord_t shift = getShiftOffsetFromInfillOriginAndRotation(infill_rotation);

    PointMatrix rotation_matrix(infill_rotation);
    ZigzagConnectorProcessor zigzag_processor(rotation_matrix, result, use_endpieces, connected_zigzags, skip_some_zags, zag_skip_count, minimum_zag_line_length);
    generateLinearBasedInfill(outline_offset - infill_line_width / 2, result, line_distance, rotation_matrix, zigzag_processor, connected_zigzags, shift);
}

/* 
 * algorithm:
 * 1. for each line segment of each polygon:
 *      store the intersections of that line segment with all scanlines in a mapping (vector of vectors) from scanline to intersections
 *      (zigzag): add boundary segments to result
 * 2. for each scanline:
 *      sort the associated intersections 
 *      and connect them using the even-odd rule
 * 
 * rough explanation of the zigzag algorithm:
 * while walking around (each) polygon (1.)
 *  if polygon intersects with even scanline
 *      start boundary segment (add each following segment to the [result])
 *  when polygon intersects with a scanline again
 *      stop boundary segment (stop adding segments to the [result])
 *  (see infill/ZigzagConnectorProcessor.h for actual implementation details)
 * 
 * 
 * we call the areas between two consecutive scanlines a 'scansegment'.
 * Scansegment x is the area between scanline x and scanline x+1
 * Edit: the term scansegment is wrong, since I call a boundary segment leaving from an even scanline to the left as belonging to an even scansegment, 
 *  while I also call a boundary segment leaving from an even scanline toward the right as belonging to an even scansegment.
 */
void Infill::generateLinearBasedInfill(const int outline_offset, Polygons& result, const int line_distance, const PointMatrix& rotation_matrix, ZigzagConnectorProcessor& zigzag_connector_processor, const bool connected_zigzags, coord_t extra_shift)
{
    if (line_distance == 0)
    {
        return;
    }
    if (in_outline.size() == 0)
    {
        return;
    }

    coord_t shift = extra_shift + this->shift;

    if (outline_offset != 0 && perimeter_gaps)
    {
        const Polygons gaps_outline = in_outline.offset(outline_offset + infill_line_width / 2 + perimeter_gaps_extra_offset);
        perimeter_gaps->add(in_outline.difference(gaps_outline));
    }

    Polygons outline = in_outline.offset(outline_offset + infill_overlap);

    if (outline.size() == 0)
    {
        return;
    }
    //TODO: Currently we find the outline every time for each rotation.
    //We should compute it only once and rotate that accordingly.
    //We'll also have the guarantee that they have the same size every time.
    //Currently we assume that the above operations are all rotation-invariant,
    //which they aren't if vertices fall on the same coordinate due to rounding.
    crossings_on_line.resize(outline.size()); //One for each polygon.

    outline.applyMatrix(rotation_matrix);

    if (shift < 0)
    {
        shift = line_distance - (-shift) % line_distance;
    }
    else
    {
        shift = shift % line_distance;
    }

    AABB boundary(outline);

    int scanline_min_idx = computeScanSegmentIdx(boundary.min.X - shift, line_distance);
    int line_count = computeScanSegmentIdx(boundary.max.X - shift, line_distance) + 1 - scanline_min_idx;

    std::vector<std::vector<coord_t>> cut_list; // mapping from scanline to all intersections with polygon segments

    for(int scanline_idx = 0; scanline_idx < line_count; scanline_idx++)
    {
        cut_list.push_back(std::vector<coord_t>());
    }

    //When we find crossings, keep track of which crossing belongs to which scanline and to which polygon line segment.
    //Then we can later join two crossings together to form lines and still know what polygon line segments that infill line connected to.
    struct Crossing
    {
        Crossing(Point coordinate, size_t polygon_index, size_t vertex_index): coordinate(coordinate), polygon_index(polygon_index), vertex_index(vertex_index) {};
        Point coordinate;
        size_t polygon_index;
        size_t vertex_index;
        bool operator <(const Crossing& other) const //Crossings will be ordered by their Y coordinate so that they get ordered along the scanline.
        {
            return coordinate.Y < other.coordinate.Y;
        }
    };
    std::vector<std::vector<Crossing>> crossings_per_scanline; //For each scanline, a list of crossings.
    const int min_scanline_index = computeScanSegmentIdx(boundary.min.X - shift, line_distance) + 1;
    const int max_scanline_index = computeScanSegmentIdx(boundary.max.X - shift, line_distance) + 1;
    crossings_per_scanline.resize(max_scanline_index - min_scanline_index);

    for(size_t poly_idx = 0; poly_idx < outline.size(); poly_idx++)
    {
        PolygonRef poly = outline[poly_idx];
        crossings_on_line[poly_idx].resize(poly.size()); //One for each line in this polygon.
        Point p0 = poly.back();
        zigzag_connector_processor.registerVertex(p0); // always adds the first point to ZigzagConnectorProcessorEndPieces::first_zigzag_connector when using a zigzag infill type

        for(size_t point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            Point p1 = poly[point_idx];
            if (p1.X == p0.X)
            {
                zigzag_connector_processor.registerVertex(p1); 
                // TODO: how to make sure it always adds the shortest line? (in order to prevent overlap with the zigzag connectors)
                // note: this is already a problem for normal infill, but hasn't really bothered anyone so far.
                p0 = p1;
                continue; 
            }

            int scanline_idx0;
            int scanline_idx1;
            // this way of handling the indices takes care of the case where a boundary line segment ends exactly on a scanline:
            // in case the next segment moves back from that scanline either 2 or 0 scanline-boundary intersections are created
            // otherwise only 1 will be created, counting as an actual intersection
            int direction = 1;
            if (p0.X < p1.X) 
            {
                scanline_idx0 = computeScanSegmentIdx(p0.X - shift, line_distance) + 1; // + 1 cause we don't cross the scanline of the first scan segment
                scanline_idx1 = computeScanSegmentIdx(p1.X - shift, line_distance); // -1 cause the vertex point is handled in the next segment (or not in the case which looks like >)
            }
            else
            {
                direction = -1;
                scanline_idx0 = computeScanSegmentIdx(p0.X - shift, line_distance); // -1 cause the vertex point is handled in the previous segment (or not in the case which looks like >)
                scanline_idx1 = computeScanSegmentIdx(p1.X - shift, line_distance) + 1; // + 1 cause we don't cross the scanline of the first scan segment
            }

            for(int scanline_idx = scanline_idx0; scanline_idx != scanline_idx1 + direction; scanline_idx += direction)
            {
                int x = scanline_idx * line_distance + shift;
                int y = p1.Y + (p0.Y - p1.Y) * (x - p1.X) / (p0.X - p1.X);
                assert(scanline_idx - scanline_min_idx >= 0 && scanline_idx - scanline_min_idx < int(cut_list.size()) && "reading infill cutlist index out of bounds!");
                cut_list[scanline_idx - scanline_min_idx].push_back(y);
                Point scanline_linesegment_intersection(x, y);
                zigzag_connector_processor.registerScanlineSegmentIntersection(scanline_linesegment_intersection, scanline_idx);
                crossings_per_scanline[scanline_idx - min_scanline_index].emplace_back(scanline_linesegment_intersection, poly_idx, point_idx);
            }
            zigzag_connector_processor.registerVertex(p1);
            p0 = p1;
        }
        zigzag_connector_processor.registerPolyFinished();
    }
    
    //Gather all crossings per scanline and find out which crossings belong together, then store them in crossings_on_line.
    for (int scanline_index = min_scanline_index; scanline_index < max_scanline_index; scanline_index++)
    {
        std::sort(crossings_per_scanline[scanline_index - min_scanline_index].begin(), crossings_per_scanline[scanline_index - min_scanline_index].end()); //Sorts them by Y coordinate.
        for (long crossing_index = 0; crossing_index < static_cast<long>(crossings_per_scanline[scanline_index - min_scanline_index].size()) - 1; crossing_index += 2) //Combine each 2 subsequent crossings together.
        {
            const Crossing& first = crossings_per_scanline[scanline_index - min_scanline_index][crossing_index];
            const Crossing& second = crossings_per_scanline[scanline_index - min_scanline_index][crossing_index + 1];
            //Avoid creating zero length crossing lines
            const Point unrotated_first = rotation_matrix.unapply(first.coordinate);
            const Point unrotated_second = rotation_matrix.unapply(second.coordinate);
            if (unrotated_first == unrotated_second)
            {
                continue;
            }
            InfillLineSegment* new_segment = new InfillLineSegment(unrotated_first, first.vertex_index, first.polygon_index, unrotated_second, second.vertex_index, second.polygon_index);
            //Put the same line segment in the data structure twice: Once for each of the polygon line segment that it crosses.
            crossings_on_line[first.polygon_index][first.vertex_index].push_back(new_segment);
            crossings_on_line[second.polygon_index][second.vertex_index].push_back(new_segment);
        }
    }

    if (cut_list.size() == 0)
    {
        return;
    }
    if (connected_zigzags && cut_list.size() == 1 && cut_list[0].size() <= 2)
    {
        return;  // don't add connection if boundary already contains whole outline!
    }

    addLineInfill(result, rotation_matrix, scanline_min_idx, line_distance, boundary, cut_list, shift);
}

void Infill::connectLines(Polygons& result_lines)
{
    //TODO: We're reconstructing the outline here. We should store it and compute it only once.
    Polygons outline = in_outline.offset(outline_offset + infill_overlap);

    UnionFind<InfillLineSegment*> connected_lines; //Keeps track of which lines are connected to which.
    for (std::vector<std::vector<InfillLineSegment*>>& crossings_on_polygon : crossings_on_line)
    {
        for (std::vector<InfillLineSegment*>& crossings_on_polygon_segment : crossings_on_polygon)
        {
            for (InfillLineSegment* infill_line : crossings_on_polygon_segment)
            {
                if (connected_lines.find(infill_line) == (size_t)-1)
                {
                    connected_lines.add(infill_line); //Put every line in there as a separate set.
                }
            }
        }
    }

    for (size_t polygon_index = 0; polygon_index < outline.size(); polygon_index++)
    {
        if (outline[polygon_index].empty())
        {
            continue;
        }

        InfillLineSegment* previous_crossing = nullptr; //The crossing that we should connect to. If nullptr, we have been skipping until we find the next crossing.
        InfillLineSegment* previous_segment = nullptr; //The last segment we were connecting while drawing a line along the border.
        Point vertex_before = outline[polygon_index].back();
        for (size_t vertex_index = 0; vertex_index < outline[polygon_index].size(); vertex_index++)
        {
            Point vertex_after = outline[polygon_index][vertex_index];

            //Sort crossings on every line by how far they are from their initial point.
            struct CompareByDistance
            {
                CompareByDistance(Point to_point, size_t polygon_index, size_t vertex_index): to_point(to_point), polygon_index(polygon_index), vertex_index(vertex_index) {};
                Point to_point; //The distance to this point is compared.
                size_t polygon_index; //The polygon which the vertex_index belongs to.
                size_t vertex_index; //The vertex indicating a line segment. This determines which endpoint of each line should be used.
                inline bool operator ()(InfillLineSegment*& left_hand_side, InfillLineSegment*& right_hand_side) const
                {
                    //Find the two endpoints that are relevant.
                    const Point left_hand_point = (left_hand_side->start_segment == vertex_index && left_hand_side->start_polygon == polygon_index) ? left_hand_side->start : left_hand_side->end;
                    const Point right_hand_point = (right_hand_side->start_segment == vertex_index && right_hand_side->start_polygon == polygon_index) ? right_hand_side->start : right_hand_side->end;
                    return vSize(left_hand_point - to_point) < vSize(right_hand_point - to_point);
                }
            };
            std::sort(crossings_on_line[polygon_index][vertex_index].begin(), crossings_on_line[polygon_index][vertex_index].end(), CompareByDistance(vertex_before, polygon_index, vertex_index));

            for (InfillLineSegment* crossing : crossings_on_line[polygon_index][vertex_index])
            {
                if (!previous_crossing) //If we're not yet drawing, then we have been trying to find the next vertex. We found it! Let's start drawing.
                {
                    previous_crossing = crossing;
                    previous_segment = crossing;
                }
                else
                {
                    const size_t crossing_handle = connected_lines.find(crossing);
                    assert (crossing_handle != (size_t)-1);
                    const size_t previous_crossing_handle = connected_lines.find(previous_crossing);
                    assert (previous_crossing_handle != (size_t)-1);
                    if (crossing_handle == previous_crossing_handle) //These two infill lines are already connected. Don't create a loop now. Continue connecting with the next crossing.
                    {
                        continue;
                    }

                    //Join two infill lines together with a connecting line.
                    //Here the InfillLineSegments function as a linked list, so that they can easily be joined.
                    const Point previous_point = (previous_segment->start_segment == vertex_index && previous_segment->start_polygon == polygon_index) ? previous_segment->start : previous_segment->end;
                    const Point next_point = (crossing->start_segment == vertex_index && crossing->start_polygon == polygon_index) ? crossing->start : crossing->end;
                    InfillLineSegment* new_segment;
                    // If the segment is zero length, we avoid creating it but still want to connect the crossing with the previous segment
                    if (previous_point == next_point)
                    {
                        if (previous_segment->start_segment == vertex_index && previous_segment->start_polygon == polygon_index)
                        {
                            previous_segment->previous = crossing;
                        }
                        else
                        {
                            previous_segment->next = crossing;
                        }
                        new_segment = previous_segment;
                    }
                    else
                    {
                        new_segment = new InfillLineSegment(previous_point, vertex_index, polygon_index, next_point, vertex_index, polygon_index); //A connecting line between them.
                        new_segment->previous = previous_segment;
                        if (previous_segment->start_segment == vertex_index && previous_segment->start_polygon == polygon_index)
                        {
                            previous_segment->previous = new_segment;
                        }
                        else
                        {
                            previous_segment->next = new_segment;
                        }
                        new_segment->next = crossing;
                    }

                    if (crossing->start_segment == vertex_index && crossing->start_polygon == polygon_index)
                    {
                        crossing->previous = new_segment;
                    }
                    else
                    {
                        crossing->next = new_segment;
                    }
                    connected_lines.unite(crossing_handle, previous_crossing_handle);
                    previous_crossing = nullptr;
                    previous_segment = nullptr;
                }
            }

            //Upon going to the next vertex, if we're drawing, put an extra vertex in our infill lines.
            if (previous_crossing)
            {
                InfillLineSegment* new_segment;
                if (vertex_index == previous_segment->start_segment && polygon_index == previous_segment->start_polygon)
                {
                    if (previous_segment->start == vertex_after)
                    {
                        //Edge case when an infill line ends directly on top of vertex_after: We skip the extra connecting line segment, as that would be 0-length.
                        previous_segment = nullptr;
                        previous_crossing = nullptr;
                    }
                    else
                    {
                        new_segment = new InfillLineSegment(previous_segment->start, vertex_index, polygon_index, vertex_after, (vertex_index + 1) % outline[polygon_index].size(), polygon_index);
                        previous_segment->previous = new_segment;
                        new_segment->previous = previous_segment;
                        previous_segment = new_segment;
                    }
                }
                else
                {
                    if (previous_segment->end == vertex_after)
                    {
                        //Edge case when an infill line ends directly on top of vertex_after: We skip the extra connecting line segment, as that would be 0-length.
                        previous_segment = nullptr;
                        previous_crossing = nullptr;
                    }
                    else
                    {
                        new_segment = new InfillLineSegment(previous_segment->end, vertex_index, polygon_index, vertex_after, (vertex_index + 1) % outline[polygon_index].size(), polygon_index);
                        previous_segment->next = new_segment;
                        new_segment->previous = previous_segment;
                        previous_segment = new_segment;
                    }
                }
            }

            vertex_before = vertex_after;
        }
    }

    //Save all lines, now connected, to the output.
    std::unordered_set<size_t> completed_groups;
    for (InfillLineSegment* infill_line : connected_lines)
    {
        const size_t group = connected_lines.find(infill_line);
        if (completed_groups.find(group) != completed_groups.end()) //We already completed this group.
        {
            continue;
        }

        //Find where the polyline ends by searching through previous and next lines.
        //Note that the "previous" and "next" lines don't necessarily match up though, because the direction while connecting infill lines was not yet known.
        Point previous_vertex = infill_line->start; //Take one side arbitrarily to start from. This variable indicates the vertex that connects to the previous line.
        InfillLineSegment* current_infill_line = infill_line;
        while (current_infill_line->next && current_infill_line->previous) //Until we reached an endpoint.
        {
            const Point next_vertex = (previous_vertex == current_infill_line->start) ? current_infill_line->end : current_infill_line->start;
            current_infill_line =     (previous_vertex == current_infill_line->start) ? current_infill_line->next : current_infill_line->previous;
            previous_vertex = next_vertex;
        }

        //Now go along the linked list of infill lines and output the infill lines to the actual result.
        InfillLineSegment* old_line = current_infill_line;
        const Point first_vertex = (!current_infill_line->previous) ? current_infill_line->start : current_infill_line->end;
        previous_vertex =          (!current_infill_line->previous) ? current_infill_line->end : current_infill_line->start;
        current_infill_line = (first_vertex == current_infill_line->start) ? current_infill_line->next : current_infill_line->previous;
        result_lines.addLine(first_vertex, previous_vertex);
        delete old_line;
        while (current_infill_line)
        {
            old_line = current_infill_line; //We'll delete this after we've traversed to the next line.
            const Point next_vertex = (previous_vertex == current_infill_line->start) ? current_infill_line->end : current_infill_line->start; //Opposite side of the line.
            current_infill_line =     (previous_vertex == current_infill_line->start) ? current_infill_line->next : current_infill_line->previous;
            result_lines.addLine(previous_vertex, next_vertex);
            previous_vertex = next_vertex;
            delete old_line;
        }

        completed_groups.insert(group);
    }
}

void Infill::generateConcentricArcInfill(const Polygons& in_outline, coord_t outline_offset, Polygons& result_lines, const int line_distance)
{
    if (line_distance == 0)
    {
        logWarning("No outline");
        return;
    }

    Polygons outline = in_outline.offset(outline_offset);

    if (outline.size() == 0)
    {
        logWarning("No outline");
        return;
    }

    crossings_on_line.resize(outline.size());

    // translate polygons to ensure infill_origin is at (0, 0)
    outline.translate(-infill_origin);

    AABB boundary(outline);

    // potential issue with type conversions here?
    double max_x_sq = pow(std::max(abs(boundary.max.X), abs(boundary.min.X)), 2);
    double max_y_sq = pow(std::max(abs(boundary.max.Y), abs(boundary.min.Y)), 2);
    double furthest_point = sqrt(max_x_sq + max_y_sq);
    int arc_count = static_cast<int>((furthest_point - (infill_line_width / 2)) / line_distance);

    logDebug(("arc_count: " + std::to_string(arc_count) + "\n").c_str());

    // scanarc -> intersections with polygon segments
    std::vector<std::vector<double>> cut_list;

    for (int scanarc_idx = 0; scanarc_idx < arc_count + 1; scanarc_idx++)
    {
        cut_list.push_back(std::vector<double>());
    }

    struct Crossing
    {
        Crossing(Point coordinate, size_t polygon_index, size_t vertex_index): coordinate(coordinate), polygon_index(polygon_index), vertex_index(vertex_index) {};
        Point coordinate;
        size_t polygon_index;
        size_t vertex_index;
        bool operator <(const Crossing& other) const
        {
            // do this by angle from 0 - 360
            float this_theta = atan2(coordinate.Y, coordinate.X);// + M_PI;
            float other_theta = atan2(other.coordinate.Y, other.coordinate.X);// + M_PI;

            return this_theta < other_theta;
        }
    };

    // list of crossings for each scan arc
    std::vector<std::vector<Crossing>> crossings_per_scanarc;
    crossings_per_scanarc.resize(arc_count + 1);

    for (size_t poly_idx = 0; poly_idx < outline.size(); poly_idx++)
    {
        PolygonRef poly = outline[poly_idx];
        crossings_on_line[poly_idx].resize(poly.size());

        Point p0 = poly.back();

        for (size_t point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            Point p1 = poly[point_idx];
            
            double p0_r = sqrt(pow(p0.X, 2) + pow(p0.Y, 2));
            double p1_r = sqrt(pow(p1.X, 2) + pow(p1.Y, 2));

            if (p0_r == p1_r)
            {
                p0 = p1;
                continue;
            }

            int scanarc_idx0;
            int scanarc_idx1;

            int direction = 1;
            if (p0_r < p1_r)
            {
                scanarc_idx0 = static_cast<int>(floor((p0_r - (infill_line_width / 2)) / line_distance)) + 1;
                scanarc_idx1 = static_cast<int>(floor((p1_r - (infill_line_width / 2)) / line_distance));
            }
            else
            {
                direction = -1;
                scanarc_idx0 = static_cast<int>(floor((p0_r - (infill_line_width / 2)) / line_distance));
                scanarc_idx1 = static_cast<int>(floor((p1_r - (infill_line_width / 2)) / line_distance)) + 1;
            }

            // want to rotate line segment so it's horizontal
            float horiz_angle = -1 * atan2(p1.Y - p0.Y, p1.X - p0.X);
            PointMatrix horiz_matrix(horiz_angle * 180 / M_PI);
            Point p0_horiz = horiz_matrix.apply(p0);
            Point p1_horiz = horiz_matrix.apply(p1);
            int y_prime = p0_horiz.Y;
//            logDebug(("p0_horiz.Y: " + std::to_string(p0_horiz.Y) + "\n").c_str());
//            logDebug(("p1_horiz.Y: " + std::to_string(p1_horiz.Y) + "\n").c_str());
            logDebug(("y_prime: " + std::to_string(y_prime) + "\n").c_str());

            for (int scanarc_idx = scanarc_idx0; scanarc_idx != scanarc_idx1 + direction; scanarc_idx += direction)
            {
                logDebug(("scanarc_idx: " + std::to_string(scanarc_idx) + "\n").c_str());
                int r = (infill_line_width / 2) + (scanarc_idx * line_distance);
                logDebug(("r: " + std::to_string(r) + "\n").c_str());

                double x_squared = pow(r, 2) - pow(y_prime, 2);
                logDebug(("x_squared: " + std::to_string(x_squared) + "\n").c_str());

                if (x_squared < 0)
                {
                    continue;
                }
                else if (x_squared == 0)
                {
                    // horizontal line is tangent to scan arc => (x, y) = (0, r)
                    int x = 0;

                    // is the intersection actually on the line segment?
                    if (std::min(p0_horiz.X, p1_horiz.X) <= x && std::max(p0_horiz.X, p1_horiz.X) >= x)
                    {
                        Point intersection_orig = horiz_matrix.unapply(Point(x, r));
                        double theta = atan2(intersection_orig.Y, intersection_orig.X);// + M_PI;
//                        logDebug(("theta: " + std::to_string(theta) + "\n").c_str());
                        cut_list[scanarc_idx].push_back(theta);
                        crossings_per_scanarc[scanarc_idx].emplace_back(intersection_orig, poly_idx, point_idx);
                    }
                }
                else if (x_squared > 0)
                {
//                    logDebug("x squared > 0\n");
                    // horizontal line intersects scan arc in two places: +/- x
                    int pos_x = static_cast<int>(sqrt(x_squared));
                    int neg_x = -pos_x;
                    logDebug(("pos_x: " + std::to_string(pos_x) + "\n").c_str());

                    int segment_min_x = std::min(p0_horiz.X, p1_horiz.X);
                    int segment_max_x = std::max(p0_horiz.X, p1_horiz.X);

                    // is the positive intersection on the line segment?
                    if ((segment_min_x <= pos_x) && (segment_max_x >= pos_x))
                    {
                        logDebug("pos_x is on the line segment\n");
                        Point intersection_orig = horiz_matrix.unapply(Point(pos_x, y_prime));
                        if (!boundary.contains(intersection_orig))
                        {
                            logWarning("pos_x intersection_orig outside outline\n");
                        }
                        double theta = atan2(intersection_orig.Y, intersection_orig.X);// + M_PI;
//                        logDebug(("theta: " + std::to_string(theta) + "\n").c_str());
                        cut_list[scanarc_idx].push_back(theta);
//                        logDebug(("intersection_orig.X: " + std::to_string(intersection_orig.X) + "\n").c_str());
//                        logDebug(("intersection_orig.Y: " + std::to_string(intersection_orig.X) + "\n").c_str());
//                        logDebug(("poly_idx: " + std::to_string(poly_idx) + "\n").c_str());
//                        logDebug(("point_idx: " + std::to_string(point_idx) + "\n").c_str());
                        crossings_per_scanarc[scanarc_idx].emplace_back(intersection_orig, poly_idx, point_idx);
                    }
                   
                    // is the negative intersection on the line segment?
                    if ((segment_min_x <= neg_x) && (segment_max_x >= neg_x))
                    {
                        logDebug("neg_x is on the line segment\n");
                        Point intersection_orig = horiz_matrix.unapply(Point(neg_x, y_prime));
                        if (!boundary.contains(intersection_orig))
                        {
                            logWarning("neg_x intersection_orig outside outline\n");
                        }

                        double theta = atan2(intersection_orig.Y, intersection_orig.X);// + M_PI;
                        logDebug(("theta: " + std::to_string(theta) + "\n").c_str());
                        cut_list[scanarc_idx].push_back(theta);
                        crossings_per_scanarc[scanarc_idx].emplace_back(intersection_orig, poly_idx, point_idx);
                    }
                }
            }
            p0 = p1;
        }
    }

    // okay, we've found all the crossings - now we need to connect the adjacent ones for crossings_on_line
    // we don't usually just want straight lines between our crossings
    // instead, we want to find the arc and sample along it
/**    for (int scanarc_index = 0; scanarc_index < arc_count; scanarc_index++)
    {
        // first possible point along the sorted arc
        int radius = (infill_line_width / 2) + (scanarc_index * line_distance);

        Point arc_origin(radius, 0);

        // sort the crossings along this scanarc by angle (from zero to 2pi)
        std::sort(crossings_per_scanarc[scanarc_index].begin(), crossings_per_scanarc[scanarc_index].end());

        // if the first crossing goes from inside the polygon to outside, we get the opposite of the print arcs we want
        // so, rotate the sorted vector of crossings if necessary to avoid this case
        //
        if (crossings_per_scanarc[scanarc_index].size() == 0)
        {
            continue;
        }

        if (outline.inside(arc_origin))
        {
            // want to start with the second crossing instead of the first
            std::rotate(crossings_per_scanarc[scanarc_index].begin(), crossings_per_scanarc[scanarc_index].begin() + 1, crossings_per_scanarc[scanarc_index].end());
        }

        // with the correct starting index calculated, we can find the endpoints of each arc we want to print
        // then we can sample that arc to generate a sequence of InfillLineSegments and fill crossings_on_line

        for (long crossing_index = 0; crossing_index < static_cast<long>(crossings_per_scanarc[scanarc_index].size() - 1); crossing_index += 2)
        {
            const Crossing& first = crossings_per_scanarc[scanarc_index][crossing_index];
            const Crossing& second = crossings_per_scanarc[scanarc_index][crossing_index + 1];

            // first and second are the end points of an arc - want to sample that arc finely enough to capture it
            double first_theta = atan2(first.coordinate.Y, first.coordinate.X) + M_PI;
            double second_theta = atan2(second.coordinate.Y, second.coordinate.X) + M_PI;

            double theta_diff = second_theta - first_theta;

            // happens if we rotated the vector
            if (theta_diff < 0)
            {
                theta_diff += (2 * M_PI);
            }

            // line_distance should probably infill_line_width - need to check that we have access to that
            int num_samples = floor(radius * theta_diff / (line_distance / 5));

            Point prev_sample = first.coordinate;

            for (int i = 1; i < num_samples; i++)
            {
                double sample_theta = (i / (num_samples - 1) * theta_diff) + first_theta;
                int sample_x = radius * cos(sample_theta);
                int sample_y = radius * sin(sample_theta);

                Point current_sample(sample_x, sample_y);
                
                Point untranslated_prev = prev_sample + infill_origin;
                Point untranslated_current = current_sample + infill_origin;

                if (untranslated_prev == untranslated_current)
                {
                    continue;
                }
                InfillLineSegment* new_segment = new InfillLineSegment(untranslated_prev, first.vertex_index, first.polygon_index, untranslated_current, second.vertex_index, second.polygon_index);
                crossings_on_line[first.polygon_index][first.vertex_index].push_back(new_segment);
                crossings_on_line[second.polygon_index][second.vertex_index].push_back(new_segment);
            }
        }

        // also, if (r, 0) is in the polygon but there are no intersections, we want to print a full circle
        // but I don't think it matters here because there are no ends to connect
    }
**/
    // didn't find any intersections
    if (cut_list.size() == 0)
    {
        logWarning("No cut_list");
        return;
    }

    addArcInfill(outline, result_lines, boundary, cut_list);
}

void Infill::addArcInfill(const Polygons& outline, Polygons& result, const AABB boundary, std::vector<std::vector<double>>& cut_list)
{
    unsigned int scanarc_idx = 0;
    double max_radius = sqrt(pow(std::max(abs(boundary.max.X), abs(boundary.min.X)), 2) + pow(std::max(abs(boundary.max.Y), abs(boundary.min.Y)), 2));
    unsigned int max_scanarc = static_cast<int>((max_radius - (infill_line_width / 2)) / line_distance);

//    for (coord_t radius = (infill_line_width / 2) + (scanarc_idx * line_distance); radius < max_radius; radius += line_distance)
    for (unsigned int scanarc_idx = 0; scanarc_idx < max_scanarc; scanarc_idx++)
    {
        // because we're adding two each time
        if (scanarc_idx >= cut_list.size())
        {
            break;
        }

//        logDebug(("Adding arcs at radius " + std::to_string(radius) + "\n").c_str());

        int radius = (infill_line_width / 2) + (scanarc_idx * line_distance);

        std::vector<double>& crossings = cut_list[scanarc_idx];

        Point arc_origin(-radius, 0);

        // sort the crossings along this scanarc by angle (from zero to 2pi)
        std::sort(crossings.begin(), crossings.end());

        if (crossings.size() == 0)
        {
            if (outline.inside(arc_origin))
            {
                crossings.push_back(M_PI);
                crossings.push_back(-1 * M_PI);
            }
            else
            {
                logDebug("No crossings and starting point outside Polygons\n");
                continue;
            }
        }

        for (int crossing_idx = 0; crossing_idx < crossings.size(); crossing_idx++)
        {
            logDebug(("Crossing angle " + std::to_string(crossing_idx) + ": " + std::to_string(crossings[crossing_idx]) + "\n").c_str());
        }

        // make sure the correct arcs are printed
        if (outline.inside(arc_origin))
        {
            // want to start with the second crossing instead of the first
            std::rotate(crossings.begin(), crossings.begin() + 1, crossings.end());
        }

        for (unsigned int crossing_idx = 0; crossing_idx + 1 < crossings.size(); crossing_idx += 2)
        {
            double first_theta = crossings[crossing_idx];
            double second_theta = crossings[crossing_idx + 1];
            double theta_diff = second_theta - first_theta;

//            logDebug(("first_theta: " + std::to_string(first_theta) + " second_theta: " + std::to_string(second_theta) + "\n").c_str());
//            logDebug(("radius: " + std::to_string(radius) + "\n").c_str());

//            logDebug(("theta_diff: " + std::to_string(theta_diff) + "\n").c_str());

            // happens if we rotated crossings
            if (theta_diff < 0)
            {
                theta_diff += 2 * M_PI;//= abs(theta_diff);
            }

            // arc length / minimum sample length
            int arc_segment_length = 2 * infill_line_width;
            int num_samples = floor(radius * theta_diff / arc_segment_length);

//            logDebug(("Number of samples: " + std::to_string(num_samples) + "\n").c_str());
            // figure out how to sample the arc of interest here
            Point prev_sample(radius * cos(first_theta), radius * sin(first_theta));

            if (!boundary.contains(prev_sample))
            {
                logWarning("First crossing outside boundary\n");
            }
            if (!boundary.contains(Point(radius * cos(second_theta), radius * sin(second_theta))))
            {
                logWarning("Second crossing outside boundary\n");
            }

            for (int i = 1; i < num_samples; i++)
            {
                double sample_theta = (((double) i) / ((double) (num_samples - 1)) * theta_diff) + first_theta;
//                logDebug(("sample_theta: " + std::to_string(sample_theta) + "\n").c_str());
                int sample_x = radius * cos(sample_theta);
                int sample_y = radius * sin(sample_theta);

                Point current_sample(sample_x, sample_y);

                if (!boundary.contains(current_sample))
                {
                    logWarning("Sample out of bounds\n");
                }
                /** if (!outline.inside(current_sample))
                {
                    logWarning("sample outside outline\n");
                }**/
                
                Point untranslated_prev = prev_sample + infill_origin;
                Point untranslated_current = current_sample + infill_origin;

                if (untranslated_prev == untranslated_current)
                {
                    continue;
                }

//                logDebug("Adding result line\n");
                result.addLine(untranslated_prev, untranslated_current);

                prev_sample = current_sample;
            }
        }
    }
}

bool Infill::InfillLineSegment::operator ==(const InfillLineSegment& other) const
{
    return start == other.start && end == other.end;
}

}//namespace cura
