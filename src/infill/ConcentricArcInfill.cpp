#include "../infill/ConcentricArcInfill.h"

void ConcentricArcInfill::generateConcentricArcInfill(const Polygons& in_outline, Polygons& result_lines, const int line_distance)
{
    if (line_distance == 0)
    {
        return;
    }

    Polygons outline = in_outline;

    if (outline.size() == 0)
    {
        return;
    }

    // contained in Infill class, defined on infill.h#L221
    // should maybe pass as an argument, but I don't know if that's necessary
    // polygons -> polygon lines -> infill lines
    std::vector<std::vector<std::vector<InfillLineSegment*>>> crossings_on_line;

    crossings_on_line.resize(outline.size());

    // translate polygons to ensure infill_origin is at (0, 0)
    outline.translate(-infill_origin)

    AABB boundary(outline);

    int boundary_width = boundary.max.X - boundary.min.X;
    int boundary_height = boundary.max.Y - boundary.min.Y;

    // potential issue with type conversions here?
    int arc_count = sqrt(pow(max(abs(boundary.max.X), abs(boundary.min.X)), 2) + pow(max(abs(boundary.max.Y), abs(boundary.min.Y)), 2)) / line_distance;

    // scanarc -> intersections with polygon segments
    std::vector<std::vector<coord_t>> cut_list;

    for (int scanarc_idx = 0; scanarc_idx < arc_count; scanarc_idx++)
    {
        cut_list.push_back(std::vector<coord_t>());
    }

    struct Crossing
    {
        Point coordinate;
        size_t polygon_index;
        size_t vertex_index;
        bool operator <(const Crossing& other) const
        {
            // do this by angle from 0 - 360
            float this_theta = atan2(coordinate.Y, coordinate.X) + M_PI;
            float other_theta = atan2(other.coordinate.Y, other.coordinate.X) + M_PI;

            return this_theta < other_theta;
        }
    }

    // list of crossings for each scan arc
    std::vector<std::vector<Crossing>> crossings_per_scanarc;
    crossings_per_scanarc.resize(arc_count);

    for (size_t poly_idx = 0; poly_idx < outline.size(); poly_idx++)
    {
        PolygonRef poly = outline[poly_idx];
        crossings_on_line[poly_idx].resize(poly.size());

        Point p0 = poly.back();

        for (size_t point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            Point p1 = poly[point_idx];
            
            float p0_r = sqrt(pow(p0.X, 2) + pow(p0.Y, 2));
            float p1_r = sqrt(pow(p1.X, 2) + pow(p1.Y, 2));

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
                scanarc_idx0 = floor(p0_r / line_distance) + 1;
                scanarc_idx1 = floor(p1_r / line_distance)
            }
            else
            {
                direction = -1;
                scanarc_idx0 = floor(p0_r / line_distance);
                scanarc_idx1 = floor(p1_r / line_distance) + 1;
            }

            // want to rotate line segment so it's horizontal
            horiz_angle = -1 * atan2(p1.Y - p0.Y, p1.X - p0.X);
            PointMatrix horiz_matrix(horiz_angle);

            for (int scanarc_idx = scanarc_idx0; scanarc_idx != scanarc_idx1 + direction; scanarc_idx += direction)
            {
                int r = scanarc_idx * line_distance;
                Point p0_horiz = horiz_matrix.apply(p0);
                Point p1_horiz = horiz_matrix.apply(p1);
                int y_prime = p0_horiz.Y;

                int x_squared = pow(r, 2) - pow(y_prime, 2);

                if (x_squared < 0)
                {
                    continue;
                }
                else if (x_squared == 0)
                {
                    // horizontal line is tangent to scan arc => (x, y) = (0, r)
                    int x = 0;

                    // is the intersection actually on the line segment?
                    if (min(p0_horiz.X, p1_horiz.X) <= x && max(p0_horiz.X, p1_horiz.X) >= x)
                    {
                        Point intersection_orig = horiz_matrix.unapply(Point(x, r));
                        double theta = atan2(intersection_orig.Y, intersection_orig.X) + M_PI;
                        cut_list[scanarc_idx].push_back(theta);
                        crossings_per_scanline[scanarc_idx].emplace_back(intersection_orig, poly_idx, point_idx);
                    }
                }
                else if (x_squared > 0)
                {
                    // horizontal line intersects scan arc in two places: +/- x
                    int pos_x = sqrt(x_squared);
                    int neg_x = -1 * pos_x;

                    int p0_min_x = min(p0_horiz.X, p1_horiz.X);
                    int p0_max_x = max(p0_horiz.X, p1_horiz.X);

                    if ((p0_min_x <= pos_x) && (p0_max_x >= x))
                    {
                        Point intersection_orig = horiz_matrix.unapply(Point(pos_x, r));
                        double theta = atan2(intersection_orig.Y, intersection_orig.X) + M_PI;
                        cut_list[scanarc_idx].push_back(theta);
                        crossings_per_scanline[scanarc_idx].emplace_back(intersection_orig, poly_idx, point_idx);
                    }
                    if ((p0_min_x <= neg_x) && (p0_max_x >= neg_x))
                    {
                        Point intersection_orig = horiz_matrix.unapply(Point(neg_x, r));
                        double theta = atan2(intersection_orig.Y, intersection_orig.X) + M_PI;
                        cut_list[scanarc_idx].push_back(theta);
                        crossings_per_scanline[scanarc_idx].emplace_back(intersection_orig, poly_idx, point_idx);
                    }
                }
            }
        }
    }
}

