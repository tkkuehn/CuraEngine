#include "ConcentricArcInfill.h"
#include "../utils/logoutput.h"

namespace cura
{

void ConcentricArcInfill::generateConcentricArcInfill(const Polygons& in_outline, coord_t outline_offset, Polygons& result_lines, const int line_distance, const Point infill_origin, const coord_t infill_line_width)
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

    // translate polygons to ensure infill_origin is at (0, 0)
    outline.translate(-infill_origin);

    AABB boundary(outline);

    double max_x_sq = pow(std::max(abs(boundary.max.X), abs(boundary.min.X)), 2);
    double max_y_sq = pow(std::max(abs(boundary.max.Y), abs(boundary.min.Y)), 2);

    // maximum radius that would still be in the bounding box
    double furthest_point = sqrt(max_x_sq + max_y_sq);

    // no more than this many arcs will be necessary
    int arc_count = static_cast<int>((furthest_point - (infill_line_width / 2)) / line_distance);

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
            float this_theta = atan2(coordinate.Y, coordinate.X);
            float other_theta = atan2(other.coordinate.Y, other.coordinate.X);

            return this_theta < other_theta;
        }
    };

    // list of crossings for each scan arc
    std::vector<std::vector<Crossing>> crossings_per_scanarc;
    crossings_per_scanarc.resize(arc_count + 1);

    for (size_t poly_idx = 0; poly_idx < outline.size(); poly_idx++)
    {
        PolygonRef poly = outline[poly_idx];

        Point p0 = poly.back();

        for (size_t point_idx = 0; point_idx < poly.size(); point_idx++)
        {
            Point p1 = poly[point_idx];

            // figure out which arcs this segment might cross
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

            // rotate line segment so it's horizontal (to simplify finding crossings)
            float horiz_angle = -1 * atan2(p1.Y - p0.Y, p1.X - p0.X);
            PointMatrix horiz_matrix(horiz_angle * 180 / M_PI);
            double p0_h_x = static_cast<double>(p0.X) * horiz_matrix.matrix[0] + static_cast<double>(p0.Y) * horiz_matrix.matrix[1];

            double p1_h_x = static_cast<double>(p1.X) * horiz_matrix.matrix[0] + static_cast<double>(p1.Y) * horiz_matrix.matrix[1];
            double p1_h_y = static_cast<double>(p1.X) * horiz_matrix.matrix[2] + static_cast<double>(p1.Y) * horiz_matrix.matrix[3];
            double y_prime = p1_h_y;

            for (int scanarc_idx = scanarc_idx0; scanarc_idx != scanarc_idx1 + direction; scanarc_idx += direction)
            {
                // find all possible intersections between line segment and arc
                double r = (static_cast<double>(infill_line_width) / 2) + static_cast<double>(scanarc_idx * line_distance);

                double x_squared = pow(r, 2) - pow(y_prime, 2);

                if (x_squared < 0)
                {
                    continue;
                }
                else if (x_squared == 0)
                {
                    // horizontal line is tangent to scan arc => (x, y) = (0, r)
                    int x = 0;

                    // is the intersection actually on the line segment?
                    if (std::min(p0_h_x, p1_h_x) <= 0 && std::max(p0_h_x, p1_h_x) >= 0)
                    {
                        Point intersection_orig = horiz_matrix.unapply(Point(x, static_cast<int>(r)));
                        double theta = atan2(intersection_orig.Y, intersection_orig.X);// + M_PI;
                        cut_list[scanarc_idx].push_back(theta);
                        crossings_per_scanarc[scanarc_idx].emplace_back(intersection_orig, poly_idx, point_idx);
                    }
                }
                else if (x_squared > 0)
                {
                    // horizontal line segment could intersect scan arc in two places: +/- x
                    double pos_x = sqrt(x_squared);
                    double neg_x = -pos_x;

                    double segment_min_x = std::min(p0_h_x, p1_h_x);
                    double segment_max_x = std::max(p0_h_x, p1_h_x);

                    // is the positive intersection on the line segment?
                    if ((static_cast<double>(segment_min_x) <= pos_x) && (static_cast<double>(segment_max_x) >= pos_x))
                    {
                        Point intersection_orig = horiz_matrix.unapply(Point(static_cast<int>(pos_x), y_prime));
                        double theta = atan2(intersection_orig.Y, intersection_orig.X);
                        cut_list[scanarc_idx].push_back(theta);
                        crossings_per_scanarc[scanarc_idx].emplace_back(intersection_orig, poly_idx, point_idx);
                    }
                   
                    // is the negative intersection on the line segment?
                    if ((static_cast<double>(segment_min_x) <= neg_x) && (static_cast<double>(segment_max_x) >= neg_x))
                    {
                        Point intersection_orig = horiz_matrix.unapply(Point(static_cast<int>(neg_x), y_prime));
                        double theta = atan2(intersection_orig.Y, intersection_orig.X);
                        cut_list[scanarc_idx].push_back(theta);
                        crossings_per_scanarc[scanarc_idx].emplace_back(intersection_orig, poly_idx, point_idx);
                    }
                }
            }
            p0 = p1;
        }
    }

    // didn't find any intersections
    if (cut_list.size() == 0)
    {
        logWarning("No intersections found");
        return;
    }

    addArcInfill(outline, result_lines, boundary, cut_list, line_distance, infill_origin, infill_line_width);
}

void ConcentricArcInfill::addArcInfill(const Polygons& outline, Polygons& result, const AABB boundary, std::vector<std::vector<double>>& cut_list, const int line_distance, const Point infill_origin, const coord_t infill_line_width)
{
    double max_radius = sqrt(pow(std::max(abs(boundary.max.X), abs(boundary.min.X)), 2) + pow(std::max(abs(boundary.max.Y), abs(boundary.min.Y)), 2));
    unsigned int max_scanarc = static_cast<int>((max_radius - (infill_line_width / 2)) / line_distance);

    for (unsigned int scanarc_idx = 0; scanarc_idx < max_scanarc; scanarc_idx++)
    {
        // may be able to just get rid of this
        if (scanarc_idx >= cut_list.size())
        {
            break;
        }

        int radius = (infill_line_width / 2) + (scanarc_idx * line_distance);

        // vector of crossing angles along this scanarc
        std::vector<double>& crossings = cut_list[scanarc_idx];

        // lowest angle is -pi
        Point arc_origin(-radius, 0);

        // sort the crossings along this scanarc by angle (from -pi to pi)
        std::sort(crossings.begin(), crossings.end());

        if (crossings.size() == 0)
        {
            if (outline.inside(arc_origin))
            {
                // the full circle is contained in the polygons, so add "crossings" to generate the circle
                crossings.push_back(M_PI);
                crossings.push_back(-1 * M_PI);
            }
            else
            {
                continue;
            }
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

            // happens if we rotated crossings
            if (theta_diff < 0)
            {
                theta_diff += 2 * M_PI;
            }

            // arc length / minimum sample length
            int arc_segment_length = 2 * infill_line_width;
            int num_samples = floor(radius * theta_diff / arc_segment_length);

            // sample the arc of interest
            Point prev_sample(radius * cos(first_theta), radius * sin(first_theta));

            for (int i = 1; i < num_samples; i++)
            {
                double sample_theta = (((double) i) / ((double) (num_samples - 1)) * theta_diff) + first_theta;
                int sample_x = radius * cos(sample_theta);
                int sample_y = radius * sin(sample_theta);

                Point current_sample(sample_x, sample_y);

                Point untranslated_prev = prev_sample + infill_origin;
                Point untranslated_current = current_sample + infill_origin;

                if (untranslated_prev == untranslated_current)
                {
                    continue;
                }

                result.addLine(untranslated_prev, untranslated_current);

                prev_sample = current_sample;
            }
        }
    }
}

}
