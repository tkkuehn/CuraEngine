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
    outline.translate(-infill_origin);

    AABB boundary(outline);

    int boundary_width = boundary.max.X - boundary.min.X;
    int boundary_height = boundary.max.Y - boundary.min.Y;

    // potential issue with type conversions here?
    int arc_count = sqrt(pow(max(abs(boundary.max.X), abs(boundary.min.X)), 2) + pow(max(abs(boundary.max.Y), abs(boundary.min.Y)), 2)) / line_distance;

    // scanarc -> intersections with polygon segments
    std::vector<std::vector<double>> cut_list;

    for (int scanarc_idx = 0; scanarc_idx < arc_count; scanarc_idx++)
    {
        cut_list.push_back(std::vector<double>());
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
                scanarc_idx0 = floor((p0_r - (infill_line_width / 2)) / line_distance) + 1;
                scanarc_idx1 = floor((p1_r - (infill_line_width / 2)) / line_distance)
            }
            else
            {
                direction = -1;
                scanarc_idx0 = floor((p0_r - (infill_line_width / 2)) / line_distance);
                scanarc_idx1 = floor((p1_r - (infill_line_width / 2)) / line_distance) + 1;
            }

            // want to rotate line segment so it's horizontal
            horiz_angle = -1 * atan2(p1.Y - p0.Y, p1.X - p0.X);
            PointMatrix horiz_matrix(horiz_angle);

            for (int scanarc_idx = scanarc_idx0; scanarc_idx != scanarc_idx1 + direction; scanarc_idx += direction)
            {
                int r = (infill_line_width / 2) + (scanarc_idx * line_distance);
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
                        crossings_per_scanarc[scanarc_idx].emplace_back(intersection_orig, poly_idx, point_idx);
                    }
                }
                else if (x_squared > 0)
                {
                    // horizontal line intersects scan arc in two places: +/- x
                    int pos_x = sqrt(x_squared);
                    int neg_x = -1 * pos_x;

                    int segment_min_x = min(p0_horiz.X, p1_horiz.X);
                    int segment_max_x = max(p0_horiz.X, p1_horiz.X);

                    // is the positive intersection on the line segment?
                    if ((segment_min_x <= pos_x) && (segment_max_x >= x))
                    {
                        Point intersection_orig = horiz_matrix.unapply(Point(pos_x, r));
                        double theta = atan2(intersection_orig.Y, intersection_orig.X) + M_PI;
                        cut_list[scanarc_idx].push_back(theta);
                        crossings_per_scanarc[scanarc_idx].emplace_back(intersection_orig, poly_idx, point_idx);
                    }
                   
                    // is the negative intersection on the line segment?
                    if ((segment_min_x <= neg_x) && (segment_max_x >= neg_x))
                    {
                        Point intersection_orig = horiz_matrix.unapply(Point(neg_x, r));
                        double theta = atan2(intersection_orig.Y, intersection_orig.X) + M_PI;
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
    for (int scanarc_index = 0; scanarc_index < max_scanarc_index; scanarc_index++)
    {
        // first possible point along the sorted arc
        int radius = (infill_line_width / 2) + (scanarc_index * line_distance);

        Point arc_origin(radius, 0);

        // sort the crossings along this scanarc by angle (from zero to 2pi)
        std::sort(crossings_per_scanarc[scanarc_index].begin(), crossings_per_scanarc[scanarc_index].end());

        // if the first crossing goes from inside the polygon to outside, we get the opposite of the print arcs we want
        // so, rotate the sorted vector of crossings if necessary to avoid this case

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
            double first_theta = atan2(first.Y, first.X) + M_PI;
            double second_theta = atan2(second.Y, second.X) + M_PI;

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
                crossings_on_arc[first.polygon_index][first.vertex_index].push_back(new_segment);
                crossings_on_arc[second.polygon_index][second.vertex_index].push_back(new_segment);
            }
        }

        // also, if (r, 0) is in the polygon but there are no intersections, we want to print a full circle
        // but I don't think it matters here because there are no ends to connect
    }

    // didn't find any intersections
    if (cut_list.size() == 0)
    {
        return;
    }

    addArcInfill(outline, result, rotation_matrix, scanarc_min_idx, arc_distance, boundary, cut_list);
}

void ConcentricArcInfill::addArcInfill(const Polygons& outline, Polygons& result, const PointMatrix& rotation_matrix, const int scanline_min_idx, const int arc_distance, const AABB boundary, std::vector<std::vector<coord_t>>& cut_list)
{
    unsigned int scanarc_idx = 0;
    int max_radius = sqrt(pow(max(abs(boundary.max.X), abs(boundary.min.X)), 2) + pow(max(abs(boundary.max.Y), abs(boundary.min.Y)), 2));

    for (coord_t radius = (infill_line_width / 2) + (scanarc_idx * line_distance); r < max_radius; r += line_distance)
    {
        // because we're adding two each time
        if (scanarc_idx >= cut_list.size())
        {
            break;
        }

        std::vector<double>& crossings = cut_list[scanarc_idx];

        if (crossings.size() == 0)
        {
            if (outline.inside(arc_origin))
            {
                crossings.push_back(0);
                crossings.push_back(2 * M_PI);
            }
            else
            {
                continue;
            }
        }

        // sort the crossings along this scanarc by angle (from zero to 2pi)
        std::sort(crossings.begin(), crossings.end());

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
                theta_diff += (2 * M_PI);
            }

            // arc length / minimum sample length
            int num_samples = floor(radius * theta_diff / (infill_line_width / 5));

            // figure out how to sample the arc of interest here
            Point prev_sample(radius * cos(first_theta), radius * sin(first_theta));

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

                result.addLine(untranslated_prev, untranslated_current);

                prev_sample = current_sample;
            }
        }
    }
}
