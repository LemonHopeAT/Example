#include "ConvexHull.h"
#include "Primitives/Headers/Points.h"
#include "Primitives/Headers/strobe_rectangle.h"
#include "UAC/Headers/drawingFigures.hpp"
#include "KeyPoints/Headers/match_container.hpp"


std::vector<SmartPoint> ConvexHull::grahamScan(std::vector<SmartPoint> stack)
{
    std::vector<SmartPoint> convex_hull;
    size_t n = stack.size();

    if (n < 3)
        return convex_hull;

    std::sort(stack.begin(), stack.end(), [](SmartPoint & a, SmartPoint & b) {return a.coord.x_ < b.coord.x_ || (a.coord.x_ == b.coord.x_ && a.coord.y_ < b.coord.y_); });

    for (size_t i = 0; i < n; ++i)
    {
        while (convex_hull.size() >= 2 && cross(convex_hull[convex_hull.size() - 2], convex_hull.back(), stack[i]) <= 0)
            convex_hull.pop_back();
        convex_hull.push_back(stack[i]);
    }

    for (size_t i = n - 1, t = convex_hull.size() + 1; i > 0; --i)
    {
        while (convex_hull.size() >= t && cross(convex_hull[convex_hull.size() - 2], convex_hull.back(), stack[i - 1]) <= 0)
            convex_hull.pop_back();
        convex_hull.push_back(stack[i - 1]);
    }
    return convex_hull;
}

std::vector<SmartPoint> ConvexHull::grahamScan(const MatchContainer &corresponds)
{
    assert(corresponds.size() > 0);
    std::vector<SmartPoint> points;
    for(const auto & el : corresponds)
    {
           points.emplace_back(el);
    }
    return grahamScan(points);
}

std::vector<SmartPoint*> ConvexHull::pointInsidePolygon(std::vector<SmartPoint> & stack, std::vector<SmartPoint> & convex_hull)
{
    std::vector<SmartPoint*> point_to_add;

    for (auto & it : stack)
    {
        if (contains(it.coord, convex_hull))
        {
            point_to_add.push_back(&it);
            //it.parameters.color = Blue;
        }

    }
    return point_to_add;
}


void ConvexHull::pointInsidePolygon(std::vector<SmartPoint> & stack, std::vector<SmartPoint> & convex_hull, std::vector<SmartPoint> & points_object)
{
    for (auto & it : stack)
    {
        if (contains(it.coord, convex_hull))
        {
            points_object.push_back(it);
            //it.parameters.color = Blue;
        }
    }
}

float ConvexHull::polygonArea(std::vector<SmartPoint> & polygon)
{
    if (polygon.empty())
        return 0.0f;

    size_t j;
    double area = 0;

    for (size_t i = 0; i < polygon.size(); i++)
    {
        j = (i + 1) % polygon.size();
        area += polygon[i].coord.x_ * polygon[j].coord.y_;
        area -= polygon[i].coord.y_ * polygon[j].coord.x_;
    }

    area /= 2;
    return(area < 0 ? -area : area);
}


float ConvexHull::perimeterArea(std::vector<SmartPoint> & polygon)
{
    if (polygon.empty())
        return 0.0f;

    float perimeter = 0.0f;

    for (size_t i = 0; i < polygon.size() - 1; i++)
    {
        perimeter += polygon[i].coord.distance(polygon[i + 1].coord);
    }

    perimeter += polygon.back().coord.distance(polygon[0].coord);

    return perimeter;
}


Point<float> ConvexHull::calculatePolygonBaricenter(std::vector<SmartPoint> const& polygon)
{
    Point<float> mean_centroid {-1.0f, -1.0f};
    if(polygon.empty())
        return mean_centroid;

    for (auto const& point : polygon)
        mean_centroid = mean_centroid + point.coord;

    mean_centroid = mean_centroid / polygon.size();

    return mean_centroid;
}


void ConvexHull::findTriangleStripRatio(std::vector<SmartPoint> const& polygon,
                                        size_t start_idx,
                                        Point<float> const& mass_center,
                                        std::array<float, min_edge_ratio_to_compare> & ratio_array)
{
    auto polygon_size = polygon.size();

    if (polygon_size < min_triangles_to_compare)
        return;

    size_t i;

    for (i = 0; i < min_triangles_to_compare; ++i)
    {
        auto idx_cur = (start_idx + i) % polygon_size;
        auto idx_next = (start_idx + i + 1) % polygon_size;

        ratio_array[2*i] = polygon[idx_cur].coord.distance(mass_center);
        ratio_array[2*i + 1] = polygon[idx_cur].coord.distance(polygon[idx_next].coord);
    }

    ratio_array[2*i] = polygon[(start_idx + i) % polygon_size].coord.distance(mass_center);
}

void ConvexHull::findTriangleStripRatio_inverse(std::vector<SmartPoint> const& polygon,
                                                size_t start_idx,
                                                Point<float> const& mass_center,
                                                std::array<float, min_edge_ratio_to_compare> & ratio_array)
{
    findTriangleStripRatio(polygon, start_idx, mass_center, ratio_array);
    invertTriangleStripRatio(ratio_array);
}


void ConvexHull::invertTriangleStripRatio(std::array<float, min_edge_ratio_to_compare> & ratio_array_1)
{
    for(auto & element : ratio_array_1)
    {
        if (element != 0.0f)
            element = 1.0f / element;
    }
}

float ConvexHull::compareRatioArrays(std::array<float, min_edge_ratio_to_compare> const& ratio_array_1,
                                     std::array<float, min_edge_ratio_to_compare> & ratio_array_2)
{
    float diff = 0.0f;

    for(size_t i = 0; i < min_edge_ratio_to_compare; ++i)
    {
        auto diff_i = ratio_array_1[i] * ratio_array_2[i];
        ratio_array_2[i] = diff_i;
        diff += diff_i;
    }

    diff *= min_edge_ratio_to_compare_inverse;

    return diff;
}

std::tuple<float, float, std::vector<SmartPoint>> ConvexHull::compareHulls(std::vector<SmartPoint> & polygon_1, std::vector<SmartPoint> & polygon_2)
{
    if (polygon_1.size() < min_triangles_to_compare || polygon_2.size() < min_triangles_to_compare)
        return {0.0f, 0.0f, {}};

    auto mass_center_1 = calculatePolygonBaricenter(polygon_1);
    auto mass_center_2 = calculatePolygonBaricenter(polygon_2);

    std::array <float, min_edge_ratio_to_compare> array_of_ratio_1;
    std::array <float, min_edge_ratio_to_compare> array_of_ratio_2;

    findTriangleStripRatio(polygon_1, 0, mass_center_1, array_of_ratio_1);
    invertTriangleStripRatio(array_of_ratio_1);

    float best_diff = std::numeric_limits<float>::max();
    float best_scale = std::numeric_limits<float>::max();
    int32_t best_index = -1;

    for (size_t i = 0; i < polygon_2.size(); ++i)
    {
        findTriangleStripRatio(polygon_2, i, mass_center_2, array_of_ratio_2);

        float difference_raw = compareRatioArrays(array_of_ratio_1, array_of_ratio_2);

        float difference_abs = fabs(difference_raw - 1.0f);

        if(difference_abs < best_diff)
        {
            best_diff = difference_abs;
            best_scale = difference_raw;
            best_index = i;
        }
    }

    std::vector<SmartPoint> outliers;
    float average_diff = 0.0f;

    if(best_index >= 0)
    {
        for (size_t i = 0; i < polygon_2.size(); ++i)
        {
            findTriangleStripRatio_inverse(polygon_1, i, mass_center_1, array_of_ratio_1);

            findTriangleStripRatio(polygon_2, i + best_index, mass_center_2, array_of_ratio_2);

            float difference = compareRatioArrays(array_of_ratio_1, array_of_ratio_2);

            difference = fabs(difference - 1.0f);

            average_diff += difference;

            difference = fabs(difference - best_diff) / (difference + best_diff + 1e-5);

            if(difference > 0.25f)
            {
                difference = fabs(array_of_ratio_2[1] - best_diff)/ (array_of_ratio_2[1] + best_diff + 1e-5);

                if(difference > 0.25f)
                {
                    outliers.emplace_back(polygon_2[(i + best_index) % polygon_2.size()]);
                }
            }
        }
    }
    else
    {
        return {0.0f, 0.0f, {}};
    }

    average_diff /= polygon_2.size();

    return {1.0f - average_diff, 1.0f + best_scale, outliers};
}

double ConvexHull::cross(const SmartPoint &O, const SmartPoint &A, const SmartPoint &B)
{
    return (A.coord.x_ - O.coord.x_) * (B.coord.y_ - O.coord.y_) - (A.coord.y_ - O.coord.y_) * (B.coord.x_ - O.coord.x_);
}
