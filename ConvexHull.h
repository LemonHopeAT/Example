#ifndef TRAKING_CONVEXHULL_H_
#define TRAKING_CONVEXHULL_H_

#include <vector>
#include <array>
#include "Primitives/Headers/Points.h"

class MatchContainer;


#define COLINEAR 0
#define CLOCK_WISE 1
#define COUNTER_CLOCK_WISE 2


class ConvexHull
{
public:
    ConvexHull() = default;
    ~ConvexHull() = default;

	/**
	* @brief
	* Builds a convex hull around dots
	* 
	* @param stack some points
	*
	* @return convex hull stack
	*/
    std::vector<SmartPoint> grahamScan(std::vector<SmartPoint> stack);

	/**
	* @brief
	* Builds a convex hull around dots
	*
	* @param corresponds points tracked object and points scene
	*
	* @return convex hull stack
	*/
     std::vector<SmartPoint> grahamScan(const MatchContainer &corresponds);

	/**
	* @brief
	* Finds points in convex hull
	* 
	* @param stack some points
	* @param convex_hull some convex hull points
	*
	* @return points inside convex hull
	*/
    std::vector<SmartPoint*> pointInsidePolygon(std::vector<SmartPoint> & stack, std::vector<SmartPoint> & convex_hull);

	/**
	* @brief
	* Finds points in convex hull
	*
	* @param stack some points
	* @param convex_hull some convex hull points
	* @param points_object points inside convex hull
	* 
	*/
    void pointInsidePolygon(std::vector<SmartPoint> & stack, std::vector<SmartPoint> & convex_hull, std::vector<SmartPoint> & points_object);

	/**
	* @brief
	* Contains point in convex hull or not
	* 
	* @param point some points
	* @param convex_hull some convex hull points
	* 
	* @return contains point in convex hull or not
    */
    template <typename T>
    bool contains(Point<T> & point, std::vector<SmartPoint> & convex_hull);

	/**
	* @brief
	* Search for the polygon area
	* 
    * @param some polygon(convex hull)
	*
    * @return polygon area
	*/
    float polygonArea(std::vector<SmartPoint> & polygon);

	/**
	* @brief
    * Search for the polygon perimeter
	*
    * @param some polygon(convex hull)
	*
    * @return polygon perimeter
	*/
    float perimeterArea(std::vector<SmartPoint> & polygon);

    /**
    * @brief
    * Compares two convex hulls and tries to find outlier points in polygon_2
    *
    * @param polygon_1 and polygon_2 to compare
    *
    * @return similarity, scale ratio and vector of outliers in polygon_2
    */
    std::tuple<float, float, std::vector<SmartPoint>> compareHulls(std::vector<SmartPoint> & polygon_1, std::vector<SmartPoint> & polygon_2);

private:

    constexpr static size_t min_triangles_to_compare = 3;
    constexpr static size_t min_edges_to_compare = min_triangles_to_compare + 1;
    constexpr static size_t min_edge_ratio_to_compare = min_triangles_to_compare * 2 + 1;
    constexpr static float min_edge_ratio_to_compare_inverse = 1.0f / min_edge_ratio_to_compare;

    void invertTriangleStripRatio(std::array<float, min_edge_ratio_to_compare> & ratio_array_1);

    void findTriangleStripRatio(std::vector<SmartPoint> const& polygon_1,
                                size_t start_idx,
                                Point<float>const & mass_center,
                                std::array<float, min_edge_ratio_to_compare> & ratio_array);

    void findTriangleStripRatio_inverse(std::vector<SmartPoint> const& polygon,
                                        size_t start_idx,
                                        Point<float> const& mass_center,
                                        std::array<float, min_edge_ratio_to_compare> & ratio_array);

    float compareRatioArrays(std::array<float, min_edge_ratio_to_compare> const& ratio_array_1,
                             std::array<float, min_edge_ratio_to_compare> & ratio_array_2);

    Point<float> calculatePolygonBaricenter(std::vector<SmartPoint> const& polygon_1);

    double cross(const SmartPoint &O, const SmartPoint &A, const SmartPoint &B);

};





template <typename T>
bool ConvexHull::contains(Point<T> & point, std::vector<SmartPoint> & convex_hull)
{
    bool point_in_polygon = false;
    size_t size = convex_hull.size();

    if (size < 3)
        return point_in_polygon;

    for (size_t i = 0, j = size - 1; i < size; j = i, i++)
    {
        const auto & x1 = convex_hull[i].coord.x_;
        const auto & y1 = convex_hull[i].coord.y_;
        const auto & x2 = convex_hull[j].coord.x_;
        const auto & y2 = convex_hull[j].coord.y_;

        if ((x1 == point.x_ && y1 == point.y_) || (x2 == point.x_ && y2 == point.y_))
        {
            point_in_polygon = false;
            break;
        }

        if ((y1 < point.y_ && y2 >= point.y_) || (y1 >= point.y_ && y2 < point.y_))
        {
            float crossX = (point.y_ - y1) * (x2 - x1) / (y2 - y1) + x1;

            if (crossX == point.x_)
            {
                break;
            }

            if (crossX > point.x_)
                point_in_polygon = !point_in_polygon;
        }
    }

    return point_in_polygon;
}




#endif // TRAKING_CONVEXHULL_H_

