//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <limits>
#include "AABB.h"
#include "polygon.h" //To create the AABB of a polygon.

namespace cura
{


AABB::AABB()
: min(POINT_MAX, POINT_MAX), max(POINT_MIN, POINT_MIN)
{
}

AABB::AABB(const Point& min, const Point& max)
: min(min), max(max)
{
}

AABB::AABB(const Polygons& polys)
: min(POINT_MAX, POINT_MAX), max(POINT_MIN, POINT_MIN)
{
    calculate(polys);
}

AABB::AABB(ConstPolygonRef poly)
: min(POINT_MAX, POINT_MAX), max(POINT_MIN, POINT_MIN)
{
    calculate(poly);
}

Point AABB::getMiddle() const
{
    return (min + max) / 2;
}

void AABB::calculate(const Polygons& polys)
{
    min = Point(POINT_MAX, POINT_MAX);
    max = Point(POINT_MIN, POINT_MIN);
    for (unsigned int i = 0; i < polys.size(); i++)
    {
        for (unsigned int j = 0; j < polys[i].size(); j++)
        {
            include(polys[i][j]);
        }
    }
}

void AABB::calculate(ConstPolygonRef poly)
{
    min = Point(POINT_MAX, POINT_MAX);
    max = Point(POINT_MIN, POINT_MIN);
    for (const Point& p : poly)
    {
        include(p);
    }
}

bool AABB::contains(const Point& point) const
{
    return point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y;
}

bool AABB::hit(const AABB& other) const
{
    if (max.x < other.min.x) return false;
    if (min.x > other.max.x) return false;
    if (max.y < other.min.y) return false;
    if (min.y > other.max.y) return false;
    return true;
}

void AABB::include(Point point)
{
    min.x = std::min(min.x,point.x);
    min.y = std::min(min.y,point.y);
    max.x = std::max(max.x,point.x);
    max.y = std::max(max.y,point.y);
}

void AABB::include(const AABB other)
{
    // Note that this is different from including the min and max points, since when 'min > max' it's used to denote an negative/empty box.
    min.x = std::min(min.x, other.min.x);
    min.y = std::min(min.y, other.min.y);
    max.x = std::max(max.x, other.max.x);
    max.y = std::max(max.y, other.max.y);
}

void AABB::expand(int dist)
{
    if (min == Point(POINT_MAX, POINT_MAX) || max == Point(POINT_MIN, POINT_MIN))
    {
        return;
    }
    min.x -= dist;
    min.y -= dist;
    max.x += dist;
    max.y += dist;
}

Polygon AABB::toPolygon() const
{
    Polygon ret;
    ret.add(min);
    ret.add(Point(max.x, min.y));
    ret.add(max);
    ret.add(Point(min.x, max.y));
    return ret;
}

}//namespace cura

