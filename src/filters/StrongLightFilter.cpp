#include <map>
#include <math.h>
#include "core/math/angles.h"
#include "StrongLightFilter.h"

#define MIN_VALUE 1e-8
#define IS_ZERO(v) (abs(v) < MIN_VALUE)
#define IS_EQUAL(v1, v2) IS_ZERO(v1 - v2)

StrongLightFilter::StrongLightFilter()
{
}

StrongLightFilter::~StrongLightFilter()
{
}

void StrongLightFilter::filter(
    const LaserScan &in,
    int lidarType,
    int version,
    LaserScan &out)
{
    int size = in.points.size(); // Score
    // Sort by angle
    std::map<float, int> ais;
    for (int i = 0; i < size; ++i)
    {
        const LaserPoint &p = in.points.at(i);
        if (IS_ZERO(p.range))
            continue;
        ais[p.angle] = i;
    }
    // printf("Sort by angle from point [%d] to point [%d]\n", size, ais.size());

    size = ais.size(); // Update points (points after filtering out invalid points).
    out = in;
    out.points.resize(size); // 更新点数
    std::map<float, int>::iterator it;
    int i = 0;
    for (it = ais.begin(); it != ais.end(); ++it)
    {
        out.points[i++] = in.points.at(it->second);
    }
    // Initialize variables
    std::vector<bool> noises(size, false); // Is it a noise marker?
    // Expand the traversal range to 104% of the original array to handle the first and last points.
    int sizeEx = int(size * 1.04);
    int startI = -1;  // Mark the starting point of the trail index.
    LaserPoint lastP; // Previous point information

    // main loop function
    for (int i = 0; i < sizeEx; ++i)
    {
        const LaserPoint &p = out.points.at(i % size);
        if (i != 0)
        {
            
            Point p1 = Point::polar2Angular(Point(p.angle, p.range));
            Point p2 = Point::polar2Angular(Point(lastP.angle, lastP.range));
            Point p3 = Point(0, 0); //origin
            Point p4 = Point((p1.x + p2.x) / 2.0f, (p1.y + p2.y) / 2.0f); //两点中点
            // Calculate the distance from the line connecting two points to the origin (in a rectangular coordinate system).
            float d = Point::calcDist(p3, p1, p2);
            //Calculate the angle between the lines formed by the two line segments (in a rectangular coordinate system).
            float a = Point::calcAngle(p1, p2, p3, p4);

            // printf("Point[%d] distance[%.03f]\n", i % size, d);
            // If the current distance is less than the standard, and the angle is less than the standard, then it is considered a trailing point
            if (d < maxDist && a < maxAngle)
            {
                // If the starting point is invalid, mark it
                if (-1 == startI)
                    startI = i;
            }
            // If the distance to a point is increasing, and the current distance is less than twice the standard distance, and the angle is less than the standard angle, then it is considered a trailing point.
            else if (-1 != startI &&
                p.range > lastP.range &&
                d < maxDist * 2 && 
                a < maxAngle)
            {
                // No processing
            }
            else
            {
                // Determine if the statistical location is valid; if valid, mark it.
                if (-1 != startI &&
                    i - startI >= minNoise)
                {
                    for (int j = startI; j <= i; ++j)
                    {
                        noises[j % size] = true;
                        const LaserPoint &pp = out.points.at(j % size);
                        // printf("noise[%d] a[%.02f] r[%.02f]\n",
                        //     j % size, ydlidar::core::math::to_degrees(pp.angle), pp.range);
                    }
                }

                startI = -1;
            }
        }
        lastP = p;
    }

    // Process the marked points
    int noiseCount = 0;
    for (int i = 0; i < size; ++i)
    {
        if (noises[i])
        {
            out.points[i].range = 0.0f;
            noiseCount++;
        }
    }

    // printf("Strong light filtering noise count[%d]\n", noiseCount);
}

StrongLightFilter::Point::Point(float x, float y)
    : x(x),
      y(y)
{
}

StrongLightFilter::Point StrongLightFilter::Point::angular2Polar(
    const StrongLightFilter::Point &p)
{
    // 1.Two coordinates in polar coordinate system r and θ The following formula can be used to convert the coordinates x in a rectangular coordinate system. = r*cos（θ），y = r*sin（θ）。
    // 2.From the two formulas above, we can obtain how to calculate the polar coordinates from the x and y coordinates in a rectangular coordinate system, r. = sqrt(x^2 + y^2),θ = arctan(y/x)
    //        float r = qSqrt(x * x + y * y);
    //        float theta = qAtan(y / x);

    // From the radian value[-M_PI/2,M_PI/2]Convert to[0, 2*M_PI]
    float theta = .0;
    if (!IS_ZERO(p.x)) // When x is not 0
    {
        theta = atan(p.y / p.x);
        if (p.x > 0.0)
        {
            if (p.y < 0.0)
                theta += (M_PI * 2.0);
        }
        else
        {
            theta += M_PI;
        }
    }
    return Point(theta, sqrt(p.x * p.x + p.y * p.y));
}

StrongLightFilter::Point StrongLightFilter::Point::polar2Angular(
    const StrongLightFilter::Point &p)
{
    return Point(p.y * cos(p.x), p.y * sin(p.x));
}

float StrongLightFilter::Point::calcDist(
    const StrongLightFilter::Point &p,
    const StrongLightFilter::Point &p1,
    const StrongLightFilter::Point &p2)
{
    // Calculate the shortest distance from a point to a line.
    // Input point P(x0,y0)and line AB（x1,y1,x2,y2）,Output the shortest distance from the point to the line.
    // # If two points are the same, output the coordinates of one point as the foot of the perpendicular.
    // if x1 == x2 and y1 == y2:
    //     return math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0))
    if (IS_EQUAL(p1.x, p2.x) &&
        IS_EQUAL(p1.y, p2.y))
        return sqrt((p1.x - p.x) * (p1.x - p.x) +
                    (p1.y - p.y) * (p1.y - p.y));

    // # Calculate the area using the vector outer product.
    // s = (x0 - x1) * (y2 - y1) - (y0 - y1) * (x2 - x1)
    float s = (p.x - p1.x) * (p2.y - p1.y) -
              (p.y - p1.y) * (p2.x - p1.x);
    // # Calculate the distance between two points on a straight line
    // d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    float d = sqrt((p2.x - p1.x) * (p2.x - p1.x) +
                   (p2.y - p1.y) * (p2.y - p1.y));

    // return math.fabs(s / d)
    return fabs(s / d);
}

float StrongLightFilter::Point::calcLen(
    const StrongLightFilter::Point &v)
{
    return sqrt(v.x * v.x + v.y * v.y);
}

float StrongLightFilter::Point::calcDot(
    const StrongLightFilter::Point &v1,
    const StrongLightFilter::Point &v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}

float StrongLightFilter::Point::calcAngle(
    const StrongLightFilter::Point &p1,
    const StrongLightFilter::Point &p2,
    const StrongLightFilter::Point &p3,
    const StrongLightFilter::Point &p4)
{
    Point v1(p2.x - p1.x, p2.y - p1.y); // Vector 1
    Point v2(p4.x - p3.x, p4.y - p3.y); // Vector 2
    // Calculate the angle between two vectors（acute angle）
    float theta = calcDot(v1, v2) / (calcLen(v1) * calcLen(v2));
    float a = acos(theta) * 180.0f / M_PI; 
    if (a < .0f)
        a = -a;
    if (a > 90.0f)
        a = 180.0f - a;
    return a;
}
