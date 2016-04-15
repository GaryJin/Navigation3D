#ifndef BEZIER3TEST
#define BEZIER3TEST

#include <qdebug.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

//Quadratic bezier curve point
Vector2f BezierPoint(vector<Vector2f> cp, float t)
{
    float xa, xb;
    float ya, yb;
    float tsquare;

    xa = cp[2].x() + cp[0].x() - 2 * cp[1].x();
    xb = 2 * cp[1].x() - 2 * cp[0].x();

    ya = cp[2].y() + cp[0].y() - 2 * cp[1].y();
    yb = 2 * cp[1].y() - 2*cp[0].y();

    tsquare = t * t;


    double x = xa * tsquare + t * xb +cp[0].x();
    double y = ya * tsquare + t * yb +cp[0].y();

    Vector2f result(x,y);
    qDebug()<<"result"<<result.x()<<result.y();
    return result;
}
//Quadratic bezier curve
void ComputeBezier2( vector<Vector2f> cp, int numberOfPoints, vector<Vector2f> &curve )
{
    float   dt;
    int    i;

    dt = 1.0 / ( numberOfPoints - 1 );

    for( i = 0; i < numberOfPoints; i++)
        curve.push_back( BezierPoint( cp, i*dt ) );
}



/*
 * @brief Cumpute a point of cubic bezier curve
 * @param Points of Start point Control point1 Control point2 and End point
 * @param The bezier curve point in time t
 */
Vector2f PointOnCubicBezier3( vector<Vector2f> cp, float t )
{
    float   ax, bx, cx;
    float   ay, by, cy;
    float   tSquared, tCubed;
    Vector2f result;

    /*The coefficient of the cubit bezier polyomial*/
    cx = 3.0 * (cp[1].x() - cp[0].x());
    bx = 3.0 * (cp[2].x() - cp[1].x()) - cx;
    ax = cp[3].x() - cp[0].x() - cx - bx;

    cy = 3.0 * (cp[1].y() - cp[0].y());
    by = 3.0 * (cp[2].y() - cp[1].y()) - cy;
    ay = cp[3].y() - cp[0].y() - cy - by;

    /*Calculate the point in time t*/

    tSquared = t * t;
    tCubed = tSquared * t;

    result.x() = (ax * tCubed) + (bx * tSquared) + (cx * t) + cp[0].x();
    result.y() = (ay * tCubed) + (by * tSquared) + (cy * t) + cp[0].y();

    return result;
}

/*
 * @brief Calculate the cubic bezier curve
 * @param Four points 1.Start point 2.Control point1 3.Control point2 4.End point
 * @param The number of the points that interpolation to the curve
 * @param The cubic bezier curve
 */
void ComputeBezier3( vector<Vector2f> cp, int numberOfPoints, vector<Vector2f> &curve )
{
    float   dt;
    int    i;

    dt = 1.0 / ( numberOfPoints - 1 );

    for( i = 0; i < numberOfPoints; i++)
        curve.push_back(PointOnCubicBezier3( cp, i*dt ));
}

/*
 * @brief Cumpute a point of cubic bezier curve
 * @param Points of Start point Control point1 Control point2 and End point
 * @param The bezier curve point in time t
 */
Vector3f PointOnCubicBezier3D( vector<Vector3f> cp, float t )
{
    float   ax, bx, cx;
    float   ay, by, cy;
    float   az, bz, cz;
    float   tSquared, tCubed;
    Vector3f result;

    /*The coefficient of the cubit bezier polyomial*/
    cx = 3.0 * (cp[1].x() - cp[0].x());
    bx = 3.0 * (cp[2].x() - cp[1].x()) - cx;
    ax = cp[3].x() - cp[0].x() - cx - bx;

    cy = 3.0 * (cp[1].y() - cp[0].y());
    by = 3.0 * (cp[2].y() - cp[1].y()) - cy;
    ay = cp[3].y() - cp[0].y() - cy - by;

    cz = 3.0 * (cp[1].z() - cp[0].z());
    bz = 3.0 * (cp[2].z() - cp[1].z()) - cz;
    az = cp[3].z() - cp[0].z() - cz - bz;

    /*Calculate the point in time t*/

    tSquared = t * t;
    tCubed = tSquared * t;

    result.x() = (ax * tCubed) + (bx * tSquared) + (cx * t) + cp[0].x();
    result.y() = (ay * tCubed) + (by * tSquared) + (cy * t) + cp[0].y();
    result.z() = (az * tCubed) + (bz * tSquared) + (cz * t) + cp[0].z();

    return result;
}

/*
 * @brief Calculate the cubic bezier curve
 * @param Four points 1.Start point 2.Control point1 3.Control point2 4.End point
 * @param The number of the points that interpolation to the curve
 * @param The cubic bezier curve
 */
void ComputeBezier3D( vector<Vector3f> cp, int numberOfPoints, vector<Vector3f> &curve )
{
    float   dt;
    int    i;

    dt = 1.0 / ( numberOfPoints - 1 );

    for( i = 0; i < numberOfPoints; i++)
        curve.push_back(PointOnCubicBezier3D( cp, i*dt ));
}


#endif // BEZIER3TEST

