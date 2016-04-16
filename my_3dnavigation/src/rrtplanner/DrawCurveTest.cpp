#include <ros/ros.h>

//msgs
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cmath>
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

class ReceiveAndDraw
{
public:
    ReceiveAndDraw()
    {
        ros::NodeHandle private_nh;
        pubLine = private_nh.advertise<visualization_msgs::Marker>("ResultLine", 100);
        pubCurve = private_nh.advertise<visualization_msgs::Marker>("BezierCurve", 100);  
        sub = private_nh.subscribe<visualization_msgs::Marker>("SolutionPoints" ,100, &ReceiveAndDraw::DrawCallback, this);

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
    void ComputeBezier3d( vector<Vector3f> cp, int numberOfPoints, vector<Vector3f> &curve )
    {
        float   dt;
        int    i;

        dt = 1.0 / ( numberOfPoints - 1 );

        for( i = 0; i < numberOfPoints; i++)
            curve.push_back(PointOnCubicBezier3D( cp, i*dt ));
    }

private:
    ros::Publisher pubLine;
    ros::Publisher pubCurve;
    ros::Subscriber sub;
    vector<Vector3f> beziercurve;
    visualization_msgs::Marker LinesToDraw;
    visualization_msgs::Marker CurveToDraw;
    void DrawCallback(const visualization_msgs::Marker::ConstPtr& points)
    {
        //Set the line style
        CurveToDraw.header.frame_id = LinesToDraw.header.frame_id = points->header.frame_id;
        CurveToDraw.header.stamp = LinesToDraw.header.stamp = points->header.stamp;
        CurveToDraw.ns = LinesToDraw.ns = points->ns;
        CurveToDraw.action = LinesToDraw.action = points->action;
        CurveToDraw.pose.orientation.w = LinesToDraw.pose.orientation.w = points->pose.orientation.w;
        CurveToDraw.id = 2;
        LinesToDraw.id = 1;
        CurveToDraw.type = LinesToDraw.type = visualization_msgs::Marker::LINE_STRIP;
        CurveToDraw.scale.x = LinesToDraw.scale.x = 0.1;

        LinesToDraw.color.r = 1.0;
        LinesToDraw.color.g = 1.0;
        LinesToDraw.color.a = 1.0;

        CurveToDraw.color.r = 1.0;
        CurveToDraw.color.a = 1.0;

        LinesToDraw.points.clear ();
        beziercurve.clear();
	    CurveToDraw.points.clear();
        vector<Vector3f> _previousSolution;
        for(int i=0; i<points->points.size (); i++)
        {
            Vector3f point(points->points.data()[i].x, points->points.data()[i].y, points->points.data()[i].z);
            _previousSolution.push_back(point);
        }

        beziercurve.push_back(_previousSolution[0]);

        const float VelocityDrawingMultiplier = 0.1;
        Vector3f _startVel = Vector3f(1, 0, 0);
        Vector3f _goalVel = Vector3f(0, -1, 0);
        Vector3f prevControlDiff = -_startVel*VelocityDrawingMultiplier;
        for (int i = 1; i < _previousSolution.size(); i++) {
            //Calculate the two control points
            Vector3f waypoint = _previousSolution[i];
            Vector3f prevWaypoint = _previousSolution[i-1];

            Vector3f controlDir;
            float controlLength;
            if (i == _previousSolution.size() - 1) {
                controlLength = _goalVel.norm() * VelocityDrawingMultiplier;
                controlDir = -_goalVel.normalized();
            } else {

                Vector3f nextWaypoint = _previousSolution[i+1];
                controlLength = 0.5*min( (waypoint - prevWaypoint).norm(), (nextWaypoint - waypoint).norm() );
                controlDir = ((prevWaypoint - waypoint).normalized() - (nextWaypoint - waypoint).normalized()).normalized();
            }


            Vector3f controlDiff = controlDir * controlLength;

            vector<Vector3f> tempforbezier;

            //To add the start point and two control point and the aim point
            tempforbezier.push_back(beziercurve.back ());
            tempforbezier.push_back(prevWaypoint - prevControlDiff);
            tempforbezier.push_back(waypoint + controlDiff);
            tempforbezier.push_back(waypoint);
            ComputeBezier3d(tempforbezier, 10, beziercurve);
        }
        for(int i=0; i<points->points.size (); i++)
        {
            geometry_msgs::Point p;
            p.x = points->points.data()[i].x;
            p.y = points->points.data()[i].y;
            p.z = points->points.data()[i].z;
            LinesToDraw.points.push_back (p);
        }

        for(vector<Vector3f>::iterator i = beziercurve.begin(); i<beziercurve.end(); i++)
    	{
            geometry_msgs::Point p;
            p.x = i->x();
            p.y = i->y();
            p.z = i->z();
            CurveToDraw.points.push_back(p);
        }
    	pubLine.publish(LinesToDraw);
        pubCurve.publish(CurveToDraw);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ReceiveAndDraw");
    ReceiveAndDraw my_receiver;

    ros::spin ();
    return 0;
}