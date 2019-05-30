#include <cfloat>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <piksi_rtk_msgs/VelNed.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <array>
#include <bitset>

#define NODE_NAME "ltu_actor_waypoint"

#define WATCH(x) ROS_ERROR_STREAM(#x << ": " << x);

/* Class TwistNode
 */
class GotoWaypoint {
public:
    GotoWaypoint();
    void run();

private:
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void waypointCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void gpsVelnedCallback(const piksi_rtk_msgs::VelNed::ConstPtr& msg);

    ros::NodeHandle    nh;
    ros::ServiceServer service;

    ros::Subscriber    gps_sub;
    ros::Subscriber    gps_velned_sub;
    ros::Subscriber    waypoint_sub;

    ros::Publisher     twist_pub;
    ros::Publisher     debug_angle_pub;

    sensor_msgs::NavSatFix gps_fix;
    piksi_rtk_msgs::VelNed gps_velned;

    sensor_msgs::NavSatFix target;

    ros::Rate limiter;

    double accumulate;
    double mult_accumulate;
    double mult;
    double speed;
    bool reverse;
};

// Set up publisher and subscriber
GotoWaypoint::GotoWaypoint()
    : nh{"~"},
      limiter(10),
      speed(1.5),
      mult(0.5),
      mult_accumulate(0.0),
      accumulate(0.0),
      reverse(false)
{

#define LOAD_PARAM_TO_STRING(param, name)   \
    std::string name;                       \
    if (!nh.getParam(param, name)) {        \
        ROS_ERROR_STREAM("Route-Waypoint: parameter '" << param << "' not defined."); exit(0); }

    // Load all rosparams
    LOAD_PARAM_TO_STRING("gps_fix", gps_fix_topic); // /piksi/navsatfix_best_fix
    LOAD_PARAM_TO_STRING("gps_vel_ned", gps_vel_ned_topic); // /piksi/vel_ned

#undef LOAD_PARAM_TO_STRING

    // Load configuration params
    nh.getParam("speed", speed);
    nh.getParam("mult", mult);
    nh.getParam("mult_accumulate", mult_accumulate);
    nh.getParam("accumulate", accumulate);
    nh.getParam("reverse", reverse);

    // Global
    gps_sub         = nh.subscribe<sensor_msgs::NavSatFix>(gps_fix_topic,  4, &GotoWaypoint::gpsCallback, this);
    gps_velned_sub  = nh.subscribe<piksi_rtk_msgs::VelNed>(gps_vel_ned_topic, 4, &GotoWaypoint::gpsVelnedCallback, this);

    // Local
    waypoint_sub    = nh.subscribe<sensor_msgs::NavSatFix>("waypoint", 4, &GotoWaypoint::waypointCallback, this);
    debug_angle_pub = nh.advertise<std_msgs::Float64>("debug_angle", 1);

    gps_fix.status.status = -1; // NO FIX
}

void GotoWaypoint::run()
{
    static unsigned int counter = 0;
    while (ros::ok())
    {
        counter++;

        ros::spinOnce();
        geometry_msgs::Twist command;
        std_msgs::Float64 angle_msg;
        command.linear.x = speed;


        if (gps_fix.status.status == -1)
        {
            if (counter % 50 == 0) ROS_ERROR_STREAM(NODE_NAME << ": No GPS Fix!");
        }
        else if (gps_velned.n_sats < 2)
        {
            if (counter % 50 == 0) ROS_ERROR_STREAM(NODE_NAME << ": Bad vel_ned (not enough sats)!");
        }
        else
        {
            float diff_lat, diff_lon, curr_lat, curr_lon;

            // Target direction
            diff_lat = target.latitude - gps_fix.latitude;
            diff_lon = cos(M_PI/180.0*target.latitude)*(target.longitude - gps_fix.longitude);

            // Current direction
            curr_lat = gps_velned.n;
            curr_lon = gps_velned.e;


            // normalize
            float dist = std::sqrt(diff_lat * diff_lat + diff_lon * diff_lon);
            diff_lat /= dist;
            diff_lon /= dist;

            float speed = std::sqrt(curr_lat * curr_lat + curr_lon * curr_lon);
            curr_lat /= speed;
            curr_lon /= speed;

            float velned_angle = atan2(curr_lat, curr_lon);
            //WATCH(accumulate);
            //WATCH(mult_accumulate);

            // https://stackoverflow.com/a/21486462
            // atan2(2DCross(A,B), 2DDot(A,B));
            float angle = -1 * std::atan2(curr_lat * diff_lon - curr_lon * diff_lat,
                                     curr_lat * diff_lat + curr_lon * diff_lon);

            //float angle = atan2(curr_lat, curr_lon) - atan2(diff_lat, diff_lon);
            //if (angle > 3.14159) angle -= 3.14159;
            //else if (angle < -3.14159) angle += 3.14159;

            if (std::isnan(angle)) angle = 0;

            angle_msg.data = angle;

            if (speed > 600) accumulate += angle * limiter.cycleTime().nsec/1000000000.0f * mult_accumulate;

            if (accumulate > 0.1) accumulate = 0.1;
            else if (accumulate < -0.1) accumulate = -0.1;

            command.angular.z = speed > 500 ? angle * mult : 0; 

            if (!std::isnan(accumulate))
                command.angular.z += accumulate;

            //ROS_ERROR_STREAM("accumulate: " << accumulate << "\nangle: " << angle << "\nspeed: " << speed << "\nvelnedangle: " << velned_angle);

            float scale = dist * 4503.0f;
            scale *= scale;

            if (std::abs(command.angular.z) > scale)
            {
                command.angular.z = copysignf(scale, command.angular.z);
            }

            if (reverse) command.angular.z *= -1;

            if (twist_pub.getNumSubscribers() > 0) twist_pub.publish(command);
            debug_angle_pub.publish(angle_msg);
        }

        limiter.sleep();
    }
}

void GotoWaypoint::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    gps_fix = *msg;
}

void GotoWaypoint::waypointCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    target = *msg;
}

void GotoWaypoint::gpsVelnedCallback(const piksi_rtk_msgs::VelNed::ConstPtr& msg)
{
    gps_velned = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    GotoWaypoint().run();
}
