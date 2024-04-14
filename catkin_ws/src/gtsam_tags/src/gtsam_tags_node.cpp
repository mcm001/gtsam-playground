#include "ros/ros.h"
#include "localizer.h"
#include "geometry_msgs/TwistStamped.h"
#include "apriltag_ros/AprilTagDetectionArray.h"

Localizer CreateLocalizer()
{

    // setup noise using fake numbers
    Cal3_S2 K(1300, 1300, 0, 1000, 500);
    // Pixel noise, in u,v coordinates
    auto measurementNoise =
        noiseModel::Isotropic::Sigma(2, 2.0);

    // Noise on the prior factor we use to anchor the first pose.
    // TODO: If we initialize with enough measurements, we might be able to delete this prior?
    Vector6 sigmas;
    sigmas << Vector3::Constant(0.1), Vector3::Constant(0.3);
    auto posePriorNoise = noiseModel::Diagonal::Sigmas(sigmas);

    // odometry noise
    Vector6 odomSigma;
    odomSigma << Vector3::Constant(0.001), Vector3::Constant(0.05);
    auto odometryNoise = noiseModel::Diagonal::Sigmas(odomSigma);

    Cal3_S2_ cal(K);

    return Localizer{
        cal,
        Pose3{},
        measurementNoise, odometryNoise, posePriorNoise, Pose3{}};
}

Localizer localizer = CreateLocalizer();

void odomCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    ROS_INFO("Twist: linear %f %f %f : angular %f %f %f",
             msg->linear.x,
             msg->linear.y,
             msg->linear.z,
             msg->angular.x,
             msg->angular.y,
             msg->angular.z);

    // TODO does gtsam's betweenfactor expect a twist or a pose delta?
    Pose3 gtsamTwist{
        Rot3::Rodrigues(
            msg->angular.x,
            msg->angular.y,
            msg->angular.z),
        Point3{
            msg->linear.x,
            msg->linear.y,
            msg->linear.z}};

    localizer.AddOdometry(gtsamTwist);
}
void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    ROS_INFO("> Saw %lu tags", msg->detections.size());

    // For now ignore latency compensation. we will need to deal with this at some point.
    for (size_t i = 0; i < msg->detections.size(); i++)
    {
        auto det = msg->detections.at(i);
        vector<Point2> corners;
        // todo apriltag_ros just does NOT tell us corners. LAME
        localizer.AddTagObservation(
            // assume not tag bundle
            localizer.currStateIdx, det.id[0], corners);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_isam");

    ros::NodeHandle n;

    ros::Subscriber odom = n.subscribe("odom", 1000, odomCallback);
    ros::Subscriber tags = n.subscribe("tags", 1000, tagCallback);

    ros::spin();

    return 0;
}