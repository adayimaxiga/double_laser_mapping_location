//
// Created by droid on 18-8-16.
//
#include "../include/loop_closer_check/ceres_scan_matcher_2d.h"

#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "gtest/gtest.h"
#include "ros/ros.h"

#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"

#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"

#include <cmath>
#include <iostream>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"

#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

//思路 map中进行接收地图数据，激光直接接收并处理，
//amcl_pose中做扫描匹配，确定是不是进行branch and bound


namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {


    static const std::string scan_topic_1 = "scan_1";
    static const std::string scan_topic_2 = "scan_2";
    static const std::string tracking_frame_ = "base_link";
    class LoopClosuerCheck{
    public:
        LoopClosuerCheck()
        {
            map_sub_= nh_.subscribe("map", 1, &LoopClosuerCheck::mapReceived,this);
            amcl_pose_sub_ = nh_.subscribe("amcl_pose", 1, &LoopClosuerCheck::amclPoseReceived,this);
            laser1_sub_ = nh_.subscribe(scan_topic_1, 1, &LoopClosuerCheck::LaserScanReceived_1,this);
            laser2_sub_ = nh_.subscribe(scan_topic_2, 1, &LoopClosuerCheck::LaserScanReceived_2,this);

            tfb_.reset(new tf2_ros::TransformBroadcaster());
            tf_.reset(new tf2_ros::Buffer());
            tfl_.reset(new tf2_ros::TransformListener(*tf_));
        }
        ~LoopClosuerCheck()
        {
        }

        std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
        std::shared_ptr<tf2_ros::TransformListener> tfl_;
        std::shared_ptr<tf2_ros::Buffer> tf_;


        void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
        void
        HandleLaserScan(
                const std::string& sensor_id, const cartographer::common::Time time,
                const std::string& frame_id,
                const cartographer::sensor::PointCloudWithIntensities& points);

        void printMap();
        //call_back;
        void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
        void LaserScanReceived_1(const sensor_msgs::LaserScanConstPtr& laser_scan);
        void LaserScanReceived_2(const sensor_msgs::LaserScanConstPtr& laser_scan);
        void amclPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

        std::tuple<::cartographer::sensor::PointCloudWithIntensities, ::cartographer::common::Time>
        LaserScanToPointCloudWithIntensities(const sensor_msgs::LaserScan& msg);
        ::cartographer::common::Time FromRos(const ::ros::Time& time);
        ::ros::Time ToRos(::cartographer::common::Time time);


        ::cartographer::transform::Rigid3d ToRigid3d(const geometry_msgs::TransformStamped& transform);
        ::cartographer::transform::Rigid3d ToRigid3d(const geometry_msgs::Pose& pose);
        Eigen::Vector3d ToEigen(const geometry_msgs::Vector3& vector3);
        Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion& quaternion);
        protected:
        ProbabilityGrid *probability_grid_;
        ros::NodeHandle nh_;
        ros::Subscriber map_sub_,amcl_pose_sub_;
        ros::Subscriber laser1_sub_;
        ros::Subscriber laser2_sub_;
        nav_msgs::OccupancyGrid map_save;
        float origin_pose_x;
        float origin_pose_y;
        float origin_pose_w;

    };

//***********************convert tools****************************
    ::cartographer::transform::Rigid3d
    LoopClosuerCheck::ToRigid3d(const geometry_msgs::TransformStamped& transform) {
        return ::cartographer::transform::Rigid3d(ToEigen(transform.transform.translation),
                       ToEigen(transform.transform.rotation));
    }

    ::cartographer::transform::Rigid3d
    LoopClosuerCheck::ToRigid3d(const geometry_msgs::Pose& pose) {
        return ::cartographer::transform::Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                       ToEigen(pose.orientation));
    }
    Eigen::Vector3d
    LoopClosuerCheck::ToEigen(const geometry_msgs::Vector3& vector3) {
        return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
    }

    Eigen::Quaterniond
    LoopClosuerCheck::ToEigen(const geometry_msgs::Quaternion& quaternion) {
        return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                                  quaternion.z);
    }




    ::ros::Time
    LoopClosuerCheck::ToRos(::cartographer::common::Time time) {
        int64_t uts_timestamp = ::cartographer::common::ToUniversal(time);
        int64_t ns_since_unix_epoch =
                (uts_timestamp -
                 ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds *
                 10000000ll) *
                100ll;
        ::ros::Time ros_time;
        ros_time.fromNSec(ns_since_unix_epoch);
        return ros_time;
    }

    ::cartographer::common::Time
    LoopClosuerCheck::FromRos(const ::ros::Time& time) {
        // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
        // exactly 719162 days before the Unix epoch.
        return ::cartographer::common::FromUniversal(
                (time.sec +
                 ::cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
                10000000ll +
                (time.nsec + 50) / 100);  // + 50 to get the rounding correct.
    }


//***********************handle map****************************
    void
    LoopClosuerCheck::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
    {
        ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
                 msg.info.width,
                 msg.info.height,
                 msg.info.resolution);
        map_save = msg;
        //这里给出概率珊格地图的界限
        probability_grid_ = new ProbabilityGrid(MapLimits(msg.info.resolution,
                Eigen::Vector2d(abs(msg.info.origin.position.x) + (msg.info.width/ 2) * msg.info.resolution,
                        abs(msg.info.origin.position.y) + (msg.info.height/ 2) * msg.info.resolution),
                        CellLimits(msg.info.width, msg.info.height)));
        origin_pose_x = msg.info.origin.position.x;
        origin_pose_y = msg.info.origin.position.y;

        //地图坐标自底部向上，自左向右。
        for(int j=0;j<msg.info.height;j++) {
            for (int i = 0; i < msg.info.width; i++) {

                probability_grid_->SetProbability(Eigen::Array2i(i,j),
                       (float)msg.data[i+j*msg.info.width]/100.f);
            }
        }
        //printMap();

    }
    //调试用，打印地图和机器人位置。
    void
    LoopClosuerCheck::printMap() {
        std::cout<<"robot position in resolution is x:"
                 <<(int)((0-origin_pose_x)/map_save.info.resolution)
                 <<" y:"<<(int)((0-origin_pose_y)/map_save.info.resolution);
        for(int j=0;j<map_save.info.height;j++) {
            for (int i = 0; i < map_save.info.width; i++) {
                //这里机器人位置是相对于地图origin 的位置，因此要减去origin_pose_x；
                //后面把0改成要减去的机器人位置。
                if((i==((int)((0-origin_pose_x)/map_save.info.resolution)))
                   &&(j==((int)((0-origin_pose_y)/map_save.info.resolution)))) {
                    std::cout << "0";
                    //输出机器人位置
                }else
                {
                    if ((float) map_save.data[i + j * map_save.info.width] / 100.f > 0.65) {
                        std::cout << "*";
                    } else
                        std::cout << " ";
                }
            }
            std::cout<<std::endl;
        }

    }
//***************************************************handle laser**************

//转换到珊格地图下
std::tuple<::cartographer::sensor::PointCloudWithIntensities, ::cartographer::common::Time>
    LoopClosuerCheck::LaserScanToPointCloudWithIntensities(const sensor_msgs::LaserScan& msg) {
        ::cartographer::sensor::PointCloudWithIntensities point_cloud;

        float angle = msg.angle_min;
        for (size_t i = 0; i < msg.ranges.size(); ++i) {
            const auto& echoes = msg.ranges[i];
            const float first_echo = echoes;
            if (msg.range_min <= first_echo && first_echo <= msg.range_max)
            {
                const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
                Eigen::Vector4f point;
                point << rotation * (first_echo * Eigen::Vector3f::UnitX()),
                        i * msg.time_increment;
                point_cloud.points.push_back(point);
                if (msg.intensities.size() > 0) {
                    const auto& echo_intensities = msg.intensities[i];
                    point_cloud.intensities.push_back(echo_intensities);
                } else {
                    point_cloud.intensities.push_back(0.f);
                }
            }

            angle += msg.angle_increment;

        }
        ::cartographer::common::Time timestamp = FromRos(msg.header.stamp);
        if (!point_cloud.points.empty()) {
            const double duration = point_cloud.points.back()[3];
            timestamp += cartographer::common::FromSeconds(duration);
            for (Eigen::Vector4f& point : point_cloud.points) {
                point[3] -= duration;
            }
        }
        return std::make_tuple(point_cloud, timestamp);
    }
    //这里做坐标系变换。
    void
    LoopClosuerCheck::HandleLaserScan(
        const std::string& sensor_id, const cartographer::common::Time time,
        const std::string& frame_id,
        const cartographer::sensor::PointCloudWithIntensities& points
        ) {

        if (points.points.empty()) {
            return;
        }
        ::ros::Time requested_time = ToRos(time);
        //tfbuffer
        ::cartographer::transform::Rigid3d sensor_to_tracking(ToRigid3d(tf_->lookupTransform(
                tracking_frame_, frame_id, requested_time, ros::Duration(1.0))));


            cartographer::sensor::TimedPointCloudData point_cloud_processed{
                    time, sensor_to_tracking.translation().cast<float>(),
                    cartographer::sensor::TransformTimedPointCloud(
                            points.points, sensor_to_tracking.cast<float>())};

    }
//***********************callback***********************

    void
    LoopClosuerCheck::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
    {
        handleMapMessage( *msg );
    }

    void
    LoopClosuerCheck::amclPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
    {

    }
    void
    LoopClosuerCheck::LaserScanReceived_1(const sensor_msgs::LaserScanConstPtr& msg)
    {
        ::cartographer::sensor::PointCloudWithIntensities point_cloud;
        ::cartographer::common::Time time;
        std::tie(point_cloud, time) = LaserScanToPointCloudWithIntensities(*msg);
        HandleLaserScan("0", time, msg->header.frame_id, point_cloud);
    }
    void
    LoopClosuerCheck::LaserScanReceived_2(const sensor_msgs::LaserScanConstPtr& laser_scan)
    {

    }


}
}
}
}

//主函数没啥东西，构造一下就等着收货吧
int main(int argc, char** argv) {

    ros::init(argc, argv, "LoopClosuerCheck");
    cartographer::mapping::scan_matching::LoopClosuerCheck loop;

    ros::spin();


    return 0;
}
