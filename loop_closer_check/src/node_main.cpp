//
// Created by droid on 18-8-16.
//
#include "../include/loop_closer_check/ceres_scan_matcher_2d.h"
#include "../include/loop_closer_check/voxel_filter.h"
#include "../include/loop_closer_check/fast_correlative_scan_matcher_2d.h"
#include <memory>
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/sensor/odometry_data.h"

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
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
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
            cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("cloud", 5);
            pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("b_b_pose", 1, true);
            tfb_.reset(new tf2_ros::TransformBroadcaster());
            tf_.reset(new tf2_ros::Buffer());
            tfl_.reset(new tf2_ros::TransformListener(*tf_));

            auto parameter_dictionary = common::MakeDictionary(R"text(
        return {
          occupied_space_weight = 1.,
          translation_weight = 1.0,
          rotation_weight = 1.0,
          ceres_solver_options = {
            use_nonmonotonic_steps = true,
            max_num_iterations = 500,
            num_threads = 1,
          },
        })text");

            auto voxel_filter_parameter_dictionary = common::MakeDictionary(R"text(
        return {
          max_length = 0.5,
          min_num_points = 300,
          max_range = 50.,
        })text");

            voxel_options =
                    cartographer::sensor::CreateAdaptiveVoxelFilterOptions(voxel_filter_parameter_dictionary.get());

            const proto::CeresScanMatcherOptions2D options =
                    CreateCeresScanMatcherOptions2D(parameter_dictionary.get());
            ceres_scan_matcher_ = common::make_unique<CeresScanMatcher2D>(options);
        }
        ~LoopClosuerCheck()
        {
        }
        cartographer::sensor::proto::AdaptiveVoxelFilterOptions voxel_options;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
        std::shared_ptr<tf2_ros::TransformListener> tfl_;
        std::shared_ptr<tf2_ros::Buffer> tf_;


        void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
        void
        HandleLaserScan(
                const int& sensor_id, const cartographer::common::Time time,
                const std::string& frame_id,
                const cartographer::sensor::PointCloudWithIntensities& points);

        void printMap();
        //call_back;
        void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
        void LaserScanReceived_1(const sensor_msgs::LaserScanConstPtr& laser_scan);
        void LaserScanReceived_2(const sensor_msgs::LaserScanConstPtr& laser_scan);
        void amclPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

        void PublishPointCloud(const cartographer::sensor::PointCloud&  point_cloud_show);

        std::tuple<::cartographer::sensor::PointCloudWithIntensities, ::cartographer::common::Time>
        LaserScanToPointCloudWithIntensities(const sensor_msgs::LaserScan& msg);
        ::cartographer::common::Time FromRos(const ::ros::Time& time);
        ::ros::Time ToRos(::cartographer::common::Time time);

        std::unique_ptr<cartographer::sensor::OdometryData>
        ToOdometryData(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
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
        ros::Publisher cloud_pub_,pose_pub_;
        nav_msgs::OccupancyGrid map_save;
        float origin_pose_x;
        float origin_pose_y;
        float origin_pose_w;

        transform::Rigid3d amcl_data;
        std::unique_ptr<CeresScanMatcher2D> ceres_scan_matcher_;
        cartographer::sensor::TimedPointCloudData laser_scan_point_data[2];

        geometry_msgs::Pose FastCorrelativeScanMatcher(const cartographer::transform::Rigid2d& initial_pose_estimate,const cartographer::sensor::PointCloud&  point_cloud_match,float &score_return);
        int map_receive = 0;
        int laser_receive_1 = 0;
        int laser_receive_2 = 0;
    };
//***********************fast full map match****************************
    proto::FastCorrelativeScanMatcherOptions2D
    CreateFastCorrelativeScanMatcherTestOptions2D(
            const int branch_and_bound_depth) {
        auto parameter_dictionary =
                common::MakeDictionary(R"text(
      return {
         linear_search_window = 5.,
         angular_search_window = math.rad(30.),
         branch_and_bound_depth = )text" +
                                       std::to_string(branch_and_bound_depth) + "}");
        return CreateFastCorrelativeScanMatcherOptions2D(parameter_dictionary.get());
    }

    geometry_msgs::Pose
    LoopClosuerCheck::FastCorrelativeScanMatcher(const cartographer::transform::Rigid2d& initial_pose_estimate,const cartographer::sensor::PointCloud&  point_cloud_match,float &score_return)
    {
        const auto options = CreateFastCorrelativeScanMatcherTestOptions2D(8);
        FastCorrelativeScanMatcher2D fast_correlative_scan_matcher(*probability_grid_,
                                                                   options);
        transform::Rigid2d pose_estimate;
        constexpr float kMinScore = 0.5f;
        float score;
        fast_correlative_scan_matcher.Match(initial_pose_estimate,
                point_cloud_match, kMinScore, &score, &pose_estimate);
        geometry_msgs::Pose match_result;
        tf2::Quaternion q;
        q.setRPY(0, 0, pose_estimate.rotation().angle());
        tf2::convert(q, match_result.orientation);
        match_result.position.x =  pose_estimate.translation().x();
        match_result.position.y =  pose_estimate.translation().y();
        score_return = score;
        std::cout<<std::endl<<"全局匹配位置为："<<transform::ToProto(pose_estimate).DebugString()<<"score is : "<<score;
        return match_result;
    }


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

    std::unique_ptr<cartographer::sensor::OdometryData>
    LoopClosuerCheck::ToOdometryData(
            const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
        const cartographer::common::Time time = FromRos(msg->header.stamp);

        return cartographer::common::make_unique<cartographer::sensor::OdometryData>(
                cartographer::sensor::OdometryData{
                        time, ToRigid3d(msg->pose.pose)});
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
                Eigen::Vector2d(msg.info.origin.position.x + msg.info.width * msg.info.resolution,
                        msg.info.origin.position.y + msg.info.height * msg.info.resolution),
                        CellLimits(msg.info.height, msg.info.width)));
        origin_pose_x = msg.info.origin.position.x;
        origin_pose_y = msg.info.origin.position.y;
        std::cout<<"cell index :"<<probability_grid_->limits().GetCellIndex(Eigen::Vector2f(0.f, 0.f));
        //地图坐标自底部向上，自左向右。
        for(int j=0;j<msg.info.height;j++) {
            for (int i = 0; i < msg.info.width; i++) {
                //std::cout << "x : " << i << "y :" << j << std::endl;
                //std::cout<<probability_grid_->limits().GetCellIndex(
                //        Eigen::Vector2f((i+1)*msg.info.resolution + origin_pose_x ,(j+1)*msg.info.resolution + origin_pose_y))<<std::endl;

                if(msg.data[i + j * msg.info.width]!=-1) {
                    float probably_=0.1f;
                    if((float) msg.data[i + j * msg.info.width] / 100.f>0.9f)
                    {
                        probably_ = 0.9f;
                    }else if(msg.data[i + j * msg.info.width] / 100.f<0.1f)
                    {
                        probably_ = 0.1f;
                    }else
                    {
                        probably_ = msg.data[i + j * msg.info.width]/ 100.f;
                    }
                    //std::cout<<std::endl<<"Probably grid is :"<<probably_;
                    probability_grid_->ApplyLookupTable(
                            probability_grid_->limits().GetCellIndex(
                                    Eigen::Vector2f((i + 1) * msg.info.resolution + origin_pose_x,
                                                    (j + 1) * msg.info.resolution + origin_pose_y)),
                            ComputeLookupTableToApplyCorrespondenceCostOdds(Odds(probably_)));
                    probability_grid_ ->FinishUpdate();
                }
            }
        }


        //for(int j=0;j<msg.info.height;j++) {
        //    for (int i = 0; i < msg.info.width; i++) {
        //        probability_grid_->SetProbability(Eigen::Array2i(i,j),
        //               (float)msg.data[i+j*msg.info.width]/100.f);
        //    }
        //}
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
    void
    LoopClosuerCheck::PublishPointCloud(const cartographer::sensor::PointCloud&  point_cloud_show)
    {
        sensor_msgs::PointCloud cloud;
        cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = "base_link";
        cloud.points.resize(point_cloud_show.size());
        cloud.channels.resize(1);
        cloud.channels[0].name = "la";
        cloud.channels[0].values.resize(point_cloud_show.size());

        for(int i=0;i<point_cloud_show.size();i++) {

            cloud.points[i].x = point_cloud_show[i](0);
            cloud.points[i].y = point_cloud_show[i](1);
            cloud.points[i].z = point_cloud_show[i](2);
            cloud.channels[0].values[i] = 255;
        }
        cloud_pub_.publish(cloud);
    }


    //这里做坐标系变换。
    void
    LoopClosuerCheck::HandleLaserScan(
        const int& sensor_id, const cartographer::common::Time time,
        const std::string& frame_id,
        const cartographer::sensor::PointCloudWithIntensities& points)
        {

        if (points.points.empty()) {
            return;
        }

        //std::cout<<"Receive Laser data : "<< sensor_id<<std::endl;

        ::ros::Time requested_time = ToRos(time);
        //tfbuffer
        ::cartographer::transform::Rigid3d sensor_to_tracking(ToRigid3d(tf_->lookupTransform(
                tracking_frame_, frame_id, requested_time, ros::Duration(1.0))));
        ::cartographer::transform::Rigid3d down_to_zero({0,0,-0.3},Eigen::Quaternion<double>::Identity());

        //sensor_to_tracking = sensor_to_tracking * down_to_zero;

        //std::cout<<"tracking from base_link to"<<frame_id<<"is"<<sensor_to_tracking.DebugString();
        cartographer::sensor::TimedPointCloudData point_cloud_processed{
                time, sensor_to_tracking.translation().cast<float>(),
                cartographer::sensor::TransformTimedPointCloud(
                        points.points, sensor_to_tracking.cast<float>())};

        //到这里数据由激光自己的坐标系转换到了bask_link下。接下来做scanMatch 给到最核心的节点。
        laser_scan_point_data[sensor_id] = point_cloud_processed;
    }

//***********************callback***********************

    void
    LoopClosuerCheck::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
    {
        handleMapMessage( *msg );
        map_receive =1;
    }

    void
    LoopClosuerCheck::amclPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
    {
        if(!map_receive)
            return;
        if(!laser_receive_1)
            return;
        if(!laser_receive_2)
            return;

        std::unique_ptr<cartographer::sensor::OdometryData> odometry_data = ToOdometryData(msg);

        amcl_data = odometry_data->pose;

        transform::Rigid3f range_data_pose = amcl_data.cast<float>();



        cartographer::sensor::RangeData accumulated_range_data_;
        for(int j=0;j<2;j++) {
            for (size_t i = 0; i < laser_scan_point_data[j].ranges.size(); ++i) {


                const Eigen::Vector4f &hit = laser_scan_point_data[j].ranges[i];
                //std::cout<<"laser "<<j<<" data "<<i<<"is"<<hit;
                const Eigen::Vector3f origin_in_local =
                        range_data_pose *
                                laser_scan_point_data[j].origin;
                const Eigen::Vector3f hit_in_local = range_data_pose * hit.head<3>();
                const Eigen::Vector3f delta = hit_in_local - origin_in_local;
                const float range = delta.norm();
                if (range >= 0.05f) {
                    if (range <= 10.f) {
                        accumulated_range_data_.returns.push_back(hit_in_local);
                    } else {
                        accumulated_range_data_.misses.push_back(
                                origin_in_local +
                                1.f / range * delta);
                    }
                }
            }
        }
        const cartographer::sensor::RangeData to_baselink_range_data= sensor::CropRangeData(sensor::TransformRangeData(
                accumulated_range_data_, range_data_pose.inverse()), -0.8f, 2.f);
        //probability_grid_.limits().GetCellIndex(Eigen::Vector2f(-3.5f, 2.5f)
        const cartographer::sensor::PointCloud& filtered_point_cloud =
                cartographer::sensor::AdaptiveVoxelFilter(voxel_options)
                        .Filter(to_baselink_range_data.returns);
        PublishPointCloud(filtered_point_cloud);
        //到这里点是对的。
        //for(int i =0;i< filtered_point_cloud.size();i++)
        //{
        //    std::cout<<"laser scan is"<<filtered_point_cloud[i]<<std::endl;
        //    std::cout<<" probably is "<<probability_grid_->GetProbability(probability_grid_->limits().GetCellIndex(filtered_point_cloud[i].head<2>()));
        //    std::cout<<std::endl;
        //}


        const cartographer::transform::Rigid2d pose_prediction = transform::Project2D(amcl_data);

        cartographer::transform::Rigid2d map_origin({origin_pose_x,origin_pose_y},0);

        cartographer::transform::Rigid2d initial_ceres_pose ({pose_prediction.translation().x(),pose_prediction.translation().y()},pose_prediction.rotation());

        auto pose_observation = common::make_unique<transform::Rigid2d>();
        ceres::Solver::Summary summary;

        ceres_scan_matcher_->Match(initial_ceres_pose.translation(), initial_ceres_pose,
                                   filtered_point_cloud,
                                   *probability_grid_, pose_observation.get(),
                                  &summary);
        std::cout<<"amcl pose:"<<cartographer::transform::ToProto(initial_ceres_pose).DebugString()
                            <<std::endl<<"match_pose"<<cartographer::transform::ToProto(*pose_observation).DebugString();
        std::cout<<summary.BriefReport();
        std::cout<<std::endl;
        if(summary.final_cost>0.35f)
        {
            float score=0.f;
            std::cout<<"定位失效，开始全局检测";
            geometry_msgs::PoseStamped poseStamped;
            //此时认定定位失效，全局检测、
            //直接匹配全局地图。
            geometry_msgs::Pose pose_mm =
            FastCorrelativeScanMatcher(initial_ceres_pose,filtered_point_cloud,score);
            poseStamped.pose = pose_mm;
            poseStamped.header = msg ->header;

            if((score >0.5f)&&(score<1.1f))
                pose_pub_.publish(poseStamped);
        }

    }


    void
    LoopClosuerCheck::LaserScanReceived_1(const sensor_msgs::LaserScanConstPtr& msg)
    {
        laser_receive_1 =1;
        ::cartographer::sensor::PointCloudWithIntensities point_cloud;
        ::cartographer::common::Time time;
        std::tie(point_cloud, time) = LaserScanToPointCloudWithIntensities(*msg);
        HandleLaserScan(0, time, msg->header.frame_id, point_cloud);
    }
    void
    LoopClosuerCheck::LaserScanReceived_2(const sensor_msgs::LaserScanConstPtr& msg)
    {
        laser_receive_2=1;
        ::cartographer::sensor::PointCloudWithIntensities point_cloud;
        ::cartographer::common::Time time;
        std::tie(point_cloud, time) = LaserScanToPointCloudWithIntensities(*msg);
        HandleLaserScan(1, time, msg->header.frame_id, point_cloud);
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
