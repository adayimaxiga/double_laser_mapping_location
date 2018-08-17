//
// Created by droid on 18-8-16.
//
#include "../include/loop_closer_check/ceres_scan_matcher_2d.h"

#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/sensor/point_cloud.h"
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

#include <cmath>
#include <iostream>
//思路 map中进行接收地图数据，激光直接接收并处理，
//amcl_pose中做扫描匹配，确定是不是进行branch and bound


namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {


    static const std::string scan_topic_1 = "scan_1";
    static const std::string scan_topic_2 = "scan_2";

    class LoopClosuerCheck{
    public:
        LoopClosuerCheck()
        {
            map_sub_= nh_.subscribe("map", 1, &LoopClosuerCheck::mapReceived,this);
            amcl_pose_sub_ = nh_.subscribe("amcl_pose", 1, &LoopClosuerCheck::amclPoseReceived,this);
            laser1_sub_ = nh_.subscribe("scan_1", 1, &LoopClosuerCheck::LaserScanReceived_1,this);
            laser2_sub_ = nh_.subscribe("scan_2", 1, &LoopClosuerCheck::LaserScanReceived_2,this);
        }
        ~LoopClosuerCheck()
        {
        }

        void handleMapMessage(const nav_msgs::OccupancyGrid& msg);

        void printMap();
        //call_back;
        void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
        void LaserScanReceived_1(const sensor_msgs::LaserScanConstPtr& laser_scan);
        void LaserScanReceived_2(const sensor_msgs::LaserScanConstPtr& laser_scan);
        void amclPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

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
    LoopClosuerCheck::LaserScanReceived_1(const sensor_msgs::LaserScanConstPtr& laser_scan)
    {

    }
    void
    LoopClosuerCheck::LaserScanReceived_2(const sensor_msgs::LaserScanConstPtr& laser_scan)
    {

    }


}
}
}
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "LoopClosuerCheck");
    cartographer::mapping::scan_matching::LoopClosuerCheck loop;

    ros::spin();


    return 0;
}
