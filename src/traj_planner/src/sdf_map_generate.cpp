#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <stdlib.h>

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include "sdf_tools/collision_map.hpp"
#include "sdf_tools/sdf.hpp"

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_generate");
    ros::NodeHandle node;

    // -------------------visulize endpoints and trajectory---------------------
    ros::Publisher visualization_pub =
        node.advertise<visualization_msgs::Marker>("sdf_tools_tutorial_visualization", 1, true);

    srand(ros::Time::now().toSec());
    ros::Duration(0.5).sleep();

    //---------------------create a map using sdf_tools-----------------------------

    // sdf collision map parameter
    const double resolution = 0.1;  // 定义每个方块（栅格）大小
    const double x_size = 60.0;     // 地图的大小
    const double z_size = 5.0;
    double y_size = 60.0;
    Eigen::Translation3d origin_translation(0.0, 0.0, 0.0); // 地图的原点不做平移
    Eigen::Quaterniond origin_rotation(1.0, 0.0, 0.0, 0.0);     // 不做旋转
    const Eigen::Isometry3d origin_transform = origin_translation * origin_rotation;  // 欧式变换，看上去为三维，实际上是4*4的矩阵
    const std ::string frame = "world";   // 地图的名字

    // create map
    sdf_tools ::COLLISION_CELL cell;
    cell.occupancy = 0.0;
    cell.component = 0;
    const sdf_tools ::COLLISION_CELL oob_cell = cell;
    sdf_tools ::CollisionMapGrid collision_map(origin_transform, frame, resolution, x_size, y_size,
                                                z_size, oob_cell);

    // add some obstacle randomly
    sdf_tools::COLLISION_CELL obstacle_cell(1.0);

    int obs_num = 150;
    vector<Eigen::Vector3d> obstacles;
    cout << "----------------------Add obstacles to map!---------------" << endl;
    int fail_num = 0;
    for(int i = 0; i < obs_num;)
    {
        // randomly create a obstacle point
        Eigen::Vector3d pt; 
        pt(0) = 2.0 + 26.0 * rand() / double(RAND_MAX);
        pt(1) = 2.0 + 26.0 * rand() / double(RAND_MAX);
        pt(2) = 2.0;

        // ensure that each obstacle is far enough from others
        if(i == 0)
        {
            obstacles.push_back(pt);
            ++i;
        }
        else
        {
            double min_dist = 1000.0;
            double dist_thresh = 1.85;
            for(int j = 0; j < obstacles.size(); ++j)
            {
                double dist = (obstacles[j] - pt).norm();
                if(dist < min_dist)
                    min_dist = dist;
            }

            if(min_dist > dist_thresh)
            {
                obstacles.push_back(pt);
                ++i;
                fail_num = 0;
            }
            else
            {
                ++fail_num; 
            }
        }
        if(fail_num > 10000)
        {
            break;
        }
    }

    cout << "----------------------Obstacles generated!----------------------" << endl;

    // add the generated obstacles into collision map
    const int th = 2;
    for(float z = 0; z < 3.5; z += resolution) {
        for(int i = 0; i < obstacles.size(); ++i)
        {
            for(int m = -th; m <= th; m++)
                for(int n = -th; n <= th; n++)
                {
                    collision_map.Set(obstacles[i](0) + m * resolution, obstacles[i](1) + n * resolution, z,
                                    obstacle_cell);
                }
        }
    }
    // 构建围墙
    for(float z = 0; z < 3.5; z += resolution) {
        for(int m = 0; m < 300; m++)
            collision_map.Set(m * resolution, 0.0, z, obstacle_cell);
        for(int m = 0; m < 300; m++)
            collision_map.Set(m * resolution, 30.0, z, obstacle_cell);
        for(int m = 150; m < 299; m++)
            collision_map.Set(m * resolution, 20.0, z, obstacle_cell);
        for(int m = 1; m < 150; m++)
            collision_map.Set(m * resolution, 10.0, z, obstacle_cell);
        for(int n = 1; n < 299; n++)
            collision_map.Set(0.0, n * resolution, z, obstacle_cell);
        for(int n = 1; n < 299; n++)
            collision_map.Set(30.0, n * resolution, z, obstacle_cell);
        for(int n = 150; n < 299; n++)
            collision_map.Set(10.0, n * resolution, z, obstacle_cell);
        for(int n = 1; n < 150; n++)
            collision_map.Set(20.0, n * resolution, z, obstacle_cell);
    }
        
    // visualize the collision map
    std_msgs::ColorRGBA collision_color;
    collision_color.r = 0.3;
    collision_color.g = 0.5;
    collision_color.b = 1.0;
    collision_color.a = 0.8;

    std_msgs::ColorRGBA free_color, unknown_color;
    unknown_color.a = free_color.a = 0.0;

    visualization_msgs::Marker collision_map_marker =
        collision_map.ExportForDisplay(collision_color, free_color, unknown_color);
    collision_map_marker.ns = "collision_map";
    collision_map_marker.id = 1;

    visualization_pub.publish(collision_map_marker);  // 话题发布在sdf_tools_tutorial_visualization

    // Build the signed distance field 构建带符号的距离场
    float oob_value = INFINITY;
    std::pair<sdf_tools::SignedDistanceField, std::pair<double, double>> sdf_with_extrema =
        collision_map.ExtractSignedDistanceField(oob_value);

    sdf_tools::SignedDistanceField sdf = sdf_with_extrema.first;
    cout << "----------------------Signed distance field build!----------------------" << endl;

    //-----------------------------Wait for user to click waypoint--------------------
    cout << "----------------------Please click some way_points----------------------- " << endl;


    // std::pair<float, bool> location_sdf_query = sdf.GetSafe(obstacles[0](0) , obstacles[0](1), obstacles[0](2));
    std::pair<float, bool> location_sdf_query = sdf.GetSafe(obstacles[0](0) , obstacles[0](1), 3.5);
    // std::pair<float, bool> location_sdf_query = sdf.GetSafe(0.0, 0.0, 0.0);
    float dist = location_sdf_query.first;
    cout << "距离障碍物：" << dist << endl;
    return 0;
}
