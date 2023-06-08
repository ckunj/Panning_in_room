#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <stdlib.h>

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include "sdf_tools/collision_map.hpp"
#include "sdf_tools/sdf.hpp"

#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

double mapSizeX, mapSizeY, mapSizeZ;
double _resolution, invResolution;


Vector3d startPt;
Vector3d mapLower, mapUpper;
int maxXId, maxYId, maxZId;

// ros related
ros::Subscriber ptsSub;
ros::Publisher  _grid_path_vis_pub, _visited_nodes_vis_pub;

AstarPathFinder* astarPathFinder = new AstarPathFinder();

void rcvWaypointsCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);
void visGridPath( vector<Vector3d> nodes );
void visVisitedNode( vector<Vector3d> nodes );

// 收到目标点开始A*搜索
void rcvWaypointsCallback(const geometry_msgs::PoseStamped::ConstPtr & msg) {
    // double targetX = wp.poses[0].pose.position.x, targetY = wp.poses[0].pose.position.y, targetZ = wp.poses[0].pose.position.z;
    double targetX = msg->pose.position.x, targetY = msg->pose.position.y, targetZ = msg->pose.position.z;
    if (targetX < mapLower(0) || targetY < mapLower(1) || targetZ < mapLower(2) 
        || targetX > mapUpper(0) || targetY > mapUpper(1) || targetZ > mapUpper(2)) return;
    Vector3d targetPt;
    targetPt << targetX , targetY , targetZ;
    ROS_INFO("[node] receviece the target position!");

    //--------------采用A*搜索路径---------------------
    astarPathFinder->AstarGraphSearch(startPt, targetPt);
    // 获取路径和访问的节点
    auto gridPath = astarPathFinder->getPath();
    auto visitedNodes = astarPathFinder->getVisitedNodes();
    // 可视化路径和访问过的节点
    visGridPath (gridPath);
    visVisitedNode(visitedNodes);
    // 重置节点信息
    astarPathFinder->resetUsedGrids();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_generate");
    ros::NodeHandle node;

    // 订阅导航点
    ptsSub = node.subscribe("goal", 5, rcvWaypointsCallback);

    // -------------------visulize endpoints and trajectory---------------------
    _grid_path_vis_pub            = node.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    _visited_nodes_vis_pub        = node.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);
    ros::Publisher visualization_pub =
        node.advertise<visualization_msgs::Marker>("sdf_tools_tutorial_visualization", 1, true);


    srand(ros::Time::now().toSec());
    ros::Duration(0.5).sleep();

    // 获取仿真参数
    node.param("map/resolution", _resolution, 0.2);  // 定义每个方块（栅格）大小
    node.param("map/map_size_x", mapSizeX, 30.0);  // 地图的大小
    node.param("map/map_size_y", mapSizeY, 30.0);
    node.param("map/map_size_z", mapSizeZ, 5.0);
    node.param("planning/start_x", startPt(0), -14.0);
    node.param("planning/start_y", startPt(1), -14.0);
    node.param("planning/start_z", startPt(2), 2.0);

    //---------------------create A* search map
    // 定义地图的上下界
    mapLower << - mapSizeX/2.0, - mapSizeY/2.0,     0.0;
    mapUpper << + mapSizeX/2.0, + mapSizeY/2.0, mapSizeZ;
    // 分辨率
    invResolution = 1.0 / _resolution;
    // xyz三个方向的最大栅格坐标
    maxXId = (int)(mapSizeX * invResolution);
    maxYId = (int)(mapSizeY * invResolution);
    maxZId = (int)(mapSizeZ * invResolution);
    // 初始化地图
    astarPathFinder = new AstarPathFinder();
    astarPathFinder->initGridMap(_resolution, mapLower, mapUpper, maxXId, maxYId, maxZId);

    //---------------------create a map using sdf_tools-----------------------------

    // sdf collision map parameter
    const double resolution = 0.2;  // 定义sdf地图每个方块（栅格）大小
    Eigen::Translation3d origin_translation(-15.0, -15.0, 0.0); // 地图的原点平移
    Eigen::Quaterniond origin_rotation(1.0, 0.0, 0.0, 0.0);     // 不做旋转
    const Eigen::Isometry3d origin_transform = origin_translation * origin_rotation;  // 欧式变换，看上去为三维，实际上是4*4的矩阵
    const std ::string frame = "world";   // 地图的名字

    // create map
    sdf_tools ::COLLISION_CELL cell;
    cell.occupancy = 0.0;
    cell.component = 0;
    const sdf_tools ::COLLISION_CELL oob_cell = cell;
    sdf_tools ::CollisionMapGrid collision_map(origin_transform, frame, resolution, mapSizeX, mapSizeY,
                                                mapSizeZ, oob_cell);
    // sdf_tools ::CollisionMapGrid collision_map(origin_transform, frame, resolution, x_size, y_size,
                                                // z_size, oob_cell);

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
        pt(0) = -13.0 + 26.0 * rand() / double(RAND_MAX);
        pt(1) = -13.0 + 18.0 * rand() / double(RAND_MAX);
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
    const int th = 1;
    for(float z = 0; z <= mapSizeZ; z += resolution) {
        for(int i = 0; i < obstacles.size(); ++i)
        {
            for(int m = -th; m <= th; m++)
                for(int n = -th; n <= th; n++)
                {
                    collision_map.Set(obstacles[i](0) + m * resolution, obstacles[i](1) + n * resolution, z,
                                    obstacle_cell);
                    astarPathFinder->setObs(obstacles[i](0) + m * resolution, obstacles[i](1) + n * resolution, z);
                }
        }
    }
    // 构建围墙 地图大小30*30*5 实际围墙内范围30*30*5
    for(float z = 0; z <= mapSizeZ; z += resolution) {
        // 外围墙
        for(int m = -75; m < 75; m++) {
            collision_map.Set(m * resolution, 14.9, z, obstacle_cell); 
            astarPathFinder->setObs(m * resolution, 14.9, z);
        }
        for(int m = -75; m < 75; m++) {
            collision_map.Set(m * resolution, -15.0, z, obstacle_cell);
            astarPathFinder->setObs(m * resolution, -15.0, z);
        }
        for(int n = -75; n < 75; n++) {
            collision_map.Set(14.9, n * resolution, z, obstacle_cell);
            astarPathFinder->setObs(14.9, n * resolution, z);
        }
        for(int n = -75; n < 75; n++) {
            collision_map.Set(-15.0, n * resolution, z, obstacle_cell);
            astarPathFinder->setObs(-15.0, n * resolution, z);
        }
        //内围墙
        for(int m = 25; m < 75; m++) {
            for (double n = 0.0; n < 3 * resolution; n += resolution) {
                collision_map.Set(m * resolution, 5.0 + n, z, obstacle_cell);
                astarPathFinder->setObs(m * resolution, 5.0 + n, z);
            }
        }
        for(int m = -20; m < 60; m++) {
            for (double n = 0.0; n < 3 * resolution; n += resolution) {
                collision_map.Set(m * resolution, 9.0 + n, z, obstacle_cell);
                astarPathFinder->setObs(m * resolution, 9.0 + n, z);
            }
        }
        for(int n = 25; n < 50; n++) {
            for (double m = 0.0; m < 3 * resolution; m += resolution) {
                collision_map.Set(5 + m, n * resolution, z, obstacle_cell);
                astarPathFinder->setObs(5 + m, n * resolution, z);
            }
        }
        for(int n = 15; n < 75; n++) {
            for (double m = 0.0; m < 3 * resolution; m += resolution) {
                collision_map.Set(-7.5 + m, n * resolution, z, obstacle_cell);
                astarPathFinder->setObs(-7.5 + m, n * resolution, z);
            }
        }
        for(int n = -25; n < 25; n++) {
            for (double m = 0.0; m < 3 * resolution; m += resolution) {
                collision_map.Set(m, n * resolution, z, obstacle_cell);
                astarPathFinder->setObs(m, n * resolution, z);
            }
        }
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
    // std::pair<float, bool> location_sdf_query = sdf.GetSafe(obstacles[0](0) , obstacles[0](1), 3.5);
    // std::pair<float, bool> location_sdf_query = sdf.GetSafe(1.0, 1.0, 3.0);
    // float dist = location_sdf_query.first;
    // cout << "距离障碍物：" << dist << endl;

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
    }

    delete astarPathFinder;
    return 0;
}

void visGridPath( vector<Vector3d> nodes )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    
    node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 0.0;
    
    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}

void visVisitedNode( vector<Vector3d> nodes )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "demo_node/expanded_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _visited_nodes_vis_pub.publish(node_vis);
}
