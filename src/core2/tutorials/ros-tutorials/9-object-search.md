---
title: '9 Object search'
platform: 'CORE2'
autotoc: true
layout: layout.hbs
page: 'Tutorials'
order: 9
---

# Object search #

## Introduction ##

Object search task defines a mission in which robot has to explore environment while observing if given object exists in explored area. For this purpose it is necessary to use two different approaches, one for exploration and second for object recognition. In prevoius tutorial we already discussed object environment exploration and object recognition as separate tasks. Beside launching them together, it is necessary to keep track of which obstacles were checked by the object recognition process. Task is considered as finished when object is succesfully recognized or all abstacles were checked with no object detection.

## Object search in ROS ##

It is possible to use configurations from prevoius tutorials for area exloration and object detection. We will use:

- `explore_server`node from `frontier_exploration` package

- `move_base` node from `move_base` package

- `find_object_2d` node from `find_object_2d` package

Furthermore we will need our own node to keep track of checked obstacles.

### Requirements regarding robot ###

Before continuing with object detection task certain requirements must be met, robot should:

-   subscribe `/move_base/goal` topic with message type `geometry_msgs/PoseStamped` in which robot desired positions are included.

-   Publish map to `/map` topic with message type `nav_msgs/OccupancyGrid`.

-   Publish to `/tf` topic transformations between robot starting point relative to map, robot relative to its starting point, laser scanner relative to robot and camera realative to robot.

-   Be equipped with RGB-D camera (Orbbec Astra is used in tutorial)

### System architecture

Our search system will consist of many cooperating ROS nodes, before we start configuring them, we need to specify overall data flow and principle of operation. For performing the search task we will use two main sensors, this will be laser scanner and RGB-D camera. Laser scanner will be used for robot localization and mapping, RDB-D camera will be used for object detection. The key role of the system will be played by our own node, we will name it `search_manager`, this node will be controlling state of other tasks like exploration or path planning. Furhtermore, `search_manager` will keep track of found obstacles and which of them were checked, for this, it will need to subscribe `/map` from `gmapping`, `/objects` from `find_object_2d` and `proj_scan` containing `sensor_msgs/LaserScan` projected from depth image.

Due to the fact that all computations would be exccesive load for SBC in the robot, some of the tasks will be moved to other computer.

### Configuration of `explore_server` and `move_base` nodes ###

`Explore server` and `move_base` nodes can be used with the same configuration as in prevoius tutorials, make sure you have `costmap_common_params.yaml`, `local_costmap_params.yaml`, `global_costmap_params.yaml`, `trajectory_planner.yaml` and `exploration.yaml` file in `tutorial_pkg/config` directory.

### Configuration of `find_object_2d` node

`Find_object_2d` node will be used to detect the searched object, beside the anaysis of RGB image it can utilise depth image to measure detected object position and publish it to `/tf` topic.

You can use the same images that you scanned in tutorial 4. Searching node will be running until any object is recognized.

For the node we will define below parameters:

-   `objects_path` with value `$(find tutorial_pkg)/image_rec/`

-   `object_prefix` with value `object`

-   `gui` with value `true`

-   `subscribe_depth` with value `true`

We will also need to remap topics:

-   from `rgb/image_rect_color` to `/rgb_raw`

-   from `depth_registered/image_raw` to `/depth_raw`

-   from `depth_registered/camera_info` to `/camera/depth/camera_info`

### Configuration of `depthimage_to_laserscan` node

Node `depthimage_to_laserscan` from package `depthimage_to_laserscan` makes projection of depth images from RGB-D camera to planar `sensor_msgs/LaserScan`. These projections will be used to mark obstacles that were checked by the camera.

For the node we will define below parameters:

-   `scan_height` with value `1`

-   `range_min` with value `0.45`

-   `range_max` with value `1.5`

We will also need to remap topics:

-   from= `/image` to `/camera/depth/image`

-   from= `/scan` to `/proj_scan`

### Configuration of video streaming to external computer

We will be performing image analysis on external computer, this could be PC connected through LAN or remote server connected through husarnet. Though it is possible to stream uncompressed images to other device, it is not adviced due to the huge bandwidth usage. Much better way is to use `image_transport` package to stream compressed images. For this we will need to start few aditional nodes. Two of them will be running on robot, they will subscribe respectively raw RGB and depth image and publish compressed images. Another two nodes will be running on another machine, their task will be to decompress images and publish them for further usage. All mentioned nodes are from `image_transport` package and are of type `republish`.

First node on robot will be defined with argument:` raw in:=/camera/rgb/image_raw compressed out:=/rgb_republish` and no parameters.

Second node on robot will be defined with argument: `raw in:=/camera/depth/image_raw compressed out:=/depth_republish` and below parameters:

-   `compressed/format` with value `png`

-   `compressed/png_level` with value `1`


First node on external computer will be defined with argument:` compressed in:=/rgb_republish raw out:=/rgb_raw` and below parameter:

-   `compressed/mode` with value `color`

Second node on external computer will be defined with argument: `compressed in:=/depth_republish raw out:=/depth_raw` and below parameter:

-   `compressed/mode` with value `unchanged`


### Key methods in `search_manager` node

The `search_manager` node that we will use in this tutorial is responsible for managing exploration and trajectory planning tasks. It also takes care of marking checked obstacles.


For controlling exploration and path planning tasks, we will use `actionlib` library.

We need to include appropriate headers in `search_manager_node.h`:

```
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
```

Define function for triggering exploration task:

```
void start_frontier_exploration()
{
    exploration_in_progress = true;
    actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> exploreClient("explore_server", true);
    exploreClient.waitForServer();
    ROS_INFO("Sending goal");
    exploreClient.sendGoal(sm->createExplorationGoal());
}
```

Method `createExplorationGoal()` of class `SearchManager` creates data structure containing parameters of exploration task.


To stop the exploration task we will define function:

```
void cancel_exploration_action()
{
    actionlib_msgs::GoalID cancel_exploration;
    cancel_exploration.id = "";
    cancel_exploration.stamp = ros::Time::now();
    explore_canceller.publish(cancel_exploration);
}
```
If empty goal ID is published, this causes all exploration tasks to be cancelled.

Similarly we define function for cancelling path planning task:

```
void cancel_move_base_action()
{
    actionlib_msgs::GoalID move_base_goal;
    move_base_goal.id = "";
    move_base_goal.stamp = ros::Time::now();
    goal_pub.publish(move_base_goal);
}
```

We do not need to specify any function for initializaion of path planning, this is done by the exploration server or when destination point is published.



We also need to monitor statuses of other tasks. Define callback for path planning:

```
void status_callback(const actionlib_msgs::GoalStatusArrayConstPtr &status)
{
    int size = status->status_list.size();
    if (size > 0)
    {
        int status_value = status->status_list[size - 1].status;
        if (status_value == 1)
        {
            goal_accessible = true;
        }
        else if (status_value == 4)
        {
            goal_accessible = false;
        }
    }
    else
    {
        goal_accessible = true;
    }
}
```


Define callback for exploration:

```
void explore_status_callback(const actionlib_msgs::GoalStatusArrayConstPtr &status)
{
    int size = status->status_list.size();
    if (size > 0)
    {
        int status_value = status->status_list[size - 1].status;
        if (status_value == 1) // ACTIVE
        {
            exploration_in_progress = true;
        }
        else if (status_value == 3) // SUCCEEDED
        {
            exploration_in_progress = false;
            exploration_failed = false;
        }
        else if (status_value == 4) // ABORTED
        {
            exploration_in_progress = false;
            exploration_failed = true;
            ROS_ERROR("Exploration failed, will not search for object.");
        }
    }
}
```

Both callbacks are checking status of currently executed task and update appropriate global variable. These variables will be used in node main loop. 

Now we will proceed to `main()` function of the node. In the body of the function we begin  with node initialization:

```
    ros::init(argc, argv, "search_manager_node");
    ros::NodeHandle node("~");
```

Then we create instance of `SearchManager`, this class contains methods for processing data during object search task.

```
    sm = new SearchManager();
```

Init values for search status variables:

```
    exploration_failed = false;
    object_search_in_progress = false;
    object_found = false;
```

Init subscribers and publishers:

```
    ros::Subscriber objects_found_sub = node.subscribe("/objects", 5, objects_found_callback);
    ros::Subscriber sub = node.subscribe("/proj_scan", 1, scanCallback);
    ros::Subscriber gridMapSub = node.subscribe("/map", 1, gridMapCallback);
    ros::Subscriber move_base_status_sub = node.subscribe("/move_base/status", 1, status_callback);
    ros::Subscriber exploration_status_sub = node.subscribe("/explore_server/status", 1, explore_status_callback);

    publisher_obstacles_found = node.advertise<nav_msgs::OccupancyGrid>("/obstacles/found", 1);
    publisher_checked_obstacles = node.advertise<nav_msgs::OccupancyGrid>("/obstacles/checked", 1);
    publisher_pending_obstacles = node.advertise<nav_msgs::OccupancyGrid>("/obstacles/pending", 1);
    publisher_exploration_goal = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    goal_pub = node.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    explore_canceller = node.advertise<actionlib_msgs::GoalID>("/explore_server/cancel", 1);
    listener = new tf::TransformListener();

```

For keeping track of which obstacles were checked or still need to be inspected we will use `grid_map` [library](http://wiki.ros.org/grid_map).

To use this library, it is necessary to include some headers in `search_manager_node.h`:

```
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
```

Initialize `map` and `obstacles` instances of `GridMap` class to store map layers:

```
    map = new grid_map::GridMap({"input_og"});

    map->setFrameId("map");
    map->setGeometry(grid_map::Length(2.0, 2.0), 0.01);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
             map->getLength().x(), map->getLength().y(),
             map->getSize()(0), map->getSize()(1));

    std::vector<std::string> map_layers;
    map_layers.push_back("rgbd_scan");
    map_layers.push_back("obstacles_found");
    map_layers.push_back("pending_obstacles");
    obstacles = new grid_map::GridMap(map_layers);
    obstacles->setFrameId("map");
    obstacles->setGeometry(map->getLength(), map->getResolution());
    obstacles->setPosition(map->getPosition());
```

Set the main loop frequency:

```
    ros::Rate rate(50.0);
```

Trigger the explortion task:

```
    start_frontier_exploration();
```

Wait until `explore_server` finishes its work or `find_object_2d` detects object:

```
    while (exploration_in_progress && node.ok() && !object_found)
    {
        ros::spinOnce();
        update_robot_pos();
        rate.sleep();
    }
```

Status of exploration task and object detection are checked in callbacks.

Check if the object was already found, if so, the mission is finished, we can stop all other task and shutdown the node:

```
    if (object_found)
    {
        ROS_INFO("Object detected during exploration, stop exploration task");
        cancel_exploration_action();
        cancel_move_base_action();
        node.shutdown();
        return 0;
    }
    else
    {
        ROS_INFO("Detected exploration finsh");
    }
```

We need to make sure, that `explore_server` finished with success. If there were any errors, flag `exploration_failed` will be set to true. In such case we stop operation.

```
    if (exploration_failed)
    {
        ROS_ERROR("Shutting down node due to exploration error");
        node.shutdown();
        return 0;
    }
```

At this moment we should have complete map of environment and marked all the obstacle which should be checked. Now we begin to set robot goals to inspect all pending obstacles:

```
    ROS_INFO("Begin searching for object");
    object_search_in_progress = true;
    set_new_goal();

    while (node.ok() && object_search_in_progress && !object_found)
    {
        update_robot_pos();
        goal_reached = sm->is_goal_reached(exploration_goal, ROSbot2_base_to_map_transform, 0.3, 0.5);
        if (goal_reached)
        {
            ROS_INFO("Goal is reached, can set new destination");
            set_new_goal();
        }
        else if (goal_accessible)
        {
            // wait until goal is reached
        }
        else
        {
            set_new_goal();
        }

        ros::spinOnce();
        rate.sleep();
    }
```

When the above loop is stopped, the object was found or all obstacles were checked with no detection, report to the user and shutdown the node:

```
    if (node.ok())
    {
        if (object_found)
        {
            ROS_INFO("Object search succeeded");
        }
        else
        {
            ROS_INFO("Object search stopped, all area checked, nothing found");
        }
        cancel_move_base_action();
        node.shutdown();
    }
    else
    {
        ROS_WARN("Object search cancelled with external signal");
    }
    return 0;
}
```

Above functions are only the most important for node operation. Node is using some more functions that should be self explanatory.

### `search_manager` node complete code

Create file `search_manager_node.cpp` inside the `src` folder under `tutorial_pkg` and paste below code:

```
#include <search_manager_node.h>

/**
 * brief Computes the bearing in degrees from the point A(a1,a2) to the point B(b1,b2).
 * param a1 x coordiante of point A
 * param a2 y coordinate of point A
 * param b1 x coordiante of point B
 * param b2 y coordinate of point B
 */
double bearing(double a1, double a2, double b1, double b2)
{
    static const double TWOPI = 6.2831853071795865;
    static const double RAD2DEG = 57.2957795130823209;
    double theta = atan2(b1 - a1, b2 - a2);
    if (theta < 0.0)
        theta += TWOPI;
    return theta;
}

bool is_area_free(grid_map::Position point, float radius)
{
    bool area_free = true;
    for (grid_map::CircleIterator it(*obstacles, point, radius); !it.isPastEnd(); ++it)
    {
        if (obstacles->at("obstacles_found", *it) == 1)
        {
            area_free = false;
        }
    }
    return area_free;
}

void clear_area(grid_map::Position point, float radius)
{
    for (grid_map::CircleIterator it(*obstacles, point, radius); !it.isPastEnd(); ++it)
    {
        obstacles->at("pending_obstacles", *it) = 0;
    }
}

bool is_point_single(grid_map::Position point)
{
    float res = obstacles->getResolution();
    bool single = true;
    grid_map::CircleIterator point_it(*obstacles, point, 0.01);
    grid_map::Position point_it_position;
    obstacles->getPosition(*point_it, point_it_position);

    grid_map::Position circle_it_position;

    for (grid_map::CircleIterator it(*obstacles, point, 10 * res); !it.isPastEnd(); ++it)
    {
        obstacles->getPosition(*it, circle_it_position);
        if (obstacles->at("pending_obstacles", *it) == 1)
        {
            float x_dist = point_it_position.x() - circle_it_position.x();
            float y_dist = point_it_position.y() - circle_it_position.y();
            float x_abs = std::abs(x_dist);
            float y_abs = std::abs(y_dist);
            if (x_abs > res && y_abs > res)
            {
                single = false;
            }
        }
    }
    return single;
}

bool find_nearest_obstacle(grid_map::Position *new_pos)
{
    grid_map::Position current_robot_position(robot_position[0], robot_position[1]);
    return find_nearest_obstacle(new_pos, current_robot_position);
}

bool find_nearest_obstacle(grid_map::Position *new_pos, grid_map::Position current_pos)
{
    grid_map::Position new_obstacle;
    bool new_iteration = true;
    bool obstacle_found = false;
    double distance_to_nearest_obstacle;
    double distance_to_current_obstacle;
    double x_dist, y_dist;
    grid_map::Position element_position;

    for (grid_map::GridMapIterator it(*obstacles); !it.isPastEnd(); ++it)
    {
        if (obstacles->at("pending_obstacles", *it) > 0.5)
        {
            obstacle_found = true;
            obstacles->getPosition(*it, element_position);
            x_dist = current_pos[0] - element_position[0];
            y_dist = current_pos[1] - element_position[1];
            distance_to_current_obstacle = sqrt((x_dist * x_dist) + (y_dist * y_dist));
            if (new_iteration)
            {
                distance_to_nearest_obstacle = distance_to_current_obstacle;
                new_obstacle = element_position;
                new_iteration = false;
            }
            else if (distance_to_current_obstacle < distance_to_nearest_obstacle)
            {
                distance_to_nearest_obstacle = distance_to_current_obstacle;
                new_obstacle = element_position;
            }
        }
    }
    *new_pos = new_obstacle;
    return obstacle_found;
}

bool check_pending_obstacles()
{
    for (grid_map::GridMapIterator it(*obstacles); !it.isPastEnd(); ++it)
    {
        if (obstacles->at("pending_obstacles", *it) > 0.5)
        {
            return true;
        }
    }
    ROS_INFO("No pending obstacles found");
    return false;
}

bool was_obstacle_checked(grid_map::Position obstacle_position)
{
    float radius = 0.1;
    bool obstacle_already_checked = false;
    for (grid_map::CircleIterator it(*obstacles, obstacle_position, radius); !it.isPastEnd(); ++it)
    {
        if (obstacles->at("pending_obstacles", *it) > 0.1)
        {
            obstacle_already_checked = true;
        }
    }
    return obstacle_already_checked;
}

void publish_exploration_goal(grid_map::Position goal_pos, float goal_angle)
{
    dest_orientation.setRPY(0, 0, goal_angle);
    dest_orientation.getRotation(tf_dest_quaternion);
    exploration_goal.header.frame_id = "map";
    exploration_goal.pose.position.x = goal_pos[0];
    exploration_goal.pose.position.y = goal_pos[1];
    exploration_goal.pose.position.z = 0;
    exploration_goal.pose.orientation.x = tf_dest_quaternion.x();
    exploration_goal.pose.orientation.y = tf_dest_quaternion.y();
    exploration_goal.pose.orientation.z = tf_dest_quaternion.z();
    exploration_goal.pose.orientation.w = tf_dest_quaternion.w();
    publisher_exploration_goal.publish(exploration_goal);
}

void status_callback(const actionlib_msgs::GoalStatusArrayConstPtr &status)
{
    int size = status->status_list.size();
    if (size > 0)
    {
        int status_value = status->status_list[size - 1].status;
        if (status_value == 1)
        {
            goal_accessible = true;
        }
        else if (status_value == 4)
        {
            goal_accessible = false;
        }
    }
    else
    {
        goal_accessible = true;
    }
}

void explore_status_callback(const actionlib_msgs::GoalStatusArrayConstPtr &status)
{
    int size = status->status_list.size();
    if (size > 0)
    {
        int status_value = status->status_list[size - 1].status;
        if (status_value == 1) // ACTIVE
        {
            exploration_in_progress = true;
        }
        else if (status_value == 3) // SUCCEEDED
        {
            exploration_in_progress = false;
            exploration_failed = false;
        }
        else if (status_value == 4) // ABORTED
        {
            exploration_in_progress = false;
            exploration_failed = true;
            ROS_ERROR("Exploration failed, will not search for object.");
        }
    }
}

void scanCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
    angle_min = msg->angle_min;
    angle_max = msg->angle_max;
    angle_increment = msg->angle_increment;
    range_min = msg->range_min;
    range_max = msg->range_max;
    ranges_size = msg->ranges.size();
    uint32_t s = msg->header.stamp.sec;
    uint32_t ns = msg->header.stamp.nsec;
    ROS_INFO("Scan time: %d,%d", s, ns);

    double camera_x;
    double camera_y;
    double camera_yaw;

    if (sm->lookup_camera_transform(&camera_x, &camera_y, &camera_yaw, ros::Time(s, ns), listener))
    {
        for (int i = 0; i < ranges_size; i++)
        {
            if (msg->ranges[i] > (camera_view_dist - camera_view_depth) && msg->ranges[i] < (camera_view_dist + camera_view_depth))
            {
                point_x = msg->ranges[i] * cos((angle_min + i * angle_increment) + camera_yaw);
                point_y = msg->ranges[i] * sin((angle_min + i * angle_increment) + camera_yaw);
                if (sm->set_point_checked(point_x + camera_x, point_y + camera_y, obstacles, nearest_obstacle))
                {
                    set_new_goal();
                }
            }
        }
        grid_map::GridMapRosConverter::toOccupancyGrid(*obstacles, "rgbd_scan", 0.0, 1.0, occupancyGridResult);
        publisher_checked_obstacles.publish(occupancyGridResult);
    }
    else
    {
        ROS_WARN("Transform lookup failed, drop this scan");
    }
}

void gridMapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    occupancyGridInput = new nav_msgs::OccupancyGrid();
    occupancyGridInput->header = msg->header;
    occupancyGridInput->info = msg->info;
    occupancyGridInput->data = msg->data;

    map = new grid_map::GridMap({"input_og"});
    grid_map::GridMapRosConverter::fromOccupancyGrid(*occupancyGridInput, "input_og", *map);
    map->setFrameId("map");
    obstacles->addDataFrom(*map, true, true, true);

    for (grid_map::GridMapIterator it(*obstacles); !it.isPastEnd(); ++it)
    {
        if (obstacles->at("input_og", *it) > 0)
        {
            obstacles->at("obstacles_found", *it) = 1;
            if (obstacles->at("rgbd_scan", *it) > 0)
            {
                obstacles->at("pending_obstacles", *it) = 0;
            }
            else
            {
                obstacles->at("pending_obstacles", *it) = 1;
            }
        }
        else
        {
            obstacles->at("obstacles_found", *it) = 0;
            obstacles->at("pending_obstacles", *it) = 0;
        }
    }

    grid_map::GridMapRosConverter::toOccupancyGrid(*obstacles, "obstacles_found", 0.0, 1.0, occupancyGridResult);
    publisher_obstacles_found.publish(occupancyGridResult);
    grid_map::GridMapRosConverter::toOccupancyGrid(*obstacles, "pending_obstacles", 0.0, 1.0, occupancyGridResult);
    publisher_pending_obstacles.publish(occupancyGridResult);
    if (is_area_free(nearest_obstacle, 0.1))
    {
        set_new_goal();
    }
    if (!sm->check_obstacle_surrounding(&current_robot_position, &obstacle_bearing, camera_view_dist, min_dist, current_obstacle, obstacles, map))
    {
        set_new_goal();
    }
}

void objects_found_callback(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    if (msg->data.size() > 0)
    {
        ROS_INFO("Object detected");
        object_found = true;
    }
}

void update_robot_pos()
{
    try
    {
        listener->lookupTransform("/map", "/base_link", ros::Time(0), ROSbot2_base_to_map_transform);
        robot_position[0] = ROSbot2_base_to_map_transform.getOrigin().x();
        robot_position[1] = ROSbot2_base_to_map_transform.getOrigin().y();
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

void cancel_move_base_action()
{
    actionlib_msgs::GoalID move_base_goal;
    move_base_goal.id = "";
    move_base_goal.stamp = ros::Time::now();
    goal_pub.publish(move_base_goal);
}

void cancel_exploration_action()
{
    actionlib_msgs::GoalID cancel_exploration;
    cancel_exploration.id = "";
    cancel_exploration.stamp = ros::Time::now();
    explore_canceller.publish(cancel_exploration);
}

void start_frontier_exploration()
{
    exploration_in_progress = true;
    actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> exploreClient("explore_server", true);
    exploreClient.waitForServer();
    ROS_INFO("Sending goal");
    exploreClient.sendGoal(sm->createExplorationGoal());
}

void set_new_goal()
{
    if (object_search_in_progress)
    {
        ROS_INFO("Set new point for object recognition.");
        if (find_nearest_obstacle(&nearest_obstacle, nearest_obstacle))
        {
            if (is_point_single(nearest_obstacle))
            {
                ROS_INFO("Point %f, %f has no surrounding obstacles, consider it as noise.", nearest_obstacle[0], nearest_obstacle[1]);
                clear_area(nearest_obstacle, 0.1);
                find_nearest_obstacle(&nearest_obstacle, nearest_obstacle);
            }
            ROS_INFO("New dest point %f, %f.", nearest_obstacle[0], nearest_obstacle[1]);

            robot_destination = get_optimal_pose(nearest_obstacle);
            double angle = bearing(robot_destination[1], robot_destination[0], nearest_obstacle[1], nearest_obstacle[0]);
            publish_exploration_goal(robot_destination, angle);
        }
        else
        {
            if (!check_pending_obstacles())
            {
                ROS_INFO("Object search finished, object not found");
                object_search_in_progress = false;
            }
        }
    }
}

grid_map::Position get_optimal_pose(grid_map::Position obstacle)
{
    obstacle_bearing = 180 * bearing(obstacle[0], obstacle[1], robot_position[0], robot_position[1]) / M_PI;
    ROS_INFO("Obstacle bearing: %f", obstacle_bearing);
    current_obstacle = obstacle;
    if (!sm->check_obstacle_surrounding(&current_robot_position, &obstacle_bearing, camera_view_dist, min_dist, current_obstacle, obstacles, map))
    {
        set_new_goal();
    }
    return current_robot_position;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "search_manager");
    ros::NodeHandle node("~");

    sm = new SearchManager();

    exploration_failed = false;
    object_search_in_progress = false;
    object_found = false;

    ros::Subscriber objects_found_sub = node.subscribe("/objects", 5, objects_found_callback);
    ros::Subscriber sub = node.subscribe("/proj_scan", 1, scanCallback);
    ros::Subscriber gridMapSub = node.subscribe("/map", 1, gridMapCallback);
    ros::Subscriber move_base_status_sub = node.subscribe("/move_base/status", 1, status_callback);
    ros::Subscriber exploration_status_sub = node.subscribe("/explore_server/status", 1, explore_status_callback);

    publisher_obstacles_found = node.advertise<nav_msgs::OccupancyGrid>("/obstacles/found", 1);
    publisher_checked_obstacles = node.advertise<nav_msgs::OccupancyGrid>("/obstacles/checked", 1);
    publisher_pending_obstacles = node.advertise<nav_msgs::OccupancyGrid>("/obstacles/pending", 1);
    publisher_exploration_goal = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    goal_pub = node.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    explore_canceller = node.advertise<actionlib_msgs::GoalID>("/explore_server/cancel", 1);
    listener = new tf::TransformListener();

    map = new grid_map::GridMap({"input_og"});

    map->setFrameId("map");
    map->setGeometry(grid_map::Length(2.0, 2.0), 0.01);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
             map->getLength().x(), map->getLength().y(),
             map->getSize()(0), map->getSize()(1));

    std::vector<std::string> map_layers;
    map_layers.push_back("rgbd_scan");
    map_layers.push_back("obstacles_found");
    map_layers.push_back("pending_obstacles");
    obstacles = new grid_map::GridMap(map_layers);
    obstacles->setFrameId("map");
    obstacles->setGeometry(map->getLength(), map->getResolution());
    obstacles->setPosition(map->getPosition());

    ros::Rate rate(50.0);

    start_frontier_exploration();

    while (exploration_in_progress && node.ok() && !object_found)
    {
        ros::spinOnce();
        update_robot_pos();
        rate.sleep();
    }

    if (object_found)
    {
        ROS_INFO("Object detected during exploration, stop exploration task");
        cancel_exploration_action();
        cancel_move_base_action();
        node.shutdown();
        return 0;
    }
    else
    {
        ROS_INFO("Detected exploration finsh");
    }

    if (exploration_failed)
    {
        ROS_ERROR("Shutting down node due to exploration error");
        node.shutdown();
        return 0;
    }

    ROS_INFO("Begin searching for object");
    object_search_in_progress = true;
    set_new_goal();

    while (node.ok() && object_search_in_progress && !object_found)
    {
        update_robot_pos();
        goal_reached = sm->is_goal_reached(exploration_goal, ROSbot2_base_to_map_transform, 0.3, 0.5);
        if (goal_reached)
        {
            ROS_INFO("Goal is reached, can set new destination");
            set_new_goal();
        }
        else if (goal_accessible)
        {
            // wait until goal is reached
        }
        else
        {
            set_new_goal();
        }

        ros::spinOnce();
        rate.sleep();
    }
    if (node.ok())
    {
        if (object_found)
        {
            ROS_INFO("Object search succeeded");
        }
        else
        {
            ROS_INFO("Object search stopped, all area checked, nothing found");
        }
        cancel_move_base_action();
        node.shutdown();
    }
    else
    {
        ROS_WARN("Object search cancelled with external signal");
    }
    return 0;
}
```

Then file `SearchManager.cpp` inside the `src` folder under `tutorial_pkg` and paste below code:

```
#include <SearchManager.h>

SearchManager::SearchManager()
{
}

bool SearchManager::lookup_camera_transform(double *cam_x, double *cam_y, double *cam_yaw, ros::Time scan_time, tf::TransformListener *listener)
{
    try
    {
        tf::StampedTransform rgbd_scan_to_map_transform;
        listener->lookupTransform("/map", "/camera_link", scan_time, rgbd_scan_to_map_transform);
        std::cout << "LookupTime: " << scan_time.sec << ", " << scan_time.nsec << std::endl;
        *cam_x = rgbd_scan_to_map_transform.getOrigin().x();
        *cam_y = rgbd_scan_to_map_transform.getOrigin().y();
        tf::Quaternion quat = rgbd_scan_to_map_transform.getRotation();
        tf::Matrix3x3 rotation_matrix;
        double roll;
        double pitch;
        double yaw;
        rotation_matrix.setRotation(quat);
        rotation_matrix.getRPY(roll, pitch, yaw);
        *cam_yaw = yaw;
        return true;
    }
    catch (tf::TransformException ex)
    {
        return false;
    }
}

bool SearchManager::check_obstacle_surrounding(grid_map::Position *robot_dest, double *obstacle_bearing, float dist_from_obstacle, float min_dist, grid_map::Position current_obstacle, grid_map::GridMap *obstacles, grid_map::GridMap *map)
{
    double current_bearing;
    for (int deg = 0; deg < 36; deg++)
    {
        current_bearing = *obstacle_bearing + (deg * 10);

        if (check_space_occupation(robot_dest, current_bearing, dist_from_obstacle, min_dist, current_obstacle, obstacles, map))
        {
            *obstacle_bearing = current_bearing;
            return true;
        }
    }
    return false;
}

bool SearchManager::check_space_occupation(grid_map::Position *robot_dest, double bearing, float dist_from_obstacle, float min_dist, grid_map::Position current_obstacle, grid_map::GridMap *obstacles, grid_map::GridMap *map)
{
    bool polygon_free = true;
    grid_map::Position current_robot_destination;
    grid_map::Position middle_corner;
    grid_map::Position left_corner;
    grid_map::Position right_corner;
    std::vector<grid_map::Position> vertices;
    grid_map::Polygon polygon;

    // calculate possible position
    current_robot_destination[0] = current_obstacle[0] + dist_from_obstacle * (sin((bearing)*M_PI / 180));
    current_robot_destination[1] = current_obstacle[1] + dist_from_obstacle * (cos((bearing)*M_PI / 180));
    middle_corner[0] = current_obstacle[0] + min_dist * (sin((bearing)*M_PI / 180));
    middle_corner[1] = current_obstacle[1] + min_dist * (cos((bearing)*M_PI / 180));
    left_corner[0] = middle_corner[0] + min_dist * (sin((bearing + 90) * M_PI / 180));
    left_corner[1] = middle_corner[1] + min_dist * (cos((bearing + 90) * M_PI / 180));
    right_corner[0] = middle_corner[0] + min_dist * (sin((bearing - 90) * M_PI / 180));
    right_corner[1] = middle_corner[1] + min_dist * (cos((bearing - 90) * M_PI / 180));
    vertices.push_back(current_robot_destination);
    vertices.push_back(middle_corner);
    vertices.push_back(left_corner);
    vertices.push_back(right_corner);
    polygon = grid_map::Polygon(vertices);

    int checked_points = 0;
    for (grid_map::PolygonIterator it(*obstacles, polygon); !it.isPastEnd(); ++it)
    {
        checked_points++;
        if (obstacles->at("obstacles_found", *it) > 0.5)
        {
            polygon_free = false;
        }
    }

    for (grid_map::CircleIterator it(*map, current_robot_destination, 0.1); !it.isPastEnd(); ++it)
    {
        float map_val = map->at("input_og", *it);
        if (map_val >= 0)
        {
        }
        else
        {
            polygon_free = false;
            grid_map::Position known_pos;
            map->getPosition(*it, known_pos);
        }
    }

    if (polygon_free)
    {
        *robot_dest = current_robot_destination;
        return true;
    }
    else
    {
        return false;
    }
}

bool SearchManager::set_point_checked(float x, float y, grid_map::GridMap *gm, grid_map::Position obstacle)
{
    float radius = 3 * gm->getResolution();
    return set_point_checked(x, y, gm, obstacle, radius);
}

bool SearchManager::set_point_checked(float x, float y, grid_map::GridMap *gm, grid_map::Position obstacle, float radius)
{
    for (grid_map::CircleIterator it(*gm, grid_map::Position(x, y), radius); !it.isPastEnd(); ++it)
    {
        gm->at("rgbd_scan", *it) = 1;
        gm->at("pending_obstacles", *it) = 0;
    }
    float x_dist = obstacle.x() - x;
    float y_dist = obstacle.y() - y;
    float dist = sqrt(x_dist * x_dist + y_dist * y_dist);
    if (dist < radius)
    {
        // set_new_goal();
        return true;
    }
    return false;
}

frontier_exploration::ExploreTaskGoal SearchManager::createExplorationGoal()
{
    frontier_exploration::ExploreTaskGoal goal;
    geometry_msgs::PointStamped center;
    center.header.frame_id = "map";
    center.point.x = 1.0;
    center.point.y = 0;
    center.point.z = 0;
    goal.explore_center = center;
    geometry_msgs::PolygonStamped square;
    square.header.frame_id = "map";
    std::vector<geometry_msgs::Point32> square_points;
    geometry_msgs::Point32 point_a;
    point_a.x = 30;
    point_a.y = 30;
    point_a.z = 0;
    geometry_msgs::Point32 point_b;
    point_b.x = -30;
    point_b.y = 30;
    point_b.z = 0;
    geometry_msgs::Point32 point_c;
    point_c.x = -30;
    point_c.y = -30;
    point_c.z = 0;
    geometry_msgs::Point32 point_d;
    point_d.x = 30;
    point_d.y = -30;
    point_d.z = 0;
    square_points.push_back(point_a);
    square_points.push_back(point_b);
    square_points.push_back(point_c);
    square_points.push_back(point_d);
    square.polygon.points = square_points;
    goal.explore_boundary = square;
    return goal;
}

bool SearchManager::is_goal_reached(geometry_msgs::PoseStamped goal, tf::StampedTransform current_tf, double linear_threshold, double angular_threshold)
{
    tf::Matrix3x3 goal_m, current_m;
    tf::Quaternion goal_q;
    double current_roll, current_pitch, current_yaw;
    double goal_roll, goal_pitch, goal_yaw;

    goal_q.setX(goal.pose.orientation.x);
    goal_q.setY(goal.pose.orientation.y);
    goal_q.setZ(goal.pose.orientation.z);
    goal_q.setW(goal.pose.orientation.w);
    current_m.setRotation(current_tf.getRotation());
    current_m.getRPY(current_roll, current_pitch, current_yaw);
    goal_m.setRotation(goal_q);
    goal_m.getRPY(goal_roll, goal_pitch, goal_yaw);

    double x_distance = current_tf.getOrigin().x() - goal.pose.position.x;
    double y_distance = current_tf.getOrigin().y() - goal.pose.position.y;
    double yaw_distance = current_yaw - goal_yaw;

    if (x_distance < linear_threshold && x_distance > -linear_threshold)
    {
        if (y_distance < linear_threshold && y_distance > -linear_threshold)
        {
            if (yaw_distance < angular_threshold && yaw_distance > -linear_threshold)
            {
                return true;
            }
        }
    }
    return false;
}
```

Now we will create required header files. In `tutorial_pkg` create `include` directory. Inside the `include` directory create `search_manager_node.h` and paste:

```
#include <math.h>
#include <cmath>

#include <ros/ros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <frontier_exploration/ExploreTaskAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <SearchManager.h>

SearchManager *sm;
grid_map::GridMap *obstacles;
grid_map::GridMap *map;
std::vector<std::string> layers_to_import;
nav_msgs::OccupancyGrid *occupancyGridInput;
nav_msgs::OccupancyGrid occupancyGridResult;
nav_msgs::OccupancyGrid occupancyGridScanned;
nav_msgs::OccupancyGrid occupancyGridObstaclesFound;
nav_msgs::OccupancyGrid occupancyGridPendingObstacles;
geometry_msgs::PoseStamped exploration_goal;
frontier_exploration::ExploreTaskAction exploreTaskAction;

ros::Publisher publisher_obstacles_found;
ros::Publisher publisher_pending_obstacles;
ros::Publisher publisher_checked_obstacles;
ros::Publisher publisher_exploration_goal;
ros::Publisher vis_pub;
ros::Publisher goal_pub;
ros::Publisher explore_canceller;

double angle_min;
double angle_max;       //        # end angle of the scan [rad]
double angle_increment; //  # angular distance between measurements [rad]
double range_min;       //        # minimum range value [m]
double range_max;       //        # maximum range value [m]
std::uint16_t ranges_size;
double point_x;
double point_y;
tf::Quaternion tf_q, tf_dest_quaternion;
tf::TransformListener *listener;
tf::StampedTransform ROSbot2_base_to_map_transform;
geometry_msgs::Pose circle_element;
grid_map::Position robot_position;
grid_map::Position nearest_obstacle;
grid_map::Position robot_destination;
grid_map::Position current_obstacle;
tf::Matrix3x3 m, dest_orientation;
double x_dest_pos, y_dest_pos;
bool goal_accessible;
bool goal_reached;
bool destination_free_to_go;
bool exploration_in_progress;
bool exploration_failed;
bool object_search_in_progress;
bool object_found;
double camera_view_dist = 0.7;  // [meters]
double camera_view_depth = 0.2; // [meters]
std::vector<geometry_msgs::PoseStamped> poses(36);

double obstacle_bearing;
float min_dist = 0.15;
grid_map::Position current_robot_position;

void set_new_goal();
bool find_nearest_obstacle(grid_map::Position *new_pos);
bool find_nearest_obstacle(grid_map::Position *new_pos, grid_map::Position current_pos);
grid_map::Position get_optimal_pose(grid_map::Position obstacle);

void cancel_move_base_action();
void cancel_exploration_action();
```

Then create `SearchManager.h` and paste:

```
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <grid_map_core/grid_map_core.hpp>
#include <frontier_exploration/ExploreTaskAction.h>

class SearchManager
{
  public:
    SearchManager();
    bool check_space_occupation(
        grid_map::Position *robot_dest,
        double bearing,
        float dist_from_obstacle,
        float min_dist,
        grid_map::Position current_obstacle,
        grid_map::GridMap *obstacles,
        grid_map::GridMap *map);

    bool check_obstacle_surrounding(
        grid_map::Position *robot_dest,
        double *obstacle_bearing,
        float dist_from_obstacle,
        float min_dist,
        grid_map::Position current_obstacle,
        grid_map::GridMap *obstacles,
        grid_map::GridMap *map);

    bool lookup_camera_transform(
        double *cam_x,
        double *cam_y,
        double *cam_yaw,
        ros::Time scan_time,
        tf::TransformListener *listener);

    /**
     * brief Mark given point as checked
     * param x X position of point to be marked
     * param y Y position of point to be marked
     * param gm GridMap at which point is to be marked
     * param obstacle Obstacle which is currently set as destination
     * return true if obstacle was inside the checked area
     */
    bool set_point_checked(float x, float y, grid_map::GridMap *gm, grid_map::Position obstacle);

    /**
     * brief Mark given point as checked
     * param x X position of point to be marked
     * param y Y position of point to be marked
     * param gm GridMap at which point is to be marked
     * param obstacle Obstacle which is currently set as destination
     * param radius Radius of the circle to be marked
     * return true if obstacle was inside the checked area
     */
    bool set_point_checked(float x, float y, grid_map::GridMap *gm, grid_map::Position obstacle, float radius);

    frontier_exploration::ExploreTaskGoal createExplorationGoal();

    bool is_goal_reached(geometry_msgs::PoseStamped goal, tf::StampedTransform current_tf, double linear_threshold, double angular_threshold);
};
```

### Building the node

Open `CmakeLists.txt` file and find:

```
find_package(catkin REQUIRED COMPONENTS
  roscpp tf
)
```

And add below components to the list:

```
  grid_map_core
  grid_map_msgs
  grid_map_ros
  nav_msgs
  roscpp
  sensor_msgs
  frontier_exploration
```

Find section `include_directories` and, at the front of the list add line:

```
include
```

After edit it should look like below:

```
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
```

At the end of `add_executable()` directives list add line:

```
add_executable(search_manager src/search_manager_node.cpp src/SearchManager.cpp)
```

At the end of `target_link_libraries()` directives list add:

```
target_link_libraries(search_manager
  ${catkin_LIBRARIES}
)
```

Now you can build your workspace with `catkin_make`.

### Launching search task ###


To remind, you will need to run following nodes:

-   `CORE2` bridge node -
    `/opt/husarion/tools/rpi-linux/ros-core2-client /dev/ttyCORE2 `

-   `rplidarNode` - driver for rpLidar laser scanner

-   `astra_camera` - driver for Orbbec Astra RGB-D camera

-   `drive_controller_node` - `tf` publisher for transformation of robot
    relative to starting point

-    `republish` nodes for managing image streaming

Or instead ot these nodes, `Gazebo`:

-   `roslaunch rosbot_gazebo maze_world.launch`

And: 

-   `static_transform_publisher` - `tf` publisher for transformation of
    laser scanner relative to robot and camera relative to robot

-   `slam_gmapping` - map building node

-   `move_base` - trajectory planner

-   `explore_server` - exploration task

-   `rviz` - visualization tool

-   `find_object_2d` - image recognition tool

-   `search_manager` - executive node for search task that you just build

It is necessary to distnguish launching search task on ROSbot and in Gazebo. Essential difference is that some processes will be moved from ROSbot to separate device to improve performance.
When using Gazebo all processes can be executed on single device, we assume that worksatation is capable enough to run all of them. If you want, there are no restrictions to use Gazebo with similar setup as for ROSbot.


#### Gazebo version

For Gazebo you can use below `launch` file:

``` launch
<launch>

    <param name="use_sim_time" value="true"/>
    <arg name="world" default="empty"/>
    <arg name="paused" default="false"/>
    <arg name="use_sim_time" default="true"/>
    <arg name="gui" default="true"/>
    <arg name="headless" default="false"/>
    <arg name="debug" default="false"/>

    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find rosbot_gazebo)/worlds/search.world"/>
        <arg name="paused" value="$(arg paused)"/>
        <arg name="use_sim_time" value="$(arg use_sim_time)"/>
        <arg name="gui" value="$(arg gui)"/>
        <arg name="headless" value="$(arg headless)"/>
        <arg name="debug" value="$(arg debug)"/>
    </include>

    <param name="robot_description" command="$(find xacro)/xacro.py '$(find rosbot_description)/urdf/rosbot.xacro'"/>

    <node name="rosbot_spawn" pkg="gazebo_ros" type="spawn_model" output="screen" args="-urdf -param robot_description -model rosbot" />

    <node pkg="tf" type="static_transform_publisher" name="laser_broadcaster" args="0.019 0 0 3.14 0 0 base_link laser 100" />

    <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"/>

    <node pkg="depthimage_to_laserscan" type="depthimage_to_laserscan" name="depthimage_to_laserscan" required="true">
        <remap from="/image" to="/camera/depth/image_raw"/>
        <remap from="scan" to="proj_scan"/>
        <param name="scan_height" value="1"/>
        <param name="range_min" value="0.45"/>
        <param name="range_max" value="1.5"/>
    </node>

    <node pkg="gmapping" type="slam_gmapping" name='gmapping_node' output='log'>
        <param name="base_frame" value="base_link"/>
        <param name="odom_frame" value="odom"/>
        <param name="delta" value="0.01"/>
        <param name="xmin" value="-5"/>
        <param name="ymin" value="-5"/>
        <param name="xmax" value="5"/>
        <param name="ymax" value="5"/>
        <param name="maxUrange" value="8"/>
        <param name="maxRange" value="8"/>
        <param name="map_update_interval" value="1"/>
        <param name="linearUpdate" value="0.05"/>
        <param name="angularUpdate" value="0.05"/>
        <param name="temporalUpdate" value="0.1"/>
        <param name="particles" value="100"/>
    </node>

    <node pkg="move_base" type="move_base" name="move_base" output="screen">
        <param name="controller_frequency" value="5.0"/>
        <rosparam file="$(find rosbot_navigation)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find rosbot_navigation)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find rosbot_navigation)/config/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find rosbot_navigation)/config/global_costmap_params.yaml" command="load" />
        <rosparam file="$(find rosbot_navigation)/config/trajectory_planner.yaml" command="load" />
    </node>

    <node name="rviz" pkg="rviz" type="rviz"/>

    <node name="find_object_3d" pkg="find_object_2d" type="find_object_2d" output="log" required="true">
        <param name="gui" value="true" type="bool"/>
        <param name="subscribe_depth" value="true" type="bool"/>
        <param name="objects_path" value="$(find tutorial_pkg)/image_rec/" type="str"/>
        <param name="object_prefix" value="object" type="str"/>
        <remap from="rgb/image_rect_color" to="/camera/rgb/image_raw"/>
        <remap from="depth_registered/image_raw" to="/camera/depth/image_raw"/>
        <remap from="depth_registered/camera_info" to="/camera/depth/camera_info"/>
    </node>

    <node pkg="frontier_exploration" type="explore_client" name="explore_client" output="screen"/>

    <node pkg="frontier_exploration" type="explore_server" name="explore_server" output="screen">
        <param name="frequency" type="double" value="1.0"/>
        <param name="goal_aliasing" type="double" value="0.1"/>
        <rosparam ns="explore_costmap" subst_value="true" file="$(find rosbot_navigation)/config/exploration.yaml" command="load" />
        <param name="explore_clear_space" type="boolean" value="true"/>
        <param name="frontier_travel_point" type="string" value="middle"/>
    </node>

    <node pkg="tutorial_pkg" type="search_manager" name="search_manager" output="screen"/>

</launch>
```


#### ROSbot version

When running search task on ROSbot, you will need two `launch` files, first to be run on ROSbot:

```launch
<launch>

    <include file="$(find astra_launch)/launch/astra.launch"></include>

    <include file="$(find rplidar_ros)/launch/rplidar.launch"></include>

    <node pkg="depthimage_to_laserscan" type="depthimage_to_laserscan" name="depthimage_to_laserscan" required="true">
        <remap from="/image" to="/camera/depth/image_raw"/>
        <remap from="scan" to="proj_scan"/>
        <param name="scan_height" value="1"/>
        <param name="range_min" value="0.45"/>
        <param name="range_max" value="1.5"/>
    </node>

    <node pkg="image_transport" type="republish" name="rgb_compress" args=" raw in:=/camera/rgb/image_raw compressed out:=/rgb_republish"/>

    <node pkg="image_transport" type="republish" name="depth_compress" args=" raw in:=/camera/depth/image_raw compressed out:=/depth_republish">
        <param name="compressed/format" value="png"/>
        <param name="compressed/png_level" value="1"/>
    </node>

    <node pkg="tutorial_pkg" type="drive_controller_node" name="drive_controller"/>

    <node pkg="tf" type="static_transform_publisher" name="ROSbot2_laser" args="0.019 0 0 3.14 0 0 base_link laser 100" />

    <node pkg="tf" type="static_transform_publisher" name="ROSbot2_camera" args="0.0 0 0.15 0 0 0 base_link camera_link 100" />

    <node pkg="gmapping" type="slam_gmapping" name='gmapping_node' output='log'>
        <param name="base_frame" value="base_link"/>
        <param name="odom_frame" value="odom"/>
        <param name="delta" value="0.01"/>
        <param name="xmin" value="-5"/>
        <param name="ymin" value="-5"/>
        <param name="xmax" value="5"/>
        <param name="ymax" value="5"/>
        <param name="maxUrange" value="5"/>
        <param name="map_update_interval" value="1"/>
        <param name="linearUpdate" value="0.05"/>
        <param name="angularUpdate" value="0.05"/>
        <param name="temporalUpdate" value="0.1"/>
        <param name="particles" value="100"/>
    </node>

    <node pkg="move_base" type="move_base" name="move_base" output="screen">
        <param name="controller_frequency" value="10.0"/>
        <rosparam file="$(find rosbot_navigation)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find rosbot_navigation)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find rosbot_navigation)/config/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find rosbot_navigation)/config/global_costmap_params.yaml" command="load" />
        <rosparam file="$(find rosbot_navigation)/config/trajectory_planner.yaml" command="load" />
    </node>

</launch>
```

And second to be run on another device:

```launch
<launch>
    <node name="rviz" pkg="rviz" type="rviz"/>

    <node pkg="image_transport" type="republish" name="rgb_decompress" args=" compressed in:=/rgb_republish raw out:=/rgb_raw">
        <param name="compressed/mode" value="color"/>
    </node>

    <node pkg="image_transport" type="republish" name="depth_decompress" args=" compressed in:=/depth_republish raw out:=/depth_raw ">
        <param name="compressed/mode" value="unchanged"/>
    </node>

    <node name="find_object_3d" pkg="find_object_2d" type="find_object_2d" output="log" required="true">
        <param name="gui" value="true" type="bool"/>
        <param name="subscribe_depth" value="true" type="bool"/>
        <param name="objects_path" value="$(find tutorial_pkg)/image_rec/" type="str"/>
        <param name="object_prefix" value="object" type="str"/>
        <remap from="rgb/image_rect_color" to="/rgb_raw"/>
        <remap from="depth_registered/image_raw" to="/depth_raw"/>
        <remap from="depth_registered/camera_info" to="/camera/depth/camera_info"/>
    </node>

    <node pkg="frontier_exploration" type="explore_server" name="explore_server" output="screen">
        <param name="frequency" type="double" value="1.0"/>
        <param name="goal_aliasing" type="double" value="0.1"/>
        <rosparam ns="explore_costmap" subst_value="true" file="$(find rosbot_navigation)/config/exploration.yaml" command="load" />
        <param name="explore_clear_space" type="boolean" value="true"/>
        <param name="frontier_travel_point" type="string" value="middle"/>
    </node>

    <node pkg="tutorial_pkg" type="search_manager" name="search_manager" output="screen"/>

</launch>
```

## Summary ##

After completing this tutorial you should be familiar with controlling tasks using `actionlib` library. You will also know basic usage of `grid_map` library to load, edit, create from scratch and publish `nav_msgs/OccupancyGrid` maps. Finally you will be able to configure your robot to search for an object in selected area.

---------

*by Łukasz Mitka, Husarion*

*Do you need any support with completing this tutorial or have any difficulties with software or hardware? Feel free to describe your thoughts on our community forum: https://community.husarion.com/ or to contact with our support: support@husarion.com*
