#include "navigation/global_planner/breadth_first_search.h"

nav_msgs::Path BFS::createPath(Node* goal_node, Map* map)
{
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";

  if (goal_node && goal_node->parent == nullptr)
  {
    ROS_WARN_STREAM("Unable to find path / path does not exist");
  }
  else
  {
    Node* current = goal_node;

    while (current != nullptr)
    {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = current->x * map->getResolution();
      pose.pose.position.y = current->y * map->getResolution();
      path.poses.push_back(pose);
      current = current->parent;
    }
  }

  return path;
}

nav_msgs::Path BFS::getGlobalPath(const geometry_msgs::PointStamped& start,
                                  const geometry_msgs::PointStamped& goal, Map* map)
{
  nav_msgs::Path global_path;

  // Step-1: Verify whether goal node is an obstacle or not
  Node* goal_node = map->getNodeFromMap(goal.point.x, goal.point.y);

  if (goal_node->x >= map->getWidthInPixels() || goal_node->y >= map->getHeightInPixels() ||
      goal_node->is_obstacle || !map)
  {
    return global_path;
  }

  // Step-2: Start breadth first search on the graph and find shortest path
  // between start and goal nodes
  std::queue<Node*> search_list;
  std::vector<std::vector<Node*>> graph = map->getGraph();

  Node* start_node = map->getNodeFromMap(start.point.x, start.point.y);

  ROS_INFO_STREAM("Start: [" << start_node->x << "," << start_node->y << " => " << goal_node->x
                             << "," << goal_node->y << "] (in pixels)");

  ROS_INFO_STREAM("Start: [" << start.point.x << "," << start.point.y << " => " << goal.point.x
                             << "," << goal.point.y << "] (in meters)");

  graph[start_node->x][start_node->y]->is_visited = true;
  search_list.push(graph[start_node->x][start_node->y]);

  Node* last_node;

  while (!search_list.empty())
  {
    Node* current_node = search_list.front();
    last_node = current_node;
    search_list.pop();

    if (current_node->x == goal_node->x && current_node->y == goal_node->y)
      break;

    std::vector<Node*> neighbors;

    if (int(current_node->x - 1) >= 0 &&
        !graph[current_node->x - 1][current_node->y]->is_obstacle &&
        !graph[current_node->x - 1][current_node->y]->is_visited)
      neighbors.push_back(graph[current_node->x - 1][current_node->y]);
    if (current_node->x + 1 < map->getWidthInPixels() &&
        !graph[current_node->x + 1][current_node->y]->is_obstacle &&
        !graph[current_node->x + 1][current_node->y]->is_visited)
      neighbors.push_back(graph[current_node->x + 1][current_node->y]);
    if (int(current_node->y - 1) >= 0 &&
        !graph[current_node->x][current_node->y - 1]->is_obstacle &&
        !graph[current_node->x][current_node->y - 1]->is_visited)
      neighbors.push_back(graph[current_node->x][current_node->y - 1]);
    if (current_node->y + 1 < map->getHeightInPixels() &&
        !graph[current_node->x][current_node->y + 1]->is_obstacle &&
        !graph[current_node->x][current_node->y + 1]->is_visited)
      neighbors.push_back(graph[current_node->x][current_node->y + 1]);

    for (auto& neighbor : neighbors)
    {
      neighbor->parent = current_node;
      neighbor->is_visited = true;
      search_list.push(neighbor);
    }
  }

  ROS_DEBUG_STREAM("Search completed: final node => (" << last_node->x << "," << last_node->y
                                                       << ")");

  // Step-3: From goal node, traverse back to start node and create path with
  // vector of poses
  global_path = createPath(goal_node, map);

  ROS_DEBUG_STREAM("Valid global path found");

  // Step-4 Reset graph for creating another path
  map->resetGraph();

  // Step-5: Attach orientation to all poses by identifyin the slope between two
  // consecutive waypoints
  for (int i = 0; i < global_path.poses.size(); i++)
  {
    if (i + 1 != global_path.poses.size())
    {
      double orientation =
          atan2((global_path.poses[i].pose.position.y - global_path.poses[i + 1].pose.position.y),
                (global_path.poses[i].pose.position.x - global_path.poses[i + 1].pose.position.x));
      tf2::Quaternion q;
      q.setRPY(0, 0, orientation);
      global_path.poses[i].pose.orientation.x = q.x();
      global_path.poses[i].pose.orientation.y = q.y();
      global_path.poses[i].pose.orientation.z = q.z();
      global_path.poses[i].pose.orientation.w = q.w();
    }
  }

  // Step-6: Downsample number of poses to create a smooth trajectory for local
  // planner algorithm
  nav_msgs::Path downsampled_global_path;

  if (!global_path.poses.empty())
  {
    downsampled_global_path.header = global_path.header;
    downsampled_global_path.poses.push_back(global_path.poses[0]);
    for (int i = 1; i < global_path.poses.size() - 2; i++)
    {
      geometry_msgs::PoseStamped pt_a, pt_b, pt_c;
      float slope_ab = 0.0;
      float slope_bc = 0.0;

      pt_a = global_path.poses[i];
      pt_b = global_path.poses[i + 1];
      pt_c = global_path.poses[i + 2];

      slope_ab = atan2((pt_b.pose.position.y - pt_a.pose.position.y),
                       (pt_b.pose.position.x - pt_a.pose.position.x));

      slope_bc = atan2((pt_c.pose.position.y - pt_b.pose.position.y),
                       (pt_c.pose.position.x - pt_b.pose.position.x));

      if(slope_ab != slope_bc)
      {
        downsampled_global_path.poses.push_back(pt_a);
      }
    }
    downsampled_global_path.poses.push_back(global_path.poses[global_path.poses.size() - 1]);
  }

  ROS_DEBUG_STREAM("Downsampling complete");

  return downsampled_global_path;
}
