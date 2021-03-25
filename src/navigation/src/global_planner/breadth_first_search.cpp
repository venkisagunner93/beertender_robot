#include "navigation/global_planner/breadth_first_search.h"

nav_msgs::Path BFS::createPath(Node* goal_node, Map* map)
{
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";

  if (goal_node->parent == nullptr)
  {
    ROS_WARN_STREAM("Unable to find path / path does not exist");
    return path;
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

  while (!search_list.empty())
  {
    Node* current_node = search_list.front();
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

  ROS_DEBUG_STREAM("Search completed");

  // Step-3: From goal node, traverse back to start node and create path with
  // vector of poses
  global_path = createPath(goal_node, map);

  ROS_DEBUG_STREAM("Valid global path found");

  // Step-4 Reset graph for creating another path
  map->resetGraph();

  // Step-5: Downsample number of poses to create a smooth trajectory for local
  // planner algorithm
  nav_msgs::Path downsampled_global_path;

  if (!global_path.poses.empty())
  {
    downsampled_global_path.header = global_path.header;
    downsampled_global_path.poses.push_back(global_path.poses[0]);

    int sample_interval = global_path.poses.size() / 10;

    for (int i = 1; i < global_path.poses.size() - 1; i++)
    {
      if (i % sample_interval == 0)
      {
        downsampled_global_path.poses.push_back(global_path.poses[i]);
      }
    }

    downsampled_global_path.poses.push_back(global_path.poses[global_path.poses.size() - 1]);
  }

  ROS_DEBUG_STREAM("Downsampling complete");

  // Step-6: Attach orientation to all poses by identifyin the slope between two
  // consecutive waypoints
  for (int i = 0; i < downsampled_global_path.poses.size(); i++)
  {
    if (i + 1 != downsampled_global_path.poses.size())
    {
      double orientation = atan2((downsampled_global_path.poses[i].pose.position.y -
                                  downsampled_global_path.poses[i + 1].pose.position.y),
                                 (downsampled_global_path.poses[i].pose.position.x -
                                  downsampled_global_path.poses[i + 1].pose.position.x));
      tf2::Quaternion q;
      q.setRPY(0, 0, orientation);
      downsampled_global_path.poses[i].pose.orientation.x = q.x();
      downsampled_global_path.poses[i].pose.orientation.y = q.y();
      downsampled_global_path.poses[i].pose.orientation.z = q.z();
      downsampled_global_path.poses[i].pose.orientation.w = q.w();
    }
  }

  return downsampled_global_path;
}
