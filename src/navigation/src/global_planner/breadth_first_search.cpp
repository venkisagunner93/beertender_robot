#include "navigation/global_planner/breadth_first_search.h"

std::vector<Node*> BFS::findNeighbors(Node* node)
{
  std::vector<Node*> neighbors;

  if (int(node->x - 1) >= 0 && !graph_[node->x - 1][node->y]->is_obstacle &&
      !graph_[node->x - 1][node->y]->is_visited)
    neighbors.push_back(graph_[node->x - 1][node->y]);
  if (node->x + 1 < map_.info.height && !graph_[node->x + 1][node->y]->is_obstacle &&
      !graph_[node->x + 1][node->y]->is_visited)
    neighbors.push_back(graph_[node->x + 1][node->y]);
  if (int(node->y - 1) >= 0 && !graph_[node->x][node->y - 1]->is_obstacle &&
      !graph_[node->x][node->y - 1]->is_visited)
    neighbors.push_back(graph_[node->x][node->y - 1]);
  if (node->y + 1 < map_.info.width && !graph_[node->x][node->y + 1]->is_obstacle &&
      !graph_[node->x][node->y + 1]->is_visited)
    neighbors.push_back(graph_[node->x][node->y + 1]);

  return neighbors;
}

nav_msgs::Path BFS::createPath(Node* goal_node)
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
      pose.pose.position.x = current->x * map_.info.resolution;
      pose.pose.position.y = current->y * map_.info.resolution;
      path.poses.push_back(pose);
      current = current->parent;
    }
  }

  return path;
}

nav_msgs::Path BFS::getGlobalPath(const geometry_msgs::PointStamped& start,
                                  const geometry_msgs::PointStamped& goal)
{
  // Step-1: Reset the graph where all nodes are set back to not visited
  resetGraph();
  nav_msgs::Path global_path;

  // Step-2: Verify whether goal node is an obstacle or not
  if (static_cast<int>(goal.point.x / map_.info.resolution) >= map_.info.width ||
      static_cast<int>(goal.point.y / map_.info.resolution) >= map_.info.height)
  {
    return global_path;
  }

  // Step-3: Start breadth first search on the graph and find shortest path
  // between start and goal nodes
  Node* goal_node = graph_[static_cast<int>(goal.point.x / map_.info.resolution)]
                          [static_cast<int>(goal.point.y / map_.info.resolution)];

  std::queue<Node*> search_list;

  ROS_INFO_STREAM("Start: [" << static_cast<int>(start.point.x / map_.info.resolution) << ","
                             << static_cast<int>(start.point.y / map_.info.resolution)
                             << "] => Goal: ["
                             << static_cast<int>(goal.point.x / map_.info.resolution) << ","
                             << static_cast<int>(goal.point.y / map_.info.resolution) << "]");

  graph_[static_cast<int>(start.point.x / map_.info.resolution)]
        [static_cast<int>(start.point.y / map_.info.resolution)]
            ->is_visited = true;
  search_list.push(graph_[static_cast<int>(start.point.x / map_.info.resolution)]
                         [static_cast<int>(start.point.y / map_.info.resolution)]);

  while (!search_list.empty())
  {
    Node* current_node = search_list.front();
    search_list.pop();

    if (current_node->x == goal_node->x && current_node->y == goal_node->y)
      break;

    for (auto& neighbor : findNeighbors(current_node))
    {
      neighbor->parent = current_node;
      neighbor->is_visited = true;
      search_list.push(neighbor);
    }
  }

  // Step-4: From goal node, traverse back to start node and create path with
  // vector of poses
  global_path = createPath(goal_node);

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

void BFS::updateMap(const nav_msgs::OccupancyGrid& msg)
{
  map_ = msg;
  createGraphFromMap();
}

void BFS::resetGraph()
{
  for (unsigned int i = 0; i < map_.info.width; i++)
  {
    for (unsigned int j = 0; j < map_.info.height; j++)
    {
      graph_[i][j]->is_visited = false;
    }
  }
}

void BFS::createGraphFromMap()
{
  graph_.resize(map_.info.width);
  for (unsigned int i = 0; i < map_.info.height; i++)
  {
    graph_[i].resize(map_.info.height);
  }

  int m = 0;
  int n = 0;

  if (!map_.data.empty())
  {
    for (int i = 0; i < map_.data.size(); i++)
    {
      if (i > 0 && i % map_.info.height == 0)
      {
        n++;
        m = 0;
      }

      Node* new_node = new Node();
      new_node->x = m;
      new_node->y = n;

      if (map_.data[i] == 100)
      {
        new_node->is_obstacle = true;
      }

      graph_[m][n] = new_node;
      m++;
    }
  }
}

void BFS::displayGraph() const
{
  for (unsigned int i = 0; i < map_.info.width; i++)
  {
    for (unsigned int j = 0; j < map_.info.height; j++)
    {
      std::cout << graph_[i][j]->is_obstacle << "\t";
    }
    std::cout << std::endl;
  }
}