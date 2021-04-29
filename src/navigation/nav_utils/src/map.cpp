#include "nav_utils/map.h"

Map::Map(ros::NodeHandle* nh)
{
  get_map_client_ = nh->serviceClient<nav_msgs::GetMap>("static_map");

  nav_msgs::GetMap get_map_srv;

  if (get_map_client_.waitForExistence(ros::Duration(5.0)))
  {
    if (get_map_client_.call(get_map_srv))
    {
      map_ = get_map_srv.response.map;
      width_ = map_.info.width;
      height_ = map_.info.height;
      resolution_ = map_.info.resolution;
      createGraphFromMap();
      ROS_INFO_STREAM("[Map]: Global map constructed successfully");
    }
  }
  else
  {
    ROS_WARN_STREAM("[Map]: Timed out. 'static_map' server not available");
  }
}

int Map::getWidthInPixels() const
{
  return width_;
}

int Map::getHeightInPixels() const
{
  return height_;
}

float Map::getResolution() const
{
  return resolution_;
}

float Map::getWidthInMeters()
{
  return width_ * resolution_;
}

float Map::getHeightInMeters()
{
  return height_ * resolution_;
}

void Map::createGraphFromMap()
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

Node* Map::getNodeFromMap(const float& x, const float& y)
{
  if (x < 0.0 || y < 0.0 || x > getWidthInMeters() || y > getHeightInMeters())
  {
    return nullptr;
  }
  else
  {
    return graph_[static_cast<int>(x / resolution_)][static_cast<int>(y / resolution_)];
  }
}

std::vector<std::vector<Node*>> Map::getGraph() const
{
  return graph_;
}

void Map::resetGraph()
{
  for (unsigned int i = 0; i < map_.info.width; i++)
  {
    for (unsigned int j = 0; j < map_.info.height; j++)
    {
      graph_[i][j]->is_visited = false;
      graph_[i][j]->parent = nullptr;
    }
  }
}

void Map::displayGraph() const
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