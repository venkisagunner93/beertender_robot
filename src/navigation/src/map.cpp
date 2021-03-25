#include "navigation/map.h"

void Map::setMap(const nav_msgs::OccupancyGrid& map)
{
  map_ = map;
  width_ = map_.info.width;
  height_ = map_.info.height;
  resolution_ = map_.info.resolution;
  createGraphFromMap();
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
  return graph_[static_cast<int>(x / resolution_)][static_cast<int>(y / resolution_)];
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