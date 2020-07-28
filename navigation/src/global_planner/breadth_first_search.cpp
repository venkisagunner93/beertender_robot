#include "navigation/global_planner/breadth_first_search.h"

std::vector<Node*> BFS::findNeighbors(Node* node)
{
    std::vector<Node*> neighbors;

    if(int(node->x - 1) >= 0 && !graph_[node->x - 1][node->y]->is_obstacle && !graph_[node->x - 1][node->y]->is_visited)
        neighbors.push_back(graph_[node->x - 1][node->y]);
    if(node->x + 1 < map_.info.height && !graph_[node->x + 1][node->y]->is_obstacle && !graph_[node->x + 1][node->y]->is_visited)
        neighbors.push_back(graph_[node->x + 1][node->y]);
    if(int(node->y - 1) >= 0 && !graph_[node->x][node->y - 1]->is_obstacle && !graph_[node->x][node->y - 1]->is_visited)
        neighbors.push_back(graph_[node->x][node->y - 1]);
    if(node->y + 1 < map_.info.width && !graph_[node->x][node->y + 1]->is_obstacle && !graph_[node->x][node->y + 1]->is_visited)
        neighbors.push_back(graph_[node->x][node->y + 1]);

    return neighbors;
}

nav_msgs::Path BFS::createPath(Node* goal_node)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    
    if(goal_node->parent == nullptr)
    {
        ROS_WARN_STREAM("Unable to find path / path does not exist");
        return path;
    }
    else
    {
        Node* current = goal_node;

        while(current != nullptr)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = current->y * map_.info.resolution;
            pose.pose.position.y = current->x * map_.info.resolution;
            path.poses.push_back(pose);
            current = current->parent;
        }
    }

    return path;
}

nav_msgs::Path BFS::getGlobalPath(const Coordinate& start, const Coordinate& goal)
{
    resetGraph();
    Node* goal_node = graph_[int(goal.x / map_.info.resolution)][int(goal.y / map_.info.resolution)];

    std::queue<Node*> search_list;

    ROS_INFO_STREAM("Start: [" << int(start.x / map_.info.resolution) << "," << int(start.y / map_.info.resolution) << 
                    "] => Goal: [" << int(goal.x / map_.info.resolution) << "," << int(goal.y / map_.info.resolution) << "]");

    graph_[int(start.x / map_.info.resolution)][int(start.y / map_.info.resolution)]->is_visited = true;
    search_list.push(graph_[int(start.x / map_.info.resolution)][int(start.y / map_.info.resolution)]);

    while(!search_list.empty())
    {
        Node* current_node = search_list.front();
        search_list.pop();

        if(current_node->x == goal_node->x && current_node->y == goal_node->y)
            break;
        
        for(auto neighbor : findNeighbors(current_node))
        {
            neighbor->parent = current_node;
            neighbor->is_visited = true;
            search_list.push(neighbor);
        }
    }

    return createPath(goal_node);
}

void BFS::updateMap(const nav_msgs::OccupancyGrid& msg)
{
    map_ = msg;
    createGraphFromMap();
}

void BFS::resetGraph()
{
    for(unsigned int i = 0; i < map_.info.height; i++)
    {
        for(unsigned int j = 0; j < map_.info.width; j++)
        {
            graph_[i][j]->is_visited = false;
        }
    }
}

void BFS::createGraphFromMap()
{
    int k = 0;

    graph_.resize(map_.info.height);
    for(unsigned int i = 0; i < map_.info.width; i++)
    {
        graph_[i].resize(map_.info.width);
    }

    for(unsigned int i = 0; i < map_.info.height; i++)
    {
        for(unsigned int j = 0; j < map_.info.width; j++)
        {
            Node* node = new Node();

            node->x = i;
            node->y = j;
            
            if(int(map_.data[k]) == 100)
                node->is_obstacle = true;

            graph_[i][j] = node;

            k++;
        }
    }
}

void BFS::displayGraph() const
{
    int k = 0;
    for(unsigned int i = 0; i < map_.info.height; i++)
    {
        for(unsigned int j = 0; j < map_.info.width; j++)
        {
            std::cout << graph_[i][j]->is_obstacle << "\t";
        }
        std::cout << std::endl;
    }
}