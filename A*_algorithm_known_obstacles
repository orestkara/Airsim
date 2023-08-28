#include <iostream>

#include <vector>

#include <queue>

#include <cmath>

#include <unordered_map>

#include <unordered_set>


#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"

#include <chrono>
#include <thread>
#include <tuple>

using namespace std;

struct Node
{

    int x, y, z;

    double g, h, f;

    Node* parent;

    Node(int x, int y, int z, double g, double h)
        : x(x), y(y), z(z), g(g), h(h), f(g + h), parent(nullptr) {}
};

class AStar
{

private:
    int grid_x_size;

    int grid_y_size;

    int grid_z_size;

    int drone_height;

    double grid_resolution = 5.0;

    struct NodeComparator
    {

        bool operator()(const Node* a, const Node* b) const
        {

            return a->f > b->f;
        }
    };

    bool isValid(int x, int y, int z)
    {

        return x >= 80 && x <= 480 && y >= 60 && y <= 460 && z >= 160 && z <= 560;
    }
    bool isKnownObstacle(int x, int y, int z, const vector<tuple<int, int, int>>& obstacles) 
    {

      for (const auto& obstacle : obstacles) {

      int obstacle_x, obstacle_y, obstacle_z;

      tie(obstacle_x, obstacle_y, obstacle_z) = obstacle;

      if (x == obstacle_x && y == obstacle_y && z == obstacle_z) {

            return true;

        }

    }

    return false;

}

    double heuristic(int x, int y, int z, int goal_x, int goal_y, int goal_z)
    {

        return euclideanDistance(x, y, z, goal_x, goal_y, goal_z);
    }

    double euclideanDistance(int x1, int y1, int z1, int x2, int y2, int z2)
    {

        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
    }

    unordered_set<string> closed;

    bool isClosed(int x, int y, int z)
    {

        string cell_key = to_string(x) + "," + to_string(y) + "," + to_string(z);

        return closed.find(cell_key) != closed.end();
    }

    void addToClosed(int x, int y, int z)
    {

        string cell_key = to_string(x) + "," + to_string(y) + "," + to_string(z);

        closed.insert(cell_key);
    }

public:
    AStar()
        : grid_x_size(5), grid_y_size(5), grid_z_size(5), drone_height(160) {}

    void setGridSize(int x, int y, int z)
    {
        grid_x_size = x;
        grid_y_size = y;
        grid_z_size = z;
    }

    void setDroneHeight(int height)
    {
        drone_height = height;
    }

    void moveDroneOnPath(vector<Node*> path, msr::airlib::MultirotorRpcLibClient& client)
    {
        for (int i = path.size() -1; i >= 0; i--) {
            Node* current_node = path[i];
            int next_x = current_node->x;
            int next_y = current_node->y;
            int next_z = current_node->z * -1; //Due to NED coordinates system, reverse the z-axis

            std::cout << "\n";
            std::cout << "x: " << next_x << " y: " << next_y << " z: " << next_z;

            //Failed to implement this function
            //// Move the drone to the next waypoint
            //msr::airlib::Pose pose;
            //pose.position.x() = next_x * .01;
            //pose.position.y() = next_y * .01;
            //pose.position.z() = next_z * .01; // Adding drone's height to reach the specified z-coordinate
            ////pose.orientation.w() = 1.0; // Assuming no rotation

            //// Set the new pose for the drone
            //client.simSetVehiclePose(pose, true);
            client.moveToPositionAsync(next_x * .01f, next_y * .01f, next_z * .01f, 1);

            // Sleep for a short period to allow the drone to reach the waypoint
            std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // Adjust as needed
        }
    }

    vector<Node*> findPath(int start_x, int start_y, int start_z, int goal_x, int goal_y, int goal_z, const vector<tuple<int, int, int>>& obstacles)
     {
        vector<Node*> result;

        //Create the open and closed lists
       
        priority_queue<Node*, vector<Node*>, NodeComparator> open;

        // Create the start node and add it to the open list
       
        Node* start_node = new Node(start_x, start_y, start_z, 0.0, heuristic(start_x, start_y, start_z, goal_x, goal_y, goal_z));
        open.push(start_node);

        while (!open.empty()) {

            // Get the node with the smallest f value from the open list
            Node* current_node = open.top();
            open.pop();

            // Goal check

            if (current_node->x == goal_x && current_node->y == goal_y && current_node->z == goal_z) {

                // Goal reached, reconstruct path and return
                while (current_node != nullptr) {
                    result.push_back(current_node);
                    current_node = current_node->parent;
                }
                break;
            }

          
            // Mark the current node as closed
            addToClosed(current_node->x, current_node->y, current_node->z);


            // Generate successors and add them to the open list
            for (int dx = -100; dx <= 100; dx+=100) {

                for (int dy = -100; dy <= 100; dy+=100) {

                    for (int dz = -100; dz <= 100; dz+=100) {

                        int new_x = current_node->x + dx;

                        int new_y = current_node->y + dy;

                        int new_z = current_node->z + dz;

                        // Check if the new coordinates are valid
                        if (isValid(new_x, new_y, new_z) && !isClosed(new_x, new_y, new_z) && !isKnownObstacle (new_x, new_y, new_z, obstacles)) {

                            double g = current_node->g + euclideanDistance(current_node->x, current_node->y, current_node->z, new_x, new_y, new_z);

                            double h = heuristic(new_x, new_y, new_z, goal_x, goal_y, goal_z);

                            Node* new_node = new Node(new_x, new_y, new_z, g, h);

                            new_node->parent = current_node;

                            open.push(new_node);
                        }

                    }
                }
            }
        }

        return result;
    }
};

int main()
{

    // Initialize AirSim connection
    
    msr::airlib::MultirotorRpcLibClient client;

    typedef msr::airlib::ImageCaptureBase::ImageRequest ImageRequest;

    typedef msr::airlib::ImageCaptureBase::ImageResponse ImageResponse;

    typedef msr::airlib::ImageCaptureBase::ImageType ImageType;

    typedef common_utils::FileSystem FileSystem;

    std::cout << "Press Enter to enable API control\n";
    std::cin.get();
    client.enableApiControl(true);

    std::cout << "Press Enter to arm the drone\n";
    std::cin.get();
    client.armDisarm(true);

    std::cout << "Press Enter to takeoff\n";
    std::cin.get();
    client.takeoffAsync(5)->waitOnLastTask();

    std::cout << "Press Enter to follow found path\n";
    std::cin.get();


    // Create the A* planner
    AStar astar;
  
    // Set the grid size and drone height
  
    astar.setGridSize(5, 5, 5); //Values are indicative. Change according to the grid you want to traverse.

    astar.setDroneHeight(160); //Values are indicative. Change according to the grid you want to traverse.

    int start_x = 80; //Values are indicative. Change according to the grid you want to traverse.
  
    int start_y = 60; //Values are indicative. Change according to the grid you want to traverse.

    int start_z = 160; //Values are indicative. Change according to the grid you want to traverse.

    int goal_x = 480; //Values are indicative. Change according to the grid you want to traverse.

    int goal_y = 460; //Values are indicative. Change according to the grid you want to traverse.

    int goal_z = 160; //Values are indicative. Change according to the grid you want to traverse.

    std::const vector<tuple<int, int, int>> obstacles = { make_tuple(180, 160, 160), make_tuple(180, 60, 160)};

    std::vector<Node*> path;
    // Find the path using A* algorithm
  
    path = astar.findPath(start_x, start_y, start_z, goal_x, goal_y, goal_z, obstacles);

    // Move the drone along the path
  
    astar.moveDroneOnPath(path, client);

    // Clean up memory
    
    for (Node* node : path) {

        delete node;
    }

    std::cout << "Press Enter to land\n";
    std::cin.get();
    client.landAsync()->waitOnLastTask();

    return 0;

}
