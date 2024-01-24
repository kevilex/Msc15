#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <Eigen/Geometry> 

struct Point {
    double x, y, z;
};


std::vector<Point> parsePLYFile(const std::string& filename) {
    std::ifstream file(filename);
    std::vector<Point> points;
    std::string line;

    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file");
    }

    // Read vertex data
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        Point p;
        if (!(iss >> p.x >> p.y >> p.z)) { 
            break; // Error or end of file
        }
        points.push_back(p);
    }

    return points;
}

Eigen::Quaterniond vectorToQuaternion(const Point& target_vector) {
    // Assuming the initial direction is along the z-axis
    Eigen::Vector3d initial_direction(0, 0, 1);
    Eigen::Vector3d target_direction(target_vector.x, target_vector.y, target_vector.z);

    // Normalize the target direction
    target_direction.normalize();

    // Compute the rotation axis and angle
    Eigen::Vector3d rotation_axis = initial_direction.cross(target_direction);
    double angle = acos(initial_direction.dot(target_direction));

    // Create and return the quaternion representing this rotation
    Eigen::Quaterniond quaternion;
    quaternion = Eigen::AngleAxisd(angle, rotation_axis.normalized());

    return quaternion;
}

int main(int argc, char* argv[])
{

    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "Manipulator_Node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("Manipulator_Node");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
    // Define multiple target poses and add them to a vector
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // Retrieve waypoints
    std::string Path = "/home/kevin/workspace/points_output.txt";
    std::vector<Point> points = parsePLYFile(Path);


    // Calculate the normal vector for the waypoints
    Point NormalVector;
    NormalVector.x = 0.0;
    NormalVector.y = 1.0;
    NormalVector.z = 0.0;
    Eigen::Quaternion angle = vectorToQuaternion(NormalVector);

    // Adding all the points to the waypoints vector
    int num_points = static_cast<int>(points.size());
    RCLCPP_INFO(node->get_logger(), "Adding %i points:", num_points);
    for (int i = 0; i < num_points; ++i) {

        geometry_msgs::msg::Pose temp;
        temp.orientation.x = angle.x();
        temp.orientation.y = angle.y();
        temp.orientation.z = angle.z();
        temp.orientation.w = angle.w();

        temp.position.x = points[i].x * 2;
        temp.position.y = points[i].y * 2;
        temp.position.z = points[i].z * (-1);
        waypoints.push_back(temp);

        RCLCPP_INFO(node->get_logger(), "x: %f, y: %f, z: %f", temp.position.x, temp.position.y, temp.position.z);
    }



    // Plan the Cartesian path connecting the waypoints
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0; // Threshold for deciding if a movement is a 'jump'
    const double eef_step = 0.001; // Resolution of the path
    
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction > 0.01)
    {
        // Execute the plan
        move_group_interface.execute(trajectory);
    }
    else
    {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}