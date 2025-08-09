/**
 * @brief WeightedTopologyNeighbors class for calculating weighted topology neighbors based on vehicle positions.
 * @details This class extends the NearestNeighbors class template to calculate weighted topology neighbors.
 * It provides functionality to process neighbor positions, enrich neighborhoods, and handle weighted topology neighbors.
 * @author Arthur Astier
 */

#include "SwarmControllers/WeightedTopology/WeightedTopologyNeighbors.hpp"
#include <numeric>  // for std::accumulate
#include <memory>   // for std::bind and shared_ptr

namespace Neighborhood {

/**
 * @brief Constructor for WeightedTopologyNeighbors class.
 */
WeightedTopologyNeighbors::WeightedTopologyNeighbors()
        : NearestNeighbors<custom_msgs::msg::WeightedTopologyNeighbors>() {
    this->declare_parameter<std::vector<bool>>("leaders");
    this->declare_parameter<std::vector<double>>("x_formation");
    this->declare_parameter<std::vector<double>>("y_formation");
    this->declare_parameter<std::vector<double>>("z_formation");

    const auto x_formation{this->get_parameter("x_formation").as_double_array()};
    const auto y_formation{this->get_parameter("y_formation").as_double_array()};
    const auto z_formation{this->get_parameter("z_formation").as_double_array()};

    vectors_to_Vector3d(x_formation, y_formation, z_formation);

    leaders = this->get_parameter("leaders").as_bool_array();

    // Initialize prcs and prcs_neighborhood matrices (assumed square with nb_drones dimension)
    prcs = PRCS::Constant(nb_drones, nb_drones);
    prcs_neighborhood = PRCS::Constant(nb_drones, nb_drones);

    // Initialize the service for changing formation at runtime
    change_formation_service_ = this->create_service<custom_msgs::srv::ChangeFormation>(
        "change_formation",
        std::bind(&WeightedTopologyNeighbors::handle_change_formation_request, this,
                  std::placeholders::_1, std::placeholders::_2));
}

/**
/**
 * @brief Service callback to change the formation dynamically.
 */
void WeightedTopologyNeighbors::handle_change_formation_request(
    const std::shared_ptr<custom_msgs::srv::ChangeFormation::Request> request,
    std::shared_ptr<custom_msgs::srv::ChangeFormation::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Received request to change formation: %s", request->formation_type.c_str());

    formation.clear();
    formation.reserve(nb_drones);

    if (request->formation_type == "line") {
        // Drones in a horizontal line along the x-axis
        for (size_t i = 0; i < nb_drones; i++) {
            formation.emplace_back(Vector3d(i * request->follower_distance, 0.0, 0.0));
        }
        response->success = true;
        response->message = "Formation changed to line formation.";

    } else if (request->formation_type == "row") {
        // Drones stacked vertically along the z-axis
        for (size_t i = 0; i < nb_drones; i++) {
            formation.emplace_back(Vector3d(0.0, 0.0, i * request->follower_distance));
        }
        response->success = true;
        response->message = "Formation changed to row formation.";

    } else if (request->formation_type == "custom") {
        // Diagonal custom formation (x and y increase)
        for (size_t i = 0; i < nb_drones; i++) {
            formation.emplace_back(Vector3d(i * request->follower_distance, i * request->leader_distance, 0.0));
        }
        response->success = true;
        response->message = "Formation changed to custom formation.";

    } else if (request->formation_type == "triangle") {
        if (nb_drones >= 3) {
            // First 3 drones form a triangle on the XY plane
            formation.emplace_back(Vector3d(0.0, request->leader_distance, 0.0)); // Top vertex
            formation.emplace_back(Vector3d(-request->follower_distance / 2.0, 0.0, 0.0)); // Bottom left
            formation.emplace_back(Vector3d(+request->follower_distance / 2.0, 0.0, 0.0)); // Bottom right

            // Remaining drones line up behind the center
            for (size_t i = 3; i < nb_drones; ++i) {
                formation.emplace_back(Vector3d(0.0, -i * request->leader_distance, 0.0));
            }

            response->success = true;
            response->message = "Formation changed to triangle formation.";
        } else {
            response->success = false;
            response->message = "Triangle formation needs at least 3 drones.";
        }

    } else {
        response->success = false;
        response->message = "Unknown formation type: " + request->formation_type;
    }
}



/**
 * @brief Converts vectors representing x, y, and z coordinates to Eigen Vector3d objects.
 */
void WeightedTopologyNeighbors::vectors_to_Vector3d(const std::vector<double> &x,
                                                    const std::vector<double> &y,
                                                    const std::vector<double> &z) {
    if (x.size() == nb_drones && y.size() == nb_drones && z.size() == nb_drones) {
        formation.resize(nb_drones);
        for (size_t idx = 0; idx < nb_drones; ++idx) {
            formation[idx] = Vector3d(x[idx], y[idx], z[idx]);
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "Drone formation provided is not correct!");
    }
}

/**
 * @brief Processes the position of a neighbor drone.
 */
void WeightedTopologyNeighbors::process_neighbor_position(const std::size_t drone_idx,
                                                          const std::size_t neighbor_idx,
                                                          const VehicleLocalPosition &position,
                                                          VehicleLocalPosition neighbor_position,
                                                          WeightedTopologyNeighborsMsg &neighborhood) {
    neighbor_position.x =
        position.x - neighbor_position.x -
        static_cast<float>(formation[drone_idx].x() - formation[neighbor_idx].x());
    neighbor_position.y =
        position.y - neighbor_position.y -
        static_cast<float>(formation[drone_idx].y() - formation[neighbor_idx].y());
    neighbor_position.z =
        position.z - neighbor_position.z -
        static_cast<float>(formation[drone_idx].z() - formation[neighbor_idx].z());

    neighbor_position.vx = position.vx - neighbor_position.vx;
    neighbor_position.vy = position.vy - neighbor_position.vy;
    neighbor_position.vz = position.vz - neighbor_position.vz;

    neighborhood.neighbors_position.emplace_back(neighbor_position);
    neighborhood.neighbors_ids.emplace_back(neighbor_idx);
    prcs_neighborhood[neighbor_idx] = prcs[neighbor_idx];
}

/**
 * @brief Processes the neighborhood based on the positions of the neighbor drones.
 */
void WeightedTopologyNeighbors::process_neighborhood(const std::size_t drone_idx,
                                                     WeightedTopologyNeighborsMsg &neighborhood) {
    if (!neighborhood.neighbors_position.empty()) {
        if (leaders[drone_idx]) {
            prcs[drone_idx] = 1;
        } else {
            const auto min_prc_neighborhood{prcs_neighborhood.minCoeff() + 1};
            prcs[drone_idx] = static_cast<std::size_t>(std::copysign(1u, min_prc_neighborhood)) *
                              std::min(min_prc_neighborhood, nb_drones);
            prcs_neighborhood.setConstant(nb_drones);
        }
    } else {
        // اگر همسایه ای نیست، می‌توانیم prcs[drone_idx] را مقداردهی کنیم
        prcs[drone_idx] = nb_drones;  // یا مقدار مناسبی که شما می‌خواهید
    }
}

/**
 * @brief Enriches the neighborhood message with weights calculated based on the PRCS.
 */
void WeightedTopologyNeighbors::enrich_neighborhood(WeightedTopologyNeighborsMsg &neighborhood) {
    if (neighborhood.neighbors_ids.empty()) {
        return;
    }

    const double sum_inverse_prcs = std::accumulate(
        neighborhood.neighbors_ids.begin(), neighborhood.neighbors_ids.end(), 0.0,
        [this](double sum, auto id) {
            return sum + 1.0 / static_cast<double>(prcs[id]);
        });

    Weights weights;
    weights.reserve(neighborhood.neighbors_ids.size());

    for (auto id : neighborhood.neighbors_ids) {
        weights.emplace_back((1.0 / static_cast<double>(prcs[id])) / sum_inverse_prcs);
    }

    neighborhood.set__weights(weights);
}

}  // namespace Neighborhood

/**
 * @brief Main function to initialize and spin the node.
 */
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Neighborhood::WeightedTopologyNeighbors>());
    rclcpp::shutdown();
    return 0;
}
