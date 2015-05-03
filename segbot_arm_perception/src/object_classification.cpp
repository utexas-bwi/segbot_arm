// #include <ros/ros.h>
// #include <ros/package.h>

#include <mlpack/core.hpp>
// #include <mlpack/core/tree/cover_tree.hpp>
// #include <mlpack/methods/neighbor_search/neighbor_search.hpp>

using namespace mlpack;

int num_neighbors = 3;

bool knnClassifier() {
    // arma::mat<double> train_data (4, 4);
    // std::string input_file_name, input_file_path;
    // std::string segbot_arm_perception_path = "/home/bwi/catkin_ws/src/segbot_arm/segbot_arm_perception";

    // // TODO parse data feature vectors to corresponding objects (hash?)
    // // Get file names
    // // ROS_INFO("Train file: ");
    // // std::getline (std::cin, input_file_name);
    // // input_file_name += "/";
    // // ROS_INFO("input_file_name = %s", input_file_name.c_str());
    // while (1);
    // input_file_path = segbot_arm_perception_path + "/feature_data_csv/" + "train.csv";
    // data::Load(input_file_path.c_str(), train_data, true);
    // // input_file_path = segbot_arm_perception_path + "/feature_data_csv/" + "test.csv";
    // // data::Load(input_file_path.c_str(), test_data, true);

    // // Init with reference set and query set
    // neighbor::AllkNN nn (train_data);

    // arma::Col<size_t> neighbors;
    // arma::vec distances;

    // nn.Search(num_neighbors, neighbors, distances);

    // for (int i = 0; i < neighbors.n_elem; i++) {
    //     std::cout << "Nearest neighbor of point " << i << " is point "
    //               << neighbors[i] << " and the distance is " << distances[i] << ".\n";
    // }
}


int main(int argc, char** argv) {
    while (1);
    // // Initialize ROS
	// ros::init (argc, argv, "segbot_arm_button_detector");
	// ros::NodeHandle nh;
    std::cout << "HI" << std::endl;
    while(1);
    knnClassifier();
}
