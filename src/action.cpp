#include "action.h"
#include <cmath>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#define NUM_JOINTS 8
#define NUM_BINS 10

using std::ofstream;
using std::endl;
using std::string;
using std::vector;
using geometry_msgs::Pose;

Action::Action(const string& line) {
    // Getting the label and values
    vector<double> values;
    label = split_line(line, values);

    // Looping through all the values
    int index = 0;
    while (index < values.size()) {
        // Creating the bin
        bin b;

        // Getting the time
        b.time = ros::Time(values[index++]);

        // Building the joint_states
        for (int i = index; i < 24; i += 3) {
            b.joints.push_back(get_joint_state(values, i));
        }

        // Getting the pose
        b.pose = get_pose(values, index++);

        // Adding to the action
        data.push_back(b);
    }
}

Action::Action(const vector<Pose>& poses, const joint_list& joints,
        const vector<ros::Time>& times) {
    // Blank label for unlabeled action
    label = "";

    // Building the bins
    for (int bin = 0; bin < NUM_BINS; bin++) {
        data.push_back(build_bin(bin, poses, joints, times));
    }
}

string Action::get_label() const {
    return label;
}

void Action::set_label(const string& new_label) {
    label = new_label;
}

void Action::print(ofstream& os) const {
    // Adding to dataset file
    for (bin_cit bin_it = data.begin(); bin_it != data.end(); bin_it++) {
        // Writing the time
        os << bin_it->time.toSec() << ",";

        // Writing the joint_states
        for (joint_cit joint_it = bin_it->joints.begin();
                joint_it != bin_it->joints.end(); joint_it++) {
            os << joint_it->vel << "," << joint_it->pos << ","
               << joint_it->eff << ",";
        }

        // Writing the pose
        print_pose(os, bin_it->pose);
    }

    // Writing the label
    os << label << endl;
}

void Action::print_pose(ofstream& os, const Pose& pose) const {
    os << pose.position.x << "," << pose.position.y << "," << pose.position.z
       << "," << pose.orientation.x << "," << pose.orientation.y << ","
       << pose.orientation.z << "," << pose.orientation.w;
}

bool Action::is_num(char c) {
    return isdigit(c) || c == '.' || c == 'e' || c== '+' || c == '-';
}

string Action::split_line(const string& line, vector<double>& values) {
    // Iterating over entire line
    string::const_iterator begin = line.begin();
    while (is_num(*begin)) {
        // Moving end down until at next non-digit character
        string::const_iterator end = begin;
        while (is_num(*end)) {
            end++;
        }

        // Building string from iterators and pushing double value to vector
        string val(begin, end);
        values.push_back(atof(val.c_str()));

        // Moving begin to one past end
        begin = end + 1;
    }

    // Getting the classification from the remaining characters
    string classification(begin, line.end());

    return classification;
}

Action::bin Action::build_bin(int bin_num, const vector<Pose>& poses,
        const joint_list& joints, const vector<ros::Time>& times) {
    bin b;

    // Getting the pose
    b.pose = poses[bin_num];

    // Getting the time
    b.time = times[bin_num];

    // Getting the joints
    int offset = bin_num * NUM_BINS;
    for (int i = 0; i < NUM_JOINTS; i++) {
        b.joints.push_back(joints[i + offset]);
    }

    return b;
}

Action::joint_state Action::get_joint_state(const vector<double>& values, int i) {
    joint_state state;
    state.vel = values[i];
    state.pos = values[i + 1];
    state.eff = values[i + 2];

    return state;
}

Pose Action::get_pose(const std::vector<double>& values, int i) {
    Pose pose;

    // Getting the position
    pose.position.x = values[i];
    pose.position.y = values[i + 1];
    pose.position.z = values[i + 2];

    // Getting the orientation
    pose.orientation.x = values[i + 3];
    pose.orientation.y = values[i + 4];
    pose.orientation.z = values[i + 5];
    pose.orientation.w = values[i + 6];

    return pose;
}

double Action::joint_dist_sum(const bin& recorded_bin, const bin& this_bin) const {
    double sum = 0.0;

    for (int i = 0; i < recorded_bin.joints.size(); i++) {
        sum += joint_dist(recorded_bin.joints[i], this_bin.joints[i]);
    }

    return sum;
}

/**
 * Calculates the distance between two joint_states
 * Returns the distance
 */ 
double Action::joint_dist(const joint_state& data, const joint_state& recorded)  const {
    double dvel = pow(recorded.vel - data.vel, 2);
    double dpos = pow(recorded.pos - data.pos, 2);
    double deff = pow(recorded.eff - data.eff, 2);
            
    return sqrt(dvel + dpos + deff);
}

/**
 * Calculates the euclidean distance between two Points
 * Returns the distance
 */
double Action::euclidean_dist(const geometry_msgs::Point& data, const geometry_msgs::Point& action) const {
	double dx = pow(action.x - data.x, 2);
	double dy = pow(action.y - data.y, 2);
	double dz = pow(action.z - data.z, 2);
	double dist = sqrt(dx + dy + dz);

	// Returning the sum of the distances
	return dist;
}

/**
 * Calculates the distance between two Quaternions
 * Returns the distance
 */
double Action::quarterion_dist(const geometry_msgs::Quaternion& c, const geometry_msgs::Quaternion& d) const {
    Eigen::Vector4f dv;
 	dv[0] = d.w; dv[1] = d.x; dv[2] = d.y; dv[3] = d.z;
	Eigen::Matrix<float, 3,4> inv;
	inv(0,0) = -c.x; inv(0,1) = c.w; inv(0,2) = -c.z; inv(0,3) = c.y;
  	inv(1,0) = -c.y; inv(1,1) = c.z; inv(1,2) = -c.w; inv(1,3) = -c.x;
	inv(2,0) = -c.z; inv(2,1) = -c.y; inv(2,2) = -c.x; inv(2,3) = c.w;

    Eigen::Vector3f m = inv * dv * -2.0;
    return m.norm();
}

vector<Action::bin> Action::get_data() const {
    return data;
}


double Action::get_dist(const Action& currentAction) const {
	double sumOfAll = 0.0;
	
	// Within each bin, add up the joint dist, euclidean dist, and quaternion dist.
    vector<bin> current_bins = currentAction.get_data();
    vector<bin> this_bins = this->get_data();

    for (int i = 0; i < NUM_BINS; i++) {
		sumOfAll = joint_dist_sum(current_bins[i], this_bins[i]) + euclidean_dist(current_bins[i].pose.position, this_bins[i].pose.position) + quarterion_dist(current_bins[i].pose.orientation, this_bins[i].pose.orientation);
    }
       // Then get an average of the 10 bins and return it.
	double average = sumOfAll / 10.0;

	return average;
}
