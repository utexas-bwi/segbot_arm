#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <std_msgs/Time.h>

#ifndef GUARD_action_h
#define GUARD_action_h

class Action {
    public:
        // Struct for the joint_states for a temporal bin
        struct joint_state {
            double vel;
            double pos;
            double eff;
        };

        // Struct for a bin of an action
        struct bin {
            std::vector<joint_state> joints;
            geometry_msgs::Pose pose;
            ros::Time time;
        };

        // List types
        typedef std::vector<joint_state> joint_list;
        typedef std::vector<geometry_msgs::Pose> pose_list;
        typedef std::vector<bin> bin_list;
        typedef std::vector<Action> action_list;

        // List iterator types
        typedef joint_list::const_iterator joint_cit;
        typedef pose_list::const_iterator pose_cit;
        typedef bin_list::const_iterator bin_cit;
        typedef action_list::const_iterator action_cit;

        /**
         * Builds an action given a string with the values for the action
         * and the label for the action
         */
        Action(const std::string&);

        /**
         * Builds an action given the vector of poses and joint_states
         * recorded for the action
         */
        Action(const std::vector<geometry_msgs::Pose>&, const joint_list&,
                const std::vector<ros::Time>&);

        std::string get_label() const;

        std::vector<bin> get_data() const;

        /**
         * Calculates the distance between this Action and another Action
         * Returns the distance
         */
        double get_dist(const Action&) const;

        void set_label(const std::string&);

        void print(std::ofstream&) const;

    private:
        std::string label;
        std::vector<bin> data;
        std::vector<ros::Time> times;

        void print_pose(std::ofstream&, const geometry_msgs::Pose&) const;

        bool is_num(char c);

        /**
         * Splits a vector of joint_states into the temporal bins
         * Returns the vector of temporal bins
         */
        joint_list split_bins(const joint_list&);

        /**
         * Builds the temporal bin given the bin number and vectors of Poses
         * and joint_states
         */
        bin build_bin(int, const std::vector<geometry_msgs::Pose>&,
                const joint_list&, const std::vector<ros::Time>&);

        /**
         * Splits a string into a vector of double values and returns
         * the string label at the end of the line. Returns the vector of
         * values through a reference to the vector
         */
        std::string split_line(const std::string&, std::vector<double>&);

        /**
         * Gets the joint_state from a vector of values
         */
        joint_state get_joint_state(const std::vector<double>&, int);

        /**
         * Gets the Pose from a vector of values
         */
        geometry_msgs::Pose get_pose(const std::vector<double>&, int);

        double joint_dist_sum(const bin&, const bin&) const;

        /**
         * Calculates the distance between two joint_states
         * Returns the distance
         */
        double joint_dist(const joint_state&, const joint_state&) const;

        /**
         * Calculates the euclidean distance between two Points
         * Returns the distance
         */
        double euclidean_dist(const geometry_msgs::Point&, const geometry_msgs::Point&) const;

        /**
         * Calculates the distance between two Quaternions
         * Returns the distance
         */
        double quarterion_dist(const geometry_msgs::Quaternion&, const geometry_msgs::Quaternion&) const;

};

#endif
