#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "ros/ros.h"
#include <cstdlib>
#include <iostream>
#include <vector>

// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class Trajectory
{
private:
    std::vector<cv::Point> list_;
    cv::Vec3b color_;
    unsigned int thickness_;

public:
    /**
     * Construct a new dummy Trajectory.
     */
    //Trajectory() = default;

    /**
     * Construct a new Trajectory.
     * 
     * @param col An Opencv 3d vector for the color of the trajectory in BGR.
     * @param thickness The thickness of the trajectory.
     */
    Trajectory(cv::Vec3b col = cv::Vec3b(0, 0, 255), unsigned int thickness = 1);

    /**
     * Destructor of the Trajectory class.
     */
    ~Trajectory();

    /**
     * Initialize the different points of the trajectory.
     * It destroys the previous defined points of the trajectory.
     * 
     * @param checkpoints A ros service Checkpoints.
     * @param gridmap A gridmap containing the map of the environment.
     * @return True if all points of the checkpoints are in the map, else return false.
     */
    // bool setTrajectory(const planification::Checkpoints &checkpoints, const gridmap_2d::GridMap2D &gridmap);

    /**
     * Display the trajectory on an Opencv matrix.
     * 
     * @param dest The Opencv matrix to display the trajectory on.
     * @return Wether the trajectory is correctly displayed or not.
     */
    bool show(cv::Mat &dest) const;

    /**
     * Show the points of the trajectory in the terminal.
     */
    void print() const;

    /**
     * Get the size of the vector of points.
     * 
     * @return The number of points in the trajectory.
     */
    int size() {return list_.size();}
};

#endif