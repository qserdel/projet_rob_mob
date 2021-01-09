#include "trajectory.h"

Trajectory::Trajectory(cv::Vec3b col, unsigned int thickness) : 
list_(),
color_(col),
thickness_(thickness) {}

Trajectory::~Trajectory()
{
    list_.clear();
}

bool Trajectory::setTrajectory(const planification::Checlpoints &checkpoints, const gridmap_2d::GridMap2D &gridmap)
{
    list_.clear();
    unsigned int mx, my;

    for (const geometry_msgs::Point &pt : cp_list.response.points.points)
    {
        if (gridmap.inMapBounds(pt.x, pt.y))
        {
            gridmap.worldToMap(pt.x, pt.y, mx, my);
            cpt.points.push_back(cv::Point(my, mx));
        }
        else
        {
            list_.clear();
            return false;
        }
    }
    return true;
}

bool Trajectory::displayTraj(cv::Mat &dest) const
{
    if (!dest.empty())
    {
        return false;
    }
    for (int i = 1; i < cpt.len; i++)
    {
        cv::line(dest, cpt.points[i - 1], cpt.points[i], cpt.color, cpt.thickness);
    }
    return true;
}

void Trajectory::print() const
{
    std::cout << "Trajectoire de checkpoints:\n";
    if (list_.size() == 0){
        std::cout << "empty\n";
    }
    else
    {
        for (const cv::Point &pt : cpt.points)
        {
            std::cout << pt << ",";
        }
        std::cout << "\n";
    }
}