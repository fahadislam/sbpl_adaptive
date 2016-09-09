#include <sbpl_adaptive_collision_checking/sbpl_collision_space.h>

namespace adim {

SBPLCollisionSpace::SBPLCollisionSpace(
    std::shared_ptr<adim::SBPLCollisionModel> model,
    const sbpl::OccupancyGridPtr& grid)
:
    model_(model),
    grid_(grid),
    padding_(0.0),
    contact_padding_(0.0)
{
}

SBPLCollisionSpace::~SBPLCollisionSpace()
{
}

void SBPLCollisionSpace::setPadding(double padding)
{
    padding_ = padding;
}

void SBPLCollisionSpace::setContactPadding(double padding)
{
    contact_padding_ = padding;
}

/// \brief Return whether a state is free of collisions with the environment
///
/// The state is in collision with the environment if any collision sphere is
/// in collision with the environment
bool SBPLCollisionSpace::checkCollision(
    const ModelCoords_t &coords,
    double &dist)
{
    std::vector<Sphere> collision_spheres;
    if (!model_->getModelCollisionSpheres(coords, collision_spheres)) {
        ROS_ERROR("[cspace] Failed to get model spheres");
        return false;
    }

    for (const Sphere& s : collision_spheres) {
        ROS_DEBUG("Checking sphere '%s' with radius %0.3f at (%0.3f, %0.3f, %0.3f)", s.name_.c_str(), s.radius, s.v.x(), s.v.y(), s.v.z());
        int x, y, z;
        grid_->worldToGrid(s.v.x(), s.v.y(), s.v.z(), x, y, z);
        if (!grid_->isInBounds(x, y, z)) {
            ROS_DEBUG("Sphere %s out of bounds!", s.name_.c_str());
            dist = 0;
            return false;
        }
        double dist_bounds = grid_->getDistanceToBorder(x, y, z);
        double dist_temp = std::min(grid_->getDistance(x, y, z), dist_bounds);
        if (dist_temp < dist) {
            dist = dist_temp;
        }
        ROS_DEBUG(" -> dist = %0.3f", dist_temp);
        if (dist_temp < s.radius + padding_) {
            ROS_DEBUG("Cell: [%d %d %d] [%.3f %.3f %.3f]", x, y, z, s.v.x(), s.v.y(), s.v.z());
            ROS_DEBUG("Sphere %s in collision! r=%.3f+p=%.3f > d=%.3f", s.name_.c_str(), s.radius, padding_, dist_temp);
            return false;
        }
    }
    return true;
}

/// \brief Return whether a path between two states is free of collisions with
///     the environment
///
/// The path is checked for collisions at discrete points generated by the
/// associated collision model. The path is free of collisions if all
/// intermediate states are free of collision.
bool SBPLCollisionSpace::checkCollision(
    const ModelCoords_t &coords0,
    const ModelCoords_t &coords1,
    int steps,
    double &dist)
{
    //TODO:
    int x, y, z;
    double sum = 0, dist_temp = 100;
    std::vector<Sphere> collision_spheres;

    if (!model_->getModelPathCollisionSpheres(
            coords0, coords1, steps, collision_spheres))
    {
        ROS_ERROR("[cspace] Failed to get model spheres");
        return false;
    }

    for (Sphere s : collision_spheres) {
        grid_->worldToGrid(s.v.x(), s.v.y(), s.v.z(), x, y, z);
        dist_temp = grid_->getDistance(x, y, z);
        if (dist_temp < dist) {
            dist = dist_temp;
        }
        if (dist_temp < s.radius + padding_) {
            //ROS_WARN("Cell: [%d %d %d] [%.3f %.3f %.3f]", x, y, z, s.v.x(), s.v.y(), s.v.z());
            //ROS_WARN("Sphere %s in collision! r=%.3f+p=%.3f > d=%.3f", s.name_.c_str(), s.radius, padding_, dist_temp);
            return false;
        }
    }
    return true;
}

/// \brief Return whether a state is in contact with the environment
///
/// The state is in contact with the environment if all contact spheres are in
/// collision with the environment.
bool SBPLCollisionSpace::checkContact(const ModelCoords_t &coords, double &dist)
{
    int x, y, z;
    double sum = 0, dist_temp = 0;
    std::vector<Sphere> contact_spheres;

    if (!model_->getModelContactSpheres(coords, contact_spheres)) {
        ROS_ERROR("[cspace] Failed to get model spheres");
        return false;
    }

    for (Sphere s : contact_spheres) {
        grid_->worldToGrid(s.v.x(), s.v.y(), s.v.z(), x, y, z);
        dist_temp = grid_->getDistance(x, y, z);
        if (dist_temp > dist) {
            dist = dist_temp;
        }
        if (dist_temp > s.radius + contact_padding_) {
            //ROS_WARN("Sphere %s not in contact! r=%.3f+p=%.3f < d=%.3f", s.name_.c_str(), s.radius, contact_padding_, dist_temp);
            return false;
        }
    }
    return true;
}

/// \brief Return whether a link in the collision model is in contact with the
///     environment
bool SBPLCollisionSpace::checkContact(
    const ModelCoords_t &coords,
    const std::string &link_name,
    double &dist)
{
    std::vector<Sphere> contact_spheres;
    int x, y, z;
    double dist_temp = 0;
    if (!model_->getModelContactSpheres(coords, link_name, contact_spheres)) {
        ROS_ERROR("[cspace] Failed to get model spheres");
        return false;
    }

    for (Sphere s : contact_spheres) {
        grid_->worldToGrid(s.v.x(), s.v.y(), s.v.z(), x, y, z);
        double dist_bounds = grid_->getDistanceToBorder(x, y, z);
        dist_temp = std::min(grid_->getDistance(x, y, z), dist_bounds);
        if (dist_temp > dist) {
            dist = dist_temp;
        }
        if (dist_temp > s.radius + contact_padding_) {
            ROS_DEBUG("Sphere %s for link %s not in contact! r=%.3f+p=%.3f < d=%.3f", s.name_.c_str(), link_name.c_str(), s.radius, contact_padding_, dist_temp);
            return false;
        }
    }
    //all contact spheres in contact!
    return true;
}

/// \brief Return whether a path between two states is in contact with the
///     environment
///
/// The path is checked for contact at discrete points generated by the
/// associated collision model. The path is in contact with the environment if
/// all intermediate states are in contact with the environment.
bool SBPLCollisionSpace::checkContact(
    const ModelCoords_t &coords0,
    const ModelCoords_t &coords1,
    int steps,
    double &dist)
{
    int x, y, z;
    double sum = 0, dist_temp = 0;
    std::vector<Sphere> contact_spheres;

    if (!model_->getModelPathContactSpheres(coords0, coords1, steps,
            contact_spheres)) {
        ROS_ERROR("[cspace] Failed to get model spheres");
        return false;
    }

    for (Sphere s : contact_spheres) {
        grid_->worldToGrid(s.v.x(), s.v.y(), s.v.z(), x, y, z);
        dist_temp = grid_->getDistance(x, y, z);
        if (dist_temp > dist) {
            dist = dist_temp;
        }
        if (dist_temp > s.radius + contact_padding_) {
            //ROS_WARN("Sphere %s not in contact! r=%.3f+p=%.3f < d=%.3f", s.name_.c_str(), s.radius, contact_padding_, dist_temp);
            return false;
        }
    }
    return true;
}

/// \brief Compute the occupied voxels of a state
///
/// The returned voxels are the grid coordinates of each occupied voxel.
/// \return true if the collision model returned valid spheres; false otherwise
bool SBPLCollisionSpace::getModelVoxelsInGrid(
    const ModelCoords_t &coords,
    std::vector<Eigen::Vector3i> &voxels)
{
    std::vector<Sphere> spheres;
    if (!model_->getModelCollisionSpheres(coords, spheres)) {
        return false;
    }
    if (!model_->getModelContactSpheres(coords, spheres)) {
        return false;
    }
    // 1. find the extents of the spheres (axis-aligned bounding box)
    int min_x = 0, max_x = 0, dim_x = 0;
    int min_y = 0, max_y = 0, dim_y = 0;
    int min_z = 0, max_z = 0, dim_z = 0;
    grid_->getGridSize(dim_x, dim_y, dim_z);
    min_x = dim_x;
    min_y = dim_y;
    min_z = dim_z;
    for (Sphere s : spheres) {
        double mn_x = s.v.x() - s.radius;
        double mx_x = s.v.x() + s.radius;
        double mn_y = s.v.y() - s.radius;
        double mx_y = s.v.y() + s.radius;
        double mn_z = s.v.z() - s.radius;
        double mx_z = s.v.z() + s.radius;
        int mins[3];
        int maxs[3];
        grid_->worldToGrid(mn_x, mn_y, mn_z, mins[0], mins[1], mins[2]);
        grid_->worldToGrid(mx_x, mx_y, mx_z, maxs[0], maxs[1], maxs[2]);
        if (mins[0] < min_x) {
            min_x = mins[0];
        }
        if (mins[1] < min_y) {
            min_y = mins[1];
        }
        if (mins[2] < min_z) {
            min_z = mins[2];
        }
        if (maxs[0] > max_x) {
            min_x = mins[0];
        }
        if (maxs[1] > max_y) {
            min_y = mins[1];
        }
        if (maxs[2] > max_z) {
            min_z = mins[2];
        }
    }
    min_x = std::max(0, min_x);
    min_y = std::max(0, min_y);
    min_z = std::max(0, min_z);
    max_x = std::min(dim_x, max_x);
    max_y = std::min(dim_y, max_y);
    max_z = std::min(dim_z, max_z);
    // 2. loop through all the voxels in the bounding box and check if in sphere
    for (int x = min_x; x <= max_x; x++) {
        for (int y = min_y; y <= max_y; y++) {
            for (int z = min_z; z <= max_z; z++) {
                bool bIn = false;
                double wx, wy, wz;
                grid_->gridToWorld(x, y, z, wx, wy, wz);
                Eigen::Vector3d v(wx, wy, wz);
                Eigen::Vector3i vi(x, y, z);
                for (Sphere s : spheres) {
                    Eigen::Vector3d d = s.v - v;
                    if (d.norm() <= s.radius) {
                        bIn = true;
                        break;
                    }
                }
                if (bIn) {
                    voxels.push_back(vi);
                }
            }
        }
    }
    return true;
}

double SBPLCollisionSpace::isValidLineSegment(
    const std::vector<int>& a,
    const std::vector<int>& b,
    const int radius)
{
    bresenham3d_param_t params;
    int nXYZ[3], retvalue = 1;
    double cell_val, min_dist = 100.0;
    CELL3V tempcell;
    std::vector<CELL3V>* pTestedCells = NULL;

    // iterate through the points on the segment
    get_bresenham3d_parameters(a[0], a[1], a[2], b[0], b[1], b[2], &params);
    do {
        get_current_point3d(&params, &(nXYZ[0]), &(nXYZ[1]), &(nXYZ[2]));

        if (!grid_->isInBounds(nXYZ[0], nXYZ[1], nXYZ[2])) {
            return 0;
        }

        cell_val = grid_->getDistance(nXYZ[0], nXYZ[1], nXYZ[2]);
        if (cell_val <= radius) {
            if (pTestedCells == NULL) {
                return cell_val;   //return 0
            }
            else {
                retvalue = 0;
            }
        }

        if (cell_val < min_dist) {
            min_dist = cell_val;
        }

        // insert the tested point
        if (pTestedCells) {
            if (cell_val <= radius) {
                tempcell.bIsObstacle = true;
            }
            else {
                tempcell.bIsObstacle = false;
            }
            tempcell.x = nXYZ[0];
            tempcell.y = nXYZ[1];
            tempcell.z = nXYZ[2];
            pTestedCells->push_back(tempcell);
        }
    }
    while (get_next_point3d(&params));

    if (retvalue) {
        return min_dist;
    }
    else {
        return 0;
    }
}

} // namespace sbpl_adaptive_collision_checking
