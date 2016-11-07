/*
 * sbpl_adaptive_grid3D.h
 *
 *  Created on: Nov 4, 2014
 *      Author: Kalin Gochev
 */

#ifndef _SBPL_ADAPTIVE_GRID3D_H_
#define _SBPL_ADAPTIVE_GRID3D_H_

// system includes
#include <leatherman/utils.h>
#include <ros/ros.h>
#include <sbpl_adaptive/macros.h>
#include <smpl/occupancy_grid.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <sbpl_adaptive_components/sbpl_adaptive_grid.h>

namespace adim {

SBPL_CLASS_FORWARD(AdaptiveGrid3D)

class AdaptiveGrid3D : public AdaptiveGrid
{
public:

    static const int InvalidDim = 0;

    AdaptiveGrid3D(const sbpl::OccupancyGridPtr& grid);

    ~AdaptiveGrid3D();

    /// \name (Voxel) Grid Functionality
    ///@{
    void getDimensions(int &sizeX, int &sizeY, int &sizeZ) const;

    bool isInBounds(int gx, int gy, int gz) const;
    bool isInBounds(double wx, double wy, double wz) const;

    const AdaptiveGridCell &getCell(int gx, int gy, int gz) const;
    const AdaptiveGridCell &getCell(double wx, double wy, double wz) const;

    double resolution() const { return oc_grid_->getResolution(); }

    void world2grid(
        double wx, double wy, double wz,
        size_t& gx, size_t& gy, size_t& gz) const;

    void grid2world(
        size_t gx, size_t gy, size_t gz,
        double& wx, double& wy, double& wz) const;
    ///@}

    /// return true if the bit was actually toggled
    bool enableDimPlanning(int gx, int gy, int gz, int dimID);
    bool disableDimPlanning(int gx, int gy, int gz, int dimID);
    bool enableDimTracking(int gx, int gy, int gz, int dimID);
    bool disableDimTracking(int gx, int gy, int gz, int dimID);
    bool enableDim(int gx, int gy, int gz, int dimID, bool mode);
    bool disableDim(int gx, int gy, int gz, int dimID, bool mode);

    bool enableNearDimPlanning(int gx, int gy, int gz, int dimID);
    bool disableNearDimPlanning(int gx, int gy, int gz, int dimID);
    bool enableNearDimTracking(int gx, int gy, int gz, int dimID);
    bool disableNearDimTracking(int gx, int gy, int gz, int dimID);
    bool enableNearDim(int gx, int gy, int gz, int dimID, bool mode);
    bool disableNearDim(int gx, int gy, int gz, int dimID, bool mode);

    bool dimEnabledPlanning(int gx, int gy, int gz, int dimID) const;
    bool dimEnabledTracking(int gx, int gy, int gz, int dimID) const;
    bool dimEnabled(int gx, int gy, int gz, int dimID, bool mode) const;

    void addPlanningSphere(
        const std::vector<int> &coord,
        int dimID,
        int rad,
        int near_rad,
        std::vector<adim::Position3D> &modCells);

    void addPlanningSphere(
        adim::AdaptiveSphere3D sphere,
        std::vector<adim::Position3D> &modCells);

    void setTrackingMode(
        const std::vector<std::vector<int>> &tunnel_centers,
        const std::vector<int> &costsToGoal,
        std::vector<adim::Position3D> &modCells);

    void setTrackingMode(
        const std::vector<adim::AdaptiveSphere3D> &tunnel,
        std::vector<adim::Position3D> &modCells);

    int getCellPlanningDim(double wx, double wy, double wz) const;
    int getCellTrackingDim(double wx, double wy, double wz) const;
    int getCellDim(bool bTrackMode, size_t x, size_t y, size_t z) const;

    unsigned int getCellCostToGoal(double wx, double wy, double wz) const;

    visualization_msgs::MarkerArray getVisualizations(
        std::string ns_prefix,
        int throttle = 1,
        double scale = -1);

    visualization_msgs::Marker getAdaptiveGridVisualization(
        std::string ns_prefix,
        int throttle = 1,
        double scale = -1);

    visualization_msgs::Marker getCostToGoalGridVisualization(
        std::string ns_prefix,
        int throttle = 1,
        double scale = 1);

    /// \name Required Public Functions From AdaptiveGrid
    ///@{

    void reset();

    void clearAllSpheres();

    void setPlanningMode();

    void setTrackingMode(
        const std::vector<std::vector<int>> &tunnel_centers,
        const std::vector<int> &costsToGoal);

    void addPlanningSphere(
        const std::vector<int> &coord,
        int dimID,
        int rad,
        int near_rad);

    int getCellPlanningDim(const std::vector<int> &gcoord) const;

    int getCellTrackingDim(const std::vector<int> &gcoord) const;

    unsigned int getCellCostToGoal(const std::vector<int> &coord) const;

    ///@}

private:

    std::vector<int> grid_sizes_;
    std::vector<std::vector<int>> spheres_;

    // used to keep track of state type (LD, NearLD, HD)
    std::vector<std::vector<std::vector<AdaptiveGridCell>>> grid_;

    int max_dimID_;
    unsigned int max_costToGoal_;

    sbpl::OccupancyGridPtr oc_grid_;

    static double getDist2(int x1, int y1, int z1, int x2, int y2, int z2);

    static double getDist(int x1, int y1, int z1, int x2, int y2, int z2);

    void addSphere(
        bool bTrackMode,
        size_t x,
        size_t y,
        size_t z,
        int rad,
        int near_rad,
        int dimID,
        unsigned int costToGoal,
        std::vector<adim::Position3D> &modCells);

    bool setCellDim(bool bTrackMode, size_t x, size_t y, size_t z, int dimID);

    bool setCellNearDim(bool bTrackMode, size_t x, size_t y, size_t z, int dimID);

    void addTrackingSphere(
        const std::vector<int> &coords,
        int dimID,
        int rad,
        int near_rad,
        int costToGoal,
        std::vector<adim::Position3D> &modCells);

    void addTrackingSphere(
        adim::AdaptiveSphere3D sphere,
        std::vector<adim::Position3D> &modCells);

    void getOverlappingSpheres(
        size_t x, size_t y, size_t z,
        int dimID,
        std::vector<std::vector<int>> &spheres);

    /// \name Required Protected Functions From AdaptiveGrid
    ///@{

    void resetTrackingGrid();

    void addTrackingSphere(
        const std::vector<int> &coords,
        int dimID,
        int rad,
        int near_rad,
        int costToGoal);

    void setCellPlanningDim(const std::vector<int> &coord, int dimID);

    void setCellTrackingDim(const std::vector<int> &coord, int dimID);

    void setCellCostToGoal(
        const std::vector<int> &coord,
        unsigned int costToGoal);

    ///@}
};

inline
bool AdaptiveGrid3D::enableDimPlanning(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

    int prev_dims = grid_[gx][gy][gz].pDimID;
    grid_[gx][gy][gz].pDefaultDimID |= (1 << dimID);
    grid_[gx][gy][gz].pDimID |= (1 << dimID);
    max_dimID_ = std::max(max_dimID_, dimID);
    return grid_[gx][gy][gz].pDimID != prev_dims;
}

inline
bool AdaptiveGrid3D::disableDimPlanning(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

    int prev_dims = grid_[gx][gy][gz].pDimID;
    grid_[gx][gy][gz].pDimID &= ~(1 << dimID);
    return grid_[gx][gy][gz].pDimID != prev_dims;
}

inline
bool AdaptiveGrid3D::enableDimTracking(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

    int prev_dims = grid_[gx][gy][gz].tDimID;
    grid_[gx][gy][gz].tDimID |= (1 << dimID);
    max_dimID_ = std::max(max_dimID_, dimID);
    return grid_[gx][gy][gz].tDimID != prev_dims;
}

inline
bool AdaptiveGrid3D::disableDimTracking(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

    int prev_dims = grid_[gx][gy][gz].tDimID;
    grid_[gx][gy][gz].tDimID &= ~(1 << dimID);
    return grid_[gx][gy][gz].tDimID != prev_dims;
}

inline
bool AdaptiveGrid3D::enableNearDimPlanning(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

    int prev_dims = grid_[gx][gy][gz].pNearDimID;
    grid_[gx][gy][gz].pDefaultDimID |= (1 << dimID);
    grid_[gx][gy][gz].pNearDimID |= (1 << dimID);
    max_dimID_ = std::max(max_dimID_, dimID);
    return grid_[gx][gy][gz].pNearDimID != prev_dims;
}

inline
bool AdaptiveGrid3D::disableNearDimPlanning(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

    int prev_dims = grid_[gx][gy][gz].pNearDimID;
    grid_[gx][gy][gz].pNearDimID &= ~(1 << dimID);
    return grid_[gx][gy][gz].pNearDimID != prev_dims;
}

inline
bool AdaptiveGrid3D::enableNearDimTracking(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

    int prev_dims = grid_[gx][gy][gz].tNearDimID;
    grid_[gx][gy][gz].tNearDimID |= (1 << dimID);
    max_dimID_ = std::max(max_dimID_, dimID);
    return grid_[gx][gy][gz].tNearDimID != prev_dims;
}

inline
bool AdaptiveGrid3D::disableNearDimTracking(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

    int prev_dims = grid_[gx][gy][gz].tNearDimID;
    grid_[gx][gy][gz].tNearDimID &= ~(1 << dimID);
    return grid_[gx][gy][gz].tNearDimID != prev_dims;
}

inline
bool AdaptiveGrid3D::enableNearDim(int gx, int gy, int gz, int dimID, bool mode)
{
    if (mode) {
        return enableNearDimTracking(gx, gy, gz, dimID);
    } else {
        return enableNearDimPlanning(gx, gy, gz, dimID);
    }
}

inline
bool AdaptiveGrid3D::disableNearDim(int gx, int gy, int gz, int dimID, bool mode)
{
    if (mode) {
        return disableNearDimTracking(gx, gy, gz, dimID);
    } else {
        return disableNearDimPlanning(gx, gy, gz, dimID);
    }
}

inline
bool AdaptiveGrid3D::enableDim(int gx, int gy, int gz, int dimID, bool mode)
{
    if (mode) {
        return enableDimTracking(gx, gy, gz, dimID);
    } else {
        return enableDimPlanning(gx, gy, gz, dimID);
    }
}

inline
bool AdaptiveGrid3D::disableDim(int gx, int gy, int gz, int dimID, bool mode)
{
    if (mode) {
        return disableDimTracking(gx, gy, gz, dimID);
    } else {
        return disableDimPlanning(gx, gy, gz, dimID);
    }
}

inline
bool AdaptiveGrid3D::dimEnabledPlanning(int gx, int gy, int gz, int dimID) const
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }
    return grid_[gx][gy][gz].pDimID & (1 << dimID);
}

inline
bool AdaptiveGrid3D::dimEnabledTracking(int gx, int gy, int gz, int dimID) const
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }
    return grid_[gx][gy][gz].tDimID & (1 << dimID);
}

inline
bool AdaptiveGrid3D::dimEnabled(int gx, int gy, int gz, int dimID, bool mode) const
{
    if (mode) {
        return dimEnabledPlanning(gx, gy, gz, dimID);
    } else {
        return dimEnabledTracking(gx, gy, gz, dimID);
    }
}

inline
void AdaptiveGrid3D::addPlanningSphere(
    const std::vector<int> &coord,
    int dimID,
    int rad,
    int near_rad,
    std::vector<adim::Position3D> &modCells)
{
    addSphere(false, coord[0], coord[1], coord[2], rad, near_rad, dimID, INFINITECOST, modCells);
}

inline
void AdaptiveGrid3D::addPlanningSphere(
    adim::AdaptiveSphere3D sphere,
    std::vector<adim::Position3D> &modCells)
{
    size_t gx, gy, gz;
    world2grid(sphere.x,sphere.y,sphere.z,gx,gy,gz);
    int r = round(sphere.rad / oc_grid_->getResolution());
    int nr = round(sphere.near_rad / oc_grid_->getResolution());
    addSphere(false, gx, gy, gz, r, nr, sphere.dimID, INFINITECOST, modCells);
}

inline
void AdaptiveGrid3D::getDimensions(int &sizeX, int &sizeY, int &sizeZ) const
{
    sizeX = grid_sizes_[0];
    sizeY = grid_sizes_[1];
    sizeZ = grid_sizes_[2];
}

inline
const AdaptiveGridCell &AdaptiveGrid3D::getCell(int gx, int gy, int gz) const
{
    if (!isInBounds(gx, gy, gz)) {
        SBPL_ERROR("Coordinates out of bounds %d, %d, %d", gx, gy, gz);
        throw SBPL_Exception();
    } else {
        return grid_[gx][gy][gz];
    }
}

inline
const AdaptiveGridCell &AdaptiveGrid3D::getCell(double wx, double wy, double wz) const
{
    size_t gcoordx, gcoordy, gcoordz;
    world2grid(wx, wy, wz, gcoordx, gcoordy, gcoordz);
    return getCell((int)gcoordx, (int)gcoordy, (int)gcoordz);
}

inline
int AdaptiveGrid3D::getCellPlanningDim(double wx, double wy, double wz) const
{
    size_t gcoordx, gcoordy, gcoordz;
    world2grid(wx,wy,wz,gcoordx,gcoordy,gcoordz);
    return this->getCellPlanningDim({(int)gcoordx, (int)gcoordy, (int)gcoordz});
}

inline
unsigned int AdaptiveGrid3D::getCellCostToGoal(double wx, double wy, double wz) const
{
    size_t gcoordx, gcoordy, gcoordz;
    world2grid(wx,wy,wz,gcoordx,gcoordy,gcoordz);
    return this->getCellCostToGoal({(int)gcoordx, (int)gcoordy, (int)gcoordz});
}

inline
int AdaptiveGrid3D::getCellTrackingDim(double wx, double wy, double wz) const
{
    size_t gcoordx, gcoordy, gcoordz;
    world2grid(wx,wy,wz,gcoordx,gcoordy,gcoordz);
    return this->getCellTrackingDim({(int)gcoordx, (int)gcoordy, (int)gcoordz});
}

inline
void AdaptiveGrid3D::setTrackingMode(
    const std::vector<std::vector<int>> &tunnel_centers,
    const std::vector<int> &costsToGoal)
{
    std::vector<adim::Position3D> modCells;
    setTrackingMode(tunnel_centers, costsToGoal, modCells);
}

inline
void AdaptiveGrid3D::addPlanningSphere(
    const std::vector<int> &coord,
    int dimID,
    int rad,
    int near_rad)
{
    std::vector<adim::Position3D> modCells;
    addSphere(false, coord[0], coord[1], coord[2], rad, near_rad, dimID, INFINITECOST, modCells);
}

inline
bool AdaptiveGrid3D::isInBounds(int gx, int gy, int gz) const
{
    return oc_grid_->isInBounds(gx, gy, gz);
}

inline
bool AdaptiveGrid3D::isInBounds(double wx, double wy, double wz) const
{
    size_t gx, gy, gz;
    world2grid(wx, wy, wz, gx, gy, gz);
    return isInBounds((int)gx, (int)gy, (int)gz);
}

inline
void AdaptiveGrid3D::addTrackingSphere(
    const std::vector<int> &coords,
    int dimID,
    int rad,
    int near_rad,
    int costToGoal,
    std::vector<adim::Position3D> &modCells)
{
    addSphere(true, coords[0], coords[1], coords[2], rad, near_rad, dimID, costToGoal, modCells);
}

inline
void AdaptiveGrid3D::addTrackingSphere(
    adim::AdaptiveSphere3D sphere,
    std::vector<adim::Position3D> &modCells)
{
    size_t gx, gy, gz;
    world2grid(sphere.x, sphere.y, sphere.z, gx, gy, gz);
    int r = round(sphere.rad / oc_grid_->getResolution());
    int nr = round(sphere.near_rad / oc_grid_->getResolution());
    addSphere(true, gx, gy, gz, r, nr, sphere.dimID, sphere.costToGoal, modCells);
}

inline
void AdaptiveGrid3D::addTrackingSphere(
    const std::vector<int> &coords,
    int dimID,
    int rad,
    int near_rad,
    int costToGoal)
{
    std::vector<adim::Position3D> modCells;
    addSphere(true, coords[0], coords[1], coords[2], rad, near_rad, dimID, costToGoal, modCells);
}

} // namespace adim

#endif
