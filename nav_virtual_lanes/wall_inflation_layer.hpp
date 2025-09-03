#ifndef NAV_VIRTUAL_LANES__WALL_INFLATION_LAYER_HPP_
#define NAV_VIRTUAL_LANES__WALL_INFLATION_LAYER_HPP_

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

namespace nav_virtual_lanes
{
class WallInflationLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  WallInflationLayer() = default;
  virtual ~WallInflationLayer() = default;

  void onInitialize() override;
  void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
};

}  // namespace nav_virtual_lanes

#endif  // NAV_VIRTUAL_LANES__WALL_INFLATION_LAYER_HPP_
