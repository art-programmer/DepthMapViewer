#pragma once

#include <Eigen/Dense>
#include <limits>
#include <vector>

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

struct Configuration;
class Navigation;
class Panorama;

struct BoundingBox {
  BoundingBox() {
    min_xyz = Eigen::Vector3d(std::numeric_limits<double>::max(),
                              std::numeric_limits<double>::max(),
                              std::numeric_limits<double>::max());
    max_xyz = Eigen::Vector3d(- std::numeric_limits<double>::max(),
                              - std::numeric_limits<double>::max(),
                              - std::numeric_limits<double>::max());
  }

  Eigen::Vector3d min_xyz;
  Eigen::Vector3d max_xyz;
};

// All local.
 struct TreeConfiguration {
   // Before bounding box.
   BoundingBox bounding_box;
   // Before center.
   Eigen::Vector3d center;

   // Offset with respect to the node one higher up (local)
   Eigen::Vector3d displacement;
   // Scale.
   double scale;
   
   // For object row.
   int row;
 };

class ViewParameters {
 public:
  ViewParameters();
  void Init(const double aspect_ratio);

  Eigen::Vector3d TransformRoom(const Vector3d& global,
                                const int room,
                                const double progress,
                                const double animation,
                                const Eigen::Vector3d& max_vertical_shift) const;
  
  Eigen::Vector3d TransformObject(const Vector3d& global,
                                  const int room,
                                  const int object,
                                  const double progress,
                                  const double animation,
                                  const Eigen::Vector3d& room_max_vertical_shift,
                                  const Eigen::Vector3d& vertical_object_top,
                                  const Eigen::Vector3d& vertical_object_bottom) const;

  Eigen::Vector3d TransformFloorplan(const Vector3d& global,
                                     const double air_to_tree_progress,
                                     const double animation,
                                     const Eigen::Vector3d& max_vertical_shift,
                                     const double max_shrink_scale) const;
  
  const Eigen::Vector3d& GetCenter() const { return center; }
  const Eigen::Vector3d GetObjectCenter(const int room, const int object) const;

  double GetAverageDistance() const { return average_distance; }
  double GetAirHeight() const { return air_height; }
  double GetFloorplanHeight() const { return floorplan_height; }
  double GetAverageFloorHeight() const { return average_floor_height; }
  double GetAverageCeilingHeight() const { return average_ceiling_height; }
  const Eigen::Vector3d& GetZAxis() const { return z_axis; }
  
  // double GetFloorplanHeight() const { return floorplan_height; }
  double GetVerticalFloorplan() const { return vertical_floorplan; }
  double GetVerticalIndoorPolygon() const { return vertical_indoor_polygon; }
  double GetVerticalTopBoundary() const { return - diameter / aspect_ratio / 6.0; }
  double GetVerticalBottomBoundary() const { return - diameter / aspect_ratio / 1.9; }

  double GetFloorplanScale() const { return floorplan_scale; }

  void SetOffsetDirection(const Eigen::Vector3d& view_direction,
                          Eigen::Vector3d* offset_direction) const;
  void SetBoundaries(const Eigen::Vector3d& offset_direction,
                     std::vector<std::vector<Eigen::Vector3d> >* boundaries) const;
  void SetLines(const Eigen::Vector3d& offset_direction,
                std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> >* top_lines,
                std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> >* bottom_lines) const;

  int GetObjectRow(const int room,
                   const int object) const { return object_configurations[room][object].row; }
  static double SetAnimationAlpha(const double animation);
  const static int kMaxNumRows;
  
  
  //???
  // private:
  void InitAxes();
  void InitCenter();
  void InitBoundingBoxes();
  void InitAirFloorplanViewpoints();
  void InitTreeConfigurationCenter();
  void SetPolygonScale();
  void SetRoomDisplacements();
  void SetObjectDisplacements();
  
  Eigen::Vector3d GlobalToLocal(const Eigen::Vector3d& global) const {
    const Eigen::Vector3d& diff = global - center;
    return Eigen::Vector3d(x_axis.dot(diff), y_axis.dot(diff), z_axis.dot(diff));
  }
  Eigen::Vector3d LocalToGlobal(const Eigen::Vector3d& local) const {
    return local[0] * x_axis + local[1] * y_axis + local[2] * z_axis + center;
  }
  Eigen::Vector3d GlobalToLocalNormal(const Eigen::Vector3d& global) const {
    return Eigen::Vector3d(x_axis.dot(global), y_axis.dot(global), z_axis.dot(global));
  }
  Eigen::Vector3d LocalToGlobalNormal(const Eigen::Vector3d& local) const {
    return local[0] * x_axis + local[1] * y_axis + local[2] * z_axis;
  }
  
  // Angle of viewing in the air.
  // Field of view.

  double aspect_ratio;

  // 3D distance corresponding to the frame width.
  double diameter;
  
  double air_height;
  double floorplan_height;

  // Best ground_center for air.
  Eigen::Vector3d best_ground_center;
  Eigen::Vector3d best_start_directions_for_air[2];
  Eigen::Vector3d best_start_directions_for_floorplan[2];
  
  // Average distance.
  double average_distance;
  double average_floor_height;
  double average_ceiling_height;
  
  //----------------------------------------------------------------------
  // Configurations.
  //----------------------------------------------------------------------
  // global.
  Eigen::Vector3d center;
  Eigen::Vector3d x_axis;
  Eigen::Vector3d y_axis;
  Eigen::Vector3d z_axis;
  BoundingBox bounding_box;
  
  // Displacement is from the current room location.
  std::vector<TreeConfiguration> room_configurations;
  // Displacement is from the moved room.
  std::vector<TreeConfiguration> floor_configurations;
  // Displacement is from the moved room.
  std::vector<std::vector<TreeConfiguration> > wall_configurations;
  // Displacement is from the moved room.  
  std::vector<std::vector<TreeConfiguration> > object_configurations;

  //----------------------------------------------------------------------
  double vertical_floorplan;
  double vertical_indoor_polygon;
  int num_rows;
  double floorplan_scale;
  double indoor_polygon_scale;

  friend class Navigation;
};
  
}  // namespace structured_indoor_modeling
