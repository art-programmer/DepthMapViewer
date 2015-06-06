#include <limits>
#include <numeric>
#include "view_parameters.h"
#include "configuration.h"
#include <iostream>

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

const int ViewParameters::kMaxNumRows = 3;
  
ViewParameters::ViewParameters() {
}

void ViewParameters::Init(const double aspect_ratio_tmp) {
  aspect_ratio = aspect_ratio_tmp;
  
//  InitAxes();
//  InitCenter();
//  InitBoundingBoxes();
//  InitAirFloorplanViewpoints();
//  InitTreeConfigurationCenter();
//  SetPolygonScale();
//  SetRoomDisplacements();
//  SetObjectDisplacements();
}


void ViewParameters::InitAxes() {
  // Compute best ground_center and start_direction for air.
    cerr << "Init axes" << endl;
    exit(1);

//  Eigen::Vector2d x_range, y_range;
//  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
//    for (int v = 0; v < floorplan.GetNumRoomVertices(room); ++v) {
//      const Eigen::Vector2d local = floorplan.GetRoomVertexLocal(room, v);
//      if (room == 0 && v == 0) {
//        x_range[0] = x_range[1] = local[0];
//        y_range[0] = y_range[1] = local[1];
//      } else {
//        x_range[0] = min(x_range[0], local[0]);
//        x_range[1] = max(x_range[1], local[0]);
//        y_range[0] = min(y_range[0], local[1]);
//        y_range[1] = max(y_range[1], local[1]);
//      }
//    }
//  }

//  if ((x_range[1] - x_range[0]) > (y_range[1] - y_range[0])) {
//    x_axis = floorplan.GetFloorplanToGlobal() * Vector3d(1, 0, 0);
//    y_axis = floorplan.GetFloorplanToGlobal() * Vector3d(0, 1, 0);
//    z_axis = floorplan.GetFloorplanToGlobal() * Vector3d(0, 0, 1);
//  } else {
//    x_axis = floorplan.GetFloorplanToGlobal() * Vector3d(0, 1, 0);
//    y_axis = floorplan.GetFloorplanToGlobal() * Vector3d(1, 0, 0);
//    z_axis = floorplan.GetFloorplanToGlobal() * Vector3d(0, 0, 1);
//  }
}

void ViewParameters::InitAirFloorplanViewpoints() {
  // diameter must be visible in the given field-of-view along air_angle.
}
  
void ViewParameters::InitCenter() {
//  center = Vector3d(0, 0, 0);
//  for (int room = 0; room < floorplan.GetNumRooms(); ++room) {
//    for (int v = 0; v < floorplan.GetNumRoomVertices(room); ++v) {
//      const Eigen::Vector3d floor_vertex = GlobalToLocal(floorplan.GetFloorVertexGlobal(room, v));
//      for (int a = 0; a < 3; ++a) {
//        bounding_box.min_xyz[a] = min(bounding_box.min_xyz[a], floor_vertex[a]);
//        bounding_box.max_xyz[a] = max(bounding_box.max_xyz[a], floor_vertex[a]);
//      }
//      const Eigen::Vector3d ceiling_vertex = GlobalToLocal(floorplan.GetCeilingVertexGlobal(room, v));
//      for (int a = 0; a < 3; ++a) {
//        bounding_box.min_xyz[a] = min(bounding_box.min_xyz[a], ceiling_vertex[a]);
//        bounding_box.max_xyz[a] = max(bounding_box.max_xyz[a], ceiling_vertex[a]);
//      }
//    }
//  }

//  center = LocalToGlobal((bounding_box.min_xyz + bounding_box.max_xyz) / 2.0);
}

void ViewParameters::InitBoundingBoxes() {
}

void ViewParameters::InitTreeConfigurationCenter() {
}

void ViewParameters::SetPolygonScale() {
  // Make the height occupy only 1/3 of the screen.
}
  
void ViewParameters::SetRoomDisplacements() {
  // To allow some margin between items.
}

void ViewParameters::SetObjectDisplacements() {
}

Eigen::Vector3d ViewParameters::TransformRoom(const Vector3d& global,
                                             const int room,
                                             const double progress,
                                             const double animation,
                                             const Eigen::Vector3d& max_vertical_shift) const {
}

Eigen::Vector3d ViewParameters::TransformObject(const Vector3d& global,
                                                const int room,
                                                const int object,
                                                const double progress,
                                                const double animation,
                                                const Eigen::Vector3d& room_max_vertical_shift,
                                                const Eigen::Vector3d& vertical_object_top,
                                                const Eigen::Vector3d& vertical_object_bottom) const {
}
  
Eigen::Vector3d ViewParameters::TransformFloorplan(const Vector3d& global,
                                                  const double air_to_tree_progress,
                                                   const double /* animation */,
                                                  const Eigen::Vector3d& max_vertical_shift,
                                                  const double max_shrink_scale) const {
}

const Eigen::Vector3d ViewParameters::GetObjectCenter(const int room, const int object) const {
}

void ViewParameters::SetOffsetDirection(const Eigen::Vector3d& view_direction,
                                        Eigen::Vector3d* offset_direction) const {
}
  
void ViewParameters::SetBoundaries(const Eigen::Vector3d& offset_direction,
                                   std::vector<std::vector<Eigen::Vector3d> >* boundaries) const {
}

void ViewParameters::SetLines(const Eigen::Vector3d& offset_direction,
                              std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> >* top_lines,
                              std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> >* bottom_lines) const {
}

double ViewParameters::SetAnimationAlpha(const double animation) {
  const double kMargin = 0.075;
  // const double pivots[4] = { 1.0 / 8, 3.0 / 8, 5.0 / 8, 7.0 / 8};
  const double pivots[4] = { 0.0 / 8, 2.0 / 8, 4.0 / 8, 6.0 / 8};
  double diff = 1.0;
  for (int i = 0; i < 4; ++i)
    diff = min(diff, fabs(animation - pivots[i]));
  return max(0.0, min(1.0, 2.0 * (kMargin - diff) / kMargin));
}
  
}  // namespace structured_indoor_modeling
