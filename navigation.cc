#include <Eigen/Geometry>
#include <iostream>
#include "navigation.h"
#include "view_parameters.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

namespace {

  /*
Eigen::Vector3d ComputeAirDirectionFromPanorama(const Eigen::Vector3d& panorama_direction,
                                                const double air_height,
                                                const double air_angle) {
  Vector3d direction = panorama_direction.normalized();
  // Lift up by the angle.
  direction += -tan(air_angle)* Vector3d(0, 0, 1);
  direction *= air_height / tan(air_angle);

  return direction;
}
  */

Eigen::Vector3d ComputePanoramaDirectionFromAir(const Eigen::Vector3d& air_direction,
                                                const double distance) {
  Vector3d direction = air_direction;
  const int kZIndex = 2;
  direction[kZIndex] = 0.0;
  direction.normalize();
  direction *= distance;
  return direction;
}

Eigen::Vector3d ComputePanoramaDirectionFromFloorplan(const Eigen::Vector3d& floorplan_direction,
                                                      const double distance) {
  Vector3d direction = floorplan_direction;
  const int kZIndex = 2;
  direction[kZIndex] = 0.0;
  direction.normalize();
  direction *= distance;
  return direction;
}
  
void RobustifyDirection(const Eigen::Vector3d& lhs, Eigen::Vector3d* rhs) {
  const double length = (lhs.norm() + rhs->norm()) / 2.0;
  const double move_length = length * 0.01;

  Vector3d lhs_normalized = lhs.normalized();
  Vector3d rhs_normalized = rhs->normalized();

  const double kAlmostOpposite = -0.98;
  if (lhs_normalized.dot(rhs_normalized) <= kAlmostOpposite) {
    // Add a small offset.
    Vector3d orthogonal(lhs_normalized[1], lhs_normalized[0], 0.0);
    orthogonal.normalize();
    *rhs += orthogonal * move_length;
  }
}

}  // namespace

void CameraPanoramaTour::GetIndexWeightPairs(const double progress,
                                             int index_pair[2],
                                             int panorama_index_pair[2],
                                             double weight_pair[2]) const {
  const double index = progress * (indexes.size() - 1);
  index_pair[0] = static_cast<int>(floor(index));
  index_pair[1] = min(index_pair[0] + 1, (int)indexes.size() - 1);
  
  panorama_index_pair[0] = indexes[index_pair[0]];
  panorama_index_pair[1] = indexes[index_pair[1]];

  weight_pair[1] = index - index_pair[0];
  weight_pair[0] = 1.0 - weight_pair[1];
}

Eigen::Vector3d CameraPanoramaTour::GetCenter(const double progress) const {
  int index_pair[2];
  int panorama_index_pair[2];
  double weight_pair[2];
  GetIndexWeightPairs(progress, index_pair, panorama_index_pair, weight_pair);
  return weight_pair[0] * centers[index_pair[0]] + weight_pair[1] * centers[index_pair[1]];
}

Eigen::Vector3d CameraPanoramaTour::GetDirection(const double progress) const {
  int index_pair[2];
  int panorama_index_pair[2];
  double weight_pair[2];
  GetIndexWeightPairs(progress, index_pair, panorama_index_pair, weight_pair);
  Vector3d direction = weight_pair[0] * directions[index_pair[0]] + weight_pair[1] * directions[index_pair[1]];

  if (direction.norm() == 0.0) {
    cerr << "zero direction vector." << endl;
    exit (1);
  }
  
  return direction;
}

Navigation::Navigation(const ViewParameters& view_parameters)
  : view_parameters(view_parameters) {
}

Vector3d Navigation::GetCenter() const {
  switch (camera_status) {
  case kPanorama: {
    return camera_panorama.start_center;
  }
  case kPanoramaTransition: {
    const double weight_start = ProgressInverse();
    const double weight_end = 1.0 - weight_start;
    return weight_start * camera_panorama.start_center +
      weight_end * camera_panorama.end_center;
  }
  case kAir:
  case kTree:
  case kTreeToAirTransition:
  case kAirToTreeTransition: {
    return camera_air.GetCenter();
  }
  case kAirTransition:
  case kTreeTransition: {
    const Vector3d direction = GetDirection();
    return camera_air.ground_center - direction;
  }
  case kFloorplan: {
    return camera_floorplan.GetCenter();
  }  
  case kFloorplanTransition: {
    const Vector3d direction = GetDirection();
    return camera_floorplan.ground_center - direction;
  }  
  case kPanoramaToAirTransition: {
    const double weight_start = ProgressInverse();
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_in_transition.camera_panorama.start_center +
      weight_end * (camera_in_transition.camera_air.GetCenter());
  }
  case kAirToPanoramaTransition: {
    const double weight_start = ProgressInverse();
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * (camera_in_transition.camera_air.GetCenter()) +
      weight_end * camera_in_transition.camera_panorama.start_center;
  }
  case kPanoramaToFloorplanTransition: {
    const double weight_start = ProgressInverse();
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_in_transition.camera_panorama.start_center +
      weight_end * (camera_in_transition.camera_floorplan.GetCenter());
  }
  case kFloorplanToPanoramaTransition: {
    const double weight_start = ProgressInverse();
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * (camera_in_transition.camera_floorplan.GetCenter()) +
      weight_end * camera_in_transition.camera_panorama.start_center;
  }
  case kAirToFloorplanTransition: {
    const double weight_start = ProgressInverse();
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * (camera_in_transition.camera_air.GetCenter()) +
      weight_end * (camera_in_transition.camera_floorplan.GetCenter());
  }
  case kFloorplanToAirTransition: {
    const double weight_start = ProgressInverse();
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * (camera_in_transition.camera_floorplan.GetCenter()) +
      weight_end * (camera_in_transition.camera_air.GetCenter());
  }    
  case kPanoramaTour: {
    return camera_panorama_tour.GetCenter(1.0 - ProgressInverse());
  }
  }
}

Vector3d Navigation::GetDirection() const {
  switch (camera_status) {
  case kPanorama: {
    return camera_panorama.start_direction;
  }
  case kPanoramaTransition: {
    // Interpolation.
    const double weight_start = (cos(camera_panorama.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    const Vector3d direction = weight_start * camera_panorama.start_direction +
      weight_end * camera_panorama.end_direction;
    const Vector3d normalized_direction = direction.normalized();
    if (normalized_direction.norm() == 0.0) {
      cout << "Internal zero " << camera_panorama.start_direction << endl
           << camera_panorama.end_direction << endl
           << weight_start << ' ' << weight_end << endl;
      exit (1);
    }
    if (std::isnan(normalized_direction[0]) ||
        std::isnan(normalized_direction[1]) ||
        std::isnan(normalized_direction[2])) {
      cout << "Internal nan " << camera_panorama.start_direction << endl
           << camera_panorama.end_direction << endl
           << weight_start << ' ' << weight_end << endl;
      exit (1);
    }
    
    return normalized_direction;
  }
  case kAir:
  case kTree:
  case kTreeToAirTransition:
  case kAirToTreeTransition: {
    return camera_air.start_direction;
  }
  case kAirTransition:
  case kTreeTransition: {
  }
  case kFloorplan: {
    return camera_floorplan.start_direction;
  }
  case kFloorplanTransition: {
  }
    
  case kPanoramaToAirTransition: {
    const double weight_start =
      (cos(camera_in_transition.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_in_transition.camera_panorama.start_direction +
      weight_end * camera_in_transition.camera_air.start_direction;
  }
  case kAirToPanoramaTransition: {
    const double weight_start =
      (cos(camera_in_transition.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_in_transition.camera_air.start_direction +
      weight_end * camera_in_transition.camera_panorama.start_direction;
  }
  case kPanoramaToFloorplanTransition: {
    const double weight_start =
      (cos(camera_in_transition.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_in_transition.camera_panorama.start_direction +
      weight_end * camera_in_transition.camera_floorplan.start_direction;
  }
  case kFloorplanToPanoramaTransition: {
    const double weight_start =
      (cos(camera_in_transition.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_in_transition.camera_floorplan.start_direction +
      weight_end * camera_in_transition.camera_panorama.start_direction;
  }
  case kAirToFloorplanTransition: {
    const double weight_start =
      (cos(camera_in_transition.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_in_transition.camera_air.start_direction +
      weight_end * camera_in_transition.camera_floorplan.start_direction;
  }
  case kFloorplanToAirTransition: {
    const double weight_start =
      (cos(camera_in_transition.progress * M_PI) + 1.0) / 2.0;
    const double weight_end = 1.0 - weight_start;
    return
      weight_start * camera_in_transition.camera_floorplan.start_direction +
      weight_end * camera_in_transition.camera_air.start_direction;
  }
    
  case kPanoramaTour: {
    return camera_panorama_tour.GetDirection(1.0 - ProgressInverse());
  }
  }
}

CameraStatus Navigation::GetCameraStatus() const {
  return camera_status;
}

const CameraPanorama& Navigation::GetCameraPanorama() const {
  return camera_panorama;
}

const CameraAir& Navigation::GetCameraAir() const {
  return camera_air;
}

const CameraFloorplan& Navigation::GetCameraFloorplan() const {
  return camera_floorplan;
}
  
const CameraInTransition& Navigation::GetCameraInTransition() const {
  return camera_in_transition;
}

const CameraPanoramaTour& Navigation::GetCameraPanoramaTour() const {
  return camera_panorama_tour;
}

void Navigation::Init() {
//  if (panoramas.empty()) {
//    cerr << "No panoramas." << endl;
//    exit (1);
//  }

  air_floorplan_field_of_view_scale = 1.0;
  
  const int kStartIndex = 0;
  camera_status = kPanorama;
  camera_panorama.start_index = kStartIndex;
//  camera_panorama.start_center = panoramas[kStartIndex].GetCenter();
//  camera_panorama.start_center = panorama.GetCenter();
  camera_panorama.start_center = Vector3d(0, 0, 0);
  camera_panorama.start_direction[2] = 0.0;
  camera_panorama.start_direction.normalize();
//  camera_panorama.start_direction *= panorama.GetAverageDistance();
  camera_panorama.progress = 0.0;
}

void Navigation::Tick() {
  switch (camera_status) {
  case kPanoramaTransition: {
    const double kStepSize = 0.02;
    camera_panorama.progress += kStepSize;
    if (camera_panorama.progress >= 1.0) {
      camera_status = kPanorama;

      camera_panorama.start_index = camera_panorama.end_index;
      camera_panorama.start_center = camera_panorama.end_center;
      camera_panorama.start_direction = camera_panorama.end_direction;
      camera_panorama.progress = 0.0;
    }
    break;
  }
  case kAirTransition:
  case kTreeTransition: {
    const double kStepSize = transition_slow? 0.01 : 0.04;
    camera_air.progress += kStepSize;
    if (camera_air.progress >= 1.0) {
      if (camera_status == kAirTransition)
        camera_status = kAir;
      else
        camera_status = kTree;

      camera_air.start_direction = camera_air.end_direction;
      camera_air.progress = 0.0;
    }
    break;
  }
  case kFloorplanTransition: {
    const double kStepSize = 0.02;
    camera_floorplan.progress += kStepSize;
    if (camera_floorplan.progress >= 1.0) {
      camera_status = kFloorplan;

      camera_floorplan.start_direction = camera_floorplan.end_direction;
      camera_floorplan.progress = 0.0;
    }
    break;
  }
  case kPanoramaToAirTransition: {
    const double kStepSize = 0.02;
    camera_in_transition.progress += kStepSize;
    if (camera_in_transition.progress >= 1.0) {
      camera_status = kAir;

      camera_air = camera_in_transition.camera_air;
      camera_air.progress = 0.0;
    }
    break;
  }
  case kAirToPanoramaTransition: {
    const double kStepSize = 0.02;
    camera_in_transition.progress += kStepSize;
    if (camera_in_transition.progress >= 1.0) {
    }
    break;
  }
  case kPanoramaToFloorplanTransition: {
    const double kStepSize = 0.02;
    camera_in_transition.progress += kStepSize;
    if (camera_in_transition.progress >= 1.0) {
      camera_status = kFloorplan;

      camera_floorplan = camera_in_transition.camera_floorplan;
      camera_floorplan.progress = 0.0;
    }
    break;
  }
  case kFloorplanToPanoramaTransition: {
    const double kStepSize = 0.02;
    camera_in_transition.progress += kStepSize;
    if (camera_in_transition.progress >= 1.0) {
    }
    break;
  }
  case kAirToFloorplanTransition: {
    const double kStepSize = 0.02;
    camera_in_transition.progress += kStepSize;
    if (camera_in_transition.progress >= 1.0) {
      camera_status = kFloorplan;

      camera_floorplan = camera_in_transition.camera_floorplan;
      camera_floorplan.progress = 0.0;
    }
    break;
  }
  case kFloorplanToAirTransition: {
    const double kStepSize = 0.02;
    camera_in_transition.progress += kStepSize;
    if (camera_in_transition.progress >= 1.0) {
      camera_status = kAir;

      camera_air = camera_in_transition.camera_air;
      camera_air.progress = 0.0;
    }
    break;
  }
    
  case kPanoramaTour: {
    double step_size;
    if (camera_panorama_tour.indexes.size() > 4)
      step_size = 0.01;
    else
      step_size = 0.02;
    camera_panorama_tour.progress += step_size;
    if (camera_panorama_tour.progress >= 1.0) {
    }
    break;
  }

  case kTreeToAirTransition: {
    const double kStepSize = 0.01;

    tree_progress += kStepSize;
    if (tree_progress >= 1.0) {
      camera_status = kAir;
      camera_air.progress = 0.0;
    }
    break;
  }
  case kAirToTreeTransition: {
    const double kStepSize = 0.01;

    tree_progress += kStepSize;
    if (tree_progress >= 1.0) {
      camera_status = kTree;
      camera_air.progress = 0.0;
      tree_progress = 0.0;
    }
    break;
  }
    
  default: {
    break;
  }
  }
}

void Navigation::RotatePanorama(const double dx, const double dy) {
  Vector3d zaxis = camera_panorama.start_direction;
  Vector3d yaxis(0, 0, 1);
  Vector3d xaxis = yaxis.cross(zaxis);
  yaxis = zaxis.cross(xaxis);

  if (dx == 0 && dy == 0)
    return;

  xaxis.normalize();
  yaxis.normalize();

  Matrix3d rotation;
  if (dx == 0) {
    rotation = AngleAxisd(-dy, xaxis).toRotationMatrix();
  } else if (dy == 0) {
    rotation = AngleAxisd(dx, yaxis).toRotationMatrix();
  } else {
    rotation = AngleAxisd(-dy, xaxis).toRotationMatrix() * AngleAxisd(dx, yaxis).toRotationMatrix();
  }

  camera_panorama.start_direction = rotation * camera_panorama.start_direction;
}

void Navigation::MoveAir(const Eigen::Vector3d& translation)  {
  camera_air.ground_center += translation;
}

void Navigation::MoveFloorplan(const Eigen::Vector3d& translation)  {
  camera_floorplan.ground_center += translation;
}

bool Navigation::Collide(const int /*from_index*/, const int /*to_index*/) const {
  return false;
}

void Navigation::MoveToPanorama(const int target_panorama_index) {
  cerr << "Move to " << target_panorama_index << endl;
  exit(1);
}

void Navigation::TourToPanorama(const std::vector<int>& indexes) {
  // Find a sequence of paths from camera_panorama.start_index to target_panorama_index.
    cerr << "Tour to " << indexes[indexes.size() - 1] << endl;
    exit(1);
}

void Navigation::MovePanorama(const Vector3d& direction) {
    cerr << "Move Panorama" << endl;
    exit(1);

  const double kMaximumAngle = 60.0 * M_PI / 180.0;
  const double kPerpScale = 4.0;
  const int kInvalid = -1;
  int best_panorama_index = kInvalid;
  double best_distance = 0.0;

  Vector3d along_direction = direction;
  along_direction[2] = 0.0;
  along_direction.normalize();
  Vector3d perp_direction(along_direction[1], -along_direction[0], 0.0);

//  for (int p = 0; p < static_cast<int>(panoramas.size()); ++p) {
//    if (p == camera_panorama.start_index)
//      continue;
//    // Behind.
//    const Vector3d diff = panoramas[p].GetCenter() - camera_panorama.start_center;
    
//    if (diff.normalized().dot(direction.normalized()) <= cos(kMaximumAngle))
//      continue;

//    // Collision check.
//    //????
//    if (Collide(camera_panorama.start_index, p))
//      continue;

//    const double distance = diff.dot(along_direction) + kPerpScale * fabs(diff.dot(perp_direction));
//    if (best_panorama_index == kInvalid || distance < best_distance) {
//      best_panorama_index = p;
//      best_distance = distance;
//    }
//  }

  if (best_panorama_index == kInvalid)
    return;

  MoveToPanorama(best_panorama_index);  
  // const int target_panorama_index = (camera_panorama.start_index + 1) % panoramas.size();
  // MoveToPanorama(target_panorama_index);
}

void Navigation::MoveForwardPanorama() {
  MovePanorama(camera_panorama.start_direction);
}

void Navigation::MoveBackwardPanorama() {
  MovePanorama(-camera_panorama.start_direction);
}

void Navigation::RotatePanorama(const double radian) {
  Matrix3d rotation;
  rotation << cos(radian), -sin(radian), 0.0,
    sin(radian), cos(radian), 0.0,
    0.0, 0.0, 1.0;

  camera_panorama.end_index = camera_panorama.start_index;
  camera_panorama.end_center = camera_panorama.start_center;
  // camera_panorama.end_direction = rotation * camera_panorama.start_direction;
//  camera_panorama.end_direction = RotateInFloorplan(rotation, camera_panorama.start_direction);
  camera_panorama.progress = 0.0;
  camera_status = kPanoramaTransition;

  RobustifyDirection(camera_panorama.start_direction, &camera_panorama.end_direction);
}

void Navigation::RotateAir(const double radian, const bool slow) {
  Matrix3d rotation;
  rotation << cos(radian), -sin(radian), 0.0,
    sin(radian), cos(radian), 0.0,
    0.0, 0.0, 1.0;

  transition_slow = slow;
  
  // camera_air.end_direction = rotation * camera_air.start_direction;
//  camera_air.end_direction = RotateInFloorplan(rotation, camera_air.start_direction);
  camera_air.progress = 0.0;
  if (camera_status == kAir) {
    camera_status = kAirTransition;
  } else if (camera_status == kTree) {
    camera_status = kTreeTransition;
  } else {
    cerr << "Impossible in RotateAir." << endl;
    exit (1);
  }
}

void Navigation::PanoramaToAir() {
  air_floorplan_field_of_view_scale = 1.0;
  
  camera_status = kPanoramaToAirTransition;

  camera_in_transition.camera_panorama = camera_panorama;
  {
    CameraAir& camera_air      = camera_in_transition.camera_air;
    camera_air.ground_center = view_parameters.best_ground_center;
    if (camera_panorama.start_direction.dot(view_parameters.best_start_directions_for_air[0]) >
        camera_panorama.start_direction.dot(view_parameters.best_start_directions_for_air[1]))
      camera_air.start_direction = view_parameters.best_start_directions_for_air[0];
    else
      camera_air.start_direction = view_parameters.best_start_directions_for_air[1];
  }
  
  camera_in_transition.progress = 0.0;  
}

void Navigation::AirToPanorama(const int panorama_index) {
    cerr << "Air to " << endl;
    exit(1);
  camera_status = kAirToPanoramaTransition;
  
  camera_in_transition.camera_air = camera_air;
//  {
//    CameraPanorama& camera_panorama =
//      camera_in_transition.camera_panorama;
//    camera_panorama.start_index     = panorama_index;
//    camera_panorama.start_center    = panoramas[panorama_index].GetCenter();
//    camera_panorama.start_direction =
//      ComputePanoramaDirectionFromAir(camera_air.start_direction,
//                                      panoramas[panorama_index].GetAverageDistance());
//  }
  camera_in_transition.progress = 0.0;  
}

void Navigation::TreeToAir() {
  camera_status = kTreeToAirTransition;
  tree_progress = 0.0; 
}

void Navigation::AirToTree() {
  camera_status = kAirToTreeTransition;
  tree_progress = 0.0;  
}
  
  
double Navigation::ProgressInverse() const {
  switch (camera_status) {
  case kPanoramaTransition:
    return cos(camera_panorama.progress * M_PI) / 2.0 + 1.0 / 2.0;
  case kAirTransition:
    return cos(camera_air.progress * M_PI) / 2.0 + 1.0 / 2.0;
  case kFloorplanTransition:
    return cos(camera_floorplan.progress * M_PI) / 2.0 + 1.0 / 2.0;
  case kPanoramaToAirTransition:
  case kAirToPanoramaTransition:
  case kPanoramaToFloorplanTransition:
  case kFloorplanToPanoramaTransition:
  case kAirToFloorplanTransition:
  case kFloorplanToAirTransition:
    {
    return cos(camera_in_transition.progress * M_PI) / 2.0 + 1.0 / 2.0;
    // sigmoid.
    /*
    const double minimum_value = 1 / (1 + exp(6.0));
    const double maximum_value = 1 / (1 + exp(-6.0));
    const double sigmoid = 1 / (1 + exp(- (6.0 - 12.0 * camera_in_transition.progress)));
    return (sigmoid - minimum_value) / (maximum_value - minimum_value);
    */
    //return 1.0 - sin(camera_in_transition.progress * M_PI / 2);
  }
  case kPanoramaTour:
    return cos(camera_panorama_tour.progress * M_PI) / 2.0 + 1.0 / 2.0;
  default:
    cerr << "Impossible in ProgressInverse." << endl;
    exit (1);
  }
}

double Navigation::GetFieldOfViewInDegrees() const {
  const double kPanoramaFieldOfViewDegrees = 120.0;
//  const double scaled_air_field_of_view_degrees =
//    view_parameters.air_field_of_view_degrees * air_floorplan_field_of_view_scale;
//  const double scaled_floorplan_field_of_view_degrees =
//    view_parameters.floorplan_field_of_view_degrees * air_floorplan_field_of_view_scale;

  switch (camera_status) {
  case kPanorama:
  case kPanoramaTransition:
  case kPanoramaTour: {
    return kPanoramaFieldOfViewDegrees;
  }
  // CameraAir handles the state.
  case kAir:
  case kAirTransition:
  case kTree:
  case kTreeTransition:
  case kTreeToAirTransition:
  case kAirToTreeTransition: {
    return -1; //scaled_air_field_of_view_degrees;
  }
  case kFloorplan:
  case kFloorplanTransition: {
    return -1; //scaled_floorplan_field_of_view_degrees;
  }
//  // CameraInTransition handles the state.
//  case kPanoramaToAirTransition: {
//    return -1; //GetFieldOfViewInTransitionInDegrees(camera_in_transition.camera_panorama.start_center[2],
//                                               camera_in_transition.camera_air.GetCenter()[2],
//                                               kPanoramaFieldOfViewDegrees * M_PI / 180.0,
//                                               scaled_air_field_of_view_degrees * M_PI / 180.0);
//  }
//  case kAirToPanoramaTransition: {
//    return -1; //GetFieldOfViewInTransitionInDegrees(camera_in_transition.camera_air.GetCenter()[2],
//                                               camera_in_transition.camera_panorama.start_center[2],
//                                               scaled_air_field_of_view_degrees * M_PI / 180.0,
//                                               kPanoramaFieldOfViewDegrees * M_PI / 180.0);
//  }
//  case kPanoramaToFloorplanTransition: {
//    return -1; //GetFieldOfViewInTransitionInDegrees(camera_in_transition.camera_panorama.start_center[2],
//                                               camera_in_transition.camera_floorplan.GetCenter()[2],
//                                               kPanoramaFieldOfViewDegrees * M_PI / 180.0,
//                                               scaled_floorplan_field_of_view_degrees * M_PI / 180.0);
//  }
//  case kFloorplanToPanoramaTransition: {
//    return -1; //GetFieldOfViewInTransitionInDegrees(camera_in_transition.camera_floorplan.GetCenter()[2],
//                                               camera_in_transition.camera_panorama.start_center[2],
//                                               scaled_floorplan_field_of_view_degrees * M_PI / 180.0,
//                                               kPanoramaFieldOfViewDegrees * M_PI / 180.0);
//  }
//  case kAirToFloorplanTransition: {
//    return -1; //GetFieldOfViewInTransitionInDegrees(camera_in_transition.camera_air.GetCenter()[2],
//                                               camera_in_transition.camera_floorplan.GetCenter()[2],
//                                               scaled_air_field_of_view_degrees * M_PI / 180.0,
//                                               scaled_floorplan_field_of_view_degrees * M_PI / 180.0);
//  }
//  case kFloorplanToAirTransition: {
//    return -1; //GetFieldOfViewInTransitionInDegrees(camera_in_transition.camera_floorplan.GetCenter()[2],
//                                               camera_in_transition.camera_air.GetCenter()[2],
//                                               scaled_floorplan_field_of_view_degrees * M_PI / 180.0,
//                                               scaled_air_field_of_view_degrees * M_PI / 180.0);
//  }
  default:
      cerr << "Get fielf of view in degrees error." << endl;
      exit(1);
  }
}

double Navigation::GetFieldOfViewInTransitionInDegrees(const double start_height,
                                                       const double end_height,
                                                       const double start_field_of_view,
                                                       const double end_field_of_view) const {
  const double start_height_diff = start_height - view_parameters.average_floor_height;
  const double end_height_diff   = end_height - view_parameters.average_floor_height;

  const double start_size = 1.0 / tan(start_field_of_view / 2.0) / start_height_diff;
  const double end_size   = 1.0 / tan(end_field_of_view / 2.0) / end_height_diff;

  const double start_weight = ProgressInverse();
  const double end_weight = 1.0 - start_weight;

  const double current_height_diff = GetCenter()[2] - view_parameters.average_floor_height;
  const double current_size = start_weight * start_size + end_weight * end_size;
  const double current_field_of_view = atan(1.0 / current_height_diff / current_size) * 2.0;

  return current_field_of_view * 180.0 / M_PI; 
}

}  // namespace structured_indoor_modeling
