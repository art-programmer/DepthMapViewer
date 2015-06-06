#ifndef PANORAMA_H_
#define PANORAMA_H_

/*
  Panorama has both RGB and D information. Both information are stored
  as a 2D array with a simple cylindrical mapping. The resolutions of
  the RGB and D data are different. The RGB and D cameras do not share
  the same optical center, but the D information was mapped to the RGB
  camera space, and one needs not worry about this issue in using this
  class.

  There are several different coordinate systems to be understood in
  using this class: global/local/image/depth. Global is the global 3D
  coordinate frame, which is the same as the global coordinate system
  in floorplan.h. The local coordinate frame is centered at the
  panorama(RGBD) center, and aligned with the XYZ axes of the device.
  Image coordinate system defines the 2D space for RGBD data. The
  rescaled version of this is the depth coordinate system. The
  conversion between the image and the depth systems can be given by
  "RGBToDepth and DepthToRGB". The conversion between the image and
  the global coordinate systems can be given by Project and Unproject
  functions. The conversions between the global and the local
  coordinate systems may not be often necessary, but are possible via
  GlobalToLocal and LocalToGlobal.


  < Background pixels >

  Each RGB panorama has many unobserved black pixels (holes) at the
  top and the bottom of the image. The following function (void
  Panorama::MakeOnlyBackgroundBlack()) guarantees that only the holes
  have the black pixel colors (0, 0, 0). All the other black pixels
  are changed to (1, 1, 1). After this function, one can perform the
  hole testing by looking at the RGB color, being black or not.
  
  < Example >

  FileIO file_io("../some_data_directory");

  const int kPanoramaID = 3;
  Panorama panorama;
  panorama.Init(file_io, kPanoramaID);
 */

#pragma clang diagnostic ignored "-Woverloaded-virtual"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

#include "file_io.h"

namespace structured_indoor_modeling {

class Panorama {
public:
  Panorama();
  bool Init(const FileIO& file_io, const int scene_index, const int layer_index);

  Eigen::Vector2d Project(const Eigen::Vector3d& global) const;
  Eigen::Vector3d Unproject(const Eigen::Vector2d& pixel,
                            const double distance) const;

  Eigen::Vector2d ProjectToDepth(const Eigen::Vector3d& global) const;

  Eigen::Vector3d GlobalToLocal(const Eigen::Vector3d& global) const;
  Eigen::Vector3d LocalToGlobal(const Eigen::Vector3d& local) const;
  
  const Eigen::Vector3d& GetCenter() const { return center; }

  double GetAverageDistance() const { return average_distance; }
  double GetMaxDepth() const { return max_depth; }
  Eigen::Vector2d RGBToDepth(const Eigen::Vector2d& pixel) const;
  Eigen::Vector2d DepthToRGB(const Eigen::Vector2d& depth_pixel) const;

  void MakeOnlyBackgroundBlack();

  // Performs bilinear interpolation.
  Eigen::Vector3f GetRGB(const Eigen::Vector2d& pixel) const;
  double GetDepth(const Eigen::Vector2d& depth_pixel) const;
  const cv::Mat GetRGBImage() const{return rgb_image;} 
  double GetPhiRange() const;
  double GetPhiPerPixel() const;
  Eigen::Matrix4d GetGlobalToLocal() const;
  Eigen::Matrix4d GetLocalToGlobal() const;
  
  int Width() const { return width; }
  int Height() const { return height; }
  int DepthWidth() const { return depth_width; }
  int DepthHeight() const { return depth_height; }

  bool IsInsideRGB(const Eigen::Vector2d& pixel) const;
  bool IsInsideDepth(const Eigen::Vector2d& depth_pixel) const;

  // Shrink only and suggested for integer shrink ratio (e.g., making a width half or 1/3).
  void Resize(const Eigen::Vector2i& size);

  // void ReleaseMemory();

  // This function needs to be used very carefully. Adjust the center
  // of the panorama at the specified location (e.g., laser scanning
  // center), so that Project and Unproject can be used in the laser
  // scanning coordinate.
  void AdjustCenter(const Eigen::Vector3d& new_center);

  std::vector<int> GetSurfaceIds() const { return surface_ids; }
  double GetSubSampleRatio() const { return depth_map_sub_sample_ratio; }

private:
  void InitDepthValues(const FileIO& file_io, const int scene_index, const int layer_index);
  void InitCameraParameters(const FileIO& file_io, const int panorama);
  void SetGlobalToLocalFromLocalToGlobal();

  void SubSampleDepthMap(const double ratio);
  void GetBoundaryIndices();
  double InterpolateDepthValue(const double x, const double y) const;

  void UpdateSurfaceIds();


  Eigen::Matrix4d global_to_local;
  Eigen::Matrix4d local_to_global;
  Eigen::Vector3d center;
  double phi_range;

  cv::Mat rgb_image;
  std::vector<double> depth_values;

  // Redundant but useful values.
  int width;
  int height;
  int depth_width;
  int depth_height;

  double phi_per_pixel;
  double phi_per_depth_pixel;

  double average_distance;
  double max_depth;
  bool only_background_black;

  std::vector<int> boundary_indices;
  std::vector<int> surface_ids;

  double depth_map_sub_sample_ratio;
};

//----------------------------------------------------------------------
void ReadPanoramas(const FileIO& file_io, std::vector<Panorama>* panoramas);
void ReadPanoramasWithoutDepths(const FileIO& file_io,
                                std::vector<Panorama>* panoramas);
void ReadPanoramaPyramids(const FileIO& file_io,
                          const int num_levels,
                          std::vector<std::vector<Panorama> >* panorama_pyramids);
 
}  // namespace structured_indoor_modeling

#endif  // PANORAMA_H_
