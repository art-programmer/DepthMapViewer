#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>

#include "panorama.h"

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

Panorama::Panorama() {
  only_background_black = false;
}

bool Panorama::Init(const FileIO& file_io,
                    const int scene_index, const int layer_index) {
//  rgb_image = cv::imread(file_io.GetPanoramaImage(panorama), 1);
//  rgb_image = cv::imread("Data/image.bmp", 1);
    rgb_image = cv::imread(file_io.GetLayerTextureImage(scene_index, layer_index));
  if (rgb_image.cols == 0 && rgb_image.rows == 0) {
    cerr << "Panorama image cannot be loaded: " << file_io.GetLayerTextureImage(scene_index, layer_index) << endl;
    return false;
  }
  width  = rgb_image.cols;
  height = rgb_image.rows;
  InitDepthValues(file_io, scene_index, layer_index);


  ifstream ifstr;
//  ifstr.open(file_io.GetSurfaceIds(panorama));
  ifstr.open(file_io.GetLayerSurfaceIds(scene_index, layer_index).c_str());
  if (!ifstr.is_open()) {
    cerr << "Cannot open surface ids file." << endl;
    exit (1);
  }

  int surface_id_width = -1, surface_id_height = -1;
  ifstr >> surface_id_width >> surface_id_height;
  surface_ids.resize(surface_id_width * surface_id_height);

  for (int y = 0; y < surface_id_height; ++y) {
    for (int x = 0; x < surface_id_width; ++x) {
      ifstr >> surface_ids[y * surface_id_width + x];
    }
  }
  ifstr.close();
//  UpdateSurfaceIds();

//  cout << depth_values[235 * depth_width + 14] << '\t' << depth_values[235 * depth_width + 15] << '\t' << depth_values[236 * depth_width + 14] << '\t' << depth_values[236 * depth_width + 15] << endl;
//  cout << surface_ids[235 * depth_width + 14] << '\t' << surface_ids[235 * depth_width + 15] << '\t' << surface_ids[236 * depth_width + 14] << '\t' << surface_ids[236 * depth_width + 15] << endl;

  InitCameraParameters(file_io, scene_index);
  phi_per_pixel = phi_range / height;
  phi_per_depth_pixel = phi_range / depth_height;
  return true;
}

//bool Panorama::InitWithoutLoadingImages(const FileIO& file_io, const int panorama) {
//  InitCameraParameters(file_io, panorama);
//  phi_per_pixel = phi_range / height;
//  phi_per_depth_pixel = phi_range / depth_height;
//  return true;
//}

//bool Panorama::InitWithoutDepths(const FileIO& file_io, const int panorama) {
//  rgb_image = cv::imread(file_io.GetPanoramaImage(panorama), 1);
////  rgb_image = cv::imread(file_io.GetLayerTextureImage(scene_index, layer_index), 1);
//  if (rgb_image.cols == 0 && rgb_image.rows == 0) {
//    cerr << "Panorama image cannot be loaded: " << file_io.GetPanoramaImage(panorama) << endl;
//    cout << "why" << endl;
//    return false;
//  }
//  width  = rgb_image.cols;
//  height = rgb_image.rows;
//  InitCameraParameters(file_io, panorama);
//  phi_per_pixel = phi_range / height;
//  return true;
//}

Eigen::Vector2d Panorama::Project(const Eigen::Vector3d& global) const {
  const Vector3d local = GlobalToLocal(global);

  // x coordinate.
  double theta = -atan2(local.y(), local.x());
  if (theta < 0.0)
    theta += 2 * M_PI;
  
  double theta_ratio = max(0.0, min(1.0, theta / (2 * M_PI)));
  if (theta_ratio == 1.0)
    theta_ratio = 0.0;

  Vector2d uv;
  uv[0] = theta_ratio * width;
  const double depth = sqrt(local.x() * local.x() +
                            local.y() * local.y());
  double phi = atan2(local.z(), depth);
  const double pixel_offset_from_center = phi / phi_per_pixel;
  // uv[1] = height / 2.0 - pixel_offset_from_center;
  uv[1] = max(0.0, min(height - 1.1, height / 2.0 - pixel_offset_from_center));

  return uv;
}

Eigen::Vector3d Panorama::Unproject(const Eigen::Vector2d& pixel,
                                    const double distance) const {
  const double theta = -2.0 * M_PI * pixel[0] / width;
  const double phi   = (height / 2.0 - pixel[1]) * phi_per_pixel;

  Vector3d local;
//  local[2] = distance * sin(phi);
//  local[0] = distance * cos(phi) * cos(theta);
//  local[1] = distance * cos(phi) * sin(theta);
  double scale = max(width, height);
  local[0] = (pixel[0] - 0.5 * width) * distance / scale;
  local[1] = (0.5 * height - pixel[1]) * distance / scale;
  local[2] = -distance;
//  cout << local[2] << endl;
//  cout << LocalToGlobal(local)(2) << endl;
  return LocalToGlobal(local);
}

Eigen::Vector2d Panorama::ProjectToDepth(const Eigen::Vector3d& global) const {
  const Eigen::Vector2d pixel = Project(global);
  return RGBToDepth(pixel);
}

Eigen::Vector3d Panorama::GlobalToLocal(const Eigen::Vector3d& global) const {
  const Vector4d global4(global[0], global[1], global[2], 1.0);
  const Vector4d local4 = global_to_local * global4;
  return Vector3d(local4[0], local4[1], local4[2]); 
}

Eigen::Vector3d Panorama::LocalToGlobal(const Eigen::Vector3d& local) const {
  const Vector4d local4(local[0], local[1], local[2], 1.0);
  const Vector4d global4 = local_to_global * local4;
  return Vector3d(global4[0], global4[1], global4[2]); 
}

Eigen::Vector2d Panorama::RGBToDepth(const Eigen::Vector2d& pixel) const {
  return Vector2d(pixel[0] * depth_width / width,
                  min(depth_height - 1.1, pixel[1] * depth_height / height));
}

Eigen::Vector2d Panorama::DepthToRGB(const Eigen::Vector2d& depth_pixel) const {
  return Vector2d(min(width - 0.0, depth_pixel[0] * width / depth_width),
                  min(height - 0.0, depth_pixel[1] * height / depth_height));
}

Eigen::Vector3f Panorama::GetRGB(const Eigen::Vector2d& pixel) const {
  if (!IsInsideRGB(pixel)) {
    cerr << "Pixel outside: " << pixel[0] << ' ' << pixel[1] << ' '
         << width << ' ' << height << endl;
    exit(1);
  }
  
  // Bilinear interpolation.
  const int u0 = static_cast<int>(floor(pixel[0]));
  const int v0 = static_cast<int>(floor(pixel[1]));
  const int u1 = u0 + 1;
  int v1 = v0 + 1;
  
  const double weight00 = (u1 - pixel[0]) * (v1 - pixel[1]);
  const double weight01 = (pixel[0] - u0) * (v1 - pixel[1]);
  const double weight10 = (u1 - pixel[0]) * (pixel[1] - v0);
  const double weight11 = (pixel[0] - u0) * (pixel[1] - v0);
  const int u1_corrected = (u1 % width);

  v1 = min(v1, height - 1);
      
  const cv::Vec3b& color00 = rgb_image.at<cv::Vec3b>(v0, u0);
  const cv::Vec3b& color01 = rgb_image.at<cv::Vec3b>(v0, u1_corrected);
  const cv::Vec3b& color10 = rgb_image.at<cv::Vec3b>(v1, u0);
  const cv::Vec3b& color11 = rgb_image.at<cv::Vec3b>(v1, u1_corrected);

  const cv::Vec3b kHole(0, 0, 0);
  if (only_background_black) {
    double weight_sum = 0.0;
    Vector3f color_sum(0, 0, 0);
    if (color00 != kHole) {
      color_sum += Vector3f(weight00 * color00[0],
                            weight00 * color00[1],
                            weight00 * color00[2]);
      weight_sum += weight00;
    }
    if (color01 != kHole) {
      color_sum += Vector3f(weight01 * color01[0],
                            weight01 * color01[1],
                            weight01 * color01[2]);
      weight_sum += weight01;
    }
    if (color10 != kHole) {
      color_sum += Vector3f(weight10 * color10[0],
                            weight10 * color10[1],
                            weight10 * color10[2]);
      weight_sum += weight10;
    }
    if (color11 != kHole) {
      color_sum += Vector3f(weight11 * color11[0],
                            weight11 * color11[1],
                            weight11 * color11[2]);
      weight_sum += weight11;
    }
    if (weight_sum == 0.0)
      return Vector3f(0, 0, 0);
    else
      return color_sum / weight_sum;
  } else {
    return Vector3f((weight00 * color00[0] + weight01 * color01[0] +
                     weight10 * color10[0] + weight11 * color11[0]),
                    (weight00 * color00[1] + weight01 * color01[1] +
                     weight10 * color10[1] + weight11 * color11[1]),
                    (weight00 * color00[2] + weight01 * color01[2] +
                     weight10 * color10[2] + weight11 * color11[2]));
  }
}

double Panorama::GetDepth(const Eigen::Vector2d& depth_pixel) const {
  if (!IsInsideDepth(depth_pixel)) {
    cerr << "Depth pixel outside: " << depth_pixel[0] << ' ' << depth_pixel[1] << ' '
         << depth_width << ' ' << depth_height << endl;
    exit (1);
  }

  return InterpolateDepthValue(depth_pixel[0], depth_pixel[1]);
  
  // Bilinear interpolation.
  const int u0 = static_cast<int>(floor(depth_pixel[0]));
  const int v0 = static_cast<int>(floor(depth_pixel[1]));
  int u1 = u0 + 1;
  int v1 = v0 + 1;
  
  const double weight00 = (u1 - depth_pixel[0]) * (v1 - depth_pixel[1]);
  const double weight01 = (depth_pixel[0] - u0) * (v1 - depth_pixel[1]);
  const double weight10 = (u1 - depth_pixel[0]) * (depth_pixel[1] - v0);
  const double weight11 = (depth_pixel[0] - u0) * (depth_pixel[1] - v0);

//  const int u1_corrected = (u1 % depth_width);
  u1 = min(u1, depth_width - 1);
  v1 = min(v1, depth_height - 1);
  
  if (depth_values[v0 * depth_width + u0] < 0 ||
      depth_values[v0 * depth_width + u1] < 0 ||
      depth_values[v1 * depth_width + u0] < 0 ||
      depth_values[v1 * depth_width + u1] < 0)
      return -1;
  return
    weight00 * depth_values[v0 * depth_width + u0] +
    weight01 * depth_values[v0 * depth_width + u1] +
    weight10 * depth_values[v1 * depth_width + u0] +
    weight11 * depth_values[v1 * depth_width + u1];
}

double Panorama::GetPhiRange() const {
  return phi_range;
}

double Panorama::GetPhiPerPixel() const {
  return phi_per_pixel;
}  

Eigen::Matrix4d Panorama::GetGlobalToLocal() const {
  return global_to_local;
}
  
Eigen::Matrix4d Panorama::GetLocalToGlobal() const {
  return local_to_global;
}
  
bool Panorama::IsInsideRGB(const Eigen::Vector2d& pixel) const {
  if (pixel[0] < 0.0 || width <= pixel[0] ||
      pixel[1] < 0.0 || height - 1 < pixel[1]) {
    return false;
  } else {
    return true;
  }
}

bool Panorama::IsInsideDepth(const Eigen::Vector2d& depth_pixel) const {
  if (depth_pixel[0] < -0.5 || depth_width - 0.5 < depth_pixel[0] ||
      depth_pixel[1] < -0.5 || depth_height - 0.5 < depth_pixel[1]) {
    return false;
  } else {
    return true;
  }
}

void Panorama::Resize(const Eigen::Vector2i& size) {
  const int new_width = size[0];
  const int new_height = size[1];
  
  const int x_scale = width  / new_width;
  const int y_scale = height / new_height;

  const int new_depth_width = depth_width / x_scale;
  const int new_depth_height = depth_height / y_scale;
  
  if (only_background_black) {
    const cv::Vec3b kHole(0, 0, 0);
    cv::Mat new_rgb_image(new_height, new_width, CV_8UC3);
    
    for (int y = 0; y < new_height; ++y) {
      const int start_y = y * y_scale;
      const int end_y   = (y + 1) * y_scale;
      for (int x = 0; x < new_width; ++x) {
        const int start_x = x * x_scale;
        const int end_x   = (x + 1) * x_scale;

        Vector3f color(0, 0, 0);
        int denom = 0;
        for (int j = start_y; j < end_y; ++j) {
          for (int i = start_x; i < end_x; ++i) {
            const cv::Vec3b v3b = rgb_image.at<cv::Vec3b>(j, i);
            if (v3b != kHole) {
              color += Vector3f(v3b[0], v3b[1], v3b[2]);
              ++denom;
            }
          }
        }
        if (denom != 0)
          color /= denom;
        new_rgb_image.at<cv::Vec3b>(y, x) =
          cv::Vec3b(static_cast<int>(round(color[0])),
                    static_cast<int>(round(color[1])),
                    static_cast<int>(round(color[2])));
      }
    }
    rgb_image = new_rgb_image;
  } else {
    cv::Mat new_rgb_image;
    cv::resize(rgb_image, new_rgb_image, cv::Size(new_width, new_height));
    rgb_image = new_rgb_image;
  }

  // Resize depth.
  {
    const double kInvalid = -1.0;
    vector<double> new_depth_values(new_depth_width * new_depth_height);
    
    for (int y = 0; y < new_depth_height; ++y) {
      const int start_y = y * y_scale;
      const int end_y   = (y + 1) * y_scale;
      for (int x = 0; x < new_depth_width; ++x) {
        const int start_x = x * x_scale;
        const int end_x   = (x + 1) * x_scale;
        
        double sum_depth = 0.0;
        int denom = 0;
        for (int j = start_y; j < end_y; ++j) {
          for (int i = start_x; i < end_x; ++i) {
            const double depth = depth_values[j * depth_width + i];
            if (depth != kInvalid) {
              sum_depth += depth;
              ++denom;
            }
          }
        }
        if (denom != 0)
          sum_depth /= denom;
        else
          sum_depth = kInvalid;
        new_depth_values[y * new_depth_width + x] = sum_depth;
      }
    }
    depth_values.swap(new_depth_values);
  }

  width  = new_width;
  height = new_height;
    
  phi_per_pixel = phi_range / height;

  depth_width = new_depth_width;
  depth_height = new_depth_height;

  phi_per_depth_pixel = phi_range / depth_height;
}

void Panorama::AdjustCenter(const Eigen::Vector3d& new_center) {
  const Vector3d translation = new_center - center;
  Matrix4d adjustment;
  adjustment.setIdentity();
  for (int i = 0; i < 3; ++i)
    adjustment(i, 3) = translation[i];
  
  local_to_global = local_to_global * adjustment;
  
  SetGlobalToLocalFromLocalToGlobal();
}
  
  /*
void Panorama::ReleaseMemory() {
  rgb_image.release();
  vector<double>().swap(depth_values);
}
  */

void Panorama::InitDepthValues(const FileIO& file_io,
                               const int scene_index,
                               const int layer_index) {
  // Interpolate myself.
  ifstream ifstr;
//  ifstr.open(file_io.GetDepthPanorama(panorama));
  ifstr.open(file_io.GetLayerDepthValues(scene_index, layer_index).c_str());
  if (!ifstr.is_open()) {
    cerr << "Cannot open depth file." << endl;
    exit (1);
  }

  string header;
  double min_depth;
  ifstr >> depth_width >> depth_height >> min_depth >> max_depth;

  depth_values.resize(depth_width * depth_height);
  
  average_distance = 0.0;
  int num_positive_depths = 0;
  int index = 0;
  for (int y = 0; y < depth_height; ++y) {
    for (int x = 0; x < depth_width; ++x, ++index) {
      ifstr >> depth_values[index];
      if (depth_values[index] > 0) {
          average_distance += depth_values[index];
          num_positive_depths++;
      }
    }
  }
  ifstr.close();

  average_distance /= num_positive_depths;

  depth_map_sub_sample_ratio = depth_width / width;

//  SubSampleDepthMap(depth_map_sub_sample_ratio);
//  GetBoundaryIndices();
}
  
void Panorama::InitCameraParameters(const FileIO& file_io,
                                    const int panorama) {
//  const string buffer = file_io.GetPanoramaToGlobalTransformation(panorama);

//  ifstream ifstr;
//  ifstr.open(buffer.c_str());
//  string stmp;
//  ifstr >> stmp;
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 4; ++x)
      local_to_global(y, x) = 0;
  
//  for (int y = 0; y < 4; ++y) {
//    for (int x = 0; x < 4; ++x)
//      ifstr >> local_to_global(y, x);
//  }
  for (int i = 0; i < 4; ++i)
      local_to_global(i, i) = 1;

  for (int y = 0; y < 3; ++y)
    center(y) = local_to_global(y, 3);

  SetGlobalToLocalFromLocalToGlobal();

  phi_range = M_PI;

//  ifstr >> phi_range;
  
//  ifstr.close();
}

void Panorama::SetGlobalToLocalFromLocalToGlobal() {
  const Matrix3d rotation = local_to_global.block(0, 0, 3, 3);
  global_to_local.block(0, 0, 3, 3) = rotation.transpose();
  global_to_local.block(0, 3, 3, 1) =
    - rotation.transpose() * local_to_global.block(0, 3, 3, 1);
  global_to_local(3, 0) = 0.0;
  global_to_local(3, 1) = 0.0;
  global_to_local(3, 2) = 0.0;
  global_to_local(3, 3) = 1.0;
}  

void Panorama::MakeOnlyBackgroundBlack() {
  // Background must be in [0, kTopRato] or [kBottomRatio, 1].
  const double kTopRatio = 170 / 1500.0;
  const double kBottomRatio = 1250 / 1500.0;

  const int top_height = static_cast<int>(round(height * kTopRatio));
  const int bottom_height = static_cast<int>(round(height * kBottomRatio));
  const int bottom_margin = 20;

  // Starting from the top most or the bottom most pixel, identify the black pixels.
  for (int x = 0; x < width; ++x) {
    int top_index, bottom_index;
    for (top_index = 0; top_index < top_height; ++top_index) {
      if (rgb_image.at<cv::Vec3b>(top_index, x) == cv::Vec3b(0, 0, 0))
        continue;
      else
        break;
    }
    for (bottom_index = height - 1; bottom_index > bottom_height; --bottom_index) {
      if (rgb_image.at<cv::Vec3b>(bottom_index, x) == cv::Vec3b(0, 0, 0))
        continue;
      else
        break;
    }

    for(; bottom_index > bottom_height - bottom_margin; --bottom_index){
	 rgb_image.at<cv::Vec3b>(bottom_index, x) = cv::Vec3b(0,0,0);
    }
    
    // Make black pixels between top_index and bottom_index to (1, 1, 1).
    for (int y = top_index; y < bottom_index; ++y) {
      if (rgb_image.at<cv::Vec3b>(y, x) == cv::Vec3b(0, 0, 0))
        rgb_image.at<cv::Vec3b>(y, x) = cv::Vec3b(1, 1, 1);
    }
  }
  only_background_black = true;
}

double Panorama::InterpolateDepthValue(const double x, const double y) const
{
    double interpolated_depth_value = -1;

    const double SHIFT_THRESHOLD = 0;

    int index_1 = static_cast<int>(y) * depth_width + static_cast<int>(x);
    int index_2 = static_cast<int>(y) * depth_width + static_cast<int>(x + 1);
    int index_3 = static_cast<int>(y + 1) * depth_width + static_cast<int>(x);
    int index_4 = static_cast<int>(y + 1) * depth_width + static_cast<int>(x + 1);
    double depth_value_1 = (x >= 0 && y >= 0) ? depth_values[index_1] : -1;
    double depth_value_2 = (x < depth_width - 1 && y >= 0) ? depth_values[index_2] : -1;
    double depth_value_3 = (x >= 0 && y < depth_height - 1) ? depth_values[index_3] : -1;
    double depth_value_4 = (x < depth_width - 1 && y < depth_height - 1) ? depth_values[index_4] : -1;

    int num_valid_depth_values = 0;
    if (depth_value_1 > 0)
        num_valid_depth_values++;
    if (depth_value_2 > 0)
        num_valid_depth_values++;
    if (depth_value_3 > 0)
        num_valid_depth_values++;
    if (depth_value_4 > 0)
        num_valid_depth_values++;

    switch (num_valid_depth_values) {
    case 4: {
        double area_1 = (static_cast<int>(x + 1) - x) * (static_cast<int>(y + 1) - y);
        double area_2 = (x - static_cast<int>(x)) * (static_cast<int>(y + 1) - y);
        double area_3 = (static_cast<int>(x + 1) - x) * (y - static_cast<int>(y));
        double area_4 = (x - static_cast<int>(x)) * (y - static_cast<int>(y));
        interpolated_depth_value = depth_value_1 * area_1 + depth_value_2 * area_2 + depth_value_3 * area_3 + depth_value_4 * area_4;
        break;
    }
    case 3: {
        if (depth_value_1 > 0 && depth_value_2 > 0 && depth_value_3 > 0) {
            double area_2 = 0.5 * (x - static_cast<int>(x));
            double area_3 = 0.5 * (y - static_cast<int>(y));
            if (area_2 + area_3 <= 0.5 + SHIFT_THRESHOLD) {
                double area_1 = 0.5 - area_2 - area_3;
                interpolated_depth_value = depth_value_1 * area_1 + depth_value_2 * area_2 + depth_value_3 * area_3;
                interpolated_depth_value *= 2;
            }
        } else if (depth_value_2 > 0 && depth_value_3 > 0 && depth_value_4 > 0) {
            double area_2 = 0.5 * (static_cast<int>(y + 1) - y);
            double area_3 = 0.5 * (static_cast<int>(x + 1) - x);
            if (area_2 + area_3 <= 0.5 + SHIFT_THRESHOLD) {
                double area_4 = 0.5 - area_2 - area_3;
                interpolated_depth_value = depth_value_2 * area_2 + depth_value_3 * area_3 + depth_value_4 * area_4;
                interpolated_depth_value *= 2;
            }
        } else if (depth_value_1 > 0 && depth_value_3 > 0 && depth_value_4 > 0) {
            double area_1 = 0.5 * (static_cast<int>(y + 1) - y);
            double area_4 = 0.5 * (x - static_cast<int>(x));
            if (area_1 + area_4 <= 0.5 + SHIFT_THRESHOLD) {
                double area_3 = 0.5 - area_1 - area_4;
                interpolated_depth_value = depth_value_1 * area_1 + depth_value_3 * area_3 + depth_value_4 * area_4;
                interpolated_depth_value *= 2;
            }
        } else if (depth_value_1 > 0 && depth_value_2 > 0 && depth_value_4 > 0) {
            double area_1 = 0.5 * (static_cast<int>(x + 1) - x);
            double area_4 = 0.5 * (y - static_cast<int>(y));
            if (area_1 + area_4 <= 0.5 + SHIFT_THRESHOLD) {
                double area_2 = 0.5 - area_1 - area_4;
                interpolated_depth_value = depth_value_1 * area_1 + depth_value_2 * area_2 + depth_value_4 * area_4;
                interpolated_depth_value *= 2;
            }
        }
        break;
    }
    case 2: {
        if (depth_value_1 > 0 && depth_value_2 > 0) {
            if (y - static_cast<int>(y) <= 0.5 + SHIFT_THRESHOLD) {
                double length_1 = (static_cast<int>(x + 1) - x);
                double length_2 = (x - static_cast<int>(x));
                interpolated_depth_value = depth_value_1 * length_1 + depth_value_2 * length_2;
            }
        } else if (depth_value_1 > 0 && depth_value_3 > 0) {
            if (x - static_cast<int>(x) <= 0.5 + SHIFT_THRESHOLD) {
                double length_1 = (static_cast<int>(y + 1) - y);
                double length_3 = (y - static_cast<int>(y));
                interpolated_depth_value = depth_value_1 * length_1 + depth_value_3 * length_3;
            }
        } else if (depth_value_2 > 0 && depth_value_4 > 0) {
            if (static_cast<int>(x + 1) - x <= 0.5 + SHIFT_THRESHOLD) {
                double length_2 = (static_cast<int>(y + 1) - y);
                double length_4 = (y - static_cast<int>(y));
                interpolated_depth_value = depth_value_2 * length_2 + depth_value_4 * length_4;
            }
        } else if (depth_value_3 > 0 && depth_value_4 > 0) {
            if (static_cast<int>(y + 1) - y <= 0.5 + SHIFT_THRESHOLD) {
                double length_3 = (static_cast<int>(x + 1) - x);
                double length_4 = (x - static_cast<int>(x));
                interpolated_depth_value = depth_value_3 * length_3 + depth_value_4 * length_4;
            }
        } else if (depth_value_1 > 0 && depth_value_4 > 0) {
            if (abs((x - static_cast<int>(x)) - (y - static_cast<int>(y))) <= 0.5 + SHIFT_THRESHOLD) {
                double length_1 = 1 - ((x - static_cast<int>(x)) + (y - static_cast<int>(y))) / 2;
                double length_4 = ((x - static_cast<int>(x)) + (y - static_cast<int>(y))) / 2;
                interpolated_depth_value = depth_value_1 * length_1 + depth_value_4 * length_4;
            }
        } else if (depth_value_2 > 0 && depth_value_3 > 0) {
            if (abs((x - static_cast<int>(x)) + (y - static_cast<int>(y)) - 1) <= 0.5 + SHIFT_THRESHOLD) {
                double length_2 = 0.5 + ((x - static_cast<int>(x)) - (y - static_cast<int>(y))) / 2;
                double length_3 = 0.5 - ((x - static_cast<int>(x)) - (y - static_cast<int>(y))) / 2;
                interpolated_depth_value = depth_value_2 * length_2 + depth_value_3 * length_3;
            }
        }
        break;
    }
    case 1: {
        if (depth_value_1 > 0) {
            if ((x - static_cast<int>(x)) <= 0.5 + SHIFT_THRESHOLD && (y - static_cast<int>(y)) <= 0.5 + SHIFT_THRESHOLD)
                interpolated_depth_value = depth_value_1;
        } else if (depth_value_2 > 0) {
            if ((static_cast<int>(x + 1) - x) <= 0.5 + SHIFT_THRESHOLD && (y - static_cast<int>(y)) <= 0.5 + SHIFT_THRESHOLD)
                interpolated_depth_value = depth_value_2;
        } else if (depth_value_3 > 0) {
            if ((x - static_cast<int>(x)) <= 0.5 + SHIFT_THRESHOLD && (static_cast<int>(y + 1) - y) <= 0.5 + SHIFT_THRESHOLD)
                interpolated_depth_value = depth_value_3;
        } else if (depth_value_4 > 0) {
            if ((static_cast<int>(x + 1) - x) <= 0.5 + SHIFT_THRESHOLD && (y - static_cast<int>(y + 1) - y) <= 0.5 + SHIFT_THRESHOLD)
                interpolated_depth_value = depth_value_4;
        }
        break;
    }
    case 0:
        break;
    }
//    if (interpolated_depth_value > 0 && (depth_value_1 <= 0 || interpolated_depth_value < depth_value_1) && (depth_value_2 <= 0 || interpolated_depth_value < depth_value_2)
//            && (depth_value_3 <= 0 || interpolated_depth_value < depth_value_3) && (depth_value_4 <= 0 || interpolated_depth_value < depth_value_4)) {
//        cout << num_valid_depth_values << '\t' << x << '\t' << y << endl;
//        cout << depth_value_1 << '\t' << depth_value_2 << '\t' << depth_value_3 << '\t' << depth_value_4 << '\t' << interpolated_depth_value << endl;
//        exit(1);
//    }
//    if (interpolated_depth_value > 0 && (depth_value_1 <= 0 || interpolated_depth_value > depth_value_1) && (depth_value_2 <= 0 || interpolated_depth_value > depth_value_2)
//            && (depth_value_3 <= 0 || interpolated_depth_value > depth_value_3) && (depth_value_4 <= 0 || interpolated_depth_value > depth_value_4)) {
//        cout << num_valid_depth_values << '\t' << x << '\t' << y << endl;
//        exit(1);
//    }
    if (interpolated_depth_value < 0 && num_valid_depth_values > 0)
        cout << depth_value_1 << '\t' << depth_value_2 << '\t' << depth_value_3 << '\t' << depth_value_4 << endl;
    return interpolated_depth_value;
}

void Panorama::SubSampleDepthMap(const double ratio)
{
    int new_depth_width = depth_width * ratio;
    int new_depth_height = depth_height * ratio;

    vector<double> new_depth_values(new_depth_width * new_depth_height);

    for (int y = 0; y < new_depth_height; ++y) {
      for (int x = 0; x < new_depth_width; ++x) {
          double ori_x = x / ratio;
          double ori_y = y / ratio;
          new_depth_values[y * new_depth_width + x] = InterpolateDepthValue(ori_x, ori_y);
      }
    }

    depth_values = new_depth_values;
    depth_width = new_depth_width;
    depth_height = new_depth_height;
}

void Panorama::GetBoundaryIndices()
{
    boundary_indices.clear();
    for (int y = 0; y < depth_height; ++y) {
      for (int x = 0; x < depth_width; ++x) {
          vector<double> neighbor_depth_values;
          if (x > 0)
            neighbor_depth_values.push_back(depth_values[y * depth_width + (x - 1)]);
          if (x < depth_width - 1)
            neighbor_depth_values.push_back(depth_values[y * depth_width + (x + 1)]);
          if (y > 0)
            neighbor_depth_values.push_back(depth_values[(y - 1) * depth_width + x]);
          if (y < depth_height - 1)
            neighbor_depth_values.push_back(depth_values[(y + 1) * depth_width + x]);
          bool on_boundary = false;
          for (int i = 0; i < neighbor_depth_values.size(); i++) {
              if (neighbor_depth_values[i] <= 0) {
                  on_boundary = true;
                  break;
              }
          }
          if (on_boundary)
              boundary_indices.push_back(y * depth_width + x);
      }
    }
}

void Panorama::UpdateSurfaceIds()
{
    vector<int> new_surface_ids(depth_width * depth_height, -1);
    int new_id = 0;
    for (int x = 0; x < depth_width; x++) {
        for (int y = 0; y < depth_height; y++) {
            int index = y * depth_width + x;
            if (new_surface_ids[index] >= 0)
                continue;
            int id = surface_ids[index];
            if (id == -1)
                continue;
            vector<int> border_pixels;
            border_pixels.push_back(index);
            while (border_pixels.size() > 0) {
                vector<int> new_border_pixels;
                for (int i = 0; i < border_pixels.size(); i++) {
                    int pixel = border_pixels[i];
                    if (new_surface_ids[pixel] != -1 || surface_ids[pixel] != id)
                        continue;
                    new_surface_ids[pixel] = new_id;
                    int x = pixel % depth_width;
                    int y = pixel / depth_width;
                    if (x > 0)
                        new_border_pixels.push_back(pixel - 1);
                    if (x < depth_width - 1)
                        new_border_pixels.push_back(pixel + 1);
                    if (y > 0)
                        new_border_pixels.push_back(pixel - depth_width);
                    if (y < depth_height - 1)
                        new_border_pixels.push_back(pixel + depth_width);
                    if (x > 0 && y > 0)
                        new_border_pixels.push_back(pixel - 1 - depth_width);
                    if (x < depth_width - 1 && y > 0)
                        new_border_pixels.push_back(pixel + 1 - depth_width);
                    if (x > 0 && y < depth_height - 1)
                        new_border_pixels.push_back(pixel - 1 + depth_width);
                    if (x < depth_width - 1 && y < depth_height - 1)
                        new_border_pixels.push_back(pixel + 1 + depth_width);
                }
                border_pixels = new_border_pixels;
            }

            new_id++;
        }
    }
    surface_ids = new_surface_ids;
}

}  // namespace structured_indoor_modeling
  
