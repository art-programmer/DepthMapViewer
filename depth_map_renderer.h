#ifndef DEPTH_MAP_RENDERER_H__
#define DEPTH_MAP_RENDERER_H__

#include <vector>
#include <map>
#include <Eigen/Dense>
#include <QImage>
#include <QGLFunctions>
#include <QGLWidget>
#include <QOpenGLShaderProgram>

#include "configuration.h"
#include "panorama.h"

namespace structured_indoor_modeling {

class Panorama;

class DepthMapRenderer : protected QGLFunctions {
 public:
  DepthMapRenderer();
  virtual ~DepthMapRenderer();
  void Render(const double alpha, QOpenGLShaderProgram* program);
  // void Init(const PanoramaConfiguration& panorama_configuration, QGLWidget* widget);
  void Init(const FileIO& file_io, const int scene_index, std::vector<Panorama>& depth_maps_tmp, QGLWidget* widget);
  void InitGL();

//  const std::vector<Eigen::Vector3d>& DepthMesh() const { return depth_mesh; }
//  int DepthWidth() const { return depth_width; }
//  int DepthHeight() const { return depth_height; }
//  const Panorama& GetPanorama() const { return *panorama; }
  
  void setRenderingMode(const char rendering_mode_tmp);
  void increaseViewScale();
  void decreaseViewScale();
  void moveByX(const double delta_x);
  void moveByY(const double delta_y);
  void moveByZ(const double delta_z);
  void rotateByX(const double delta_angle_x);
  void rotateByY(const double delta_angle_y);
  void rotateByZ(const double delta_angle_z);

//  DepthMapRenderer& operator = (const DepthMapRenderer &right);
 private:
  void InitDepthMeshes(const FileIO& file_io, const int scene_index);

  void GetBoundaryIndices();
  void RectifyDepthMeshes();


  std::vector<Panorama> depth_maps;
  // For texture allocation and deletion.
  QGLWidget* widget;
  
  // Image.
  std::vector<QImage> rgb_images;
  std::vector<GLint> texture_ids;

  // Depthmap is turned into a grid mesh.
  int image_width;
  int image_height;
  std::vector<std::vector<std::vector<double> > > depth_meshes;
  std::vector<Eigen::Vector3d> ori_depth_mesh;
  std::vector<GLdouble *> depth_meshes_data;

  std::vector<std::map<int, std::vector<int> > > layer_surface_boundary_indices;


  int num_layers;

  char rendering_mode;


  double view_scale;
  double translate_x;
  double translate_y;
  double translate_z;
  double rotate_x;
  double rotate_y;
  double rotate_z;
};

}  // namespace structured_indoor_modeling
 
#endif  // DEPTH_MAP_RENDERER_H__
