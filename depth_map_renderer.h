#ifndef DEPTH_MAP_RENDERER_H__
#define DEPTH_MAP_RENDERER_H__

#include <vector>
#include <map>
#include <Eigen/Dense>
#include <QImage>
//#include <QGLFunctions>
#include <QGLWidget>
#include <QOpenGLShaderProgram>

#include "configuration.h"
#include "panorama.h"

namespace structured_indoor_modeling {

class Panorama;

class DepthMapRenderer {
 public:
  DepthMapRenderer();
  virtual ~DepthMapRenderer();
  void Render(const double alpha, QOpenGLShaderProgram* program);
  // void Init(const PanoramaConfiguration& panorama_configuration, QGLWidget* widget);
  void Init(const FileIO& file_io, const int scene_index, QGLWidget* widget);
  void InitGL(QOpenGLShaderProgram* program);

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
  void returnToOriginalViewPoint();
  void TurnVBOOnOff();
  void setStaticAlpha(const float alpha);

  void resetScene(const FileIO file_io, const int scene_index);

  void printPosition();

  int getWidth();
  int getHeight();

//  DepthMapRenderer& operator = (const DepthMapRenderer &right);
 private:
//  void InitDepthMeshes(const FileIO& file_io, const int scene_index);

//  void GetBoundaryIndices();
//  void RectifyDepthMeshes();
//  void TriangulateBoundaries();

  void ReadTriangles(const FileIO& file_io, const int scene_index);
  void ReadRenderingInfo(const FileIO& file_io, const int scene_index);
  void InitLayerTriangles(const FileIO& file_io, const int scene_index);
  void InitTrianglesOri(const FileIO& file_io, const int scene_index);
  void CreateVBOs(QOpenGLShaderProgram* program);



//  std::vector<Panorama> depth_maps;
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
  std::vector<std::map<int, std::vector<double> > > layer_surface_triangles_2D;

  int num_layers;

  char rendering_mode;

  int test_texture_id;


  double view_scale;
  double translate_x;
  double translate_y;
  double translate_z;
  double rotate_x;
  double rotate_y;
  double rotate_z;


  std::vector<std::map<int, std::vector<double> > > layer_surface_triangles_3D;
  std::vector<std::map<int, std::vector<double> > > layer_surface_triangles_uv;
  std::vector<std::vector<double> > layer_triangles;
  std::vector<double> triangles_ori;

  double focal_length;

//  std::vector<std::vector<GLuint> > VBO_ids;
//  std::vector<std::vector<int> > layer_num_triangle_vertices;

  bool use_VBO;

  std::vector<GLuint> VBO_ids;
  std::vector<GLuint> VAO_ids;

  float static_alpha;

//  std::vector<int> layer_num_triangle_vertices;
};

}  // namespace structured_indoor_modeling
 
#endif  // DEPTH_MAP_RENDERER_H__
