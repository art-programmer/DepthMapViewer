#include <fstream>
#include <iostream>
#include "depth_map_renderer.h"
#include "panorama.h"
#include "utils.h"

//#ifdef __linux__
#include <GL/glu.h>

#include <set>

using namespace Eigen;
using namespace std;

namespace structured_indoor_modeling {

DepthMapRenderer::DepthMapRenderer() {
//    texture_id = -1;

    view_scale = 0.3;
    translate_x = 0;
    translate_y = 0;
    translate_z = 0;
    rotate_x = 0;
    rotate_y = 0;
    rotate_z = 0;

    rendering_mode = 'A';
}

DepthMapRenderer::~DepthMapRenderer() {
    for (int layer_index = 0; layer_index < texture_ids.size(); layer_index++)
        if (texture_ids[layer_index] != -1)
            widget->deleteTexture(texture_ids[layer_index]);
}

//DepthMapRenderer& operator = (const DepthMapRenderer &right)
//{
//    depth_maps = right.depth_maps;
//    // For texture allocation and deletion.
//    widget = right.widget;

//    // Image.
//    rgb_images = right.rgb_images;
//    texture_ids = right.rgb_images;

//    // Depthmap is turned into a grid mesh.
//    image_width = right.image_width;
//    image_height = right.image_height;
//    depth_meshes = right.depth_meshes;


//    view_scale = right.view_scale;
//    translate_x = right.translate_x;
//    translate_y = right.translate_y;
//    translate_z = right.translate_z;
//    rotate_x = right.rotate_x;
//    rotate_y = right.rotate_y;
//    rotate_z = right.rotate_z;

//    return *this;
//}


void vertexCallback(GLvoid *vertex)
{
    const GLdouble *pointer = (GLdouble *) vertex;
 //   glColor3dv(pointer + 3);//在此设置颜色
    double *color = new double[3];
    color[0] = color[1] = 0;
    color[2] = 1;
    glColor3dv(color);
    glVertex3dv(pointer);
}

void beginCallback(GLenum which)
{
    glBegin(which);
}

void endCallback  ()
{
    glEnd();
}

void errorCallback(GLenum errorCode)
{
    const GLubyte *estring;
    estring = gluErrorString(errorCode);
    cerr << "Tessellation Error: " << estring << endl;
    exit(1);
}

void combineCallback(GLdouble coords[3],
                              GLdouble *vertex_data[4],
                              GLfloat weight[4], GLdouble **dataOut )
{
//    cout << "combine callback" << endl;

    GLdouble *vertex;
    int i;
    vertex = (GLdouble *) malloc(6 * sizeof(GLdouble));
    vertex[0] = coords[0];
    vertex[1] = coords[1];
    vertex[2] = coords[2];
    for (i = 3; i < 7; i++)
    {
        vertex[i] = weight[0] * vertex_data[0][i]
            + weight[1] * vertex_data[1][i]
            + weight[2] * vertex_data[2][i]
            + weight[3] * vertex_data[3][i];
    }
    *dataOut = vertex;
}

void DepthMapRenderer::Render(const double alpha, QOpenGLShaderProgram* program) {

//    const Vector3d camera_center(0, 0, 0);
//    const Vector3d direction(0, 0, -1);
//    gluLookAt(camera_center[0], camera_center[1], camera_center[2],
//            camera_center[0] + direction[0], camera_center[1] + direction[1], camera_center[2] + direction[2],
//            0, 1, 0);

//    glTranslated(-translate_x, -translate_y, -translate_z);
//    glRotated(rotate_x, 1, 0, 0);
//    glRotated(rotate_y, 0, 1, 0);
//    glRotated(rotate_z, 0, 0, 1);


    bool use_shaders = false;
    bool use_tesselator = true;
    if (use_shaders) {
      if (!program->bind()) {
        cerr << "Cannot bind." << endl;
        exit (1);
      }
      program->setUniformValue("phi_range", static_cast<float>(depth_maps[0].GetPhiRange()));
      program->setUniformValue("image_width", static_cast<float>(image_width - 1));
      program->setUniformValue("image_height", static_cast<float>(image_height - 1));

      GLfloat global_to_local[4][4];
      for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 4; ++x) {
            if (x == y)
                global_to_local[y][x] = 1;
            else
                global_to_local[y][x] = 0;
    //      global_to_local[y][x] = depth_maps[0].GetGlobalToLocal()(x, y);
        }
      }
      program->setUniformValue("global_to_local", global_to_local);
      program->setUniformValue("alpha", static_cast<float>(alpha));
      program->setUniformValue("tex0", 0);
    }


//  glTranslated(0, 0, 2);
  glTranslated(translate_x, translate_y, translate_z);
  glRotated(rotate_x, 1, 0, 0);
  glRotated(rotate_y, 0, 1, 0);
  glRotated(rotate_z, 0, 0, 1);
  glScaled(view_scale, view_scale, view_scale);

////  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//  glColor3f(1.0, 0.0, 0.0);
//  glBegin(GL_TRIANGLES);
//      glVertex3d(0, 0, -0.5);
//      glVertex3d(0, 1, -0.5);
//      glVertex3d(1, 1, -0.5);
//  glEnd();
//  return;


  set<int> rendering_layers;
  switch (rendering_mode) {
  case '0':
      rendering_layers.insert(0);
      break;
  case '1':
      rendering_layers.insert(1);
      break;
  case '2':
      rendering_layers.insert(2);
      break;
  case 'A':
      rendering_layers.insert(0);
      rendering_layers.insert(1);
      rendering_layers.insert(2);
      break;
  default:
      break;
  }


  if (use_tesselator == true) {

      double vertices[5][3] = {
          {0, 0, -2}, {2, 0, -2}, {1, 1, -2}, {2, 2, -2}, {0, 2, -2}
      };
      GLUtesselator *tobj = gluNewTess();
      if (!tobj) {
          cerr << "Cannot create tesselator." << endl;
          exit(1);
      }

      gluTessCallback(tobj, GLU_TESS_VERTEX, (void(*)())vertexCallback);
      gluTessCallback(tobj, GLU_TESS_BEGIN, (void(*)())beginCallback);
      gluTessCallback(tobj, GLU_TESS_END, (void(*)())endCallback);
      gluTessCallback(tobj, GLU_TESS_ERROR, (void(*)())errorCallback);
      gluTessCallback(tobj, GLU_TESS_COMBINE, (void(*)())combineCallback);

        // glShadeModel(GL_FLAT);

        // gluTessProperty(tobj,GLU_TESS_WINDING_RULE,GLU_TESS_WINDING_POSITIVE); //GLU_TESS_WINDING_ODD

      for (int layer_index = 0; layer_index < num_layers; layer_index++) {
          if (rendering_layers.count(layer_index) == 0)
              continue;
          vector<vector<double> > &depth_mesh = depth_meshes[layer_index];
          map<int, vector<int> > surface_boundary_indices = layer_surface_boundary_indices[layer_index];

          glActiveTexture(GL_TEXTURE0);

          glBindTexture(GL_TEXTURE_2D, texture_ids[layer_index]);
          glEnable(GL_TEXTURE_2D);

          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);


//          gluTessBeginPolygon(tobj, NULL);
//          gluTessBeginContour(tobj);
//          gluTessVertex(tobj, vertices[0], vertices[0]);
//          gluTessVertex(tobj, vertices[1], vertices[1]);
//          gluTessVertex(tobj, vertices[2], vertices[2]);
//          gluTessVertex(tobj, vertices[3], vertices[3]);
//          gluTessVertex(tobj, vertices[4], vertices[4]);
//          gluTessEndContour(tobj);
//          gluTessEndPolygon(tobj);

          for (map<int, vector<int> >::const_iterator surface_it = surface_boundary_indices.begin(); surface_it != surface_boundary_indices.end(); surface_it++) {
//              break;
//              cout << surface_it->second.size() << endl;
              if (surface_it->second.size() <= 8)
                  continue;

              gluTessBeginPolygon(tobj, NULL);
              gluTessBeginContour(tobj);
              for (vector<int>::const_iterator index_it = surface_it->second.begin(); index_it != surface_it->second.end(); index_it++) {
                  vector<double> &vertex = depth_mesh[*index_it];
//                  if (!use_shaders)
//                      glTexCoord2d((vertex[0] / vertex[2] * max(image_width - 1, image_height - 1)) / static_cast<double>(image_width - 1), 1.0 - (vertex[1] * vertex[2] * max(image_width - 1, image_height - 1)) / static_cast<double>(image_height - 1));
                  gluTessVertex(tobj, depth_meshes_data[layer_index * image_width * image_height + *index_it], depth_meshes_data[layer_index * image_width * image_height + *index_it]);
//                  cout << vertex[0] << '\t' << vertex[1] << '\t' << vertex[2] << endl;
              }
//              cout << "done" << endl;
              gluTessEndContour(tobj);
              gluTessEndPolygon(tobj);
//              cout << "done" << endl;
              break;
          }
          break;
      }
      gluDeleteTess(tobj);
      if (use_shaders)
          program->release();
      return;
  }

  for (int layer_index = 0; layer_index < num_layers; layer_index++) {
      if (rendering_layers.count(layer_index) == 0)
          continue;
      vector<vector<double> > depth_mesh = depth_meshes[layer_index];

      glActiveTexture(GL_TEXTURE0);

      glBindTexture(GL_TEXTURE_2D, texture_ids[layer_index]);
      glEnable(GL_TEXTURE_2D);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);


      glBegin(GL_TRIANGLES);
      // glColor4f(alpha, alpha, alpha, 1.0);
    //  cout << image_width << '\t' << image_height << '\t' << depth_mesh.size() << endl;
    //  exit(1);
    //  glColor4f(1, 0, 0, 1);

      for (int y = 0; y < image_height - 1; ++y) {
        for (int x = 0; x < image_width - 1; ++x) {
          const int right_x = (x + 1) % image_width;
          const int index00 = y * image_width + x;
          const int index01 = y * image_width + right_x;
          const int index10 = (y + 1) * image_width + x;
          const int index11 = (y + 1) * image_width + right_x;

          const vector<double> v00 = depth_mesh[index00];
          const vector<double> v10 = depth_mesh[index10];
          const vector<double> v01 = depth_mesh[index01];
          const vector<double> v11 = depth_mesh[index11];

          if (v00[2] > 0 || v01[2] > 0 || v10[2] > 0 || v11[2] > 0)
              continue;

    //      cout << v00[0] << '\t' << v00[1] << '\t' << v00[2] << endl;
    //      cout << v01[0] << '\t' << v01[1] << '\t' << v01[2] << endl;
    //      cout << v10[0] << '\t' << v10[1] << '\t' << v10[2] << endl;
    //      cout << v11[0] << '\t' << v11[1] << '\t' << v11[2] << endl;
    //      exit(1);

          // 00
          if (!use_shaders)
              glTexCoord2d((x) / static_cast<double>(image_width - 1), 1.0 - (y) / static_cast<double>(image_height - 1));
          glVertex3d(v00[0], v00[1], v00[2]);
          // 10
          if (!use_shaders)
              glTexCoord2d((x) / static_cast<double>(image_width - 1), 1.0 - (y + 1) / static_cast<double>(image_height - 1));
          glVertex3d(v10[0], v10[1], v10[2]);
          // 01
          if (!use_shaders)
              glTexCoord2d((x + 1) / static_cast<double>(image_width - 1), 1.0 - (y) / static_cast<double>(image_height - 1));
          glVertex3d(v01[0], v01[1], v01[2]);

          // 10
          if (!use_shaders)
              glTexCoord2d((x) / static_cast<double>(image_width - 1), 1.0 - (y + 1) / static_cast<double>(image_height - 1));
          glVertex3d(v10[0], v10[1], v10[2]);
          // 11
          if (!use_shaders)
              glTexCoord2d((x + 1) / static_cast<double>(image_width - 1), 1.0 - (y + 1) / static_cast<double>(image_height - 1));
          glVertex3d(v11[0], v11[1], v11[2]);
          // 01
          if (!use_shaders)
              glTexCoord2d((x + 1) / static_cast<double>(image_width - 1), 1.0 - (y) / static_cast<double>(image_height - 1));
          glVertex3d(v01[0], v01[1], v01[2]);
        }
      }
      glEnd();
  }

  if (rendering_mode == 'O') {
      glActiveTexture(GL_TEXTURE0);

      glBindTexture(GL_TEXTURE_2D, texture_ids[num_layers]);
      glEnable(GL_TEXTURE_2D);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

      glBegin(GL_TRIANGLES);

      QImage &ori_image = rgb_images[num_layers];
      int ori_image_width = ori_image.width();
      int ori_image_height = ori_image.height();
//      cout << ori_image_width << '\t' << ori_image_height << endl;
//      ori_image.save("test.bmp");
      for (int y = 0; y < ori_image_height - 1; ++y) {
        for (int x = 0; x < ori_image_width - 1; ++x) {
          const int right_x = (x + 1) % ori_image_width;
          const int index00 = y * ori_image_width + x;
          const int index01 = y * ori_image_width + right_x;
          const int index10 = (y + 1) * ori_image_width + x;
          const int index11 = (y + 1) * ori_image_width + right_x;

          const Vector3d v00 = ori_depth_mesh[index00];
          const Vector3d v10 = ori_depth_mesh[index10];
          const Vector3d v01 = ori_depth_mesh[index01];
          const Vector3d v11 = ori_depth_mesh[index11];

          if (v00[2] > 0 || v01[2] > 0 || v10[2] > 0 || v11[2] > 0)
              continue;

          // 00
//          if (!use_shaders)
              glTexCoord2d((x + 0.5) / static_cast<double>(ori_image_width), 1.0 - (y + 0.5) / static_cast<double>(ori_image_height));
          glVertex3d(v00[0], v00[1], v00[2]);
          // 10
//          if (!use_shaders)
              glTexCoord2d((x + 0.5) / static_cast<double>(ori_image_width), 1.0 - (y + 0.5 + 1) / static_cast<double>(ori_image_height));
          glVertex3d(v10[0], v10[1], v10[2]);
          // 01
//          if (!use_shaders)
              glTexCoord2d((x + 0.5 + 1) / static_cast<double>(ori_image_width), 1.0 - (y + 0.5) / static_cast<double>(ori_image_height));
          glVertex3d(v01[0], v01[1], v01[2]);

          // 10
//          if (!use_shaders)
              glTexCoord2d((x + 0.5) / static_cast<double>(ori_image_width), 1.0 - (y + 0.5 + 1) / static_cast<double>(ori_image_height));
          glVertex3d(v10[0], v10[1], v10[2]);
          // 11
//          if (!use_shaders)
              glTexCoord2d((x + 0.5 + 1) / static_cast<double>(ori_image_width), 1.0 - (y + 0.5 + 1) / static_cast<double>(ori_image_height));
          glVertex3d(v11[0], v11[1], v11[2]);
          // 01
//          if (!use_shaders)
              glTexCoord2d((x + 0.5 + 1) / static_cast<double>(ori_image_width), 1.0 - (y + 0.5) / static_cast<double>(ori_image_height));
          glVertex3d(v01[0], v01[1], v01[2]);
        }
      }
      glEnd();
  }

  if (use_shaders)
      program->release();
}
  
void DepthMapRenderer::Init(const FileIO& file_io,
                            const int scene_index,
                            std::vector<Panorama>& depth_maps_tmp,
                            QGLWidget* widget_tmp) {
    num_layers = depth_maps_tmp.size();
  depth_maps.resize(num_layers);
  for (int layer_index = 0; layer_index < num_layers; layer_index++)
    depth_maps[layer_index] = depth_maps_tmp[layer_index];
  widget = widget_tmp;
  rgb_images.resize(num_layers);
  for (int layer_index = 0; layer_index < num_layers; layer_index++)
    rgb_images[layer_index].load(file_io.GetLayerTextureImage(scene_index, layer_index).c_str());
   QImage ori_image(file_io.GetOriImage(scene_index).c_str());
   rgb_images.push_back(ori_image);

//  if (rgb_image.isNull()) {
//    cout << file_io.GetPanoramaImage(panorama_id) << endl;
//    exit (1);
//  }

  InitDepthMeshes(file_io, scene_index);

  GetBoundaryIndices();
  RectifyDepthMeshes();
}

void DepthMapRenderer::InitGL() {
  initializeGLFunctions();
  
  glEnable(GL_TEXTURE_2D);

  texture_ids.resize(num_layers + 1);
  for (int layer_index = 0; layer_index < num_layers + 1; layer_index++)
      texture_ids[layer_index] = widget->bindTexture(rgb_images[layer_index]);
//  texture_ids[num_layers] = widget->bindTexture(ori_image);

  // Set nearest filtering mode for texture minification
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  
  // Set bilinear filtering mode for texture magnification
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  
  // Wrap texture coordinates by repeating
  // f.ex. texture coordinate (1.1, 1.2) is same as (0.1, 0.2)
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
}
  
void DepthMapRenderer::InitDepthMeshes(const FileIO& file_io, const int scene_index) {
  /*
  ifstream ifstr;
  ifstr.open(filename.c_str());
  
  string header;
  double min_depth, max_depth;
  ifstr >> header >> image_width >> image_height >> min_depth >> max_depth;

  vector<double> depths;
  depths.reserve(image_width * image_height);
  for (int y = 0; y < image_height; ++y) {
    for (int x = 0; x < image_width; ++x) {
      double distance;
      ifstr >> distance;
      depths.push_back(distance);
    }
  }
  ifstr.close();
  */

  assert(num_layers > 0);
  image_width  = depth_maps[0].DepthWidth() + 1;
  image_height = depth_maps[0].DepthHeight() + 1;

  depth_meshes.resize(num_layers);
  depth_meshes_data.resize(num_layers * image_width * image_height);
  for (int layer_index = 0; layer_index < num_layers; layer_index++) {
      const Panorama &depth_map = depth_maps[layer_index];
      vector<vector<double> > depth_mesh(image_width * image_height, vector<double>(3));
//      double average_depth = depth_map.GetAverageDistance();
      for (int y = 0; y < image_height; ++y) {
        for (int x = 0; x < image_width; ++x) {
          const Vector2d pixel = depth_map.DepthToRGB(Vector2d(x, y));
//          cout << pixel[0] << '\t' << pixel[1] << endl;
          Vector3d vertex = depth_map.Unproject(pixel, depth_map.GetDepth(Vector2d(x - 0.5, y - 0.5)));
          for (int c = 0; c < 3; c++)
              depth_mesh[y * image_width + x][c] = vertex[c];
//          depth_mesh[y * image_width + x](2) += average_depth;
        }
      }
      depth_meshes[layer_index] = depth_mesh;

      for (int i = 0; i < image_width * image_height; i++) {
          depth_meshes_data[layer_index * image_width * image_height + i] = new GLdouble[3];
          for (int c = 0; c < 3; c++)
              depth_meshes_data[layer_index * image_width * image_height + i][c] = depth_mesh[i][c];
      }
  }


  QImage &ori_image = rgb_images[num_layers];
  double scale = max(ori_image.width(), ori_image.height());
  ifstream in_str(file_io.GetOriPointCloud(scene_index).c_str());
  int ori_num_pixels;
  in_str >> ori_num_pixels;
  ori_depth_mesh.resize(ori_image.width() * ori_image.height());
  for (int y = 0; y < ori_image.height(); ++y) {
      for (int x = 0; x < ori_image.width(); ++x) {
          Vector3d ori_point;
          in_str >> ori_point[0] >> ori_point[1] >> ori_point[2];
//          cout << ori_point[0] << '\t' << ori_point[1] << '\t' << ori_point[2] << endl;
          ori_point[1] *= -1;
          ori_point[2] *= -1;
          ori_depth_mesh[y * ori_image.width() + x] = ori_point / scale;
          //          depth_mesh[y * image_width + x](2) += average_depth;
      }
   }
  in_str.close();
}

void DepthMapRenderer::increaseViewScale()
{
    view_scale *= 1.1;
}

void DepthMapRenderer::decreaseViewScale()
{
    view_scale /= 1.1;
}

void DepthMapRenderer::moveByX(const double delta_x)
{
    translate_x += delta_x;
}

void DepthMapRenderer::moveByY(const double delta_y)
{
    translate_y += delta_y;
}

void DepthMapRenderer::moveByZ(const double delta_z)
{
    translate_z += delta_z;
}

void DepthMapRenderer::rotateByX(const double delta_angle_x)
{
    rotate_x += delta_angle_x;
}

void DepthMapRenderer::rotateByY(const double delta_angle_y)
{
    rotate_y += delta_angle_y;
}

void DepthMapRenderer::rotateByZ(const double delta_angle_z)
{
    rotate_z += delta_angle_z;
}

void DepthMapRenderer::setRenderingMode(const char rendering_mode_tmp)
{
    rendering_mode = rendering_mode_tmp;
}

void DepthMapRenderer::GetBoundaryIndices()
{
    layer_surface_boundary_indices.resize(num_layers);
    const int ori_image_width = image_width - 1;
    const int ori_image_height = image_height - 1;
    for (int layer_index = 0; layer_index < num_layers; layer_index++) {
        map<int, vector<int> > surface_boundary_indices;
//        vector<Vector3d> depth_mesh = depth_meshes[layer_index];
        vector<int> surface_ids = depth_maps[layer_index].GetSurfaceIds();

        for (int y = 0; y < image_height; ++y) {
          for (int x = 0; x < image_width; ++x) {
              double ori_x = x - 0.5;
              double ori_y = y - 0.5;

              int index_1 = (ori_x >= 0 && ori_y >= 0) ? static_cast<int>(ori_y) * ori_image_width + static_cast<int>(ori_x) : -1;
              int index_2 = (ori_x < ori_image_width - 1 && ori_y >= 0) ? static_cast<int>(ori_y) * ori_image_width + static_cast<int>(ori_x + 1) : -1;
              int index_3 = (ori_x >= 0 && ori_y < ori_image_height - 1) ? static_cast<int>(ori_y + 1) * ori_image_width + static_cast<int>(ori_x) : -1;
              int index_4 = (ori_x < ori_image_width - 1 && ori_y < ori_image_height - 1) ? static_cast<int>(ori_y + 1) * ori_image_width + static_cast<int>(ori_x + 1) : -1;

              set<int> neighbor_surfaces;
              neighbor_surfaces.insert(index_1 != -1 ? surface_ids[index_1] : -1);
              neighbor_surfaces.insert(index_2 != -1 ? surface_ids[index_2] : -1);
              neighbor_surfaces.insert(index_3 != -1 ? surface_ids[index_3] : -1);
              neighbor_surfaces.insert(index_4 != -1 ? surface_ids[index_4] : -1);
              if (neighbor_surfaces.size() > 1)
                  for (set<int>::const_iterator surface_it = neighbor_surfaces.begin(); surface_it != neighbor_surfaces.end(); surface_it++)
                      if (*surface_it != -1)
                          surface_boundary_indices[*surface_it].push_back(y * image_width + x);
          }
        }
        
        map<int, vector<int> > surface_ordered_boundary_indices;        
        for (map<int, vector<int> >::const_iterator surface_it = surface_boundary_indices.begin(); surface_it != surface_boundary_indices.end(); surface_it++) {
            QString image_filename = QString("boundary_") + QString::number(layer_index) + "_" + QString::number(surface_it->first) + ".bmp";

            vector<int> ordered_boundary_indices;
            vector<vector<int > > boundary_mask(image_width, vector<int>(image_height, 0));
            
            int start_x = image_width, start_y = image_height;
            for (vector<int>::const_iterator index_it = surface_it->second.begin(); index_it != surface_it->second.end(); index_it++) {
                int x = *index_it % image_width;
                int y = *index_it / image_width;
                boundary_mask[x][y] = 1;
                if (y < start_y) {
                    start_x = x;
                    start_y = y;
                } else if (y == start_y && x < start_x) {
                    start_x = x;
                }
            }
//            vector<bool> mask(image_width * image_height);
//            for (int y = 0; y < image_height; y++)
//                for (int x = 0; x < image_width; x++)
//                    if (boundary_mask[x][y] == 1)
//                        mask[y * image_width + x] = true;
//            DrawMask(image_filename, mask, image_width, image_height);

            int start_index = start_y * image_width + start_x;
            ordered_boundary_indices.push_back(start_index);
            while (true) {
                if (ordered_boundary_indices.size() == 0)
                    break;
                int index = ordered_boundary_indices.back();
//                int previous_index_offset = 0;
//                if (ordered_boundary_indices.size() > 1)
//                    previous_index_offset = index - ordered_boundary_indices[ordered_boundary_indices.size() - 2];
                int x = index % image_width;
                int y = index / image_width;
                boundary_mask[x][y] = -1;

                if (x < image_width - 1 && boundary_mask[x + 1][y] == 1)
                    ordered_boundary_indices.push_back(index + 1);
                else if (x < image_width - 1 && y < image_height - 1 && boundary_mask[x + 1][y + 1] == 1)
                    ordered_boundary_indices.push_back(index + 1 + image_width);
                else if (y < image_height - 1 && boundary_mask[x][y + 1] == 1)
                    ordered_boundary_indices.push_back(index + image_width);
                else if (x > 0  && y < image_height - 1 && boundary_mask[x - 1][y + 1] == 1)
                    ordered_boundary_indices.push_back(index - 1 + image_width);
                else if (x > 0 && boundary_mask[x - 1][y] == 1)
                    ordered_boundary_indices.push_back(index - 1);
                else if (x > 0 && y > 0 && boundary_mask[x - 1][y - 1] == 1)
                    ordered_boundary_indices.push_back(index - 1 - image_width);
                else if (y > 0 && boundary_mask[x][y - 1] == 1)
                    ordered_boundary_indices.push_back(index - image_width);
                else if (x < image_width - 1 && y > 0 && boundary_mask[x + 1][y - 1] == 1)
                    ordered_boundary_indices.push_back(index + 1 - image_width);
                else {
                    if (abs(x - start_x) <= 1 && (y - start_y) <= 1)
                        break;
                    else
                        ordered_boundary_indices.pop_back();
                }
            }
            if (ordered_boundary_indices.size() == 0)
                continue;
//            cout << surface_it->first << '\t' << ordered_boundary_indices.size() << endl;
            while (false) {
                int start_index = -1;
                int nearest_visited_neighbor_pixel = -1;
                for (int y = 0; y < image_height; ++y) {
                  for (int x = 0; x < image_width; ++x) {
                      if (boundary_mask[x][y] != 1)
                          continue;
                      int pixel = y * image_width + x;
                      vector<int> neighbor_pixels;
                      if (x > 0)
                          neighbor_pixels.push_back(pixel - 1);
                      if (x < image_width - 1)
                          neighbor_pixels.push_back(pixel + 1);
                      if (y > 0)
                          neighbor_pixels.push_back(pixel - image_width);
                      if (y < image_height - 1)
                          neighbor_pixels.push_back(pixel + image_width);
                      if (x > 0 && y > 0)
                          neighbor_pixels.push_back(pixel - 1 - image_width);
                      if (x < image_width - 1 && y > 0)
                          neighbor_pixels.push_back(pixel + 1 - image_width);
                      if (x > 0 && y < image_height - 1)
                          neighbor_pixels.push_back(pixel - 1 + image_width);
                      if (x < image_width - 1 && y < image_height - 1)
                          neighbor_pixels.push_back(pixel + 1 + image_width);
                      for (int i = 0; i < neighbor_pixels.size(); i++) {
                          int neighbor_pixel = neighbor_pixels[i];
                          if (boundary_mask[neighbor_pixel % image_width][neighbor_pixel / image_width] == -1) {
                              start_index = y * image_width + x;
                              nearest_visited_neighbor_pixel = neighbor_pixel;
                          }
                      }
                      if (start_index != -1)
                          break;
                  }
                  if (start_index != -1)
                      break;
                }

                if (start_index == -1)
                      break;

                vector<int> additional_boundary_indices;
                additional_boundary_indices.push_back(start_index);
                while (true) {
                    if (additional_boundary_indices.size() == 0)
                        break;
                    int index = additional_boundary_indices.back();
                    int x = index % image_width;
                    int y = index / image_width;
                    boundary_mask[x][y] = -1;

                    if (x < image_width - 1 && boundary_mask[x + 1][y] == 1)
                        additional_boundary_indices.push_back(index + 1);
                    else if (x < image_width - 1 && y < image_height - 1 && boundary_mask[x + 1][y + 1] == 1)
                        additional_boundary_indices.push_back(index + 1 + image_width);
                    else if (y < image_height - 1 && boundary_mask[x][y + 1] == 1)
                        additional_boundary_indices.push_back(index + image_width);
                    else if (x > 0  && y < image_height - 1 && boundary_mask[x - 1][y + 1] == 1)
                        additional_boundary_indices.push_back(index - 1 + image_width);
                    else if (x > 0 && boundary_mask[x - 1][y] == 1)
                        additional_boundary_indices.push_back(index - 1);
                    else if (x > 0 && y > 0 && boundary_mask[x - 1][y - 1] == 1)
                        additional_boundary_indices.push_back(index - 1 - image_width);
                    else if (y > 0 && boundary_mask[x][y - 1] == 1)
                        additional_boundary_indices.push_back(index - image_width);
                    else if (x < image_width - 1 && y > 0 && boundary_mask[x + 1][y - 1] == 1)
                        additional_boundary_indices.push_back(index + 1 - image_width);
                    else {
                        if (abs(x - start_index % image_width) <= 1 && (y - start_index / image_width) <= 1)
                            break;
                        else
                            additional_boundary_indices.pop_back();
                    }
                }
                if (additional_boundary_indices.size() >= 2) {
                    vector<int>::iterator insertion_it;
                    for (vector<int>::iterator index_it = ordered_boundary_indices.begin(); index_it != ordered_boundary_indices.end(); index_it++) {
                        if (*index_it == nearest_visited_neighbor_pixel) {
                            insertion_it = index_it;
                            break;
                        }
                    }
                    ordered_boundary_indices.insert(insertion_it, additional_boundary_indices.begin(), additional_boundary_indices.end());
                }
            }
//            cout << surface_it->first << '\t' << ordered_boundary_indices.size() << endl;

//            DrawIndices(image_filename, ordered_boundary_indices, image_width, image_height);

            vector<int> reduced_ordered_boundary_indices;
            reduced_ordered_boundary_indices.push_back(ordered_boundary_indices.front());
            for (int i = 1; i < ordered_boundary_indices.size(); i++) {
                int previous_offset = ordered_boundary_indices[i] - ordered_boundary_indices[i - 1];
                int next_offset = i < ordered_boundary_indices.size() - 1 ? ordered_boundary_indices[i + 1] - ordered_boundary_indices[i] : ordered_boundary_indices[0] - ordered_boundary_indices[i];
                if (previous_offset != next_offset)
                    reduced_ordered_boundary_indices.push_back(ordered_boundary_indices[i]);
            }
//            DrawIndices(image_filename, reduced_ordered_boundary_indices, image_width, image_height);

//            cout << surface_it->first << '\t' << reduced_ordered_boundary_indices.size() << endl;
            surface_ordered_boundary_indices[surface_it->first] = reduced_ordered_boundary_indices;
        }

        layer_surface_boundary_indices[layer_index] = surface_ordered_boundary_indices;
    }
}

void DepthMapRenderer::RectifyDepthMeshes()
{
    for (int layer_index = 0; layer_index < num_layers; layer_index++) {
        vector<vector<double> > &depth_mesh = depth_meshes[layer_index];
        map<int, vector<int> > surface_boundary_indices = layer_surface_boundary_indices[layer_index];
        for (map<int, vector<int> >::const_iterator surface_it = surface_boundary_indices.begin(); surface_it != surface_boundary_indices.end(); surface_it++) {
            vector<double> boundary_points;
            for (vector<int>::const_iterator index_it = surface_it->second.begin(); index_it != surface_it->second.end(); index_it++) {
                vector<double> &vertex = depth_mesh[*index_it];
                boundary_points.insert(boundary_points.end(), vertex.begin(), vertex.end());
            }
            if (boundary_points.size() <= 9)
                continue;
            vector<double> plane = FitPlane(boundary_points);
            for (vector<int>::const_iterator index_it = surface_it->second.begin(); index_it != surface_it->second.end(); index_it++) {
                vector<double> &vertex = depth_mesh[*index_it];
                vector<double> new_vertex(3);
                double distance = vertex[0] * plane[0] + vertex[1] * plane[1] + vertex[2] * plane[2] - plane[3];
                for (int c = 0; c < 3; c++)
                    new_vertex[c] = vertex[c] - distance * plane[c];
                depth_mesh[*index_it] = new_vertex;
            }
        }
        depth_meshes[layer_index] = depth_mesh;
    }
}

}  // namespace structured_indoor_modeling
