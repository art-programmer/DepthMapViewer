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

    view_scale = 0.1;
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

void DepthMapRenderer::CreateVBOs()
{
    VBO_ids.resize(num_layers + 1);
    layer_num_triangle_values.resize(num_layers + 1);
//    GLuint *id = new GLuint[1];
//    GLuint test_id;
//    glGenBuffers(1, id);
    glGenBuffers(num_layers + 1, &VBO_ids[0]);
    for (int layer_index = 0; layer_index < num_layers; layer_index++) {
        glBindBuffer(GL_ARRAY_BUFFER, VBO_ids[layer_index]);
        glBufferData(GL_ARRAY_BUFFER, layer_triangles[layer_index].size() / 81 * 9, &layer_triangles[layer_index][0], GL_STATIC_DRAW);
        layer_num_triangle_values[layer_index] = layer_triangles[layer_index].size() / 81 * 9;
    }
    glBindBuffer(GL_ARRAY_BUFFER, VBO_ids[num_layers]);
    glBufferData(GL_ARRAY_BUFFER, triangles_ori.size(), &triangles_ori[0], GL_STATIC_DRAW);
    layer_num_triangle_values[num_layers] = triangles_ori.size();
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


    glClear(GL_COLOR_BUFFER_BIT);

    bool use_shaders = true;
    bool use_tesselator = false;
    bool use_triangulation = false;
    bool use_triangles = true;
    bool use_VBOs = true;


    if (use_shaders) {
      if (!program->bind()) {
        cerr << "Cannot bind." << endl;
        exit (1);
      }
      program->setUniformValue("phi_range", static_cast<float>(depth_maps[0].GetPhiRange()));
      program->setUniformValue("image_width", static_cast<float>(image_width));
      program->setUniformValue("image_height", static_cast<float>(image_height));

      program->setUniformValue("focal_length", static_cast<float>(focal_length));


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
//  cout << rendering_mode << endl;
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
  case 'O':
      rendering_layers.insert(num_layers);
  default:
      break;
  }


  if (false) {
      glActiveTexture(GL_TEXTURE0);

      glBindTexture(GL_TEXTURE_2D, test_texture_id);
      glEnable(GL_TEXTURE_2D);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

      double triangle_edge_length = 1;
      glBegin(GL_TRIANGLES);
      glTexCoord2d(0, 0);
      glVertex3d(-triangle_edge_length / 2, -triangle_edge_length / 2, -1);
      glTexCoord2d(1, 0);
      glVertex3d(triangle_edge_length / 2, -triangle_edge_length / 2, -1);
      glTexCoord2d(1, 1);
      glVertex3d(triangle_edge_length / 2, triangle_edge_length / 2, -1);
      glTexCoord2d(0, 0);
      glVertex3d(-triangle_edge_length / 2, -triangle_edge_length / 2, -1);
      glTexCoord2d(0, 1);
      glVertex3d(-triangle_edge_length / 2, triangle_edge_length / 2, -1);
      glTexCoord2d(1, 1);
      glVertex3d(triangle_edge_length / 2, triangle_edge_length / 2, -1);
      glEnd();
      return;
  }

  if (use_VBOs == true) {
      for (int layer_index = 0; layer_index < num_layers + 1; layer_index++) {
          if (rendering_layers.count(layer_index) == 0)
              continue;

          QImage test_image = rgb_images[layer_index];

          glActiveTexture(GL_TEXTURE0);

          glBindTexture(GL_TEXTURE_2D, texture_ids[layer_index]);
          glEnable(GL_TEXTURE_2D);

          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

          glBindBuffer(GL_ARRAY_BUFFER, VBO_ids[layer_index]);
          glEnableClientState(GL_VERTEX_ARRAY);
          glVertexPointer(3, GL_DOUBLE, 0, 0);
          glDrawArrays(GL_TRIANGLES, 0, layer_num_triangle_values[layer_index]);
          glDisableClientState((GL_VERTEX_ARRAY));
          glBindBuffer(GL_ARRAY_BUFFER, 0);
      }
      return;
  }
  else if (use_tesselator == true) {

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
  } else {
      for (int layer_index = num_layers - 1; layer_index >= 0; layer_index--) {
          if (rendering_layers.count(layer_index) == 0)
              continue;

          glClear(GL_DEPTH_BUFFER_BIT);

          QImage test_image = rgb_images[layer_index];

          glActiveTexture(GL_TEXTURE0);

          glBindTexture(GL_TEXTURE_2D, texture_ids[layer_index]);
          glEnable(GL_TEXTURE_2D);

          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);


          // glColor4f(alpha, alpha, alpha, 1.0);
        //  cout << image_width << '\t' << image_height << '\t' << depth_mesh.size() << endl;
        //  exit(1);
        //  glColor4f(1, 0, 0, 1);

          glBegin(GL_TRIANGLES);

          if (use_triangles == true) {
            vector<double> &triangles = layer_triangles[layer_index];
//              cout << surface_triangles_3D.size() << endl;
//              cout << surface_triangles_uv.size() << endl;
            for (int triangle_index = 0; triangle_index < triangles.size() / 9; triangle_index++) {

                glVertex3d(triangles[triangle_index * 9 + 0], triangles[triangle_index * 9 + 1], triangles[triangle_index * 9 + 2]);
                glVertex3d(triangles[triangle_index * 9 + 3], triangles[triangle_index * 9 + 4], triangles[triangle_index * 9 + 5]);
                glVertex3d(triangles[triangle_index * 9 + 6], triangles[triangle_index * 9 + 7], triangles[triangle_index * 9 + 8]);

//                glVertex3d(v_1[0], v_1[1], v_1[2]);
//                glVertex3d(v_2[0], v_2[1], v_2[2]);
//                glVertex3d(v_3[0], v_3[1], v_3[2]);
            }

//              map<int, vector<double> > surface_triangles_3D = layer_surface_triangles_3D[layer_index];
//              map<int, vector<double> > surface_triangles_uv = layer_surface_triangles_uv[layer_index];
////              cout << surface_triangles_3D.size() << endl;
////              cout << surface_triangles_uv.size() << endl;
//              for (map<int, vector<double> >::const_iterator surface_it = surface_triangles_3D.begin(); surface_it != surface_triangles_3D.end(); surface_it++) {
//                vector<double> triangles_3D = surface_it->second;
//                vector<double> triangles_uv = surface_triangles_uv[surface_it->first];
//                for (int triangle_index = 0; triangle_index < triangles_3D.size() / 9; triangle_index++) {

//                    const vector<double> v_1(triangles_3D.begin() + triangle_index * 9 + 0, triangles_3D.begin() + triangle_index * 9 + 3);
//                    const vector<double> v_2(triangles_3D.begin() + triangle_index * 9 + 3, triangles_3D.begin() + triangle_index * 9 + 6);
//                    const vector<double> v_3(triangles_3D.begin() + triangle_index * 9 + 6, triangles_3D.begin() + triangle_index * 9 + 9);

//                    if (v_1[2] > 0 || v_2[2] > 0 || v_3[2] > 0) {
//                        cout << v_1[2] << '\t' << v_2[2] << '\t' << v_3[2] << endl;
////                        cout << triangles_2D[triangle_index * 6 + 0] << '\t' << triangles_2D[triangle_index * 6 + 1] << '\t' << triangles_2D[triangle_index * 6 + 2]
////                                                                     << '\t' << triangles_2D[triangle_index * 6 + 3] << '\t' << triangles_2D[triangle_index * 6 + 4]
////                                                                     << '\t' << triangles_2D[triangle_index * 6 + 5] << endl;
//                        continue;
//                    }aversense

////                    if (abs(-v_1[0] / v_1[2] * static_cast<float>(focal_length) / image_width + 0.5 - triangles_uv[triangle_index * 6 + 0]) > 0.0001) {
////                        cout << -v_1[0] / v_1[2] * focal_length / image_width + 0.5 << '\t' << triangles_uv[triangle_index * 6 + 0] << endl;
////                        exit(1);
////                    }
////                    if (abs(-v_2[0] / v_2[2] * static_cast<float>(focal_length) / image_width + 0.5 - triangles_uv[triangle_index * 6 + 2]) > 0.0001) {
////                        cout << -v_2[0] / v_2[2] * focal_length / image_width + 0.5 << '\t' << triangles_uv[triangle_index * 6 + 2] << endl;
////                        exit(1);
////                    }
////                    if (abs(-v_3[0] / v_3[2] * static_cast<float>(focal_length) / image_width + 0.5 - triangles_uv[triangle_index * 6 + 4]) > 0.0001) {
////                        cout << -v_3[0] / v_3[2] * focal_length / image_width + 0.5 << '\t' << triangles_uv[triangle_index * 6 + 4] << endl;
////                        exit(1);
////                    }

//                    // 00
//                    if (!use_shaders)
//                        glTexCoord2d(triangles_uv[triangle_index * 6 + 0], triangles_uv[triangle_index * 6 + 1]);
//                    glVertex3d(v_1[0], v_1[1], v_1[2]);
//                    // 10
//                    if (!use_shaders)
//                        glTexCoord2d(triangles_uv[triangle_index * 6 + 2], triangles_uv[triangle_index * 6 + 3]);
//                    glVertex3d(v_2[0], v_2[1], v_2[2]);
//                    // 01
//                    if (!use_shaders)
//                        glTexCoord2d(triangles_uv[triangle_index * 6 + 4], triangles_uv[triangle_index * 6 + 5]);
//                    glVertex3d(v_3[0], v_3[1], v_3[2]);
//                }aversense
//              }
          }
          else if (use_triangulation) {
              vector<vector<double> > depth_mesh = depth_meshes[layer_index];
              map<int, vector<double> > surface_triangles_2D = layer_surface_triangles_2D[layer_index];
              for (map<int, vector<double> >::const_iterator surface_it = surface_triangles_2D.begin(); surface_it != surface_triangles_2D.end(); surface_it++) {
                vector<double> triangles_2D = surface_it->second;
                for (int triangle_index = 0; triangle_index < triangles_2D.size() / 6; triangle_index++) {
                    const int index_1 = static_cast<int>(triangles_2D[triangle_index * 6 + 1] + 0.5) * image_width + static_cast<int>(triangles_2D[triangle_index * 6 + 0] + 0.5);
                    const int index_2 = static_cast<int>(triangles_2D[triangle_index * 6 + 3] + 0.5) * image_width + static_cast<int>(triangles_2D[triangle_index * 6 + 2] + 0.5);
                    const int index_3 = static_cast<int>(triangles_2D[triangle_index * 6 + 5] + 0.5) * image_width + static_cast<int>(triangles_2D[triangle_index * 6 + 4] + 0.5);

                    const vector<double> v_1 = depth_mesh[index_1];
                    const vector<double> v_2 = depth_mesh[index_2];
                    const vector<double> v_3 = depth_mesh[index_3];

                    if (v_1[2] > 0 || v_2[2] > 0 || v_3[2] > 0) {
    //                    cout << v_1[2] << '\t' << v_2[2] << '\t' << v_3[2] << endl;
    //                    cout << triangles_2D[triangle_index * 6 + 0] << '\t' << triangles_2D[triangle_index * 6 + 1] << '\t' << triangles_2D[triangle_index * 6 + 2]
    //                                                                 << '\t' << triangles_2D[triangle_index * 6 + 3] << '\t' << triangles_2D[triangle_index * 6 + 4]
    //                                                                 << '\t' << triangles_2D[triangle_index * 6 + 5] << endl;
                        continue;
                    }

              //      cout << v00[0] << '\t' << v00[1] << '\t' << v00[2] << endl;
              //      cout << v01[0] << '\t' << v01[1] << '\t' << v01[2] << endl;
              //      cout << v10[0] << '\t' << v10[1] << '\t' << v10[2] << endl;
              //      cout << v11[0] << '\t' << v11[1] << '\t' << v11[2] << endl;
              //      exit(1);

                    // 00
                    if (!use_shaders)
                        glTexCoord2d((index_1 % image_width - 0.0) / static_cast<double>(image_width - 1), 1.0 - (index_1 / image_width - 0) / static_cast<double>(image_height - 1));
                    glVertex3d(v_1[0], v_1[1], v_1[2]);
                    // 10
                    if (!use_shaders)
                        glTexCoord2d((index_2 % image_width - 0.0) / static_cast<double>(image_width - 1), 1.0 - (index_2 / image_width - 0) / static_cast<double>(image_height - 1));
                    glVertex3d(v_2[0], v_2[1], v_2[2]);
                    // 01
                    if (!use_shaders)
                        glTexCoord2d((index_3 % image_width - 0.0) / static_cast<double>(image_width - 1), 1.0 - (index_3 / image_width - 0) / static_cast<double>(image_height - 1));
                    glVertex3d(v_3[0], v_3[1], v_3[2]);
                }
              }
          } else {
              vector<vector<double> > depth_mesh = depth_meshes[layer_index];
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

    //              cout << v00[0] << '\t' << v00[1] << '\t' << v00[2] << endl;
    //              cout << v01[0] << '\t' << v01[1] << '\t' << v01[2] << endl;
    //              cout << v10[0] << '\t' << v10[1] << '\t' << v10[2] << endl;
    //              cout << v11[0] << '\t' << v11[1] << '\t' << v11[2] << endl;
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
          }
          glEnd();
      }

      if (rendering_mode == 'O') {
          program->setUniformValue("image_width", static_cast<float>(image_width));
          program->setUniformValue("image_height", static_cast<float>(image_height));

          glActiveTexture(GL_TEXTURE0);

          glBindTexture(GL_TEXTURE_2D, texture_ids[num_layers]);
          glEnable(GL_TEXTURE_2D);

          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

          glBegin(GL_TRIANGLES);

          for (int triangle_index = 0; triangle_index < triangles_ori.size() / 9; triangle_index++) {
//              glTexCoord2d((x + 0.5) / static_cast<double>(ori_image_width), 1.0 - (y + 0.5) / static_cast<double>(ori_image_height));
              glVertex3d(triangles_ori[triangle_index * 9 + 0], triangles_ori[triangle_index * 9 + 1], triangles_ori[triangle_index * 9 + 2]);
              glVertex3d(triangles_ori[triangle_index * 9 + 3], triangles_ori[triangle_index * 9 + 4], triangles_ori[triangle_index * 9 + 5]);
              glVertex3d(triangles_ori[triangle_index * 9 + 6], triangles_ori[triangle_index * 9 + 7], triangles_ori[triangle_index * 9 + 8]);
          }
          glEnd();

//          QImage &ori_image = rgb_images[num_layers];
//          int ori_image_width = ori_image.width();
//          int ori_image_height = ori_image.height();
//    //      cout << ori_image_width << '\t' << ori_image_height << endl;
//    //      ori_image.save("test.bmp");
//          for (int y = 0; y < ori_image_height - 1; ++y) {
//            for (int x = 0; x < ori_image_width - 1; ++x) {
//              const int right_x = (x + 1) % ori_image_width;
//              const int index00 = y * ori_image_width + x;
//              const int index01 = y * ori_image_width + right_x;
//              const int index10 = (y + 1) * ori_image_width + x;
//              const int index11 = (y + 1) * ori_image_width + right_x;

//              const Vector3d v00 = ori_depth_mesh[index00];
//              const Vector3d v10 = ori_depth_mesh[index10];
//              const Vector3d v01 = ori_depth_mesh[index01];
//              const Vector3d v11 = ori_depth_mesh[index11];

//              if (v00[2] > 0 || v01[2] > 0 || v10[2] > 0 || v11[2] > 0)
//                  continue;

//              // 00
//    //          if (!use_shaders)
//                  glTexCoord2d((x + 0.5) / static_cast<double>(ori_image_width), 1.0 - (y + 0.5) / static_cast<double>(ori_image_height));
//              glVertex3d(v00[0], v00[1], v00[2]);
//              // 10
//    //          if (!use_shaders)
//                  glTexCoord2d((x + 0.5) / static_cast<double>(ori_image_width), 1.0 - (y + 0.5 + 1) / static_cast<double>(ori_image_height));
//              glVertex3d(v10[0], v10[1], v10[2]);
//              // 01
//    //          if (!use_shaders)
//                  glTexCoord2d((x + 0.5 + 1) / static_cast<double>(ori_image_width), 1.0 - (y + 0.5) / static_cast<double>(ori_image_height));
//              glVertex3d(v01[0], v01[1], v01[2]);

//              // 10
//    //          if (!use_shaders)
//                  glTexCoord2d((x + 0.5) / static_cast<double>(ori_image_width), 1.0 - (y + 0.5 + 1) / static_cast<double>(ori_image_height));
//              glVertex3d(v10[0], v10[1], v10[2]);
//              // 11
//    //          if (!use_shaders)
//                  glTexCoord2d((x + 0.5 + 1) / static_cast<double>(ori_image_width), 1.0 - (y + 0.5 + 1) / static_cast<double>(ori_image_height));
//              glVertex3d(v11[0], v11[1], v11[2]);
//              // 01
//    //          if (!use_shaders)
//                  glTexCoord2d((x + 0.5 + 1) / static_cast<double>(ori_image_width), 1.0 - (y + 0.5) / static_cast<double>(ori_image_height));
//              glVertex3d(v01[0], v01[1], v01[2]);
//            }
//          }
//          glEnd();
      }
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


//   ReadTriangles(file_io, scene_index);
   ReadCameraParameters(file_io, scene_index);
   InitLayerTriangles(file_io, scene_index);
   InitTrianglesOri(file_io, scene_index);


//  InitDepthMeshes(file_io, scene_index);
//  TriangulateBoundaries();

//  cout << "done" << endl;
//  RectifyDepthMeshes();

//  exit(1);
}

void DepthMapRenderer::InitGL() {
  initializeGLFunctions();
  
  glEnable(GL_TEXTURE_2D);

  int test_image_size = 3;
  QImage test_image(test_image_size, test_image_size, QImage::Format_RGB888);
  for (int x = 0; x < test_image_size; x++) {
      for (int y = 0; y < test_image_size; y++)
          if ((x + y) % 2 == 0)
              test_image.setPixel(x, y, qRgb(255, 0, 0));
          else
              test_image.setPixel(x, y, qRgb(0, 255, 0));
      test_image.setPixel(x, 2, qRgb(0, 0, 255));
  }
  test_texture_id = widget->bindTexture(test_image);

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


  CreateVBOs();

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
          const Vector2d pixel = depth_map.DepthToRGB(Vector2d(x - 0.5, y - 0.5));
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

void DepthMapRenderer::TriangulateBoundaries()
{
    layer_surface_triangles_2D.resize(num_layers);
    const int ori_image_width = image_width - 1;
    const int ori_image_height = image_height - 1;
    for (int layer_index = 0; layer_index < num_layers; layer_index++) {
//        cout << "layer index: " << layer_index << endl;
        map<int, vector<int> > surface_boundary_indices;
//        vector<Vector3d> depth_mesh = depth_meshes[layer_index];
        vector<int> surface_ids = depth_maps[layer_index].GetSurfaceIds();

        map<int, vector<int> > surface_points;
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
              for (set<int>::const_iterator surface_it = neighbor_surfaces.begin(); surface_it != neighbor_surfaces.end(); surface_it++)
                  if (*surface_it != -1)
                      surface_points[*surface_it].push_back(y * image_width + x);
          }
        }
        for (map<int, vector<int> >::const_iterator surface_it = surface_points.begin(); surface_it != surface_points.end(); surface_it++) {
            layer_surface_triangles_2D[layer_index][surface_it->first] = TriangulateSegmentBoundary(surface_it->second, image_width, image_height, layer_index, surface_it->first);
        }

//        for (map<int, vector<int> >::const_iterator surface_it = surface_boundary_indices.begin(); surface_it != surface_boundary_indices.end(); surface_it++) {
//            cout << surface_it->first << endl;
//            layer_surface_triangles_2D[layer_index][surface_it->first] = TriangulateSegmentBoundary(surface_it->second, image_width, image_height);
//        }
    }
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


void DepthMapRenderer::ReadTriangles(const FileIO &file_io, const int scene_index)
{
    string triangles_3D_filename = file_io.GetTriangles3D(scene_index);
    ifstream triangles_3D_in_str(triangles_3D_filename.c_str());
    triangles_3D_in_str >> num_layers;
    layer_surface_triangles_3D.resize(num_layers);
    for (int layer_index = 0; layer_index < num_layers; layer_index++) {
        int num_surfaces;
        triangles_3D_in_str >> num_surfaces;
        for (int surface_index = 0; surface_index < num_surfaces; surface_index++) {
            int surface_id, num_triangles;
            triangles_3D_in_str >> surface_id >> num_triangles;
            vector<double> triangles_3D(num_triangles * 9);
            for (int value_index = 0; value_index < num_triangles * 9; value_index++)
                triangles_3D_in_str >> triangles_3D[value_index];
            layer_surface_triangles_3D[layer_index][surface_id] = triangles_3D;
        }
    }
    triangles_3D_in_str.close();

    string triangles_uv_filename = file_io.GetTrianglesUv(scene_index);
    ifstream triangles_uv_in_str(triangles_uv_filename.c_str());
    triangles_uv_in_str >> num_layers;
    layer_surface_triangles_uv.resize(num_layers);
    for (int layer_index = 0; layer_index < num_layers; layer_index++) {
        int num_surfaces;
        triangles_uv_in_str >> num_surfaces;
        for (int surface_index = 0; surface_index < num_surfaces; surface_index++) {
            int surface_id, num_triangles;
            triangles_uv_in_str >> surface_id >> num_triangles;
            vector<double> triangles_uv(num_triangles * 6);
            for (int value_index = 0; value_index < num_triangles * 6; value_index++)
                triangles_uv_in_str >> triangles_uv[value_index];
            layer_surface_triangles_uv[layer_index][surface_id] = triangles_uv;
        }
    }
    triangles_uv_in_str.close();

//    string triangles_ori_filename = file_io.GetTrianglesOri(scene_index);
//    ifstream triangles_ori_in_str(triangles_ori_filename.c_str());
//    int num_triangles_ori;
//    triangles_ori_in_str >> num_triangles_ori;
//    triangles_ori.resize(num_triangles_ori * 9);
//    for (int value_index = 0; value_index < num_triangles_ori * 9; value_index++)
//        triangles_ori_in_str >> triangles_ori[value_index];
//    triangles_ori_in_str.close();
}

void DepthMapRenderer::ReadCameraParameters(const FileIO &file_io, const int scene_index)
{
    string camera_parameters_filename = file_io.GetCameraParameters(scene_index);
    ifstream camera_parameters_in_str(camera_parameters_filename.c_str());
    camera_parameters_in_str >> focal_length >> image_width >> image_height;
    camera_parameters_in_str.close();
}

void DepthMapRenderer::InitLayerTriangles(const FileIO &file_io, const int scene_index)
{
    layer_triangles.resize(num_layers);
    for (int layer_index = 0; layer_index < num_layers; layer_index++) {
        string depth_values_filename = file_io.GetLayerDepthValues(scene_index, layer_index);
        ifstream depth_values_in_str(depth_values_filename.c_str());
        int width, height;
        depth_values_in_str >> width >> height;
        vector<double> depth_values(width * height);
        for (int i = 0; i < width * height; i++)
            depth_values_in_str >> depth_values[i];
        depth_values_in_str.close();

        for (int x = 0; x < width - 1; x++) {
           for (int y = 0; y < height - 1; y++) {
             vector<int> vertex_indices;
             if (depth_values[y * width + x] > 0 && depth_values[y * width + (x + 1)] > 0 && depth_values[(y + 1) * width + (x + 1)] > 0) {
                 vertex_indices.push_back(y * width + x);
                 vertex_indices.push_back(y * width + (x + 1));
                 vertex_indices.push_back((y + 1) * width + (x + 1));
             }
             if (depth_values[y * width + x] > 0 && depth_values[(y + 1) * width + (x + 1)] > 0 && depth_values[(y + 1) * width + x] > 0) {
                 vertex_indices.push_back(y * width + x);
                 vertex_indices.push_back((y + 1) * width + (x + 1));
                 vertex_indices.push_back((y + 1) * width + x);
             }
             for (vector<int>::const_iterator index_it = vertex_indices.begin(); index_it != vertex_indices.end(); index_it++) {
               int vertex_x = *index_it % width;
               int vertex_y = *index_it / width;
               double depth = depth_values[*index_it];
               assert(depth > 0);
               double X = (vertex_x - (width - 1) * 0.5) * depth / focal_length;
               double Y = -(vertex_y - (height - 1) * 0.5) * depth / focal_length;
               double Z = -depth;
               layer_triangles[layer_index].push_back(X);
               layer_triangles[layer_index].push_back(Y);
               layer_triangles[layer_index].push_back(Z);
             }
           }
       }
    }
}

void DepthMapRenderer::InitTrianglesOri(const FileIO &file_io, const int scene_index)
{
    string depth_values_ori_filename = file_io.GetDepthValuesOri(scene_index);
    ifstream depth_values_ori_in_str(depth_values_ori_filename.c_str());
    int width, height;
    depth_values_ori_in_str >> width >> height;
    vector<double> depth_values_ori(width * height);
    for (int i = 0; i < width * height; i++)
        depth_values_ori_in_str >> depth_values_ori[i];
    depth_values_ori_in_str.close();

    for (int x = 0; x < width - 1; x++) {
       for (int y = 0; y < height - 1; y++) {
         vector<int> vertex_indices;
         vertex_indices.push_back(y * width + x);
         vertex_indices.push_back(y * width + (x + 1));
         vertex_indices.push_back((y + 1) * width + (x + 1));
         vertex_indices.push_back(y * width + x);
         vertex_indices.push_back((y + 1) * width + (x + 1));
         vertex_indices.push_back((y + 1) * width + x);
         for (vector<int>::const_iterator index_it = vertex_indices.begin(); index_it != vertex_indices.end(); index_it++) {
           int vertex_x = *index_it % width;
           int vertex_y = *index_it / width;
           double depth = depth_values_ori[*index_it];
           double X = (vertex_x - (width - 1) * 0.5) * depth / focal_length;
           double Y = -(vertex_y - (height - 1) * 0.5) * depth / focal_length;
           double Z = -depth;
           triangles_ori.push_back(X);
           triangles_ori.push_back(Y);
           triangles_ori.push_back(Z);
         }
       }
   }
}

}  // namespace structured_indoor_modeling
