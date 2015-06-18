#include "utils.h"

#include <QImage>
#include <QColor>
#include <cassert>
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "poly2tri/poly2tri.h"

using namespace std;
using namespace Eigen;
using namespace cv;

//triangulate a segment boundary
vector<double> TriangulateSegmentBoundary(const vector<int> &segment_boundary, const int image_width, const int image_height, const int layer_index, const int surface_id)
{
  Mat image = Mat::zeros(image_height, image_width, CV_8UC1);
  for (int i = 0; i < segment_boundary.size(); i++) {
    int boundary_point = segment_boundary[i];
    int x = boundary_point % image_width;
    int y = boundary_point / image_width;
    image.at<uchar>(y, x) = 255;
  }
  Mat ori_image = image.clone();

//  static int index = 0;
  stringstream region_image_filename;
  region_image_filename << "region_image_" << layer_index << "_" << surface_id << ".bmp";
  imwrite(region_image_filename.str(), ori_image);

  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours(image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

  Mat contour_image = Mat::zeros(image.size(), CV_8UC3);
  drawContours(contour_image, contours, -1, Scalar(0, 0, 255));
  imwrite("contour_image.bmp", contour_image);

  vector<int> external_contour_indices;
  for (int i = 0; i < contours.size(); i++)
    if (hierarchy[i][3] == -1)
      external_contour_indices.push_back(i);


  vector<double> triangle_vertices;
  for (int part_index = 0; part_index < external_contour_indices.size(); part_index++) {
      int external_contour_index = external_contour_indices[part_index];
      vector<vector<p2t::Point *> > polylines(contours.size());
      for (int contour_index = 0; contour_index < contours.size(); contour_index++) {
        vector<Point> contour = contours[contour_index];
        vector<p2t::Point *> polyline(contour.size());
        for (int point_index = 0; point_index < contour.size(); point_index++)
          polyline[point_index] = new p2t::Point(contour[point_index].x, contour[point_index].y);
        polylines[contour_index] = polyline;
      }

      p2t::CDT cdt(polylines[external_contour_index]);
      for (int contour_index = 0; contour_index < contours.size(); contour_index++)
        if (hierarchy[contour_index][3] == external_contour_index)
          cdt.AddHole(polylines[contour_index]);

      cdt.Triangulate();

      vector<p2t::Triangle *> triangles;
      triangles = cdt.GetTriangles();

      for (int triangle_index = 0; triangle_index < triangles.size(); triangle_index++) {
        p2t::Triangle &triangle = *triangles[triangle_index];
        for (int vertex_index = 0; vertex_index < 3; vertex_index++) {
          p2t::Point &point = *triangle.GetPoint(vertex_index);
          triangle_vertices.push_back(point.x);
          triangle_vertices.push_back(point.y);
        }
      }
  }

  Mat triangle_image = Mat::zeros(image.size(), CV_8UC3);

    for (int triangle_index = 0; triangle_index < triangle_vertices.size() / 6; triangle_index++) {

        const int x_1 = static_cast<int>(triangle_vertices[triangle_index * 6 + 0] + 0.5);
        const int y_1 = static_cast<int>(triangle_vertices[triangle_index * 6 + 1] + 0.5);
        const int x_2 = static_cast<int>(triangle_vertices[triangle_index * 6 + 2] + 0.5);
        const int y_2 = static_cast<int>(triangle_vertices[triangle_index * 6 + 3] + 0.5);
        const int x_3 = static_cast<int>(triangle_vertices[triangle_index * 6 + 4] + 0.5);
        const int y_3 = static_cast<int>(triangle_vertices[triangle_index * 6 + 5] + 0.5);

        line(triangle_image, Point(x_1, y_1), Point(x_2, y_2), Scalar(0, 0, 255));
        line(triangle_image, Point(x_2, y_2), Point(x_3, y_3), Scalar(0, 0, 255));
        line(triangle_image, Point(x_3, y_3), Point(x_1, y_1), Scalar(0, 0, 255));
    }
    stringstream triangle_image_filename;
    triangle_image_filename << "triangle_image_" << layer_index << "_" << surface_id << ".bmp";
  imwrite(triangle_image_filename.str(), triangle_image);

  return triangle_vertices;
}



void DrawMask(const QString filename, const vector<bool> &mask, const int width, const int height)
{
    QImage image(width, height, QImage::Format_RGB888);
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
            if (mask[y * width + x] == true)
                image.setPixel(x, y, qRgb(255, 255, 255));
            else
                image.setPixel(x, y, qRgb(0, 0, 0));
    image.save(filename);
}

void DrawIndices(const QString filename, const vector<int> &indices, const int width, const int height)
{
    QImage image(width, height, QImage::Format_RGB888);
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
            image.setPixel(x, y, qRgb(0, 0, 0));
    for (int i = 0; i < indices.size(); i++) {
        int index = indices[i];
        int x = index % width;
        int y = index / width;
        image.setPixel(x, y, qRgb(255, 255, 255));
    }
    image.save(filename);
}

//fit a plane based on points.
vector<double> FitPlane(const vector<double> &points)
{
  int NUM_POINTS = points.size() / 3;
  assert(NUM_POINTS >= 3);

  VectorXd center(3);
  center << 0, 0, 0;
  for (int i = 0; i < NUM_POINTS; i++)
    for (int c = 0; c < 3; c++)
      center[c] += points[i * 3 + c];
  center /= NUM_POINTS;

  MatrixXd A(3, NUM_POINTS);
  for (int i = 0; i < NUM_POINTS; i++)
    for (int c= 0; c < 3; c++)
      A(c, i) = points[i * 3 + c] - center[c];

  Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  //    MatrixXf S = svd.singularValues();
  //    cout << S << endl;
  MatrixXd U = svd.matrixU();
  //    cout << U << endl;
  Vector3d normal = U.col(2);
  vector<double> plane(4);
  for (int c = 0; c < 3; c++)
    plane[c] = normal[c];
  plane[3] = normal.dot(center);
  return plane;
}
