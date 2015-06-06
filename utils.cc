#include "utils.h"

#include <QImage>
#include <QColor>
#include <cassert>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


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
