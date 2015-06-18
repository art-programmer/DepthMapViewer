#ifndef UTILS_H
#define UTILS_H

#include <QString>
#include <vector>

std::vector<double> TriangulateSegmentBoundary(const std::vector<int> &segment_boundary, const int image_width, const int image_height, const int layer_index, const int surface_id);
void DrawMask(const QString filename, const std::vector<bool> &mask, const int width, const int height);
void DrawIndices(const QString filename, const std::vector<int> &indices, const int width, const int height);
std::vector<double> FitPlane(const std::vector<double> &points);

#endif // UTILS_H
