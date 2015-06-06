#ifndef UTILS_H
#define UTILS_H

#include <QString>
#include <vector>

void DrawMask(const QString filename, const std::vector<bool> &mask, const int width, const int height);
void DrawIndices(const QString filename, const std::vector<int> &indices, const int width, const int height);
std::vector<double> FitPlane(const std::vector<double> &points);

#endif // UTILS_H
