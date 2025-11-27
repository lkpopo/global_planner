#pragma once
#include <mutex>
#include <queue>
#include "ieskf.h"
#include "localizer_utils.h"

bool esti_plane(PointVec &points, const double &thresh, V4D &out);

float sq_dist(const PointType &p1, const PointType &p2);
