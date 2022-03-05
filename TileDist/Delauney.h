#pragma once

#include <QPoint>
#include <QPolygon>
#include <QColor>

#include "DataTypes.h"


std::vector<std::vector<int>> delauney( const std::vector<XYZ>& v );