#pragma once

#include <QPoint>
#include <QPolygon>
#include <QColor>

#include "DataTypes.h"


QPointF toPointF( const XYZ& pos );
QColor tileColor( int idx );
QColor withAlpha( const QColor& color, double alpha );
double signedArea( const QPolygonF& poly );
double lerp( double t, double minVal, double maxVal );
double interpolateExp( double t, double minVal, double maxVal );
