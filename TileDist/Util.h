#pragma once

#include <QPoint>
#include <QPolygon>
#include <QColor>

#include "DataTypes.h"


QPointF toPointF( const XYZ& pos );
QColor tileColor( int idx );
QColor withAlpha( const QColor& color, double alpha );
double signedArea( const QPolygonF& poly );