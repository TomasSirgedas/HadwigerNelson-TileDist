#include "Util.h"

#include <vector>

QPointF toPointF( const XYZ& pos ) { return QPointF( pos.x, pos.y ); }

const std::vector<uint32_t> COLORS = { 0xFFED1D26, 0xFF3F47CB, 0xFFFFF200, 0xFFA349A4, 0xFF22B14C, 0xFFFF7F27, 0xFF00EEEE, 0xFFFFFFFF, 0xFF000000, 0xFF909090
                                     , 0xFF550000, 0xFF000066, 0xFF7F7200, 0xFF440044, 0xFF004400, 0xFF884400, 0xFF006E6E, 0xFF80FF00, 0xFFFFCCFF, 0xFFFF1493 };
QColor tileColor( int idx )
{
   return QColor::fromRgb( COLORS[idx] );
}

QColor withAlpha( const QColor& color, double alpha ) { return QColor( color.red(), color.green(), color.blue(), lround( alpha*255 ) ); }

double signedArea( const QPolygonF& poly )
{
   if ( poly.size() <= 2 )
      return 0;

   double area2 = poly.back().x() * poly[0].y() - poly[0].x() * poly.back().y();
      
   for ( int i = 0; i+1 < poly.size(); i++ )
      area2 += poly[i].x() * poly[i+1].y() - poly[i+1].x() * poly[i].y();

   return area2 / 2;
}