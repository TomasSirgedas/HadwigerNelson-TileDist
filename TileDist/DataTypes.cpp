#include "DataTypes.h"

bool XYZW::eq( const XYZW& rhs, double tolerance ) const
{
   return abs( x - rhs.x ) <= tolerance
       && abs( y - rhs.y ) <= tolerance
       && abs( z - rhs.z ) <= tolerance
       && abs( w - rhs.w ) <= tolerance;
}

Matrix4x4 Matrix4x4::inverted() const
{
   Matrix4x4 ret;
   ret.m[0].x = m[2].y*m[3].z*m[1].w - m[3].y*m[2].z*m[1].w + m[3].y*m[1].z*m[2].w - m[1].y*m[3].z*m[2].w - m[2].y*m[1].z*m[3].w + m[1].y*m[2].z*m[3].w;
   ret.m[1].x = m[3].x*m[2].z*m[1].w - m[2].x*m[3].z*m[1].w - m[3].x*m[1].z*m[2].w + m[1].x*m[3].z*m[2].w + m[2].x*m[1].z*m[3].w - m[1].x*m[2].z*m[3].w;
   ret.m[2].x = m[2].x*m[3].y*m[1].w - m[3].x*m[2].y*m[1].w + m[3].x*m[1].y*m[2].w - m[1].x*m[3].y*m[2].w - m[2].x*m[1].y*m[3].w + m[1].x*m[2].y*m[3].w;
   ret.m[3].x = m[3].x*m[2].y*m[1].z - m[2].x*m[3].y*m[1].z - m[3].x*m[1].y*m[2].z + m[1].x*m[3].y*m[2].z + m[2].x*m[1].y*m[3].z - m[1].x*m[2].y*m[3].z;
   ret.m[0].y = m[3].y*m[2].z*m[0].w - m[2].y*m[3].z*m[0].w - m[3].y*m[0].z*m[2].w + m[0].y*m[3].z*m[2].w + m[2].y*m[0].z*m[3].w - m[0].y*m[2].z*m[3].w;
   ret.m[1].y = m[2].x*m[3].z*m[0].w - m[3].x*m[2].z*m[0].w + m[3].x*m[0].z*m[2].w - m[0].x*m[3].z*m[2].w - m[2].x*m[0].z*m[3].w + m[0].x*m[2].z*m[3].w;
   ret.m[2].y = m[3].x*m[2].y*m[0].w - m[2].x*m[3].y*m[0].w - m[3].x*m[0].y*m[2].w + m[0].x*m[3].y*m[2].w + m[2].x*m[0].y*m[3].w - m[0].x*m[2].y*m[3].w;
   ret.m[3].y = m[2].x*m[3].y*m[0].z - m[3].x*m[2].y*m[0].z + m[3].x*m[0].y*m[2].z - m[0].x*m[3].y*m[2].z - m[2].x*m[0].y*m[3].z + m[0].x*m[2].y*m[3].z;
   ret.m[0].z = m[1].y*m[3].z*m[0].w - m[3].y*m[1].z*m[0].w + m[3].y*m[0].z*m[1].w - m[0].y*m[3].z*m[1].w - m[1].y*m[0].z*m[3].w + m[0].y*m[1].z*m[3].w;
   ret.m[1].z = m[3].x*m[1].z*m[0].w - m[1].x*m[3].z*m[0].w - m[3].x*m[0].z*m[1].w + m[0].x*m[3].z*m[1].w + m[1].x*m[0].z*m[3].w - m[0].x*m[1].z*m[3].w;
   ret.m[2].z = m[1].x*m[3].y*m[0].w - m[3].x*m[1].y*m[0].w + m[3].x*m[0].y*m[1].w - m[0].x*m[3].y*m[1].w - m[1].x*m[0].y*m[3].w + m[0].x*m[1].y*m[3].w;
   ret.m[3].z = m[3].x*m[1].y*m[0].z - m[1].x*m[3].y*m[0].z - m[3].x*m[0].y*m[1].z + m[0].x*m[3].y*m[1].z + m[1].x*m[0].y*m[3].z - m[0].x*m[1].y*m[3].z;
   ret.m[0].w = m[2].y*m[1].z*m[0].w - m[1].y*m[2].z*m[0].w - m[2].y*m[0].z*m[1].w + m[0].y*m[2].z*m[1].w + m[1].y*m[0].z*m[2].w - m[0].y*m[1].z*m[2].w;
   ret.m[1].w = m[1].x*m[2].z*m[0].w - m[2].x*m[1].z*m[0].w + m[2].x*m[0].z*m[1].w - m[0].x*m[2].z*m[1].w - m[1].x*m[0].z*m[2].w + m[0].x*m[1].z*m[2].w;
   ret.m[2].w = m[2].x*m[1].y*m[0].w - m[1].x*m[2].y*m[0].w - m[2].x*m[0].y*m[1].w + m[0].x*m[2].y*m[1].w + m[1].x*m[0].y*m[2].w - m[0].x*m[1].y*m[2].w;
   ret.m[3].w = m[1].x*m[2].y*m[0].z - m[2].x*m[1].y*m[0].z + m[2].x*m[0].y*m[1].z - m[0].x*m[2].y*m[1].z - m[1].x*m[0].y*m[2].z + m[0].x*m[1].y*m[2].z;
   double det     = m[0].x* ret.m[0].x + m[1].x* ret[0].y  + m[2].x* ret[0].z + m[3].x* ret[0].w;
   double invdet  = 1 / det;
   ret.m[0] *= invdet;
   ret.m[1] *= invdet;
   ret.m[2] *= invdet;
   ret.m[3] *= invdet;
   return ret;
}

Matrix4x4 Matrix4x4::pow( int pwr ) const
{
   Matrix4x4 ret;
   for ( int i = 0; i < abs(pwr); i++ )
      ret = ret * *this;
   return pwr > 0 ? ret : ret.inverted();
}

bool Matrix4x4::eq( const Matrix4x4& rhs, double tolerance ) const
{
   for ( int i = 0; i < 4; i++ )
      if ( !m[i].eq( rhs.m[i], tolerance ) )
         return false;
   return true;
}


XYZ::XYZ( const Json& json ) { x = json[0].toDouble(); y = json[1].toDouble(); z = json[2].toDouble(); }

Json XYZ::toJson() const { return JsonArray( {x,y,z} ); }
Json XYZW::toJson() const { return JsonArray( {x,y,z,w} ); }
XYZW::XYZW( const Json& json ) { x=json[0].toDouble(); y=json[1].toDouble(); z=json[2].toDouble(); w=json[3].toDouble();  }

Json Matrix4x4::toJson() const
{
   return JsonArray( {m[0].toJson(),m[1].toJson(),m[2].toJson(),m[3].toJson()} );
}
Matrix4x4::Matrix4x4( const Json& json )
{
   m[0] = XYZW( json[0] );
   m[1] = XYZW( json[1] );
   m[2] = XYZW( json[2] );
   m[3] = XYZW( json[3] );
}