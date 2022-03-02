#pragma once

//#include "CoreMacros.h"
#include "Json.h"
#include <cmath>
#include <vector>

#define PI 3.14159265359

#define CORE_API

class CORE_API XYZ
{
public:
   XYZ( double px, double py, double pz ) { set( px, py, pz ); }
   XYZ() { set( 0, 0, 0 ); }
   XYZ( const Json& json );
   ~XYZ() {}
   void set( double px, double py, double pz ) { x = px; y = py; z = pz; }

   double len2() const { return x*x + y*y + z*z; }
   double len() const { return sqrt( len2() ); }
   double dist2( const XYZ& p ) const { return (*this - p).len2(); }
   double dist( const XYZ& p ) const { return (*this - p).len(); }

   XYZ normalized() const { return *this / len(); }

   XYZ operator-() const { return XYZ( -x, -y, -z ); }
   XYZ operator-( const XYZ& p ) const { return XYZ( x - p.x, y - p.y, z - p.z ); }
   XYZ operator+( const XYZ& p ) const { return XYZ( x + p.x, y + p.y, z + p.z ); }
   XYZ operator*( double iMul ) const { return XYZ( x * iMul, y * iMul, z * iMul ); }
   XYZ operator/( double iDiv ) const { return XYZ( x / iDiv, y / iDiv, z / iDiv ); }
   const XYZ& operator-=( const XYZ& p ) { return *this = *this - p; }
   const XYZ& operator+=( const XYZ& p ) { return *this = *this + p; }
   const XYZ& operator*=( double iMul ) { return *this = *this * iMul; }
   const XYZ& operator/=( double iDiv ) { return *this = *this / iDiv; }

   XYZ operator^( const XYZ& p ) const { return XYZ( y*p.z-z*p.y, z*p.x-x*p.z, x*p.y-y*p.x ); }

   double operator*( const XYZ& p ) const { return x*p.x + y*p.y + z*p.z; }

   bool operator==( const XYZ& p ) const { return x == p.x && y == p.y && z == p.z; }
   bool operator!=( const XYZ& p ) const { return !(*this == p); }
   bool operator<( const XYZ& p ) const { return (z != p.z) ? (z < p.z) : (y != p.y) ? (y < p.y) : (x < p.x); }

   //QPointF toPointF() const { return QPointF( x, y ); }
   //QVector3D toQVector3D() const { return QVector3D( x, y, z ); }

   Json toJson() const;

public:
   double x, y, z;
};


class CORE_API XYZW
{
public:
   XYZW( double x, double y, double z, double w ) : x(x),y(y),z(z),w(w) {}
   XYZW( const XYZ& p ) : x(p.x),y(p.y),z(p.z),w(1.) {}
   XYZW() : x(0), y(0), z(0), w(0) {}
   XYZW( const Json& json );
   ~XYZW() {}
     
   XYZW operator-() const { return XYZW( -x, -y, -z, -w ); }
   XYZW operator-( const XYZW& p ) const { return XYZW( x - p.x, y - p.y, z - p.z, w - p.w ); }
   XYZW operator+( const XYZW& p ) const { return XYZW( x + p.x, y + p.y, z + p.z, w + p.w ); }
   XYZW operator*( double iMul ) const { return XYZW( x * iMul, y * iMul, z * iMul, w * iMul ); }
   XYZW operator/( double iDiv ) const { return XYZW( x / iDiv, y / iDiv, z / iDiv, w / iDiv ); }
   const XYZW& operator-=( const XYZW& p ) { return *this = *this - p; }
   const XYZW& operator+=( const XYZW& p ) { return *this = *this + p; }
   const XYZW& operator*=( double iMul ) { return *this = *this * iMul; }
   const XYZW& operator/=( double iDiv ) { return *this = *this / iDiv; }

   double operator*( const XYZW& p ) const { return x*p.x + y*p.y + z*p.z + w*p.w; }

   bool operator==( const XYZW& p ) const { return x == p.x && y == p.y && z == p.z && w == p.w; }
   bool operator!=( const XYZW& p ) const { return !(*this == p); }
   //bool operator<( const XYZW& p ) const { return (z != p.z) ? (z < p.z) : (y != p.y) ? (y < p.y) : (x < p.x); }

   //QPointF toPointF() const { return QPointF( x, y ); }
   XYZ toXYZ() const { return XYZ( x, y, z ) / w; }
   //QVector3D toQVector3D() const { return toXYZ().toQVector3D(); }

   bool eq( const XYZW& rhs, double tolerance = 1e-8 ) const;

   Json toJson() const;

public:
   double x, y, z, w;
};

class CORE_API Matrix4x4
{
public:
   Matrix4x4() { m[0] = XYZW( 1, 0, 0, 0 ); m[1] = XYZW( 0, 1, 0, 0 ); m[2] = XYZW( 0, 0, 1, 0 ); m[3] = XYZW( 0, 0, 0, 1 ); }
   Matrix4x4( const XYZW& m0, const XYZW& m1, const XYZW& m2, const XYZW& m3 ) { m[0] = m0; m[1] = m1; m[2] = m2; m[3] = m3; }
   Matrix4x4( double m00, double m01, double m02, double m03, 
              double m10, double m11, double m12, double m13, 
              double m20, double m21, double m22, double m23, 
              double m30, double m31, double m32, double m33 ) { m[0] = XYZW(m00,m10,m20,m30); m[1] = XYZW(m01,m11,m21,m31); m[2] = XYZW(m02,m12,m22,m32); m[3] = XYZW(m03,m13,m23,m33); }
   Matrix4x4( const Json& json );
   XYZW& operator[]( int idx ) { return m[idx]; }
   XYZW operator[]( int idx ) const { return m[idx]; }
   XYZW operator*( const XYZW& v ) const { return m[0] * v.x + m[1] * v.y + m[2] * v.z + m[3] * v.w; }
   //XYZ operator*( const XYZ& v ) const { return (*this * XYZW( v )).toXYZ(); }
   //QVector3D operator*( const QVector3D& p ) const { return (*this * XYZ(p.x(),p.y(),p.z())).toQVector3D(); }
   Matrix4x4 operator*( const Matrix4x4& rhs ) const { return Matrix4x4( *this * rhs[0], *this * rhs[1], *this * rhs[2], *this * rhs[3] ); }
   static Matrix4x4 scale( const XYZ& s ) { return Matrix4x4( { s.x, 0, 0, 0 }, { 0, s.y, 0, 0 }, { 0, 0, s.z, 0 }, { 0, 0, 0, 1 } ); }
   static Matrix4x4 translation( const XYZ& d ) { return Matrix4x4( { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { d.x, d.y, d.z, 1 } ); }
   static Matrix4x4 rotationX( double a ) { return Matrix4x4( { 1, 0, 0, 0 }, { 0, cos(a), sin(a), 0 }, { 0, -sin(a), cos(a), 0 }, { 0, 0, 0, 1 } ); }
   static Matrix4x4 rotationY( double a ) { return Matrix4x4( { cos(a), 0, sin(a), 0 }, { 0, 1, 0, 0 }, { -sin(a), 0, cos(a), 0 }, { 0, 0, 0, 1 } ); }
   static Matrix4x4 rotation( const XYZ& axis, double angle ) { XYZ u = axis.normalized(); double cs = cos(angle); double sn = sin(angle);    return Matrix4x4( { cs + u.x*u.x*(1-cs), u.y*u.x*(1-cs) + u.z*sn, u.z*u.x*(1-cs) - u.y*sn, 0 }, { u.x*u.y*(1-cs) - u.z*sn, cs + u.y*u.y*(1-cs), u.z*u.y*(1-cs) + u.x*sn, 0 }, { u.x*u.z*(1-cs) + u.y*sn, u.y*u.z*(1-cs) - u.x*sn, cs + u.z*u.z*(1-cs), 0 }, { 0, 0, 0, 1 } ); }
     
   void translate( double x, double y, double z ) { *this = *this * translation( XYZ(x,y,z) ); }
   double operator()( int r, int c ) const { return r == 0 ? m[c].x : r == 1 ? m[c].y : r == 2 ? m[c].z : m[c].w; }
   void rotateX( double angle ) { *this = *this * rotationX( angle ); }
   void rotateY( double angle ) { *this = *this * rotationY( angle ); }
   void scale( double s ) { *this = *this * scale( XYZ( s, s, s ) ); }
   void scale( double sx, double sy ) { *this = *this * scale( XYZ( sx, sy, 1 ) ); }

   Matrix4x4 inverted() const;

   Matrix4x4 pow( int p ) const;

   bool eq( const Matrix4x4& rhs, double tolerance = 1e-8 ) const;

   Json toJson() const;

public:
   XYZW m[4];
};

class Perm
{
public:
   Perm() {}
   Perm( int n ) { for ( int i = 0; i < n; i++ ) v.push_back( i ); }
   Perm( const std::initializer_list<int>& w ) : v(w.begin(), w.end()) {}
   Perm( const Json& json ) { for( const Json& j : json.toArray() ) v.push_back( j.toInt() ); }

   bool isIdentity() const { return *this == Perm( (int) v.size() ); }
   int operator[]( int idx ) const { if ( idx >= (int) v.size() ) return idx; return v[idx]; }
   //int& operator[]( int idx ) { if ( idx >= v.size() ) return idx; return v[idx]; }

   bool operator==( const Perm& rhs ) const
   {
      if ( v.size() != rhs.v.size() )
         return false;
      for ( int i = 0; i < (int) v.size(); i++ )
         if ( v[i] != rhs.v[i] )
            return false;
      return true;
   }
   Perm operator*( const Perm& rhs ) const
   {
      if ( rhs.v.empty() )
         return *this;
      if ( v.empty() )
         return rhs;

      Perm ret;
      for ( int i = 0; i < (int) v.size(); i++ )
         ret.v.push_back( rhs[v[i]] );
      return ret;
   }
   Perm inverted() const
   {
      Perm ret( v.size() );
      for ( int i = 0; i < (int) v.size(); i++ )
         ret.v[v[i]] = i;
      return ret;
   }
   Perm pow( int n ) const
   {
      Perm ret( v.size() );
      for ( int i = 0; i < abs( n ); i++ )
         ret = *this * ret;
      return n >= 0 ? ret : ret.inverted();
   }
   //bool circularlyMatches( const Perm& rhs ) const
   //{
   //   int idxOffset = rhs.inverted()[v[0]];
   //   for ( int i = 0; i < (int) rhs.v.size(); i++ )
   //      if ( v[i] != rhs[mod(i+idxOffset,(int)v.size())] )
   //         return false;
   //   return true;
   //}

   Json toJson() const { return Json( v ); }

public:
   std::vector<int> v;
};