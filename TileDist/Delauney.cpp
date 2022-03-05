#include "delauney.h"
#include "delaunator.hpp"

std::vector<std::vector<int>> delauney( const std::vector<XYZ>& v )
{
   std::vector<double> flattenedCoords;
   for ( const XYZ& p : v )
   {
      flattenedCoords.push_back( p.x );
      flattenedCoords.push_back( p.y );
   }  

   delaunator::Delaunator delaunator( flattenedCoords );

   std::vector<std::vector<int>> ret;
   for ( int i = 0; i < (int)delaunator.triangles.size(); i += 3 )
      ret.push_back( { (int)delaunator.triangles[i+0], (int)delaunator.triangles[i+1], (int)delaunator.triangles[i+2] } );
 
   return ret;
}