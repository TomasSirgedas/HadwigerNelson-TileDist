#include "TileDist.h"
#include "Util.h"
#include "Delauney.h"
#include <QPainter>
#include <QLabel>
#include <QMouseEvent>
#include <vector>
#include <functional>


class Vertex
{
public:
   int _Index;
   int _Color;
   XYZ _Pos;
};

class Sector
{
public:
   bool operator==( const Sector& rhs ) const { return x == rhs.x && y == rhs.y; }
   Sector operator-() const { return {-x, -y}; }

public:
   int x, y;
};

class IGraphShape
{
public:
   virtual XYZ pos( const XYZ& position, const Sector& sector ) const = 0;
};

class VertexPtr
{
public:
   VertexPtr( const Vertex* vertex, const Sector& sector, const IGraphShape* graphShape )
      : _Vertex( vertex ), _Sector( sector ), _GraphShape( graphShape )
   {
   }
   VertexPtr() {}

   bool operator==( const VertexPtr& rhs ) const { return _Vertex == rhs._Vertex && _Sector == rhs._Sector; }
   operator bool() const { return !isNull(); }
   bool isNull() const { return _Vertex == nullptr; }
   int color() const { return _Vertex->_Color; }
   XYZ pos() const { return _GraphShape->pos( _Vertex->_Pos, _Sector ); }
   int rawIndex() const { return _Vertex->_Index; }

public:
   const IGraphShape* _GraphShape = nullptr;
   const Vertex* _Vertex = nullptr;
   Sector _Sector;
};

class Simulation : public IGraphShape
{
public:
   Simulation()
   {
      double scale = 3.1;
      _U = XYZ( 1, 0, 0 ) * scale;
      _V = XYZ( .5, sqrt(.75), 0 ) * scale;
      _InvUV = Matrix4x4( XYZW( _U.x, _U.y, 0, 0 ), XYZW( _V.x, _V.y, 0, 0 ), XYZW( 0, 0, 1, 0 ), XYZW( 0, 0, 0, 1 ) ).inverted();
   }

   XYZ pos( const XYZ& position, const Sector& sector ) const override { return position + _U * sector.x + _V * sector.y; }
   std::vector<Sector> sectors() const
   {
      std::vector<Sector> ret;
      for ( int y = -1; y <= 1; y++ )
         for ( int x = -1; x <= 1; x++ )
            ret.push_back( { x, y } );
      return ret;
   }
   std::vector<VertexPtr> vertices() const
   {
      std::vector<VertexPtr> ret;
      for ( const Sector& sector : sectors() )
      {
         for ( const Vertex& a : _Vertices )
         {
            ret.push_back( VertexPtr( &a, sector, this ) );
         }
      }
      return ret;
   }
   std::vector<VertexPtr> rawVertices() const
   {
      std::vector<VertexPtr> ret;
      Sector sector = { 0, 0 };
      for ( const Vertex& a : _Vertices )
      {
         ret.push_back( VertexPtr( &a, sector, this ) );
      }
      return ret;
   }
   VertexPtr vertexAt( const XYZ& pos, double maxDist ) const
   {
      VertexPtr ret;
      double bestDist2 = maxDist * maxDist;
      for ( const VertexPtr& a : vertices() )
      {
         double dist2 = a.pos().dist2( pos );
         if ( dist2 >= bestDist2 )
            continue;
         bestDist2 = dist2;
         ret = a;
      }
      return ret;
   }
   Vertex* mutableOf( const Vertex* vertex ) const { return const_cast<Vertex*>( vertex ); }
   void setPos( const VertexPtr& a, const XYZ& pos )
   {
      //XYZ absolutePos = pos( pos, -a._Sector );
      //mutableOf( a._Vertex )->_Pos = absolutePos;

      XYZW uv = _InvUV * pos;
      mutableOf( a._Vertex )->_Pos = normalizedPos( pos );
   }
   XYZ normalizedPos( const XYZ& p ) const
   {
      XYZW uv = _InvUV * p;
      return p - ( _U * floor( uv.x ) + _V * floor( uv.y ) );
   }

   void step()
   {
      double D = 1.;

      std::vector<XYZ> vel( _Vertices.size() );

      for ( const VertexPtr& a : rawVertices() )
      {
         XYZ posA = a.pos();
         for ( const VertexPtr& b : vertices() )
         {
            if ( a == b )
               continue;
            XYZ posB = b.pos();

            double dist2 = posA.dist2( posB );
            if ( dist2 >= D*D ) 
               continue;
            double dist = sqrt( dist2 );
            double distError = D - dist;

            vel[a.rawIndex()] += ( posA - posB ).normalized() * distError * .1;
         }
      }

      for ( const VertexPtr& a : rawVertices() )
      {
         if ( vel[a.rawIndex()].len() > .1 )
            vel[a.rawIndex()] = vel[a.rawIndex()].normalized() * .1;
      }

      for ( const VertexPtr& a : rawVertices() )
      {
         if ( a._Vertex != _ClickedVertex._Vertex )
            setPos( a, a.pos() + vel[a.rawIndex()] );
      }
   }

   void step( int numSteps )
   {
      for ( int i = 0; i < numSteps; i++ )
         step();
   }

public:
   std::vector<Vertex> _Vertices;
   XYZ _U;
   XYZ _V;
   Matrix4x4 _InvUV;

public:
   VertexPtr _ClickedVertex;
};

class Drawing : public QWidget
{
public:
   Drawing()
   {
      _Layout = new QVBoxLayout( this );

      _Label = new QLabel;
      _Label->setSizePolicy( QSizePolicy::Expanding, QSizePolicy::Expanding );

      {
         _LabelContainer = new QWidget;
         _LabelContainer->setContentsMargins( 0, 0, 0, 0 );
         QLayout* labelContainerLayout = new QVBoxLayout;
         labelContainerLayout->setMargin( 0 );
         _LabelContainer->setLayout( labelContainerLayout );
         _LabelContainer->layout()->addWidget( _Label );
         _LabelContainer->setSizePolicy( QSizePolicy::Ignored, QSizePolicy::Ignored );
      }

      _Layout->addWidget( _LabelContainer );
      _Layout->setMargin( 0 );
   }

   void resizeEvent( QResizeEvent* event ) override
   {
      updateBitmap();
   }

   QPointF toBitmap( const XYZ& pt ) const
   {
      return ::toPointF( (_ModelToBitmap * pt).toXYZ() );
   }
   XYZ toModel( const QPointF& pt ) const
   {
      return (_ModelToBitmap.inverted() * XYZ( pt.x(), pt.y(), 0 )).toXYZ();
   }
   double toModel( double dist ) const
   {
      return (_ModelToBitmap.inverted() * XYZ( dist, 0, 0 )).x - (_ModelToBitmap.inverted() * XYZ( 0, 0, 0 )).x;
   }

   void updateBitmap()
   {
      _ModelToBitmap = Matrix4x4::translation( XYZ( width()/2, height()/2, 0 ) ) * Matrix4x4::scale( XYZ( 100, 100, 1 ) ) * Matrix4x4::scale( XYZ( 1, -1, 1 ) );

      auto toBitmap = [&]( const XYZ& pt ) { return toPointF( (_ModelToBitmap * pt).toXYZ() ); };

      QImage img( size(), QImage::Format_RGB32 );
      img.fill( QColor( 120,120,120 ) );


      {
         QPainter painter( &img );
         painter.setRenderHint( QPainter::Antialiasing );

         // draw delauney
         {
            painter.setPen( QPen( QColor( 0, 0, 0, 64 ), 1 ) );

            std::vector<VertexPtr> vertices = _Simulation->vertices();
            std::vector<XYZ> v;
            for ( const VertexPtr& a : vertices )
               v.push_back( a.pos() );
            auto triangulation = delauney( v );
            for ( const auto& tri : triangulation )
            {
               painter.drawLine( toBitmap( vertices[tri[0]].pos() ), toBitmap( vertices[tri[1]].pos() ) );
               painter.drawLine( toBitmap( vertices[tri[1]].pos() ), toBitmap( vertices[tri[2]].pos() ) );
               painter.drawLine( toBitmap( vertices[tri[2]].pos() ), toBitmap( vertices[tri[0]].pos() ) );
            }
         }

         // draw _U, _V
         painter.setPen( QPen( QColor( 128, 0, 0, 64 ), 5 ) );
         {
            painter.drawLine( toBitmap( XYZ(0,0,0) ), toBitmap( _Simulation->_U ) );
            painter.drawLine( toBitmap( XYZ(0,0,0) ), toBitmap( _Simulation->_V ) );
            painter.drawLine( toBitmap( _Simulation->_U + _Simulation->_V ), toBitmap( _Simulation->_U ) );
            painter.drawLine( toBitmap( _Simulation->_U + _Simulation->_V ), toBitmap( _Simulation->_V ) );
         }

         //// draw vertices
         //for ( int sectorX = -1; sectorX <= 1; sectorX++ )
         //for ( int sectorY = -1; sectorY <= 1; sectorY++ )
         //{
         //   painter.setPen( QPen( QColor( 0, 0, 0 ), 1 ) );
         //   for ( const Vertex& a : _Simulation->_Vertices )
         //   {
         //      painter.setBrush( tileColor( a._Color ) );
         //      XYZ sectorOffset = _Simulation->_U * sectorX + _Simulation->_V * sectorY;
         //      XYZ pos = a._Pos + sectorOffset;
         //      painter.drawEllipse( toBitmap( pos ), 4, 4 );
         //   }
         //}

         // draw vertices
         painter.setPen( QPen( QColor( 0, 0, 0 ), 1 ) );
         for ( const VertexPtr& a : _Simulation->vertices() )
         {
            painter.setBrush( tileColor( a.color() ) );
            XYZ pos = a.pos();
            painter.drawEllipse( toBitmap( pos ), 4, 4 );

            //if ( a == _Simulation->_ClickedVertex )
            //   painter.drawEllipse( toBitmap( pos ), 6, 6 );
         }
      }
      _Label->setPixmap( QPixmap::fromImage( img ) );
   }

   void mousePressEvent( QMouseEvent* event ) override { _OnLeftPressFunc( toModel( event->pos() ) ); }
   void mouseReleaseEvent( QMouseEvent* event ) override { _OnLeftReleaseFunc( toModel( event->pos() ) ); }
   void mouseMoveEvent( QMouseEvent* event ) override { _OnMouseMoveFunc( toModel( event->pos() ) ); }

public:
   QVBoxLayout* _Layout;
   QLabel* _Label;
   QWidget* _LabelContainer;

public:
   std::function<void(XYZ)> _OnLeftPressFunc;
   std::function<void(XYZ)> _OnLeftReleaseFunc;
   std::function<void(XYZ)> _OnMouseMoveFunc;

public:
   Matrix4x4 _ModelToBitmap;
   const Simulation* _Simulation;
};





TileDist::TileDist( QWidget* parent )
   : QWidget( parent )
{
   _Simulation.reset( new Simulation );   
   _Simulation->_Vertices.push_back( Vertex { 0, 0, XYZ( 0, 0, 0 ) } );
   _Simulation->_Vertices.push_back( Vertex { 1, 1, XYZ( 1, 0, 0 ) } );
   _Simulation->_Vertices.push_back( Vertex { 2, 2, XYZ( 2, 0.1, 0 ) } );
   _Simulation->_Vertices.push_back( Vertex { 3, 3, XYZ( 2.5, 0, 0 ) } );
   _Simulation->_Vertices.push_back( Vertex { 4, 4, XYZ( 1, 1, 0 ) } );
   _Simulation->_Vertices.push_back( Vertex { 5, 5, XYZ( 2, 1, 0 ) } );
   _Simulation->_Vertices.push_back( Vertex { 6, 6, XYZ( 3, 1.1, 0 ) } );
   _Simulation->_Vertices.push_back( Vertex { 7, 7, XYZ( 2.1, 2, 0 ) } );

   ui.setupUi( this );
   ui.horizontalLayout->removeWidget( ui.drawingPlaceholder );

   _Drawing = new Drawing();
   _Drawing->_Simulation = _Simulation.get();
   ui.horizontalLayout->insertWidget( 0, _Drawing );


   _Drawing->_OnLeftPressFunc = [&]( XYZ clickPos )
   {
      _Simulation->_ClickedVertex = _Simulation->vertexAt( clickPos, _Drawing->toModel( 5 ) );
      redraw();
   };
   _Drawing->_OnLeftReleaseFunc = [&]( XYZ clickPos )
   {
      _Simulation->_ClickedVertex = VertexPtr();
      redraw();
   };
   _Drawing->_OnMouseMoveFunc = [&]( XYZ clickPos )
   {
      if ( _Simulation->_ClickedVertex )
         _Simulation->setPos( _Simulation->_ClickedVertex, clickPos );
      redraw();
   };

   connect( &_PlayTimer, &QTimer::timeout, [&] { _Simulation->step( 50 ); redraw(); } );

   connect( ui.playButton, &QPushButton::clicked, [&]() 
   { 
      if ( _PlayTimer.isActive() )
         _PlayTimer.stop();
      else
         _PlayTimer.start();   
   } );


   _PlayTimer.setInterval( 16 );
   ui.playButton->click();
}

void TileDist::redraw()
{
   _Drawing->updateBitmap();
}