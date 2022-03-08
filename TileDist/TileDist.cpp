#include "TileDist.h"
#include "Util.h"
#include "Delauney.h"

#include <QPainter>
#include <QLabel>
#include <QMouseEvent>
#include <QShortCut>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>

#include <vector>
#include <functional>
#include <unordered_set>
#include <unordered_map>

namespace
{
   void killFocus( QWidget* w )
   {
      if ( w->hasFocus() )
         QApplication::focusWidget()->clearFocus();
   }
   double toDouble( const QString& s, double default=0 )
   {
      bool ok;
      double ret = s.toDouble( &ok );
      return ok ? ret : default;
   }
}


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
      double scale = 2.4;
      _U = XYZ( 1, 0, 0 ) * scale;
      _V = XYZ( .5, sqrt(.75), 0 ) * scale;
      _InvUV = Matrix4x4( XYZW( _U.x, _U.y, 0, 0 ), XYZW( _V.x, _V.y, 0, 0 ), XYZW( 0, 0, 1, 0 ), XYZW( 0, 0, 0, 1 ) ).inverted();
   }

   XYZ pos( const Sector& sector ) const { return _U * sector.x + _V * sector.y; }
   XYZ pos( const XYZ& position, const Sector& sector ) const override { return position + pos( sector ); }
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
   Sector sectorAt( const XYZ& p ) const
   {
      XYZW uv = _InvUV * p;
      return Sector{ (int)floor( uv.x ), (int)floor( uv.y ) };
   }
   XYZ normalizedPos( const XYZ& p ) const
   {
      return p - pos( sectorAt( p ) );
   }
   void step()
   {
      constexpr double MAX_VEL = .1;

      std::vector<XYZ> vel( _Vertices.size() );

      for ( const VertexPtr& a : rawVertices() )
      {
         XYZ posA = a.pos();
         for ( const VertexPtr& b : vertices() )
         {
            if ( a == b )
               continue;
            XYZ posB = b.pos();

            double minDist = a.color() == b.color() ? _MinDistanceAllowed_SameColor : _MinDistanceAllowed;

            double dist2 = posA.dist2( posB );
            if ( dist2 >= minDist*minDist ) 
               continue;
            double dist = sqrt( dist2 );
            double distError = minDist - dist;

            vel[a.rawIndex()] += ( posA - posB ).normalized() * distError * _Tension;
         }
      }

      for ( const VertexPtr& a : rawVertices() )
      {
         if ( vel[a.rawIndex()].len() > .1 )
            vel[a.rawIndex()] = vel[a.rawIndex()].normalized() * MAX_VEL;
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

   void addVertex( const XYZ& pos, int color )
   {
      Vertex a { (int) _Vertices.size(), color, pos };
      _Vertices.push_back( a );
   }

   void deleteVertex( const VertexPtr& a )
   {  
      int index = a.rawIndex();
      if ( index < 0 || index >= (int) _Vertices.size() )
         return;

      _Vertices.erase( _Vertices.begin() + index );

      // renumber
      for ( int i = index; i < (int) _Vertices.size(); i++ )
         _Vertices[i]._Index = i;
   }

   std::vector<VertexPtr> verticesInRange( double R ) const
   {
      std::vector<VertexPtr> ret;
      Sector sector;
      for ( sector.y = -10; sector.y <= 10; sector.y++ )
      for ( sector.x = -10; sector.x <= 10; sector.x++ )
      for ( const Vertex& aa : _Vertices )
      {
         VertexPtr a( &aa, sector, this );
         XYZ pos = a.pos();
         if ( pos.len2() > R*R )
            continue;
         ret.push_back( a );
      }
      return ret;
   }

public:
   std::vector<Vertex> _Vertices;
   double _MinDistanceAllowed = .75;
   double _MinDistanceAllowed_SameColor = 2.;
   XYZ _U;
   XYZ _V;
   Matrix4x4 _InvUV;
   double _Tension = 0;

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
         if ( _ShowTriangulation )
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
   bool _ShowTriangulation;

public:
   Matrix4x4 _ModelToBitmap;
   const Simulation* _Simulation;
};





TileDist::TileDist( QWidget* parent )
   : QWidget( parent )
{
   _Simulation.reset( new Simulation );   
   _Simulation->addVertex( XYZ( 0, 0, 0 ), 0 );
   _Simulation->addVertex( XYZ( 1, 0, 0 ), 1 );
   _Simulation->addVertex( XYZ( 2, 0.1, 0 ), 2 );
   _Simulation->addVertex( XYZ( 2.5, 0, 0 ), 3 );
   _Simulation->addVertex( XYZ( 1, 1, 0 ), 4 );
   _Simulation->addVertex( XYZ( 2, 1, 0 ), 5 );
   _Simulation->addVertex( XYZ( 3, 1.1, 0 ), 6 );
   _Simulation->addVertex( XYZ( 2.1, 2, 0 ), 7 );

   ui.setupUi( this );
   ui.horizontalLayout->removeWidget( ui.drawingPlaceholder );

   _Drawing = new Drawing();
   _Drawing->_Simulation = _Simulation.get();
   ui.horizontalLayout->insertWidget( 0, _Drawing );


   _Drawing->_OnLeftPressFunc = [this]( XYZ clickPos )
   {
      _Simulation->_ClickedVertex = _Simulation->vertexAt( clickPos, _Drawing->toModel( 30 ) );
      //redraw();
      _Drawing->_OnMouseMoveFunc( clickPos );
   };
   _Drawing->_OnLeftReleaseFunc = [this]( XYZ clickPos )
   {
      _Simulation->_ClickedVertex = VertexPtr();
      redraw();
   };
   _Drawing->_OnMouseMoveFunc = [this]( XYZ clickPos )
   {
      if ( _Simulation->_ClickedVertex )
         _Simulation->setPos( _Simulation->_ClickedVertex, clickPos );
      redraw();
   };

   connect( &_PlayTimer, &QTimer::timeout, [this] { _Simulation->step( 50 ); redraw(); } );

   connect( ui.playButton, &QPushButton::clicked, [this]() 
   { 
      if ( _PlayTimer.isActive() )
         _PlayTimer.stop();
      else
         _PlayTimer.start();   

      ui.playButton->setText( _PlayTimer.isActive() ? "||" : ">" );
   } );
   connect( ui.exportButton, &QPushButton::clicked, [this]() 
   { 
      exportAsDual();
   } );

   connect( ui.tensionSlider, &QSlider::valueChanged, [this]( int value ) {
      double t = (double) value / ui.tensionSlider->maximum();
      //ui.outerRadiusEdit->setText( QString("%1").arg( r ) );
      _Simulation->_Tension = interpolateExp( t, .001, 1. );
      //updateModelFromUI();
      redraw();
   } );
   ui.tensionSlider->valueChanged( ui.tensionSlider->value() );

   connect( ui.showTriangulationCheckBox, &QCheckBox::toggled, [this]() {
      _Drawing->_ShowTriangulation = ui.showTriangulationCheckBox->isChecked();
      redraw();
   } );
   ui.showTriangulationCheckBox->toggled( ui.showTriangulationCheckBox->isChecked() );

   connect( ui.uxLineEdit, &QLineEdit::editingFinished, [this]() {
      _Simulation->_U.x = ui.uxLineEdit->text().toDouble();
      killFocus( ui.uxLineEdit );
      redraw();
   } );
   connect( ui.uyLineEdit, &QLineEdit::editingFinished, [this]() {
      _Simulation->_U.y = ui.uyLineEdit->text().toDouble();
      killFocus( ui.uyLineEdit );
      redraw();
   } );
   connect( ui.vxLineEdit, &QLineEdit::editingFinished, [this]() {
      _Simulation->_V.x = ui.vxLineEdit->text().toDouble();
      killFocus( ui.vxLineEdit );
      redraw();
   } );
   connect( ui.vyLineEdit, &QLineEdit::editingFinished, [this]() {
      _Simulation->_V.y = ui.vyLineEdit->text().toDouble();
      killFocus( ui.vyLineEdit );
      redraw();
   } );
   connect( ui.minDistDiffLineEdit, &QLineEdit::editingFinished, [this]() {
      _Simulation->_MinDistanceAllowed = ui.minDistDiffLineEdit->text().toDouble();
      killFocus( ui.minDistDiffLineEdit );
      redraw();
   } );
   connect( ui.minDistSameLineEdit, &QLineEdit::editingFinished, [this]() {
      _Simulation->_MinDistanceAllowed_SameColor = ui.minDistSameLineEdit->text().toDouble();
      killFocus( ui.minDistSameLineEdit );
      redraw();
   } );
   ui.uxLineEdit->setText( QString::number( _Simulation->_U.x ) );
   ui.uyLineEdit->setText( QString::number( _Simulation->_U.y ) );
   ui.vxLineEdit->setText( QString::number( _Simulation->_V.x ) );
   ui.vyLineEdit->setText( QString::number( _Simulation->_V.y ) );
   ui.minDistDiffLineEdit->setText( QString::number( _Simulation->_MinDistanceAllowed ) );
   ui.minDistSameLineEdit->setText( QString::number( _Simulation->_MinDistanceAllowed_SameColor ) );


   _PlayTimer.setInterval( 16 );
   ui.playButton->click();


   QObject::connect( new QShortcut(QKeySequence(Qt::Key_Delete), this ), &QShortcut::activated, [this]() { deleteVertex(); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_0), this ), &QShortcut::activated, [this]() { addVertex( 0 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_R), this ), &QShortcut::activated, [this]() { addVertex( 0 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_1), this ), &QShortcut::activated, [this]() { addVertex( 1 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_B), this ), &QShortcut::activated, [this]() { addVertex( 1 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_2), this ), &QShortcut::activated, [this]() { addVertex( 2 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_Y), this ), &QShortcut::activated, [this]() { addVertex( 2 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_3), this ), &QShortcut::activated, [this]() { addVertex( 3 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_P), this ), &QShortcut::activated, [this]() { addVertex( 3 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_4), this ), &QShortcut::activated, [this]() { addVertex( 4 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_G), this ), &QShortcut::activated, [this]() { addVertex( 4 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_5), this ), &QShortcut::activated, [this]() { addVertex( 5 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_O), this ), &QShortcut::activated, [this]() { addVertex( 5 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_6), this ), &QShortcut::activated, [this]() { addVertex( 6 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_C), this ), &QShortcut::activated, [this]() { addVertex( 6 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_7), this ), &QShortcut::activated, [this]() { addVertex( 7 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_W), this ), &QShortcut::activated, [this]() { addVertex( 7 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_8), this ), &QShortcut::activated, [this]() { addVertex( 8 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_K), this ), &QShortcut::activated, [this]() { addVertex( 8 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_9), this ), &QShortcut::activated, [this]() { addVertex( 9 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::Key_N), this ), &QShortcut::activated, [this]() { addVertex( 9 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::SHIFT + Qt::Key_0), this ), &QShortcut::activated, [this]() { addVertex( 10 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::SHIFT + Qt::Key_1), this ), &QShortcut::activated, [this]() { addVertex( 11 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::SHIFT + Qt::Key_2), this ), &QShortcut::activated, [this]() { addVertex( 12 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::SHIFT + Qt::Key_3), this ), &QShortcut::activated, [this]() { addVertex( 13 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::SHIFT + Qt::Key_4), this ), &QShortcut::activated, [this]() { addVertex( 14 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::SHIFT + Qt::Key_5), this ), &QShortcut::activated, [this]() { addVertex( 15 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::SHIFT + Qt::Key_6), this ), &QShortcut::activated, [this]() { addVertex( 16 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::SHIFT + Qt::Key_7), this ), &QShortcut::activated, [this]() { addVertex( 17 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::SHIFT + Qt::Key_8), this ), &QShortcut::activated, [this]() { addVertex( 18 ); } );
   QObject::connect( new QShortcut(QKeySequence(Qt::SHIFT + Qt::Key_9), this ), &QShortcut::activated, [this]() { addVertex( 19 ); } );
}

void TileDist::redraw()
{
   _Drawing->updateBitmap();
}

XYZ TileDist::mousePos() const
{
   return _Drawing->toModel( _Drawing->mapFromGlobal( QCursor::pos() ) );
}

void TileDist::addVertex( int color )
{
   _Simulation->addVertex( mousePos(), color );
}

void TileDist::deleteVertex()
{
   VertexPtr a = _Simulation->vertexAt( mousePos(), _Drawing->toModel( 30 ) );
   if ( !a )
      return;
   _Simulation->deleteVertex( a );
}

QJsonArray toJson( const XYZ& p )
{
   return QJsonArray { p.x, p.y, p.z };
}

QJsonArray neighborsToJson( const std::vector<int>& v )
{
   QJsonArray ret;
   for ( int x : v )
      ret.append( QJsonObject { {"index", x}, {"sector", 0} } );
   return ret;
}

QJsonObject toJson( const std::vector<Vertex>& vertices, const std::vector<std::vector<int>>& neighbors )
{
   QJsonArray vertexArray;
   for ( int i = 0; i < (int) vertices.size(); i++ )
   {
      const auto& a = vertices[i];
      vertexArray.append( QJsonObject { {"index", i}, {"color", vertices[i]._Color}, {"pos", toJson( a._Pos ) }, {"neighbors", neighborsToJson( neighbors[i] )} } );
   }

   return QJsonObject { { "symmetry", QJsonValue() }, { "shape", QJsonObject { { "type", "plane" } } }, { "vertices", vertexArray } };   
}

void TileDist::exportAsDual()
{   
   double R = ui.exportRadiusLineEdit->text().toDouble();
   std::vector<VertexPtr> vertices = _Simulation->verticesInRange( R + 2 );
   std::sort( vertices.begin(), vertices.end(), []( const VertexPtr& a, const VertexPtr& b ) { return a.pos().len2() < b.pos().len2(); } );
   int numValidVertices;
   for ( numValidVertices = 0; numValidVertices < (int) vertices.size(); numValidVertices++ )
      if ( vertices[numValidVertices].pos().len2() >= R*R )
         break;
   std::vector<std::vector<int>> triangulation;
   {
      std::vector<XYZ> v;
      for ( int i = 0; i < (int) vertices.size(); i++ )
         v.push_back( vertices[i].pos() );
      triangulation = delauney( v );
   }

   // construct output graph
   std::vector<Vertex> v;
   std::vector<std::vector<int>> neighbors( numValidVertices );
   {
      std::vector<std::unordered_set<int>> neighbs( numValidVertices );
      for ( const std::vector<int>& v : triangulation )
      {
         if ( v[0] >= numValidVertices || v[1] >= numValidVertices || v[2] >= numValidVertices ) continue;
         neighbs[v[0]].insert( v[1] );
         neighbs[v[0]].insert( v[2] );
         neighbs[v[1]].insert( v[2] );
         neighbs[v[1]].insert( v[0] );
         neighbs[v[2]].insert( v[0] );
         neighbs[v[2]].insert( v[1] );
      }

      for ( int i = 0; i < numValidVertices; i++ )
         v.push_back( Vertex { i, vertices[i].color(), vertices[i].pos() } );
      for ( int i = 0; i < numValidVertices; i++ )
         neighbors[i] = std::vector<int>( neighbs[i].begin(), neighbs[i].end() );
   }

   // write to file
   {
      QString filename = "test.dual";
      QFile f( filename );
      f.open( QFile::WriteOnly );
      f.write( QJsonDocument( toJson( v, neighbors ) ).toJson() );
   }
}
