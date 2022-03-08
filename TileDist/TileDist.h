#pragma once

#include <QtWidgets/QWidget>
#include "ui_TileDist.h"

#include "DataTypes.h"
#include <memory>
#include <QTimer>

class Drawing;
class Simulation;

class TileDist : public QWidget
{
   Q_OBJECT

public:
   TileDist( QWidget* parent = Q_NULLPTR );
   void redraw();
   void addVertex( int color );
   XYZ mousePos() const;
   void deleteVertex();
   void exportAsDual();

private:
   Ui::TileDistClass ui;

   QTimer _PlayTimer;

   Drawing* _Drawing;
   std::shared_ptr<Simulation> _Simulation;
};
