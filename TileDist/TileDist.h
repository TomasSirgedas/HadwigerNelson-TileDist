#pragma once

#include <QtWidgets/QWidget>
#include "ui_TileDist.h"

#include "DataTypes.h"
#include <memory>

class Drawing;
class Simulation;

class TileDist : public QWidget
{
   Q_OBJECT

public:
   TileDist( QWidget* parent = Q_NULLPTR );
   void redraw();

private:
   Ui::TileDistClass ui;

   Drawing* _Drawing;
   std::shared_ptr<Simulation> _Simulation;
};
