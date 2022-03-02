#include "TileDist.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    TileDist w;
    w.show();
    return a.exec();
}
