#include "PunktewolkeSegmentierung.h"
#include <QtWidgets/QApplication>
#include "pclviewer.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    PunktewolkeSegmentierung w;
    w.show();
    return a.exec();
}
