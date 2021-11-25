#include "PunktwolkenSegmentierung.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    PunktwolkenSegmentierung w;
    w.show();
    return a.exec();
}
