#include "Punktwolkensegmentierung.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Punktwolkensegmentierung w;
    w.show();
    return a.exec();
}
