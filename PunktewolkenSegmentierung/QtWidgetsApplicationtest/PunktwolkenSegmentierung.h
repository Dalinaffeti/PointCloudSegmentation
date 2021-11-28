#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_PunktwolkenSegmentierung.h"


class PunktwolkenSegmentierung : public QMainWindow
{
    Q_OBJECT

public:
    PunktwolkenSegmentierung(QWidget *parent = Q_NULLPTR);


private slots:
    void resetAll();
    void exportResults();
    void importPCFile();
    void openDocs();
private:
    Ui::QtWidgetsApplicationtestClass ui;

};
