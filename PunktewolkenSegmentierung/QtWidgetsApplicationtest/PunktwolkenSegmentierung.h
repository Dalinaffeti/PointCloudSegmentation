#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_PunktwolkenSegmentierung.h"

class PunktwolkenSegmentierung : public QMainWindow
{
    Q_OBJECT

public:
    PunktwolkenSegmentierung(QWidget *parent = Q_NULLPTR);

private:
    Ui::QtWidgetsApplicationtestClass ui;
};
