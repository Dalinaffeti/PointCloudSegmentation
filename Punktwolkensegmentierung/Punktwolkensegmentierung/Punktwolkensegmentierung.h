#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_Punktwolkensegmentierung.h"

class Punktwolkensegmentierung : public QMainWindow
{
    Q_OBJECT

public:
    Punktwolkensegmentierung(QWidget *parent = Q_NULLPTR);

private:
    Ui::PunktwolkensegmentierungClass ui;
};
