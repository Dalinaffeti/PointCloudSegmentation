#pragma once

#include <QtWidgets/QWidget>
#include "ui_PunktewolkeSegmentierung.h"


class PunktewolkeSegmentierung : public QWidget
{
    Q_OBJECT

public:
    PunktewolkeSegmentierung(QWidget *parent = Q_NULLPTR);

private:
    Ui::PunktewolkeSegmentierungClass ui;
};
