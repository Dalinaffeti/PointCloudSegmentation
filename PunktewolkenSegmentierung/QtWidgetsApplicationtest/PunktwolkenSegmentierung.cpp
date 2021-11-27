#include "PunktwolkenSegmentierung.h"
#include "pclviewer.h"
#include <QProgressBar>
PunktwolkenSegmentierung::PunktwolkenSegmentierung(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    
    ui.statusBar->showMessage("whazzup");
    QProgressBar* progressBar = new QProgressBar(this);
    progressBar->setMinimum(0);
    progressBar->setMaximum(100);
    progressBar->setValue(80);
    ui.statusBar->addPermanentWidget(progressBar);
}
