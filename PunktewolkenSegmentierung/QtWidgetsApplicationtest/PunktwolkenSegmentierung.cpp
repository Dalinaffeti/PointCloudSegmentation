#include "PunktwolkenSegmentierung.h"
#include "pclviewer.h"
#include <QProgressBar>
#include <QMessageBox>
#include <QFileDialog>
#include <QDesktopServices>
#include <QUrl>


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
    
    connect(ui.actionNew, SIGNAL(triggered(bool)), this, SLOT(resetAll()));
    connect(ui.actionSave, SIGNAL(triggered(bool)), this, SLOT(exportResults()));
    connect(ui.actionSaveAs, SIGNAL(triggered(bool)), this, SLOT(exportResults()));
    connect(ui.actionOpen, SIGNAL(triggered(bool)), this, SLOT(importPCFile()));
    connect(ui.actionDocs, SIGNAL(triggered(bool)), this, SLOT(openDocs()));
    
    resetAll();
}

/// <summary>
/// Setzt beim Start alles zurück + falls eine neue Datei ausgewählt wird, soll alles resettet werden.
/// </summary>
void PunktwolkenSegmentierung::resetAll() {
    // Resultate reset, Viewer schwarz, fps reset, dateiname reset
    QString empty = ui.segmentResults->toPlainText();
    ui.filename->setText("");
    ui.segmentResults->setText("");
    ui.actionZoomIn->setEnabled(false);
    ui.actionZoomOut->setEnabled(false);
    ui.actionRotate->setEnabled(false);
    ui.actionSwitchView->setEnabled(false);
}

/// <summary>
/// Exportieren der Punktwolke
/// </summary>
void PunktwolkenSegmentierung::exportResults() {

    // If abfrage: wenn keine resultate bzw. keine Punktwolke hochgeladen wurde/ nichts zum speichern/export gibt, Error Message box
    QMessageBox box;
    box.setText("Keine Datei zum Exportieren!");
    box.setIcon(QMessageBox::Critical);
    box.addButton("OK", QMessageBox::AcceptRole);
    box.exec();

    // else export segementresults -> Dateiformat sollte ply oder asc sein
    // https://docs.fileformat.com/3d/ply/ - beispiel wie eine .ply aussieht https://people.sc.fsu.edu/~jburkardt/data/ply/ply.html

   
    return;
}

/// <summary>
/// Import der Punktwolke
/// </summary>
void PunktwolkenSegmentierung::importPCFile() {
    QString fileName = QFileDialog::getOpenFileName(this, 
        tr("Open Point Cloud file"), "",
        tr("PC Format (*.ply *.asc)"));
}

/// <summary>
/// Öffnet die Readme URL aus gitlab - 
/// </summary>
void PunktwolkenSegmentierung::openDocs() {
 
    QDesktopServices::openUrl(QUrl("https://gitlab.rz.htw-berlin.de/softwareentwicklungsprojekt/wise2021-22/team8/-/blob/master/readme.md", QUrl::TolerantMode));
}