#include "PunktwolkenSegmentierung.h"
#include "pclviewer.h"
#include <QProgressBar>
#include <QMessageBox>
#include <QFileDialog>
#include <QDesktopServices>
#include <QtCore/QVariant>
#include <QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <pcl/io/ascii_io.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <pcl/io/file_io.h>
using namespace pcl::io;


using std::string;



PunktwolkenSegmentierung::PunktwolkenSegmentierung(QWidget* parent)
    : QMainWindow(parent)
{
#pragma region UiCode


    resize(1000, 650);
    setMinimumSize(QSize(800, 650));

    setBaseSize(QSize(800, 650));
    actionNew = new QAction(this);
    actionNew->setObjectName(QString::fromUtf8("actionNew"));
    actionOpen = new QAction(this);
    actionOpen->setObjectName(QString::fromUtf8("actionOpen"));

    actionSave = new QAction(this);
    actionSave->setObjectName(QString::fromUtf8("actionSave"));
    actionSave->setDisabled(true);

    actionSaveAs = new QAction(this);
    actionSaveAs->setObjectName(QString::fromUtf8("actionSaveAs"));
    actionSaveAs->setDisabled(true);

    actionExit = new QAction(this);
    actionExit->setObjectName(QString::fromUtf8("actionExit"));
    actionDocs = new QAction(this);
    actionDocs->setObjectName(QString::fromUtf8("actionDocs"));
    actionZoomIn = new QAction(this);
    actionZoomIn->setObjectName(QString::fromUtf8("actionZoomIn"));
    actionZoomOut = new QAction(this);
    actionZoomOut->setObjectName(QString::fromUtf8("actionZoomOut"));
    actionSwitchView = new QAction(this);
    actionSwitchView->setObjectName(QString::fromUtf8("actionSwitchView"));
    actionRotate = new QAction(this);
    actionRotate->setObjectName(QString::fromUtf8("actionRotate"));
    actionUndo = new QAction(this);
    actionUndo->setObjectName(QString::fromUtf8("actionUndo"));
    actionRedo = new QAction(this);
    actionRedo->setObjectName(QString::fromUtf8("actionRedo"));
    actionCut = new QAction(this);
    actionCut->setObjectName(QString::fromUtf8("actionCut"));
    actionCopy = new QAction(this);
    actionCopy->setObjectName(QString::fromUtf8("actionCopy"));
    actionPaste = new QAction(this);
    actionPaste->setObjectName(QString::fromUtf8("actionPaste"));
    actionSelectAll = new QAction(this);
    actionSelectAll->setObjectName(QString::fromUtf8("actionSelectAll"));
    actionDelete = new QAction(this);
    actionDelete->setObjectName(QString::fromUtf8("actionDelete"));
    centralWidget = new QWidget(this);
    centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
    setCentralWidget(centralWidget);

    horizontalLayout = new QHBoxLayout();
    horizontalLayout->setSpacing(6);
    horizontalLayout->setContentsMargins(11, 11, 11, 11);
    horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
    horizontalLayout->setContentsMargins(0, 0, 0, 0);
    centralWidget->setLayout(horizontalLayout);

    verticalLayout = new QVBoxLayout();
    verticalLayout->setSpacing(6);
    verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
    horizontalLayout->addLayout(verticalLayout);

    filename = new QTextBrowser();
    filename->setObjectName(QString::fromUtf8("filename"));

    QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(filename->sizePolicy().hasHeightForWidth());

    
    filename->setMaximumSize(QSize(300, 100));
    filename->setBaseSize(QSize(180, 30));

    verticalLayout->addWidget(filename);

    importBtn = new QPushButton();
    importBtn->setObjectName(QString::fromUtf8("importBtn"));
    sizePolicy.setHeightForWidth(importBtn->sizePolicy().hasHeightForWidth());
    importBtn->setSizePolicy(sizePolicy);
    importBtn->setMinimumSize(QSize(180, 40));
    importBtn->setMaximumSize(QSize(180, 40));
    importBtn->setBaseSize(QSize(180, 40));
    importBtn->setLayoutDirection(Qt::LeftToRight);

    verticalLayout->addWidget(importBtn, 0, Qt::AlignHCenter);

    segBtn = new QPushButton();
    segBtn->setObjectName(QString::fromUtf8("segBtn"));
    sizePolicy.setHeightForWidth(segBtn->sizePolicy().hasHeightForWidth());
    segBtn->setSizePolicy(sizePolicy);
    segBtn->setMinimumSize(QSize(180, 40));
    segBtn->setMaximumSize(QSize(180, 40));
    segBtn->setBaseSize(QSize(180, 40));
    segBtn->setDisabled(true);

    verticalLayout->addWidget(segBtn, 0, Qt::AlignHCenter);

    saveBtn = new QPushButton();
    saveBtn->setText("Speichern");
    saveBtn->setSizePolicy(sizePolicy);
    saveBtn->setBaseSize(QSize(180, 40));
    saveBtn->setMinimumSize(QSize(180, 40));
    saveBtn->setDisabled(true);
    verticalLayout->addWidget(saveBtn, 0, Qt::AlignHCenter);

    pclviewer = new PCLViewer();
    pclviewer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    pclviewer->setMinimumSize(QSize(500, 500));
    pclviewer->setMaximumSize(QSize(1000, 1000));
    horizontalLayout->addWidget(pclviewer);

    segmentResults = new QTextBrowser();
    segmentResults->setObjectName(QString::fromUtf8("segmentResults"));
    sizePolicy.setHeightForWidth(segmentResults->sizePolicy().hasHeightForWidth());
    QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Preferred);
    segmentResults->setSizePolicy(sizePolicy2);
    segmentResults->setMinimumSize(QSize(300, 300));
    segmentResults->setMaximumSize(QSize(600, 600));

    horizontalLayout->addWidget(segmentResults);

    menuBar = new QMenuBar(this);
    menuBar->setObjectName(QString::fromUtf8("menuBar"));
    menuBar->setGeometry(QRect(0, 0, 800, 21));
    menuFile = new QMenu(menuBar);
    menuFile->setObjectName(QString::fromUtf8("menuFile"));
    menuBearbeiten = new QMenu(menuBar);
    menuBearbeiten->setObjectName(QString::fromUtf8("menuBearbeiten"));
    menuAnsicht = new QMenu(menuBar);
    menuAnsicht->setObjectName(QString::fromUtf8("menuAnsicht"));
    menuHilfe = new QMenu(menuBar);
    menuHilfe->setObjectName(QString::fromUtf8("menuHilfe"));
    setMenuBar(menuBar);
    statusBar = new QStatusBar(this);
    statusBar->setObjectName(QString::fromUtf8("statusBar"));
    statusBar->setMinimumSize(QSize(0, 20));
    setStatusBar(statusBar);

    menuBar->addAction(menuFile->menuAction());

    menuBar->addAction(menuHilfe->menuAction());

    menuFile->addAction(actionOpen);
    menuFile->addSeparator();
    menuFile->addAction(actionSaveAs);
    menuFile->addSeparator();
    menuFile->addAction(actionExit);
    menuBearbeiten->addSeparator();
    menuBearbeiten->addSeparator();
    menuBearbeiten->addAction(actionSelectAll);
    menuHilfe->addAction(actionDocs);
    retranslateUi();
    QObject::connect(actionExit, &QAction::triggered, this, qOverload<>(&QMainWindow::close));
    QObject::connect(actionSave, &QAction::triggered, this, qOverload<>(&QMainWindow::update));

#pragma endregion

    statusBar->showMessage("Willkommen! Bitte eine .ply oder .asc Datei importieren um zu starten.");


    connect(actionNew, SIGNAL(triggered(bool)), this, SLOT(resetAll()));
    connect(actionSave, SIGNAL(triggered(bool)), this, SLOT(exportResults()));
    connect(actionSaveAs, SIGNAL(triggered(bool)), this, SLOT(exportResults()));
    connect(actionOpen, SIGNAL(triggered(bool)), this, SLOT(importPCFile()));
    connect(actionDocs, SIGNAL(triggered(bool)), this, SLOT(openDocs()));

    connect(importBtn, SIGNAL(clicked()), this, SLOT(importPCFile()));
    connect(saveBtn, SIGNAL(clicked()), this, SLOT(exportResults()));
    connect(segBtn, SIGNAL(clicked()), this, SLOT(segmentierung()));

    resetAll();
}

void PunktwolkenSegmentierung::retranslateUi()
{
    setWindowTitle(QCoreApplication::translate("QtWidgetsApplicationtestClass", "QtWidgetsApplicationtest", nullptr));
    actionNew->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Neu", nullptr));
#if QT_CONFIG(shortcut)
    actionNew->setShortcut(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Ctrl+N", nullptr));
#endif // QT_CONFIG(shortcut)
    actionOpen->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "\303\226ffnen..", nullptr));
#if QT_CONFIG(shortcut)
    actionOpen->setShortcut(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Ctrl+O", nullptr));
#endif // QT_CONFIG(shortcut)
    actionSave->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Speichern", nullptr));
#if QT_CONFIG(shortcut)
    actionSave->setShortcut(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Ctrl+S", nullptr));
#endif // QT_CONFIG(shortcut)
    actionSaveAs->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Speichern unter..", nullptr));
    actionExit->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Beenden", nullptr));
#if QT_CONFIG(shortcut)
    actionExit->setShortcut(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Ctrl+Q", nullptr));
#endif // QT_CONFIG(shortcut)
    actionDocs->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Siehe Dokumentation", nullptr));
#if QT_CONFIG(shortcut)
    actionDocs->setShortcut(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Ctrl+H", nullptr));
#endif // QT_CONFIG(shortcut)
    actionZoomIn->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Reinzoomen", nullptr));
    actionZoomOut->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Rauszoomen", nullptr));
    actionSwitchView->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Sichtebene wechseln", nullptr));
    actionRotate->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Drehen", nullptr));
    actionUndo->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "R\303\274ckg\303\244ngig", nullptr));
    actionRedo->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Wiederholen", nullptr));
    actionCut->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Ausschneiden", nullptr));
#if QT_CONFIG(shortcut)
    actionCut->setShortcut(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Ctrl+X", nullptr));
#endif // QT_CONFIG(shortcut)
    actionCopy->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Kopieren", nullptr));
#if QT_CONFIG(shortcut)
    actionCopy->setShortcut(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Ctrl+C", nullptr));
#endif // QT_CONFIG(shortcut)
    actionPaste->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Einf\303\274gen", nullptr));
#if QT_CONFIG(shortcut)
    actionPaste->setShortcut(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Ctrl+V", nullptr));
#endif // QT_CONFIG(shortcut)
    actionSelectAll->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Alles ausw\303\244hlen", nullptr));
#if QT_CONFIG(shortcut)
    actionSelectAll->setShortcut(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Ctrl+A", nullptr));
#endif // QT_CONFIG(shortcut)
    actionDelete->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "L\303\266schen", nullptr));
    filename->setPlaceholderText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Datei Name", nullptr));
    importBtn->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Datei ausw\303\244hlen", nullptr));
    segBtn->setText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Segmentieren", nullptr));
    segmentResults->setHtml(QCoreApplication::translate("QtWidgetsApplicationtestClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
        "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
        "p, li { white-space: pre-wrap; }\n"
        "</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"
        "<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", nullptr));
    segmentResults->setPlaceholderText(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Segmentierungsresultate", nullptr));
    menuFile->setTitle(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Datei", nullptr));
    menuBearbeiten->setTitle(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Bearbeiten", nullptr));
    menuAnsicht->setTitle(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Ansicht", nullptr));
    menuHilfe->setTitle(QCoreApplication::translate("QtWidgetsApplicationtestClass", "Hilfe", nullptr));
}


/// <summary>
/// Setzt beim Start alles zurück + falls eine neue Datei ausgewählt wird, soll alles resettet werden.
/// </summary>
void PunktwolkenSegmentierung::resetAll() {
    QProgressBar* progressBar = new QProgressBar(this);
    progressBar->setMinimum(0);
    progressBar->setMaximum(0);

    statusBar->addPermanentWidget(progressBar);
    // Resultate reset, Viewer schwarz, fps reset, dateiname reset
    QString empty = segmentResults->toPlainText();
    filename->setText("");
    segmentResults->clear();
    actionZoomIn->setEnabled(false);
    actionZoomOut->setEnabled(false);
    actionRotate->setEnabled(false);
    actionSwitchView->setEnabled(false);
    statusBar->showMessage("");
    actionSave->setEnabled(false);
    actionSaveAs->setEnabled(false);
    saveBtn->setDisabled(true);
    statusBar->showMessage("Alles ist zurueckgesetzt.");
    progressBar->setMaximum(100);
    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(1000);
    statusBar->removeWidget(progressBar);
   
}

/// <summary>
/// Exportieren der Punktwolke
/// </summary>
void PunktwolkenSegmentierung::exportResults() {
    QProgressBar* progressBar = new QProgressBar(this);
    progressBar->setMinimum(0);
    progressBar->setMaximum(0);

    statusBar->addPermanentWidget(progressBar);
    QString file = QFileDialog::getSaveFileName(this,
        tr("SegmentResults speichern "), "", tr("PC Format (*.ply *.asc)"));
    if (!file.isEmpty())
    {
        QString mFilename = file;
        QFile sFile(mFilename);
        QFileInfo* info = new QFileInfo(mFilename);
        QString ErgebnisFile = info->baseName() + ".asc";
        QFile::copy("./SegmentLog/coloredPC.asc", QDir(info->absolutePath()).filePath(ErgebnisFile));
        

    }
    statusBar->showMessage("Die Ergebnisse sind exportiert");
    progressBar->setMaximum(100);
    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(1000);
    statusBar->removeWidget(progressBar);

}
QString fileName;

/// <summary>
/// Import der Punktwolke
/// </summary>
void PunktwolkenSegmentierung::importPCFile() {
    saveBtn->setDisabled(true);
    segmentResults->clear();
    segBtn->setEnabled(true);
    actionSaveAs->setDisabled(true);

    QProgressBar* progressBar = new QProgressBar(this);
    progressBar->setMinimum(0);
    progressBar->setMaximum(0);

    statusBar->addPermanentWidget(progressBar);
    fileName = QFileDialog::getOpenFileName(this,
        tr("Punktwolke Datei öffnen"), "",
        tr("PC Format (*.ply *.asc)"));
    
    filename->setText(fileName);

    QFile* file = new QFile(fileName);
  
    pclviewer->createPointCloud(file);
    statusBar->showMessage("Datei wird geladen");
    progressBar->setMaximum(100);

    progressBar->setValue(100);
    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->setInterval(1000);
    timer->start();
    statusBar->removeWidget(progressBar);
}

/// <summary>
/// Öffnet die Readme URL aus gitlab - 
/// </summary>
void PunktwolkenSegmentierung::openDocs() {

    QDesktopServices::openUrl(QUrl("https://gitlab.rz.htw-berlin.de/softwareentwicklungsprojekt/wise2021-22/team8/-/blob/master/readme.md", QUrl::TolerantMode));
}

void PunktwolkenSegmentierung::segmentierung(){
    
    QFileInfo* info = new QFileInfo(fileName);
    if (info->completeSuffix() == QString("asc")) {
        ;
        statusBar->showMessage("ASCII zu PLY Export gestartet..");
        const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        const std::string src = fileName.toStdString();

        pcl::PCDReader Reader;
        Reader.read(src, *cloud);
        std::string pfile = "./examples/Punktwolke.ply";

        savePLYFileASCII(pfile, *cloud);
        statusBar->showMessage("ASCII zu PLY Export fertig");
    }
   

    if (QFile::exists("./examples/Punktwolke.ply"))
    {
        QFile::remove("./examples/Punktwolke.ply");
    }

    QFile::copy(fileName, "./examples/Punktwolke.ply");
    statusBar->showMessage("Segmentierung gestartet..");
    QProgressBar* progressBar = new QProgressBar(this);
    progressBar->setMinimum(0);
    progressBar->setMaximum(0);
    statusBar->addPermanentWidget(progressBar);
    // Skript start/Aufruf des Skripts
    system(".\\venv\\Scripts\\activate && py PointNet-Segmentierungsnetzwerk.py ");

    // evtl. warten wenn Skript fertig ist und dann kommende Code
    // Automatisches Importieren der neuen .asc file - aus /SegmentLog/coloredPC.asc
    // Anschließend die SegmentResultslog.txt in der Segmentresult textbox anzeigen lassen


    // Anzeigen der Segment Resultate:

    QFile segmentResultfile("./SegmentLog/segmentResultslog.txt");
    segmentResultfile.open(QIODevice::ReadOnly);
    QTextStream stream(&segmentResultfile);
    QString content = stream.readAll();
    segmentResultfile.close();
    segmentResults->setText(content);

    QString coloredPCpath = "./SegmentLog/segmentiertePunktwolke.asc";

    QFile* endResult = new QFile(coloredPCpath);

    pclviewer->createPointCloud(endResult);

    progressBar->setValue(100);
    statusBar->showMessage("Segmentierung beendet.");
    segBtn->setDisabled(true);
    actionSave->setEnabled(true);
    actionSaveAs->setEnabled(true);
    saveBtn->setEnabled(true);
    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(1000);
    statusBar->removeWidget(progressBar);
}
