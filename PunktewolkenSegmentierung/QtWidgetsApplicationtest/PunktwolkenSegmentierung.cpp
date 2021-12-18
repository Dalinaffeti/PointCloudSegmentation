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
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <pcl/io/ply_io.h>
#include <iostream>

#include <filesystem>


using std::string;

//string getFileExt(const string s) {
//
//    size_t i = s.rfind('.', s.length());
//    if (i != string::npos) {
//        return(s.substr(i + 1, s.length() - i));
//    }
//
//    return("");
//}

PunktwolkenSegmentierung::PunktwolkenSegmentierung(QWidget* parent)
    : QMainWindow(parent)
{
#pragma region UiCode


    resize(800, 650);
    setMinimumSize(QSize(800, 650));
    setMaximumSize(QSize(800, 650));
    setBaseSize(QSize(800, 650));
    actionNew = new QAction(this);
    actionNew->setObjectName(QString::fromUtf8("actionNew"));
    actionOpen = new QAction(this);
    actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
    actionSave = new QAction(this);
    actionSave->setObjectName(QString::fromUtf8("actionSave"));
    actionSaveAs = new QAction(this);
    actionSaveAs->setObjectName(QString::fromUtf8("actionSaveAs"));
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
    horizontalLayoutWidget = new QWidget(centralWidget);
    horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
    horizontalLayoutWidget->setGeometry(QRect(0, 0, 908, 502));
    horizontalLayout = new QHBoxLayout(horizontalLayoutWidget);
    horizontalLayout->setSpacing(6);
    horizontalLayout->setContentsMargins(11, 11, 11, 11);
    horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
    horizontalLayout->setContentsMargins(0, 0, 0, 0);
    horizontalSpacer = new QSpacerItem(5, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

    horizontalLayout->addItem(horizontalSpacer);

    verticalLayout = new QVBoxLayout();
    verticalLayout->setSpacing(6);
    verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
    verticalSpacer_4 = new QSpacerItem(10, 5, QSizePolicy::Minimum, QSizePolicy::Fixed);

    verticalLayout->addItem(verticalSpacer_4);

    filename = new QTextBrowser(horizontalLayoutWidget);
    filename->setObjectName(QString::fromUtf8("filename"));
    QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(filename->sizePolicy().hasHeightForWidth());
    filename->setSizePolicy(sizePolicy);
    filename->setMinimumSize(QSize(180, 30));
    filename->setMaximumSize(QSize(180, 30));
    filename->setBaseSize(QSize(180, 30));

    verticalLayout->addWidget(filename);

    verticalSpacer_3 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Fixed);

    verticalLayout->addItem(verticalSpacer_3);

    importBtn = new QPushButton(horizontalLayoutWidget);
    importBtn->setObjectName(QString::fromUtf8("importBtn"));
    sizePolicy.setHeightForWidth(importBtn->sizePolicy().hasHeightForWidth());
    importBtn->setSizePolicy(sizePolicy);
    importBtn->setMinimumSize(QSize(180, 40));
    importBtn->setMaximumSize(QSize(180, 40));
    importBtn->setBaseSize(QSize(180, 40));
    importBtn->setLayoutDirection(Qt::LeftToRight);

    verticalLayout->addWidget(importBtn, 0, Qt::AlignHCenter);

    verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    verticalLayout->addItem(verticalSpacer);

    segBtn = new QPushButton(horizontalLayoutWidget);
    segBtn->setObjectName(QString::fromUtf8("segBtn"));
    sizePolicy.setHeightForWidth(segBtn->sizePolicy().hasHeightForWidth());
    segBtn->setSizePolicy(sizePolicy);
    segBtn->setMinimumSize(QSize(180, 40));
    segBtn->setMaximumSize(QSize(180, 40));
    segBtn->setBaseSize(QSize(180, 40));

    verticalLayout->addWidget(segBtn, 0, Qt::AlignHCenter);

    verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    verticalLayout->addItem(verticalSpacer_2);

    horizontalLayout->addLayout(verticalLayout);

    widget = new QWidget(centralWidget);
    widget->setObjectName(QString::fromUtf8("widget"));
    widget->setGeometry(QRect(0, 500, 802, 121));
    horizontalLayout_2 = new QHBoxLayout(widget);
    horizontalLayout_2->setSpacing(6);
    horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
    horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
    horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
    segmentResults = new QTextBrowser(widget);
    segmentResults->setObjectName(QString::fromUtf8("segmentResults"));
    sizePolicy.setHeightForWidth(segmentResults->sizePolicy().hasHeightForWidth());
    segmentResults->setSizePolicy(sizePolicy);
    segmentResults->setMinimumSize(QSize(800, 110));
    segmentResults->setMaximumSize(QSize(650, 100));

    horizontalLayout_2->addWidget(segmentResults);

    setCentralWidget(centralWidget);
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
    menuBar->addAction(menuBearbeiten->menuAction());
    menuBar->addAction(menuAnsicht->menuAction());
    menuBar->addAction(menuHilfe->menuAction());
    menuFile->addAction(actionNew);
    menuFile->addAction(actionOpen);
    menuFile->addSeparator();
    menuFile->addAction(actionSave);
    menuFile->addAction(actionSaveAs);
    menuFile->addSeparator();
    menuFile->addAction(actionExit);
    menuBearbeiten->addAction(actionUndo);
    menuBearbeiten->addAction(actionRedo);
    menuBearbeiten->addSeparator();
    menuBearbeiten->addAction(actionCut);
    menuBearbeiten->addAction(actionCopy);
    menuBearbeiten->addAction(actionPaste);
    menuBearbeiten->addAction(actionDelete);
    menuBearbeiten->addSeparator();
    menuBearbeiten->addAction(actionSelectAll);
    menuAnsicht->addAction(actionZoomIn);
    menuAnsicht->addAction(actionZoomOut);
    menuAnsicht->addAction(actionSwitchView);
    menuAnsicht->addAction(actionRotate);
    menuHilfe->addAction(actionDocs);

    retranslateUi();
    QObject::connect(actionExit, &QAction::triggered, this, qOverload<>(&QMainWindow::close));
    QObject::connect(actionSave, &QAction::triggered, this, qOverload<>(&QMainWindow::update));
#pragma endregion

    pclviewer = new PCLViewer();
    horizontalLayout->addWidget(pclviewer);
    pclviewer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    statusBar->showMessage("Willkommen! Bitte eine .ply oder .asc Datei importieren um zu starten.");
    QProgressBar* progressBar = new QProgressBar(this);
    progressBar->setMinimum(0);
    progressBar->setMaximum(100);
    progressBar->setValue(0);
    statusBar->addPermanentWidget(progressBar);

    connect(actionNew, SIGNAL(triggered(bool)), this, SLOT(resetAll()));
    connect(actionSave, SIGNAL(triggered(bool)), this, SLOT(exportResults()));
    connect(actionSaveAs, SIGNAL(triggered(bool)), this, SLOT(exportResults()));
    connect(actionOpen, SIGNAL(triggered(bool)), this, SLOT(importPCFile()));
    connect(actionDocs, SIGNAL(triggered(bool)), this, SLOT(openDocs()));

    connect(importBtn, SIGNAL(clicked()), this, SLOT(importPCFile()));
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
    // Resultate reset, Viewer schwarz, fps reset, dateiname reset
    QString empty = segmentResults->toPlainText();
    filename->setText("");
    segmentResults->setText("");
    actionZoomIn->setEnabled(false);
    actionZoomOut->setEnabled(false);
    actionRotate->setEnabled(false);
    actionSwitchView->setEnabled(false);
    statusBar->showMessage("");

   
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
    statusBar->showMessage("Resultate wurden exportiert.");

}

/// <summary>
/// Import der Punktwolke
/// </summary>
void PunktwolkenSegmentierung::importPCFile() {
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Punktwolke Datei öffnen"), "",
        tr("PC Format (*.ply *.asc)"));
    //std::string sfileName = fileName.toUtf8().constData();

    //if (getFileExt(sfileName) == "ply") {
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //    pcl::visualization::PCLVisualizer::Ptr viewer;
    //    
    //    pcl::PLYReader Reader;
    //    Reader.read(sfileName, *cloud);
    //    viewer->addPointCloud(cloud, "Point cloud");

    //    //pcl::io::loadPLYFile(sfileName, *cloud);
    //    statusBar->showMessage("PLY file was loaded");
   // }

    QFile* file = new QFile(fileName);
    if (!file->open(QIODevice::ReadOnly )) {
        QMessageBox::critical(nullptr, "Error", "Datei konnte nicht geffnet werden, keine Rechte!");
        statusBar->showMessage("Bitte nochmal versuchen. Ein Fehler ist aufgetreten.");

        return;
    }
    pclviewer->createPointCloud(file);
    statusBar->showMessage("Datei wird geladen");
}

/// <summary>
/// Öffnet die Readme URL aus gitlab - 
/// </summary>
void PunktwolkenSegmentierung::openDocs() {

    QDesktopServices::openUrl(QUrl("https://gitlab.rz.htw-berlin.de/softwareentwicklungsprojekt/wise2021-22/team8/-/blob/master/readme.md", QUrl::TolerantMode));
}

void PunktwolkenSegmentierung::segmentierung(){
    std::string filename = "PointNet-Segmentierungsnetzwerk.py";
    std::string command = "python  ";
    command += filename;
    system(command.c_str());
    statusBar->showMessage("Segmentierung gestartet..");

}
