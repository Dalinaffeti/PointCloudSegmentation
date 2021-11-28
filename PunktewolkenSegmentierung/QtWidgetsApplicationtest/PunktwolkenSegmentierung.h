#pragma once

#include <QtWidgets/QMainWindow>
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

#include "pclviewer.h"

class PunktwolkenSegmentierung : public QMainWindow
{
    Q_OBJECT

public:
    PunktwolkenSegmentierung(QWidget* parent = Q_NULLPTR);
    PCLViewer* pclviewer;
    QAction* actionNew;
    QAction* actionOpen;
    QAction* actionSave;
    QAction* actionSaveAs;
    QAction* actionExit;
    QAction* actionDocs;
    QAction* actionZoomIn;
    QAction* actionZoomOut;
    QAction* actionSwitchView;
    QAction* actionRotate;
    QAction* actionUndo;
    QAction* actionRedo;
    QAction* actionCut;
    QAction* actionCopy;
    QAction* actionPaste;
    QAction* actionSelectAll;
    QAction* actionDelete;
    QWidget* centralWidget;
    QWidget* horizontalLayoutWidget;
    QHBoxLayout* horizontalLayout;
    QSpacerItem* horizontalSpacer;
    QVBoxLayout* verticalLayout;
    QSpacerItem* verticalSpacer_4;
    QTextBrowser* filename;
    QSpacerItem* verticalSpacer_3;
    QPushButton* importBtn;
    QSpacerItem* verticalSpacer;
    QPushButton* segBtn;
    QSpacerItem* verticalSpacer_2;
    QSpacerItem* horizontalSpacer_2;
    QWidget* widget;
    QHBoxLayout* horizontalLayout_2;
    QTextBrowser* segmentResults;
    QMenuBar* menuBar;
    QMenu* menuFile;
    QMenu* menuBearbeiten;
    QMenu* menuAnsicht;
    QMenu* menuHilfe;
    QStatusBar* statusBar;
    void retranslateUi();

private slots:
    void resetAll();
    void exportResults();
    void importPCFile();
    void openDocs();
    void segmentierung();
};

