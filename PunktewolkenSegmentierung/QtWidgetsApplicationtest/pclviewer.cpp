#include "pclviewer.h"
#include <vtkGenericOpenGLRenderWindow.h>
#include <QVTKOpenGLStereoWidget.h>
#include <QDebug>


PCLViewer::PCLViewer(QWidget* parent)
    : QVTKOpenGLStereoWidget(parent) {

    point_count = 0;

    // Set up the QVTK window
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto rw = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    rw->AddRenderer(renderer);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, rw,
        "viewer", false));

    setRenderWindow(viewer->getRenderWindow());
    // viewer->setupInteractor(interactor(), rw());

    refreshView();
}

void PCLViewer::refreshView() { renderWindow()->Render(); }

void PCLViewer::createPointCloud(QFile* file) {
    QTextStream in(file);

    QString line;
    // Title
    in.readLine();
    // Version
    in.readLine();
    // Fields
    in.readLine();
    // Size, I guess bytes per coordinate
    in.readLine();
    // Type
    in.readLine();
    // Count
    in.readLine();
    // Width
    point_count = in.readLine().split(QChar(' '))[1].toInt();
    // Height
    in.readLine();
    // Viewpoint
    in.readLine();
    // Points
    line = in.readLine();
    // Data type
    in.readLine();

    // Setup the cloud pointer
    cloud.reset(new PointCloudT);
    // The number of points in the cloud
    qDebug() << "Number of points: " << point_count;
    cloud->resize(point_count);

    // The default color
    unsigned int red = 200;
    unsigned int green = 128;
    unsigned int blue = 128;

    // Fill the cloud with the file points
    QStringList coordinates;
    for (auto& point : *cloud) {
        coordinates = in.readLine().split(QChar(' '));
        point.x = coordinates[0].toFloat();
        point.y = coordinates[1].toFloat();
        point.z = coordinates[2].toFloat();
        point.r = red;
        point.g = green;
        point.b = blue;
    }

    viewer->removeAllPointClouds();
    viewer->addPointCloud(cloud, "Point cloud");
    viewer->resetCamera();

    refreshView();
}
