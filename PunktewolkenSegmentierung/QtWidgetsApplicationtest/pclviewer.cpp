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

    QFileInfo* info = new QFileInfo(*file);
    QString line;

    if (info->completeSuffix() == QString("asc")) {

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

    }
    else {
        // PLY format
        line = in.readLine();
        while (!line.startsWith(QString("element vertex"))) {
            line = in.readLine();
        }
        point_count = line.split(QString(" ")).last().toInt();
        qDebug() << "PC: " << point_count;

        while (line != "end_header") {
            line = in.readLine();
        }

        QDataStream* stream = new QDataStream(file);
        stream->skipRawData(in.pos());
        stream->setByteOrder(QDataStream::LittleEndian);
        stream->setFloatingPointPrecision(QDataStream::SinglePrecision);

        cloud.reset(new PointCloudT);
        cloud->resize(point_count);

        // The default color
        unsigned int red = 200;
        unsigned int green = 128;
        unsigned int blue = 128;

        float number;
        // Fill the cloud with the file points
        for (auto& point : *cloud) {
            (*stream) >> number;
            point.x = number;

            (*stream) >> number;
            point.y = number;

            (*stream) >> number;
            point.z = number;

            point.r = red;
            point.g = green;
            point.b = blue;
        }
    }
    // I guess this is a bad practice, but I don't know how to do it better
    viewer->removeAllPointClouds();
    viewer->addPointCloud(cloud, "Point cloud");
    viewer->resetCamera();

    refreshView();


}