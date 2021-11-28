#pragma once

#include <iostream>

// Qt
#include <QtWidgets>

// Point Cloud Library
#include <QVTKOpenGLStereoWidget.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class PCLViewer : public QVTKOpenGLStereoWidget {
    Q_OBJECT

public:
    explicit PCLViewer(QWidget* parent = 0);

protected:
    void refreshView();

    pcl::visualization::PCLVisualizer::Ptr viewer;
    PointCloudT::Ptr cloud;

public:
    int point_count;
    void createPointCloud(QFile* file);
};
