#include <QImage>
#include <QPainter>

#include <ros/ros.h>

#include "RosImageProvider.h"

#include <iostream>
using namespace std;
RosImageProvider::RosImageProvider()
    : QQuickImageProvider(QQuickImageProvider::Pixmap),
    _it(ros::NodeHandle()),
    _last_image(QImage(10,10, QImage::Format_RGB888))
{
    _last_image.fill(QColor("black").rgba());
}

void RosImageProvider::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // Inspired from https://github.com/KubaO/stackoverflown/blob/master/questions/wip-main.cpp
    QImage::Format format = {};
    if (msg->encoding == "rgb8" || msg->encoding == "bgr8")
       format = QImage::Format_RGB888;
    else if (msg->encoding == "rgba8" || msg->encoding == "bgra8")
       format = QImage::Format_RGBA8888_Premultiplied;
    else if (msg->encoding == "mono8")
       format = QImage::Format_Grayscale8;
    else
       return;

    if (msg->encoding == "bgr8" || msg->encoding == "bgra8"){
        _last_image = QImage(msg->data.data(), msg->width, msg->height, format).rgbSwapped();
    }
    else{
        _last_image = QImage(msg->width, msg->height, format);
        memcpy(_last_image.bits(), msg->data.data(), _last_image.byteCount());
    }
    
}

QImage RosImageProvider::requestImage(const QString &id, QSize *size, const QSize &requestedSize)
{

    if (_topic != id.toStdString()) {
        _topic = id.toStdString();
        cout << "Subscribing to topic " << _topic << endl;
        image_transport::TransportHints hints("compressed");
        cout<< "compressed" <<endl;
        _sub = _it.subscribe(_topic, 1, &RosImageProvider::imageCallback, this, hints);
    }

    QImage result;

    if (requestedSize.isValid()) {
        result = _last_image.scaled(requestedSize, Qt::KeepAspectRatio);
    } else {
        result = _last_image;
    }

    *size = result.size();
    return result;

}
