#include <QImage>
#include <QPainter>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

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

    QImage::Format format = QImage::Format_RGBA8888;
    if(msg->encoding == sensor_msgs::image_encodings::RGB8)
        format = QImage::Format_RGB888;
    else if(msg->encoding == sensor_msgs::image_encodings::RGBA8)
        format = QImage::Format_RGBA8888; 
    else if(msg->encoding == sensor_msgs::image_encodings::MONO8)
        format = QImage::Format_Mono;

    _last_image = QImage(msg->width, msg->height, format);
    memcpy(_last_image.bits(), msg->data.data(), _last_image.byteCount());

}

QImage RosImageProvider::requestImage(const QString &id, QSize *size, const QSize &requestedSize)
{

    if (_topic != id.toStdString()) {
        _topic = id.toStdString();
        cout << "Subscribing to topic " << _topic << endl;
        _sub = _it.subscribe(_topic, 1, &RosImageProvider::imageCallback, this);
    }

    cout << "Image requested" << endl;

    QImage result;

    if (requestedSize.isValid()) {
        result = _last_image.scaled(requestedSize, Qt::KeepAspectRatio);
    } else {
        result = _last_image;
    }

    *size = result.size();
    return result;

}
