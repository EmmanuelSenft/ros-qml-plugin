#ifndef ROS_H
#define ROS_H

#include <thread>
#include <QQuickItem>
#include <QStringList>
#include <QVector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <freeplay_sandbox_msgs/ListIntStamped.h>
#include <freeplay_sandbox_msgs/ContinuousAction.h>
#include <freeplay_sandbox_msgs/RewardMessage.h>

/**
 * @brief A QtQuick item that follows a ROS pose published on a topic 'topic'.
 *
 * The scaling between the ROS pose coordinates (in meters) and the QML pixels can
 * be set with the property 'pixelscale': pixels = meters / pixelscale
 *
 * The Z value of the pose is not directly used (as QML is 2D!), but can be read from the property
 * 'zvalue'.
 */
class RosPoseSubscriber : public QQuickItem {
Q_OBJECT
    Q_PROPERTY(bool position MEMBER _position NOTIFY onPositionChanged)
    Q_PROPERTY(QQuickItem* origin MEMBER _origin)
    Q_PROPERTY(double pixelscale MEMBER _pixel2meter)
    Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)
    Q_PROPERTY(qreal zvalue READ getZValue NOTIFY onZValueChanged)
    Q_PROPERTY(QString frame MEMBER _frame)

public:

    RosPoseSubscriber(QQuickItem* parent = 0);

    virtual ~RosPoseSubscriber() {}

    void setTopic(QString topic);
    qreal getZValue() {return _zvalue;}

    void onIncomingPose(const geometry_msgs::PoseStamped&);

private slots:
    void updatePos(double x, double y, double z, double rotation);

signals:
    void onPositionChanged();
    void onZValueChanged();

    void onMsgReceived(double x, double y, double z, double rotation);

private:

    QString _topic;
    QString _frame;
    QQuickItem* _origin;
    double _pixel2meter;

    bool _position; // not really used, but required tfor 'onPositionChanged' to be valid in QML

    qreal _zvalue;

    ros::NodeHandle _node;
    ros::Subscriber _incoming_poses;
};

/**
 * @brief A QtQuick item that publish a ROS pose on a topic 'topic'.
 *
 * The scaling between the ROS pose coordinates (in meters) and the QML pixels can
 * be set with the property 'pixelscale': pixels = meters / pixelscale
 *
 * The Z value of the pose is not directly used (as QML is 2D!), but can be sent from the property
 * 'zvalue'.
 */
class RosPosePublisher : public QQuickItem {
Q_OBJECT
    Q_PROPERTY(QQuickItem* target WRITE setTarget MEMBER _target)
    Q_PROPERTY(QQuickItem* origin MEMBER _origin)
    Q_PROPERTY(QString frame WRITE setFrame MEMBER _frame)
    Q_PROPERTY(double pixelscale MEMBER _pixel2meter)
    Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)

public:
    RosPosePublisher(QQuickItem* parent = 0);
    virtual ~RosPosePublisher() {}

    Q_INVOKABLE void publish();

    void setTopic(QString topic);
    void setTarget(QQuickItem* target);
    void setFrame(QString frame);
private:

    QString _topic;
    QQuickItem* _target;
    QQuickItem* _origin;
    QString _frame;

    int _width;
    int _height;

    qreal _pixel2meter;

    ros::NodeHandle _node;
    ros::Publisher _publisher;
};

/**
 * @brief A QtQuick item that publish a freeplay reward message consisting of a 
 * reward value and a list of string on a topic 'topic'.
 */
class RosRewardPublisher : public QQuickItem {
Q_OBJECT
    Q_PROPERTY(QStringList strings MEMBER _strings)
    Q_PROPERTY(bool reward MEMBER _reward)
    Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)

public:
    RosRewardPublisher(QQuickItem* parent = 0);
    virtual ~RosRewardPublisher() {}

    Q_INVOKABLE void publish();

    void setTopic(QString topic);
private:
    QString _topic;
    bool _reward;
    QStringList _strings;

    ros::NodeHandle _node;
    ros::Publisher _publisher;
};


/**
 * @brief A QtQuick item that publish a freeplay continuous action message consisting of a 
 * stamped ROS pose and a list of string on a topic 'topic'.
 */
class RosActionPublisher : public QQuickItem {
Q_OBJECT
    Q_PROPERTY(QQuickItem* target WRITE setTarget MEMBER _target)
    Q_PROPERTY(QQuickItem* origin MEMBER _origin)
    Q_PROPERTY(QStringList strings MEMBER _strings)
    Q_PROPERTY(QString frame WRITE setFrame MEMBER _frame)
    Q_PROPERTY(double pixelscale MEMBER _pixel2meter)
    Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)

public:
    RosActionPublisher(QQuickItem* parent = 0);
    virtual ~RosActionPublisher() {}

    Q_INVOKABLE void publish();

    void setTopic(QString topic);
    void setTarget(QQuickItem* target);
    void setFrame(QString frame);
private:
    qreal _pixel2meter;

    QQuickItem* _target;
    QQuickItem* _origin;

    QString _topic;
    QString _frame;
    QStringList _strings;

    int _width;
    int _height;

    ros::NodeHandle _node;
    ros::Publisher _publisher;
};


/**
 * @brief A QtQuick item that listen a freeplay continuous action message consisting of a 
 * stamped ROS pose and a list of string on a topic 'topic'.
 */
class RosActionSubscriber : public QQuickItem {
Q_OBJECT
    Q_PROPERTY(QQuickItem* origin MEMBER _origin)
    Q_PROPERTY(QStringList strings MEMBER _strings)
    Q_PROPERTY(QString frame MEMBER _frame)
    Q_PROPERTY(double pixelscale MEMBER _pixel2meter)
    Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)

public:
    RosActionSubscriber(QQuickItem* parent = 0);
    virtual ~RosActionSubscriber() {}

    void setTopic(QString topic);
    void setTarget(QQuickItem* target);
    void setFrame(QString frame);
    void onIncomingAction(const freeplay_sandbox_msgs::ContinuousAction& message);

private slots:
    void updatePose(double x, double y);
signals:
    void actionReceived();
    void onUpdatePose(double x, double y);

private:
    qreal _pixel2meter;

    QQuickItem* _origin;

    QString _topic;
    QString _frame;
    QStringList _strings;

    int _width;
    int _height;

    ros::NodeHandle _node;
    ros::Subscriber _subscriber;
};


/**
 * @brief A QtQuick item that follows a ROS String published on a topic 'topic'.
 */
class RosStringSubscriber : public QQuickItem {
Q_OBJECT
    Q_PROPERTY(QString text MEMBER _text NOTIFY onTextChanged)
    Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)

public:

    RosStringSubscriber(QQuickItem* parent = 0){}

    virtual ~RosStringSubscriber() {}

    void setTopic(QString topic);
    void onIncomingString(const std_msgs::String& str);

signals:
    void onTextChanged();

private:

    QString _topic;
    QString _text;

    ros::NodeHandle _node;
    ros::Subscriber _incoming_message;
};

/**
 * @brief A QtQuick item that publish a ROS string on a topic 'topic'.
 *
 */
class RosStringPublisher : public QQuickItem {
Q_OBJECT
    Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)
    Q_PROPERTY(QString text WRITE setText MEMBER _text )

public:
    RosStringPublisher(QQuickItem* parent = 0){}
    virtual ~RosStringPublisher() {}

    void setTopic(QString topic);
    void setText(QString text);
    Q_INVOKABLE void publish();
private:
    QString _topic;
    QString _text;
    ros::NodeHandle _node;
    ros::Publisher _publisher;
};

/**
 * @brief A QtQuick item that follows a ROS int list published on a topic 'topic'.
 */
class RosListIntSubscriber : public QQuickItem {
Q_OBJECT
    Q_PROPERTY(QList<int> list MEMBER _list NOTIFY onListChanged)
    Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)

public:

    RosListIntSubscriber(QQuickItem* parent = 0){}

    virtual ~RosListIntSubscriber() {}

    void setTopic(QString topic);
    void onIncomingList(const freeplay_sandbox_msgs::ListIntStamped &list);

signals:
    void onListChanged();

private:

    QString _topic;
    QList<int> _list;

    ros::NodeHandle _node;
    ros::Subscriber _incoming_message;
};


/**
 * @brief A QtQuick item that follows a ROS Int32MutliArray published on a topic 'topic'.
 * supports only 2D arrays
 */
class RosArrayIntSubscriber : public QQuickItem {
Q_OBJECT
    Q_PROPERTY(QVector<int> data MEMBER _data NOTIFY onDataChanged)
    Q_PROPERTY(QList<QString> headers MEMBER _headers)
    Q_PROPERTY(QVector<int> dimensions MEMBER _dimensions)
    Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)

public:

    RosArrayIntSubscriber(QQuickItem* parent = 0){}

    virtual ~RosArrayIntSubscriber() {}

    void setTopic(QString topic);
    void onIncomingMessage(const std_msgs::Int32MultiArray &message);

signals:
    void onDataChanged();

private:

    QString _topic;
    QVector<int> _data;
    QVector<int> _dimensions;
    QList<QString> _headers;

    ros::NodeHandle _node;
    ros::Subscriber _incoming_message;
};


/**
 * @brief A QtQuick item that follows a ROS TF frame.
 *
 * The TF frame is first transformed into the provided ROS 'parentframe'
 * frame, and the (x,y) coordinates of the resulting frame are used to place
 * the QtQuick item wrt to the 'origin' item (or (0,0) if 'origin' is not set).
 *
 * The scaling between the ROS coordinates (in meters) and the QML pixels can
 * be set with the property 'pixelscale': pixels = meters / pixelscale
 *
 * The Z value of the frame is not directly used (as QML is 2D!), but can be
 * read from the property 'zvalue'.
 */
class TFListener : public QQuickItem {
    Q_OBJECT
        Q_PROPERTY(bool position MEMBER _position NOTIFY onPositionChanged)
        Q_PROPERTY(QString frame WRITE setFrame MEMBER _frame)
        Q_PROPERTY(QString parentframe WRITE setParentFrame MEMBER _parentframe)
        Q_PROPERTY(QQuickItem* origin MEMBER _origin)
        Q_PROPERTY(double pixelscale MEMBER _pixel2meter)
        Q_PROPERTY(qreal zvalue READ getZValue NOTIFY onZValueChanged)

public:

    TFListener(QQuickItem* parent = 0);

    virtual ~TFListener();

    void setFrame(QString topic);
    void setParentFrame(QString topic);
    qreal getZValue() {return _zvalue;}

    void onIncomingPose(const geometry_msgs::PoseStamped&);
    Q_INVOKABLE QStringList getAllFrames(); 

private slots:
    void updatePos(double x, double y, double z, double rotation);

signals:
    void onPositionChanged();
    void onZValueChanged();

    void onMsgReceived(double x, double y, double z, double rotation);

private:

    bool _initialized;
    bool _active;
    bool _running;
    std::thread _listener_thread;

    void listen(); // method ran in the thread that actually listen to the TF updates

    QString _frame;
    QString _parentframe;

    QQuickItem* _origin;
    double _pixel2meter;

    bool _position; // not really used, but required tfor 'onPositionChanged' to be valid in QML

    qreal _zvalue;


    ros::NodeHandle _node;
    tf::TransformListener _listener;

};


/**
 * @brief A QML Item that broadcast its target's pose to TF
 */
class TFBroadcaster : public QQuickItem {
Q_OBJECT
    Q_PROPERTY(bool active MEMBER _active)
    Q_PROPERTY(QQuickItem* target WRITE setTarget MEMBER _target)
    Q_PROPERTY(QQuickItem* origin MEMBER _origin)
    Q_PROPERTY(QString frame WRITE setFrame MEMBER _frame)
    Q_PROPERTY(QString parentframe WRITE setParentFrame MEMBER _parentframe)
    Q_PROPERTY(double pixelscale MEMBER _pixel2meter)
    Q_PROPERTY(double zoffset MEMBER _zoffset)

public:

    TFBroadcaster(QQuickItem* parent = 0);

    virtual ~TFBroadcaster();

    void setTarget(QQuickItem* target);
    void setFrame(QString frame);
    void setParentFrame(QString frame);

private:

    void tfPublisher();

    bool _initialized;
    bool _active;
    bool _running;
    std::thread _broadcaster_thread;

    QQuickItem* _target;
    QQuickItem* _origin;
    QString _frame;
    QString _parentframe;

    qreal _pixel2meter;
    qreal _zoffset;

    ros::NodeHandle _node;
    tf::TransformBroadcaster _br;

};

/**
 * @brief The ImagePublisher class provides a QML object that publishes a QImage on a
 * ROS topic (set it with the 'topic' property).
 * The QML property 'target' must refer to a QML image. The property 'frame' should be set to
 * the desired ROS frame.
 *
 * The image is published everytime the method 'publish()' is called.
 *
 * The size of the image can be set with the property 'width' and 'height'. By default, the
 * actual size of the image item is used.
 *
 * The image can be published on a latched topic by setting `latched: True` (by
 * default, not latched).
 *
 * The property 'pixelscale' is used to compute the (virtual) focal length: f =
 * 1/pixelscale.  This can be used to convert the image's pixels into meters in
 * the ROS code: 1 meter = 1 pixel * 1/f
 */
class ImagePublisher : public QQuickItem {

 Q_OBJECT
    Q_PROPERTY(bool active MEMBER _active)
    Q_PROPERTY(QQuickItem* target WRITE setTarget MEMBER _target)
    Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)
    Q_PROPERTY(QString frame WRITE setFrame MEMBER _frame)
    Q_PROPERTY(bool latched WRITE setLatched MEMBER _latched)
    Q_PROPERTY(int width MEMBER _width)
    Q_PROPERTY(int height MEMBER _height)
    Q_PROPERTY(double pixelscale MEMBER _pixel2meter)

public:

    ImagePublisher(QQuickItem* parent = 0);

    virtual ~ImagePublisher() {}

    void setTarget(QQuickItem* target);
    void setFrame(QString frame);
    void setTopic(QString topic);
    void setLatched(bool latched);

    Q_INVOKABLE void publish();

private:
    bool _active;

    sensor_msgs::CameraInfo makeCameraInfo(const sensor_msgs::Image& img);
    void _rospublish(const QImage&);

    QQuickItem* _target;

    QString _topic;
    QString _frame;
    bool _latched;

    int _width;
    int _height;
    qreal _pixel2meter;

    ros::NodeHandle _node;
    image_transport::ImageTransport _it;
    image_transport::CameraPublisher _publisher;

};


/**
 * @brief The FootprintsPublisher class provides a QML object that publishes on a
 * *latched topic* (/footprints) the 2D bounding boxes of each of its 'targets'.
 * QML items in 'targets' must have a property 'boundingbox' that contains a Box2D fixture.
 */
class FootprintsPublisher : public QQuickItem {

 Q_OBJECT
    Q_PROPERTY(QVariantList targets WRITE setTargets MEMBER _targets)
    Q_PROPERTY(double pixelscale MEMBER _pixel2meter)

public:

    static const QString topic;

    FootprintsPublisher(QQuickItem* parent = 0);

    virtual ~FootprintsPublisher() {}

    void setTargets(QVariantList targets);

private:

    QVariantList _targets;

    qreal _pixel2meter;

    ros::NodeHandle _node;
    ros::Publisher _publisher;

};

/**
 * @brief The RosSignal class provides a QML object that publishes on a
 * configurable topic an empty message (ie, a signal) every time signal() is
 * called.
 */
class RosSignal : public QQuickItem {

 Q_OBJECT
    Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)
public:

    RosSignal(QQuickItem* parent = 0) {}

    virtual ~RosSignal() {}

    void setTopic(QString topic);

    Q_INVOKABLE void signal();

    void onIncomingSignal(const std_msgs::Empty);

signals:
    void triggered();

private:

    QString _topic;

    ros::NodeHandle _node;
    ros::Publisher _publisher;
    ros::Subscriber _subscriber;
};

#endif // ROS_H

