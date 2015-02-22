/*
 * mpGTui.h
 *
 *  Created on: Feb 19, 2015
 *      Author: andre
 */

#ifndef RS_LOG_LEARN_SRC_MPGTUI_H_
#define RS_LOG_LEARN_SRC_MPGTUI_H_

#include <gtkmm.h>
#include <cairomm/context.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include "ros/ros.h"

#include <roiDrawingArea.h>
#include "rs_log_learn/ImageGTAnnotation.h"

#define TIMEOUT_VALUE 500 // timer for ros spin

namespace rs_log_learn
{

Glib::Dispatcher imageDrawDispatcher_;
std::mutex imageReceiveMutex_;
Gtk::Image roi;
cv::Mat outImage_;
cv_bridge::CvImagePtr cv_ptr_;

class mpGTui: public Gtk::Window
{
public:
    mpGTui();
    virtual ~mpGTui();

    bool receive_image(rs_log_learn::ImageGTAnnotation::Request& req,
            rs_log_learn::ImageGTAnnotation::Response& res);

protected:
    void on_testbutton_clicked();
    Gtk::Button okButton;
    Gtk::Table layoutTable;
    Gtk::VBox vBox;

    roiDrawingArea roiImage;

    Gtk::Label lblDescr1;
    Gtk::Label lblDescr2;
    Gtk::Label lblLearningString;
    Gtk::Label lblLearningString2;
    Gtk::Entry entryText;

private:
    ros::NodeHandle nh_;
    ros::ServiceServer gtAnnotationService_;
    cv_bridge::CvImagePtr cv_bridge_;

    void initRosService();
    bool onTimeout();
};

} /* namespace rs_log_learn */

#endif /* RS_LOG_LEARN_SRC_MPGTUI_H_ */
