/*
 * mpGTui.cpp
 *
 *  Created on: Feb 19, 2015
 *      Author: andre
 */

#include "mpGTui.h"

namespace rs_log_learn
{

mpGTui::mpGTui() :
        okButton("OK"), layoutTable(4, 2, true)
{
    set_border_width(10);

    okButton.signal_clicked().connect(
            sigc::mem_fun(*this, &mpGTui::on_testbutton_clicked));

    lblDescr1.set_text("Learned name:");
    lblDescr2.set_text("Learned shape:");
    lblLearningString.set_text("lbl1");
    lblLearningString2.set_text("lbl2");

    layoutTable.attach(lblDescr1, 0, 1, 1, 2);
    layoutTable.attach(lblDescr2, 0, 1, 2, 3);
    layoutTable.attach(lblLearningString, 1, 2, 1, 2);
    layoutTable.attach(lblLearningString2, 1, 2, 2, 3);
    layoutTable.attach(entryText, 0, 1, 3, 4);
    layoutTable.attach(okButton, 1, 2, 3, 4);

    add(vBox);
    //vBox.pack_start(roiImage);
    vBox.pack_end(roi);
    vBox.pack_end(layoutTable);

    initRosService();

    show_all_children();
}

mpGTui::~mpGTui()
{
}

bool mpGTui::receive_image(rs_log_learn::ImageGTAnnotation::Request& req,
        rs_log_learn::ImageGTAnnotation::Response& res)
{
    imageReceiveMutex_.lock();

    ROS_INFO("got image from annotator");

    cv_ptr_ = cv_bridge::toCvCopy(req.image,
            sensor_msgs::image_encodings::BGR8);
    ROS_INFO("image converted back to cv image");

    imageReceiveMutex_.unlock();
    imageDrawDispatcher_.emit();
    return true;
}

void mpGTui::initRosService()
{
    gtAnnotationService_ = nh_.advertiseService("image_gt_annotation",
            &mpGTui::receive_image, this);
    sigc::slot<bool> spinSlot = sigc::mem_fun(*this, &mpGTui::onTimeout);
    sigc::connection conn = Glib::signal_timeout().connect(spinSlot,
    TIMEOUT_VALUE);
    ROS_INFO("connected timeout handler");
}

bool mpGTui::onTimeout()
{
    ROS_DEBUG("ros spin...");
    ros::spinOnce();
    return true;
}

void mpGTui::on_testbutton_clicked()
{
    Gtk::MessageDialog dialog(*this, "This is a test");
    dialog.run();
}

} /* namespace rs_log_learn */

using namespace rs_log_learn;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpGTui");

    Gtk::Main kit(argc, argv);
    mpGTui ui;

    // lambda dispatcher, so GTK drawing doesn't collide
    // with the ROS thread receiving a new image
    imageDrawDispatcher_.connect([&]()
    {
        imageReceiveMutex_.lock();
        cv::cvtColor(cv_ptr_->image, outImage_, CV_BGR2RGB);
        ROS_INFO("outImage is %d %d px", outImage_.cols, outImage_.rows);
        roi.set(Gdk::Pixbuf::create_from_data(outImage_.data, Gdk::COLORSPACE_RGB, false, 8,
                        outImage_.cols, outImage_.rows, outImage_.step));
        roi.queue_draw();
        imageReceiveMutex_.unlock();
    });

    ROS_INFO("init done.");

    Gtk::Main::run(ui);

    return 0;
}
