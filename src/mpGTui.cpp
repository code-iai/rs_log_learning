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
        okButton("      OK      "),
        layoutTableEntry(3, 2, true),
        layoutTableLearned(3, 2, true)
{
    set_border_width(10);
    initTreeModel();

    okButton.signal_clicked().connect(
            sigc::mem_fun(*this, &mpGTui::onOkButtonClicked));
    entryTextName.signal_activate().connect(
            sigc::mem_fun(*this, &mpGTui::onEntryEnterKeyRelease));
    signal_delete_event().connect(sigc::mem_fun(this,&mpGTui::onExit));

    lblInfo.set_text("waiting for images...");
    lblDescr1.set_text("Learned name:");
    lblDescr2.set_text("Learned shape:");
    lblLearningStringName.set_text("NamePlaceholder");
    lblLearningStringShape.set_text("ShapePlacholder");
    lblEntryNameDescr.set_text("Name:");
    lblComboDescr.set_text("Shape:");

    lblEntryNameDescr.set_alignment(Gtk::AlignmentEnum::ALIGN_RIGHT);
    lblDescr1.set_alignment(Gtk::AlignmentEnum::ALIGN_RIGHT);
    lblDescr2.set_alignment(Gtk::AlignmentEnum::ALIGN_RIGHT);
    lblComboDescr.set_alignment(Gtk::AlignmentEnum::ALIGN_RIGHT);

    lblEntryNameDescr.set_padding(30, 0);
    lblDescr1.set_padding(30, 0);
    lblDescr2.set_padding(30, 0);
    lblComboDescr.set_padding(30, 0);

    layoutTableLearned.attach(lblDescr1, 0, 1, 0, 1,
            Gtk::AttachOptions::FILL, Gtk::AttachOptions::FILL,
            0, 5);
    layoutTableLearned.attach(lblDescr2, 0, 1, 1, 2);
    layoutTableLearned.attach(lblLearningStringName, 1, 2, 0, 1);
    layoutTableLearned.attach(lblLearningStringShape, 1, 2, 1, 2);

    layoutTableEntry.attach(lblEntryNameDescr, 0, 1, 0, 1);
    layoutTableEntry.attach(entryTextName, 1, 2, 0, 1);
    layoutTableEntry.attach(lblComboDescr, 0, 1, 1, 2);
    layoutTableEntry.attach(shapeCombo, 1, 2, 1, 2);

    learnedFrame.set_label("Learned data");
    entryFrame.set_label("GroundTruth entry");
    learnedFrame.add(layoutTableLearned);
    entryFrame.add(layoutTableEntry);

    add(vBox);
    vBox.pack_start(roi);
    vBox.pack_start(lblInfo);
    vBox.pack_start(hSeparator, Gtk::PackOptions::PACK_EXPAND_WIDGET, 10);
    vBox.pack_start(learnedFrame);
    vBox.pack_start(entryFrame);
    vBox.pack_start(okButton, Gtk::PackOptions::PACK_SHRINK, 10);

    initRosService();

    show_all_children();
}

void mpGTui::initTreeModel()
{
    // create tree model:
    shaperefTreeModel = Gtk::ListStore::create(shapeColumns);
    shapeCombo.set_model(shaperefTreeModel);

    // fill combobox tree:
    Gtk::TreeModel::Row row = *(shaperefTreeModel->append());
    row[shapeColumns.colId] = 1;
    row[shapeColumns.colName] = "box";
    row = *(shaperefTreeModel->append());
    row[shapeColumns.colId] = 2;
    row[shapeColumns.colName] = "cylinder";

    // pack to combobox
    shapeCombo.set_active(0); // "box" is the default shape
    shapeCombo.pack_start(shapeColumns.colId);
    shapeCombo.pack_start(shapeColumns.colName);
}

mpGTui::~mpGTui()
{
}

bool mpGTui::receive_image(rs_log_learn::ImageGTAnnotation::Request& req,
        rs_log_learn::ImageGTAnnotation::Response& res)
{
    imageReceiveMutex_.lock();

    ROS_INFO("got image from annotator");
    if(beforeFirstImage_)
    {
        lblInfo.set_text("");
        beforeFirstImage_ = false;
    }

    cv_ptr_ = cv_bridge::toCvCopy(req.image,
            sensor_msgs::image_encodings::BGR8);
    ROS_DEBUG("image converted back to cv image");

    imageReceiveMutex_.unlock();
    imageDrawDispatcher_.emit();
    // wait for user input, then return
    ROS_INFO("waiting for user input");

    while (true)
    {
        if(inputFinished_) break;
        if(exiting_) return true;
        // while we wait for user input, make sure ros doesn't block the ui
        Gtk::Main::iteration();
    }
    inputFinished_ = false;

    ROS_DEBUG("got data from ui");
    std::cout<< entryTextName.get_text() << std::endl;
    res.global_gt = entryTextName.get_text();

    Gtk::TreeModel::iterator iter = shapeCombo.get_active();
    if(iter)
    {
        Gtk::TreeModel::Row row = *iter;
        if(row)
        {
            int id = row[shapeColumns.colId];
            Glib::ustring name = row[shapeColumns.colName];
            std::cout << " ID=" << id << ", name=" << name << std::endl;
            // TODO: set shape
            // res.shape_gt = name;
        }
    }
    else ROS_ERROR("Invalid combo iter");

    // set the input fields back to the default for the next image
    entryTextName.set_text("");
    shapeCombo.set_active(0);

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

void mpGTui::onEntryEnterKeyRelease()
{
    ROS_DEBUG("enter key released");
    onOkButtonClicked();
}

void mpGTui::onOkButtonClicked()
{
    ROS_INFO("data committed, notifying service call thread");
    inputFinished_ = true; // add mutex
}

bool mpGTui::onExit(GdkEventAny* event)
{
    exiting_ = true;
    Gtk::Main::quit();
    return true;
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
        ROS_INFO("outImage is %d x %d px", outImage_.cols, outImage_.rows);
        roi.set(Gdk::Pixbuf::create_from_data(outImage_.data, Gdk::COLORSPACE_RGB, false, 8,
                        outImage_.cols, outImage_.rows, outImage_.step));
        roi.queue_draw();
        imageReceiveMutex_.unlock();
    });

    ROS_INFO("init done.");

    Gtk::Main::run(ui);

    ui.exiting_ = true; // needs to be set by windowmanager exit callback

    return 0;
}
