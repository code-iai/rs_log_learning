#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <iai_rs/types/all_types.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

// IAI includes
#include <iai_rs/scene_cas.h>
#include <iai_rs/util/wrapper.h>
#include <iai_rs/util/time.h>
#include <iai_rs/DrawingAnnotator.h>

// MP includes
#include "rs_log_learn/ImageGTAnnotation.h"

using namespace uima;
using namespace rs_log_learn;

class mpGTAnnotator: public Annotator
{
private:

public:

    TyErrorId initialize(AnnotatorContext &ctx)
    {
        outInfo("initialize");

        return UIMA_ERR_NONE;
    }

    TyErrorId typeSystemInit(TypeSystem const &type_system)
    {
        outInfo("typeSystemInit");
        return UIMA_ERR_NONE;
    }

    TyErrorId destroy()
    {
        outInfo("destroy");
        return UIMA_ERR_NONE;
    }

    TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
    {
        outInfo("process()");
        this->processWithLock(tcas, res_spec);
        return UIMA_ERR_NONE;
    }

    TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
    {
        outInfo("process start");

        iai_rs::util::StopWatch clock;

        // init service client
        char *argv[] = { const_cast<char*>("gt_annotation_client"), NULL };
        int argc = sizeof(argv) / sizeof(char*) - 1;
        //char** fake_argv = const_cast<char**> (&argv[0]);

        ros::init(argc, argv, "gt_annotation_client");
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<rs_log_learn::ImageGTAnnotation>("image_gt_annotation");
        rs_log_learn::ImageGTAnnotation srv;

        // grab cluster images
        iai_rs::SceneCas cas(tcas);
        iai_rs::SceneWrapper scene(cas.getScene());
        cv::Mat color;
        cas.getRGBImageHires(color);
        std::vector<iai_rs::Cluster> clusters;
        scene.identifiables.filter(clusters);

        // call the service of the UI with each cluster ROI image to annotate
        for (int i = 0; i < clusters.size(); ++i)
        {
            iai_rs::Cluster cluster = clusters.at(i);
            iai_rs::ImageROI image_rois = cluster.rois.get();
            std::vector<iai_rs::Learning> learning;
            clusters[i].annotations.filter(learning);

            cv::Mat rgb, mask;
            cv::Rect roi;
            iai_rs::conversion::from(image_rois.roi_hires(), roi);
            iai_rs::conversion::from(image_rois.mask_hires(), mask);

            color(roi).copyTo(rgb, mask);

            // prepare the image for the service call
            cv_bridge::CvImagePtr cv_ptr;
            cv_bridge::CvImage srv_msg;
            if (rgb.type() == CV_8UC1)
            {
                outInfo(
                        "image from cluster " << i << " is of type CV_8UC1 with size: " << rgb.size);
                outError("no encoding header for this frame");
            }
            else if (rgb.type() == CV_8UC3)
            {
                outInfo(
                        "image from cluster " << i << " is of type CV_8UC3 with size: " << rgb.size);
                srv_msg.encoding = sensor_msgs::image_encodings::BGR8;
            }

            //srv_msg.header.frame_id = sensor_msgs::
            srv_msg.image = rgb;
            srv_msg.toImageMsg(srv.request.image);
            outInfo("converted image from cluster " << i << " is " << (int)srv.request.image.width << "x" << (int)srv.request.image.height);

            if(!learning.empty())
            {
                srv.request.lrn_name  = learning.at(0).name.get();
                srv.request.lrn_shape = learning.at(0).shape.get();
            }
            else
            {
                srv.request.lrn_name  = "<none>";
                srv.request.lrn_shape = "<none>";
            }

            // all data set, call the service
            if (client.call(srv))
            {
                outInfo("got annotation back from ui service");
                outInfo("entered - name: " << srv.response.gt_name
                        << " shape: " << srv.response.gt_shape);
            }
            else
            {
                outError("Failed to call annotation service. start mpGTui");
            }

            // set strings returned from service
            iai_rs::GroundTruth gt = iai_rs::create<iai_rs::GroundTruth>(tcas);
            gt.global_gt.set(srv.response.gt_name);
            gt.shape.set(srv.response.gt_shape);

            clusters[i].annotations.append(gt);
        }

        outInfo("took: " << clock.getTime() << " ms.");
        return UIMA_ERR_NONE;
    }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(mpGTAnnotator)
