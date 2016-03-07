#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs_log_learning/types/all_types.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

// IAI includes
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>

// MP service includes
#include "rs_log_learning/ImageGTAnnotation.h"

using namespace uima;
using namespace rs_log_learning;

class mpGTAnnotator: public Annotator
{
private:
  std::string mode;

  void writeToCsv(std::string gtName, std::string gtShape, std::string lrnName,
                  std::string lrnShape, float confidence)
  {
    outInfo("writing evaluation results to .csv");
    ofstream csv;
    std::string path(getenv("HOME"));
    path += "/evaluation.csv";
    outInfo("saving to " << path);

    csv.open(path, std::ofstream::out | std::ofstream::app);
    // write stuff
    csv << gtName << ",";
    csv << gtShape << ",";
    csv << lrnName << ",";
    csv << lrnShape << ",";
    if(confidence < 0) // prevent less than 0 value
    {
      csv << 0 << ",";
    }
    else
    {
      csv << confidence << ",";
    }

    csv << std::boolalpha << (gtName.compare(lrnName) == 0) << std::noboolalpha << "\n";
    csv.close();
  }

public:

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("mode", mode);

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

    rs::StopWatch clock;

    // init service client
    char *argv[] = { const_cast<char *>("gt_annotation_client"), NULL };
    int argc = sizeof(argv) / sizeof(char *) - 1;
    //char** fake_argv = const_cast<char**> (&argv[0]);

    ros::init(argc, argv, "gt_annotation_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<rs_log_learning::ImageGTAnnotation>("image_gt_annotation");
    rs_log_learning::ImageGTAnnotation srv;

    // grab cluster images
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    cv::Mat color;
    cas.get(VIEW_COLOR_IMAGE_HD, color);
    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);

    // call the service of the UI with each cluster ROI image to annotate
    for(int i = 0; i < clusters.size(); ++i)
    {
      rs::Cluster cluster = clusters.at(i);
      rs::ImageROI image_rois = cluster.rois.get();
      std::vector<rs_log_learning::Learning> learning;
      clusters[i].annotations.filter(learning);

      cv::Mat rgb, mask;
      cv::Rect roi;
      rs::conversion::from(image_rois.roi_hires(), roi);
      rs::conversion::from(image_rois.mask_hires(), mask);

      color(roi).copyTo(rgb, mask);

      // prepare the image for the service call
      cv_bridge::CvImagePtr cv_ptr;
      cv_bridge::CvImage srv_msg;
      if(rgb.type() == CV_8UC1)
      {
        outInfo(
          "image from cluster " << i << " is of type CV_8UC1 with size: " << rgb.size);
        outError("no encoding header for this frame");
      }
      else if(rgb.type() == CV_8UC3)
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
        srv.request.lrn_confidence = learning.at(0).confidence.get();
      }
      else
      {
        srv.request.lrn_name  = "<none>";
        srv.request.lrn_shape = "<none>";
        srv.request.lrn_confidence = 0;
      }

      // all data set, call the service
      if(client.call(srv))
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
      rs_log_learning::GroundTruthAnnotation gt = rs::create<rs_log_learning::GroundTruthAnnotation>(tcas);
      gt.global_gt.set(srv.response.gt_name);
      gt.shape.set(srv.response.gt_shape);

      clusters[i].annotations.append(gt);

      if(mode.compare("evaluate") == 0)
      {
        writeToCsv(srv.response.gt_name, srv.response.gt_shape,
                   srv.request.lrn_name, srv.request.lrn_shape,
                   srv.request.lrn_confidence);
      }
    }

    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(mpGTAnnotator)
