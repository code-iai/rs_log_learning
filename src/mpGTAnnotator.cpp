#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <iai_rs/types/all_types.h>

// IAI includes
#include <iai_rs/scene_cas.h>
#include <iai_rs/util/wrapper.h>
#include <iai_rs/util/time.h>
#include <iai_rs/DrawingAnnotator.h>

// MP includes
#include <mpGTui.h>
#include <roiDrawingArea.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


using namespace uima;
using namespace rs_log_learn;


class mpGTAnnotator : public Annotator
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
    return  UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");

    iai_rs::util::StopWatch clock;

    // init service client
    char *argv[] = {const_cast<char*>("gt_annotation_client"), NULL};
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

    for(int i = 0; i < clusters.size(); ++i)
    {
    	iai_rs::Cluster cluster = clusters.at(i);
		iai_rs::ImageROI image_rois = cluster.rois.get();

		cv::Mat rgb, mask;
		cv::Rect roi;
		iai_rs::conversion::from(image_rois.roi_hires(), roi);
		iai_rs::conversion::from(image_rois.mask_hires(), mask);

		color(roi).copyTo(rgb, mask);

        // send to srv
		cv_bridge::CvImagePtr cv_ptr;
		cv_bridge::CvImage srv_msg;
		//srv_msg.header.frame_id = sensor_msgs::
		srv_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // TODO: find out what the enc is
		srv_msg.image = rgb;
		srv_msg.toImageMsg(srv.request.image);

		if (client.call(srv))
		{
		  outInfo("got annotation back from ui service");
		}
		else
		{
		  outError("Failed to call annotation service. start mpGTui");
		}


        iai_rs::GroundTruth gt = iai_rs::create<iai_rs::GroundTruth>(tcas);
        // set string from service
        //gt.global_gt.set(<string from srv>);
        clusters[i].annotations.append(gt);
    }




	/*srv.request.image.data = ;

	if (client.call(srv))
	{
		ROS_INFO("Sum: %ld", (long int)srv.response.global_gt);
	}
	else
	{
	ROS_ERROR("Failed to call service add_two_ints");
	return 1;
	}*/

    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(mpGTAnnotator)
