#include <string>
#include <set>

// chilitags
#include <chilitags.hpp>

// opencv2
#include <opencv2/core/core.hpp>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


#include <pcl/io/pcd_io.h>
#include <sensor_msgs/Image.h>


#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <fstream>
#include <iostream>
#include <fcntl.h>

#ifdef WITH_KNOWLEDGE
#include <liboro/oro.h>
#include <liboro/socket_connector.h>
#endif

#define USE_CHILITAGS_DEFAULT_PARAM -1
#define MAPSIZE_DEF 4

const std::string cloudTopic_ = "points";
typedef cv::Matx<float, 4, 2> Quad;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> TransformationEstimationSVD;
struct _transf_
{
  Eigen::Matrix4f tf;
  int markernum;
};

class ChilitagsDetector
{
public:
    /**
       Creates an object ready to find the 3D pose of chilitags.

 \param rosNode The node which has created the object.
 \param camaera_frame The name of the camera frame for projection onto.
 \param configFilename The name of the YAML configuration file describing rigid
        clusters of tags. The chilitags library is distributed with a sample
        configuration file documenting the expected format.
 \param omitOtherTags If true, ignore the tags that are not explicitly
        listed in the configuration file. If false (default), the 3D pose of all detected
        tags will be estimated.
 \param defaultTagSize The default size of tags (used to compute their 3D pose) when not
        explicitly specified in the configuration file. To be accurate, the unit
        must match the unit used for the camera calibration (usually, millimetres).

        The default value is 20. A value of -1 will cause the default value from
        the chilitags3d library to be used.

        Note that it assumes all the tags have the same size. If tags have
        different size, you may want to list them in the configuration file.
 \param gain A value between 0 and 1 corresponding to the weight of the
        previous (filtered) position in the new filtered position. 0 means that the
        latest position of the object is returned.

        The default value is 0.9. A value of -1 will cause the default value from
        the chilitags3d library to be used.
 \param persistence the number of frames in which a 3d object, i.e. a tag or
        a rigid object containing several tags, should be absent before being
        removed from the output of estimate().

        The default value is 5. A value of -1 will cause the default value from
        the chilitags3d library to be used.

*/
    ChilitagsDetector(ros::NodeHandle& rosNode,
                      const std::string& camera_frame,
                      const std::string& configFilename,
                      bool omitOtherTags = false,
                      double tagSize = USE_CHILITAGS_DEFAULT_PARAM,
                      double gain = USE_CHILITAGS_DEFAULT_PARAM,
                      int persistence = USE_CHILITAGS_DEFAULT_PARAM);
    ~ChilitagsDetector(void);

private:

#ifdef WITH_KNOWLEDGE
    oro::SocketConnector connector;
    oro::Ontology* kb;
#endif
//thijs
    ros::NodeHandle rosNode_;
    ros::Subscriber cloud_sub_;
    sensor_msgs::PointCloud2ConstPtr cloud_;
    std::map<int, Quad> map;
    std::ofstream logging;
    int 	mapsize; // size of the map
    _transf_ 	transfs[30];
    _transf_ 	transf_map[30]; // array of transfer matrices building a world map
    _transf_ 	transf_curr[30]; // current transfer matrices linking camera to world frame
    std::map<int,int> mapset;
    int ROS_sock;


    
     
    ros::NodeHandle& rosNode;
    image_transport::ImageTransport it;
    image_transport::CameraSubscriber sub;

    tf::TransformBroadcaster br;
    tf::Transform transform;
    std::string camera_frame;

    image_geometry::PinholeCameraModel cameramodel;
    cv::Mat cameraMatrix, distCoeffs;
    bool firstUncalibratedImage;

    cv::Mat inputImage;
    chilitags::Chilitags chilitags2d;
    chilitags::Chilitags3D chilitags3d;
    std::set<std::string> objectsSeen;
    void setROSTransform(cv::Matx44d trans, tf::Transform& transform);

    void findMarkers(const sensor_msgs::ImageConstPtr& msg,
                     const sensor_msgs::CameraInfoConstPtr& camerainfo);
   
    void cloudCallback (const PointCloud::ConstPtr& msg); //pcl::PCLPointCloud2::ConstPtr& 
  

    
};

