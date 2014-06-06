#include "chilitagsdetector.hpp"

using namespace std;
using namespace cv;

// how many second in the *future* the markers transformation should be published?
// this allow to compensate for the 'slowness' of tag detection, but introduce
// some lag in TF.
#define TRANSFORM_FUTURE_DATING 0

 pcl::PointXYZRGB makeRGBPoint( float x, float y, float z )
  {
    pcl::PointXYZRGB p;
    p.x = x;
    p.y = y; 
    p.z = z;
    return p;
  }
  
  

ChilitagsDetector::ChilitagsDetector(ros::NodeHandle& rosNode,
                                     const string& camera_frame, 
                                     const string& configFilename,
                                     bool omitOtherTags,
                                     double tagSize,
                                     double gain,
                                     int persistence) :
            rosNode(rosNode),
            it(rosNode),
            camera_frame(camera_frame),
            firstUncalibratedImage(true),
#ifdef WITH_KNOWLEDGE
            connector("localhost", "6969"),
#endif
	    chilitags2d(),
            chilitags3d(cv::Size(0,0)) // will call setDefaultTagSize and setFilter with default chilitags parameter values
{
#ifdef WITH_KNOWLEDGE
    try {
        kb = oro::Ontology::createWithConnector(connector);
    } catch (oro::OntologyServerException ose) {
        ROS_ERROR_STREAM("Could not connect to the knowledge base: " << ose.what());
        exit(1);
    }
#endif
//     sub = it.subscribeCamera("image", 1, &ChilitagsDetector::findMarkers, this);
    cloud_sub_ = rosNode_.subscribe<PointCloud>(cloudTopic_, 1, &ChilitagsDetector::cloudCallback, this);

    chilitags3d.readTagConfiguration(configFilename, omitOtherTags);

    if(tagSize!=USE_CHILITAGS_DEFAULT_PARAM)
        chilitags3d.setDefaultTagSize(tagSize); // use specified value

    if(gain != USE_CHILITAGS_DEFAULT_PARAM && persistence != USE_CHILITAGS_DEFAULT_PARAM)
    {
        chilitags3d.setFilter(persistence,  gain); // use specified values
    }else{
        if( !(gain == USE_CHILITAGS_DEFAULT_PARAM && persistence == USE_CHILITAGS_DEFAULT_PARAM)){ // only one value specified
            ROS_ERROR_STREAM("You cannot just use one of the chilitags default values for gain and persistence:\n" <<
                      "it must be both or neither.");
          }
    }
    
    
    logging.open("/home/mpjansen/test/ros_markers.csv" ,  std::fstream::out|std::fstream::trunc);
	if (logging.is_open())
	{
	ROS_INFO("File opened");
	}
	else
	{
	  ROS_INFO("File not opened");
	
	}
	mapsize = MAPSIZE_DEF;
        for(int g = 0; g < mapsize; g++)
	  {
	  logging 				<< "R(0.0),"
						<< "R(1.0),"
						<< "R(2.0),"
						<< "R(0.1),"
						<< "R(1.1),"
						<< "R(2.1),"
						<< "R(0.2),"
						<< "R(1.2),"
						<< "R(2.2),"
						<< "X,"
						<< "Y,"
						<< "Z,"
						<< "Num,";
		
	  }
	  
	  logging			<<"\n";

}

ChilitagsDetector::~ChilitagsDetector(void )
{
   logging.close();
}


void ChilitagsDetector::setROSTransform(Matx44d trans, tf::Transform& transform)
{
    transform.setOrigin( tf::Vector3( trans(0,3) / 1000,
                                    trans(1,3) / 1000,
                                    trans(2,3) / 1000) );

    tf::Quaternion qrot;
    tf::Matrix3x3 mrot(
        trans(0,0), trans(0,1), trans(0,2),
        trans(1,0), trans(1,1), trans(1,2),
        trans(2,0), trans(2,1), trans(2,2));
    mrot.getRotation(qrot);
    transform.setRotation(qrot);
}

void ChilitagsDetector::cloudCallback(const PointCloud::ConstPtr& msg)
{
    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
    pcl::PCLImage pcl_image;
    pcl::toPCLPointCloud2(*msg, pcl_image);
    pcl_conversions::moveFromPCL(pcl_image, *image_msg);
     
    inputImage = cv_bridge::toCvShare(image_msg)->image; 
    map = chilitags2d.find(inputImage);

    
    
    
    
    for (int i = 0; i <  mapsize; i++)
    {  
    transfs[i].tf.setZero();
    }
    
    for(auto& kv : map)
    {
    PointCloud marker;
    Quad quad = kv.second; 
    
      try{
      marker.push_back( (*msg).at( (int)quad(0,0),(int)quad(0,1) )); // upper left
      marker.push_back( (*msg).at( (int)quad(1,0),(int)quad(1,1) )); // upper right
      marker.push_back( (*msg).at( (int)quad(2,0),(int)quad(2,1) )); // lower right
      marker.push_back( (*msg).at( (int)quad(3,0),(int)quad(3,1) ));
      }
      
      catch (const std::out_of_range& e)
      {
	std::cout << "Out of Range error.";
	continue;	
      }      
      /* create an ideal cloud */
      double w = 78.6;
      PointCloud ideal;
      ideal.push_back( makeRGBPoint(-w/2,w/2,0) );
      ideal.push_back( makeRGBPoint(w/2,w/2,0) );
      ideal.push_back( makeRGBPoint(w/2,-w/2,0) );
      ideal.push_back( makeRGBPoint(-w/2,-w/2,0) );
      
      Eigen::Matrix4f t;
      TransformationEstimationSVD obj;
      obj.estimateRigidTransformation( marker, ideal, t );

      ROS_INFO ("Num %i Pos x %f y %f z %f" ,kv.first,t(0,3),t(1,3),t(2,3));
      
      transfs[kv.first].tf = t;
      transfs[kv.first].markernum = kv.first;
    }
        for (int i =0; i < mapsize; i++)
    {
     logging 		<<	  transfs[i].tf(0,0)<<","
			<<	  transfs[i].tf(1,0)<<","
			<<	  transfs[i].tf(2,0)<<","
			<<	  transfs[i].tf(0,1)<<","
			<<	  transfs[i].tf(1,1)<<","
			<<	  transfs[i].tf(2,1)<<","
			<<	  transfs[i].tf(0,2)<<","
			<<	  transfs[i].tf(1,2)<<","
			<<	  transfs[i].tf(2,2)<<","
			<<	  transfs[i].tf(0,3)<<","
			<<	  transfs[i].tf(1,3)<<","
			<<	  transfs[i].tf(2,3)<<","
			<< 	  transfs[i].markernum<<",";
    }
    
    logging 		<< "\n";
    
}


void ChilitagsDetector::findMarkers(const sensor_msgs::ImageConstPtr& msg, 
                                    const sensor_msgs::CameraInfoConstPtr& camerainfo)
{
  
    // updating the camera model is cheap if not modified
    cameramodel.fromCameraInfo(camerainfo);
    // publishing uncalibrated images? -> return (according to CameraInfo message documentation,
    // K[0] == 0.0 <=> uncalibrated).
    if(cameramodel.intrinsicMatrix()(0,0) == 0.0) {
        if(firstUncalibratedImage) {
            ROS_ERROR("Camera publishes uncalibrated images. Can not detect markers.");
            ROS_WARN("Detection will start over again when camera info is available.");
        }
        firstUncalibratedImage = false;
        return;
    }
    firstUncalibratedImage = true;
    // TODO: can we avoid to re-set the calibration matrices for every frame? ie,
    // how to know that the camera info has changed?
    chilitags3d.setCalibration(cameramodel.intrinsicMatrix(), 
                                cameramodel.distortionCoeffs());

    // hopefully no copy here:
    //  - assignement operator of cv::Mat does not copy the data
    //  - toCvShare does no copy if the default (source) encoding is used.
    inputImage = cv_bridge::toCvShare(msg)->image; 

        /********************************************************************
        *                      Markers detection                           *
        ********************************************************************/

    auto map2 = chilitags2d.find(inputImage);
    auto foundObjects = chilitags3d.estimate(map2);
    ROS_DEBUG_STREAM(foundObjects.size() << " objects found.");

    /****************************************************************
    *                Publish TF transforms                          *
    *****************************************************************/

    
#ifdef WITH_KNOWLEDGE
    auto previouslySeen(objectsSeen);
#endif
    objectsSeen.clear();

    for (auto& kv : foundObjects) {

        objectsSeen.insert(kv.first);
        setROSTransform(kv.second, 
                        transform);

        br.sendTransform(
                tf::StampedTransform(transform, 
                                        ros::Time::now() + ros::Duration(TRANSFORM_FUTURE_DATING), 
                                        camera_frame, 
                                        kv.first));
    }

#ifdef WITH_KNOWLEDGE
    set<oro::Statement> stmts;
    for(const auto& obj : objectsSeen) {
        if (previouslySeen.find(obj) == previouslySeen.end()) {
            stmts.insert(obj + " isVisible true");
            stmts.insert(obj + " rdf:type FiducialMarker");
        }
    }
    if (!stmts.empty()) kb->add(stmts);

    stmts.clear();
    for (const auto& pobj : previouslySeen) {
        if (objectsSeen.find(pobj) == objectsSeen.end()) {
            stmts.insert(pobj + " isVisible true");
        }
    }
    if (!stmts.empty()) kb->remove(stmts);
#endif


}

