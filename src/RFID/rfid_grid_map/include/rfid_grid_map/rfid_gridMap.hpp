/**
 * 
 * 
 * 
 * 
 * */


    
#pragma once
#include <time.h>       /* time_t, time, ctime */
#include <math.h>
#include <list>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <string> 
#include <fstream>
#include <XmlRpcValue.h>
#include <boost/algorithm/string/predicate.hpp>


// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>    
#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>
// - messages
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/MapMetaData.h>  
// - grid map
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
//

// ROS - OURS
#include <rfid_node/TagReading.h>

using namespace std;
using namespace ros;
using namespace grid_map;

namespace rfid_grid_map {

/*!
 
 */
class rfid_gridMap
{
    
    struct type_area{
        double prob;
        grid_map::Polygon polygon;
        string name;
    };
    
    public:

      /*!
       * Constructor.
       * @param nodeHandle the ROS node handle.
       */
      rfid_gridMap(ros::NodeHandle& nodeHandle);

      virtual ~rfid_gridMap();
      
      //! callback for rfid messages...
      void tagCallback(const rfid_node::TagReading::ConstPtr& msg);
    
      void loadROSParams();
      
      void showROSParams();
      
      //! periodic map updates
      void updateMapCallback(const ros::TimerEvent&);
    
      void updateProbs(const ros::TimerEvent&);
      
      void updateTransform();        
      
      void publishMap();
      
      double countValuesInArea(Polygon pol);
      
      void getMapDimensions();
      
      void drawSquare(double start_x,double start_y,double end_x,double end_y,double value);
      
      void drawCircle(double x, double y, double radius, double value);
      
      void drawDetectionShape(double cx,double cy, double rh, 
        double radius,double radiusMin,  double max_heading, 
        double midProb, double lowProb, double highProb);

      void updateLastDetectionPose(double x, double y);

      void wasHere(type_area area);
    
      void drawPolygon(const grid_map::Polygon poly,double value);
      
      std::vector<rfid_gridMap::type_area> loadAreas();
      
      void saveMapCallback(const ros::TimerEvent&);
      
      void mapCallback(const nav_msgs::OccupancyGrid& msg);
      void temporalDecayCallback(const ros::TimerEvent&);
      
    private:
    
      Position lastP;
      type_area lastRegion;

      //! ROS nodehandle.
      ros::NodeHandle& nodeHandle_;
      
      //! ROS subscriber to rfid messages...
      ros::Subscriber sub_;
      //! Grid map data.
      grid_map::GridMap map_;
      ros::Subscriber map_sub_ ;      
      //! Grid map layer name (note: we could have more...)
      std::string layerName;
      
      //! Grid map layer lowest value
      double lowerValue;
      //! Grid map layer highest value
      double upperValue;

      //! gridmap actualization rates.        
      double weight_inc;
      double weight_dec;
            
      //! publisher for probs
      ros::Publisher prob_pub_ ;
      ros::Timer decayTimer;
      double tagToDecayRate;
      unsigned int numDetections;
      //! publisher topic name for probs
      std::string prob_pub_name;
      
      //! preload gridmaps from files
      bool loadGrids;
       
      //! map Size (in meters)
      double size_x; 
      double size_y;
      //! map resolution pixel/meter
      double resolution;
      //! 2d position of the grid map in the grid map frame [m].  
      double orig_x=0;
      double orig_y=0; 

      //! Grid map publisher.
      ros::Publisher gridMapPublisher_;
      //! grid map publisher topic name
      std::string grid_map_name;
      
      tf::TransformListener listener_;
      tf::StampedTransform transform_;


      bool isMapLoaded;
      std::string save_route;
      double detectRadius;
      double cone_range;
      double cone_angle;
      
      double temporalDecayPeriod;
      double temporalDecayFactor;
      
      nav_msgs::MapMetaData mapDesc;
      string mapFrame;
      
      //! Save gridmaps period [s]
      double saveTime;
      
      //! saved gridmap file name
      std::string gridmap_image_file;
      
      //! saved ROS gridmap image format
      std::string rosEncoding;
      //! rfid tag id
      std::string tagID;
      
      //! tagged object name
      std::string object_name;
      
      //! global frame id (for maps)
      std::string global_frame;
      //! robot frame id 
      std::string robot_frame;
      
      
      //! update probabilities period [s]
      double probUpdatePeriod;
      
      //! update map period [s]
      double mapUpdatePeriod;
      
      double constrainAngle2PI(double x);
      
      double constrainAnglePI(double x);
      
      void nextTimeDecay();
      
      std::vector<rfid_gridMap::type_area> mapAreas;
      std::map<std::string,std::vector<rfid_gridMap::type_area>> mapSubAreas;  

}; // End of Class rfid_gridMap

} // end of  namespace rfid_grid_map
