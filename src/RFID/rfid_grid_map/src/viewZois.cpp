#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <vector>

#include <cmath>
#include <ros/console.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include "geometry_msgs/Point.h"

using namespace std;
using namespace ros;


std::map<std::string,std::vector<geometry_msgs::Point>> loadZois(ros::NodeHandle nodeHandle_){
        std::map<std::string,std::vector<geometry_msgs::Point>> mapAreas;
    
        XmlRpc::XmlRpcValue zoi_keys;
        std::string submapParam="/mmap/zoi/submap_0/";
        std::string numSubMapParam="/mmap/numberOfSubMaps";
        int numSubmaps;
        std::string zoiPointName; 
        std::string zoiName;
        unsigned int zoiPoint_num;
        double px=0;
	    double py=0;
        std::size_t slashPos;
        geometry_msgs::Point p;
        std::map<std::string,std::vector<geometry_msgs::Point>>::iterator map_it;


	    //ROS_ASSERT_MSG(nodeHandle_.getParam(numSubMapParam, numSubmaps),"Can't determine number of sub maps from rosparam [/mmap/numberOfSubMaps] ");
        if (!nodeHandle_.getParam(numSubMapParam, numSubmaps))
        {
			numSubmaps=1;
			ROS_ERROR("Can't determine number of sub maps from rosparam [/mmap/numberOfSubMaps]. Assuming 1 ");
		}
        
	    ROS_ASSERT_MSG(numSubmaps==1,"Number of submaps different from 1 [%d]. Aborting",numSubmaps);

        ROS_ASSERT_MSG(nodeHandle_.getParam(submapParam,zoi_keys),"Can't get zoi rosparam [/mmap/zoi/submap_0/] ");
    
        ROS_ASSERT_MSG(zoi_keys.getType() == XmlRpc::XmlRpcValue::TypeStruct,"YAML ZOI rosparam has unknown format. ZOI rosparam [/mmap/zoi/submap_0/] is not a struct ");
        
        // this should be a list of tuples like this:
        //            zoiName_pointNumber : { submapName, zoiName, posX, posY  }
        //
        //  Zoi name follows format:
        //             zoi name-zoi subarea
        // subareas are parts of bigger zois, they must have same 'zoi name' before the slash.
        //  i.e. 
        //          kitchen-table
        //          kitchen-cook    both are kitchen zoi, subareas cook and table
        for(XmlRpc::XmlRpcValue::iterator itr = zoi_keys.begin(); itr != zoi_keys.end(); itr++)
        {
            try
            {
                zoiPointName = itr->first;
                std::cout<< "Raw point name: " << zoiPointName<< "\n";
                
                // zoi point number is last thing after '_'
                slashPos = zoiPointName.find_last_of("_");
                if (slashPos!=std::string::npos)
                {                
                    ROS_DEBUG_STREAM(" zoi name: " << zoiPointName.substr(0,slashPos)  <<  " \n");
                    ROS_DEBUG_STREAM(" point num: " << zoiPointName.substr(slashPos+1) <<  " \n");
                    
                    zoiPoint_num=std::stoi(zoiPointName.substr(slashPos+1));

                    //region 
                    zoiName = static_cast<std::string>(itr->second[1]);
                    px =  itr->second[2];
                    py =  itr->second[3];

                    map_it = mapAreas.find(zoiName);
                    if (map_it == mapAreas.end())
                    {
                        std::vector<geometry_msgs::Point> area;  
                        mapAreas[zoiName]=area;
                    }

                    p=geometry_msgs::Point();
                    p.x=px;
                    p.y=py;
                    p.z=1.0;
                    mapAreas[zoiName].push_back(p);
                    
                    /* lets hope vertex are added propertly...
                     
                    // check if we have space
                    if (zoiPoint_num+1>mapAreas[zoiName].polygon.vertices_.size())
                    {
                        mapAreas[zoiName].polygon.vertices_.resize(zoiPoint_num+3);
                    }
                    mapAreas[zoiName].polygon.vertices_[zoiPoint_num]=p;
                    */
                }
                else
                {// something is wrong with the zoiPoint name, does not have slash?
                    ROS_ERROR("Ommiting zoi point name: It does not contain '_': [%s]", zoiPointName.c_str());
                } 
            } 
            catch(XmlRpc::XmlRpcException e)
            {
                ROS_ERROR_STREAM("ERROR:" << e.getMessage ()<<" .... \n");
            }
            
            
            
        }
        
        ROS_DEBUG("Loaded [%lu] zones:", mapAreas.size());
        for (std::map<std::string,std::vector<geometry_msgs::Point>>::iterator mapIt=mapAreas.begin(); mapIt!=mapAreas.end(); ++mapIt)		
        { 
            ROS_DEBUG("- [%s]: %lu points", mapIt->first.c_str(),mapIt->second.size() );
        }
        
        return mapAreas;
      }


int main( int argc, char** argv )
{
  ros::init(argc, argv, "view");
  ros::NodeHandle n;
  
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
  {
   ros::console::notifyLoggerLevelsChanged();
  }
  
  
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(10);
 
  std::map<std::string,std::vector<geometry_msgs::Point>> mapAreas;
  mapAreas = loadZois(n);
     
  while (ros::ok())
  {
    visualization_msgs::Marker line_list;
    line_list.id = 0;
    line_list.header.frame_id = "/map";
    line_list.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.scale.x = 0.1;
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    for (std::map<std::string,std::vector<geometry_msgs::Point>>::iterator mapIt=mapAreas.begin(); mapIt!=mapAreas.end(); ++mapIt)
    {
        //ROS_DEBUG_STREAM(" zoi name: " << mapIt->first  <<  " \n");
        line_list.ns = mapIt->first;
        for(geometry_msgs::Point p : mapIt->second) 
        {
            line_list.points.push_back(p);
        }
        line_list.points.push_back(line_list.points[0]);
        marker_pub.publish(line_list);  
        line_list.id++;
        line_list.points.clear();
    }
    
    
    r.sleep();
    //ROS_DEBUG_STREAM(" Republishing! \n");

  }
}
