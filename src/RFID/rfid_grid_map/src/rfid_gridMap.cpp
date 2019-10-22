    
/**
 * ToDo: parametrize decay time, prob publish time, intensity
 * 
 * 
 * */


#include "rfid_grid_map/rfid_gridMap.hpp"


namespace rfid_grid_map {
    
    
    rfid_gridMap::rfid_gridMap(ros::NodeHandle& n)
    : nodeHandle_(n)
    {

      loadROSParams();
      
      getMapDimensions();
      //these parameters are hardcoded... 
      layerName="type";
      rosEncoding="mono16";
      lowerValue=0.0;
      upperValue=1.0;

      // map Size (m.)
      size_x=mapDesc.width*resolution;
      size_y=mapDesc.height*resolution;

      map_= GridMap(vector<string>({layerName}));
      map_.setGeometry(Length(size_x, size_y), resolution, Position(orig_x, orig_y));
      map_.setFrameId(global_frame);      
      map_.clearAll();
      
      if (loadGrids) {                
        // file to image...
        cv::Mat imageCV = cv::imread((save_route+gridmap_image_file), CV_LOAD_IMAGE_UNCHANGED );
        sensor_msgs::ImagePtr imageROS = cv_bridge::CvImage(std_msgs::Header(), rosEncoding, imageCV).toImageMsg();                      
        
        // Loaded gridmap image does not match current specs
        ROS_ASSERT_MSG(GridMapRosConverter::addLayerFromImage(*imageROS, layerName, map_, lowerValue, upperValue,0.5),
        "Mismatch loading layer image (%u w,%u h) Into map (%u r,%u c). Aborting",imageROS->width,imageROS->height,map_.getSize()(0),map_.getSize()(1));         
      } 
      
      // Param load and setup finished....................................
      //showROSParams();
    
       //for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
       //     map_.at(layerName, *iterator)=0;
       //}
      
      
      lastRegion.name=std::string(" ");
      
      gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(grid_map_name, 1, true);      
      publishMap();
      
      prob_pub_ = nodeHandle_.advertise<std_msgs::String>(prob_pub_name, 1000);
        
      // get tag readings
      ros::Subscriber sub_ = nodeHandle_.subscribe("/lastTag", 1000, &rfid_gridMap::tagCallback, this);
      
      // Update displayed map periodically
      ros::Timer timer = nodeHandle_.createTimer(ros::Duration(mapUpdatePeriod),  &rfid_gridMap::updateMapCallback,this);

      // publish updated probabilities every reasonable time.
      ros::Timer timer2 = nodeHandle_.createTimer(ros::Duration(probUpdatePeriod),  &rfid_gridMap::updateProbs,this);
      
      // publish updated probabilities every reasonable time.
      ros::Timer timer3 = nodeHandle_.createTimer(ros::Duration(saveTime),  &rfid_gridMap::saveMapCallback,this);



      
      //ROS_DEBUG("Spin...");     
      
            
      ros::spin(); 
       
    }
    
    void rfid_gridMap::showROSParams(){
        
      //...........................................
      ROS_DEBUG("Configuration params:");
      
      ROS_DEBUG("GRID MAP________________________");
      ROS_DEBUG("size_x: %2.2f", size_x);
      ROS_DEBUG("size_y: %2.2f", size_y);
      ROS_DEBUG("orig_x: %2.2f", orig_x);
      ROS_DEBUG("orig_y: %2.2f", orig_y);
      ROS_DEBUG("resolution: %2.2f", resolution);
      ROS_DEBUG("layerName: \"%s\"", layerName.c_str());
      
      ROS_ASSERT_MSG(isMapLoaded,"Cant show ROS params without map loaded. ShowROSparams must be called AFTER mapCallback");      
      ROS_DEBUG("mapDesc.width: %u", mapDesc.width);
      ROS_DEBUG("mapDesc.height: %u", mapDesc.height);

      
      
      ROS_DEBUG("Confidence shape________________________");
      ROS_DEBUG("detectRadius: %2.1f", detectRadius);    
      ROS_DEBUG("wi: %3.3f", weight_inc);
      ROS_DEBUG("wd: %3.3f", weight_dec);
      ROS_DEBUG("cone_range: %3.3f", cone_range);
      ROS_DEBUG("cone_angle: %3.3f", cone_angle*(180)/M_PI);
      
      
      ROS_DEBUG("ROS Params________________________");
      
      ROS_DEBUG("global_frame: %s", global_frame.c_str());    
      ROS_DEBUG("robot_frame: %s", robot_frame.c_str());
      ROS_DEBUG("tagID: %s", tagID.c_str());
      
      ROS_DEBUG("grid_map_name: %s", grid_map_name.c_str());      
      ROS_DEBUG("load/save_route is [%s]",save_route.c_str());          
      if (loadGrids){
        ROS_DEBUG("Load Saved data [TRUE]");          
      }else{
        ROS_DEBUG("Load Saved data [FALSE]");          
      }
      
  }
  
  
    void rfid_gridMap::getMapDimensions(){
        isMapLoaded=false;

        // connect to map server and get dimensions and resolution (meters)          
        while (!isMapLoaded){          
            // Try getting map properties from topic
            boost::shared_ptr<nav_msgs::OccupancyGrid const> mapPointer;            
            mapPointer=ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map",nodeHandle_,ros::Duration(0.5));

            if(mapPointer==NULL){
                ROS_ERROR("Topic \"map\" not available, trying service ");
            }else{
                mapCallback(*mapPointer);    
            }

            if (!isMapLoaded){
                // Try getting map properties from service...
                if (ros::service::waitForService("/static_map",500)) {
                    nav_msgs::GetMap::Request  req;
                    nav_msgs::GetMap::Response resp;
                    ros::service::call("/static_map", req, resp);
                    mapCallback(resp.map);
                } else {
                    ROS_ERROR("Service at \"/static_map\" unavailable too, trying topic again ...");
                }             
            } 
        }
    }



          

    void rfid_gridMap::loadROSParams(){
              ros::NodeHandle private_node_handle("~");
                
      // LOAD ROS PARAMETERS ....................................
      std::string temp;

      private_node_handle.param("global_frame", global_frame, std::string("/map"));    
      private_node_handle.param("robot_frame", robot_frame, std::string("base_link"));
      
      private_node_handle.param("map_resolution", temp,std::string("0.1"));
      resolution=std::stod(temp);

      private_node_handle.param("weight_inc", temp,std::string("0.005"));
      weight_inc=std::stod(temp);

      private_node_handle.param("weight_dec", temp,std::string("0.001"));
      weight_dec=std::stod(temp);


      private_node_handle.param("tagID", tagID, std::string("390000010000000000000007"));
      private_node_handle.param("grid_map_name", grid_map_name, std::string("grid_map"));
      private_node_handle.param("prob_pub_name", prob_pub_name, std::string("probs"));

      private_node_handle.param("saveRoute", save_route,std::string(""));
      
      private_node_handle.param("detectRadius", temp,std::string("20.0"));
      detectRadius=std::stod(temp);
      
      private_node_handle.param("cone_range", temp,std::string("20.0"));
      cone_range=std::stod(temp);
      
      private_node_handle.param("cone_angle", temp,std::string("20.0"));
      cone_angle=std::stod(temp);
      cone_angle=cone_angle*M_PI/(180);
      
      private_node_handle.param("loadGrids", temp, std::string("not found"));

      if (boost::iequals(temp, std::string("true"))) {
          loadGrids=true;
      } else {
          loadGrids=false;
      }
      
      if (boost::iequals(temp, std::string("not found"))) {
          loadGrids=false;
      }
      
      
      private_node_handle.param("mapUpdatePeriod", mapUpdatePeriod, 5.0);
      private_node_handle.param("probUpdatePeriod", probUpdatePeriod, 1.0);
      private_node_handle.param("saveTime", saveTime, 10.0);
      
      //Default values. Will change depending on how frequently tag is detected
      private_node_handle.param("temporalDecayPeriod",  temp,std::string("1"));
      temporalDecayPeriod=std::stod(temp);
      
      private_node_handle.param("temporalDecayFactor", temp,std::string("0.05"));
      temporalDecayFactor=std::stod(temp);
      
      private_node_handle.param("tagToDecayRate", temp,std::string("50"));
      tagToDecayRate=std::stod(temp);
      
  
      numDetections=0;
      
      
      private_node_handle.param("object", object_name, std::string("noname_object"));
      
      gridmap_image_file=object_name+"_grid.png";
      
      // basically loads a lot of ROS parameters 
      mapAreas=loadAreas();

    
  }

    rfid_gridMap::~rfid_gridMap(){
        ros::TimerEvent ev;
        rfid_gridMap::saveMapCallback(ev);
    }
    
    void rfid_gridMap::tagCallback(const rfid_node::TagReading::ConstPtr& msg){
        
        
            
            
        // robot position
        double x,y,rh;
        double roll, pitch, yaw;        
        
        // use just a circle as confidence area
        bool oldMode=false;
        
        
        // probability increments:
        //    inside frontal cone        
        double highProb;
        //    inside close circle
        double midProb;        
        //    decrement prob
        double lowProb;
        
        // tag reading data
        double txPower; //dBm        
        double txLoss; //received power in nanoWatts

       if (msg->ID.compare(tagID)==0){           
        //update robot position
        updateTransform();
        //where to plot circle (m)
        x=transform_.getOrigin().x();
        y=transform_.getOrigin().y();
        
        if ((x!=0.0)&&(y!=0.0))
        {
            // count as a good one
            numDetections+=1;
            
            tf::Quaternion 	q=transform_.getRotation();
            tf::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            rh=yaw;

            txPower=msg->txP;        
            txLoss=std::pow( 10.0 , 9+(msg->rssi -30-txPower) /10.0   ); // 9 is for received power in nanoWatts
		
			txLoss= -(msg->rssi) /100.0;
			if (txLoss<0.0){
				txLoss=0.0;
			}
            
            lowProb =       txLoss * weight_dec;
			midProb = 0.5 * txLoss * weight_inc;
            highProb =      txLoss * weight_inc;
            
            //ROS_DEBUG("%3.5f: %3.5f, %3.5f, %3.5f",txLoss,lowProb,midProb,highProb);
            
            //weight_dec=0.02*(weight_inc+0.1);
            
            updateLastDetectionPose(x,y);

            //We decrease probability outside the confidence region
            drawSquare(-size_x/2,-size_y/2,size_x/2,size_y/2,-weight_dec);            
            
            //And increase inside 
            if (oldMode){
                drawCircle( x,  y,  detectRadius, weight_inc);      
            } else {
                drawDetectionShape(x,y, rh, 
                   detectRadius, cone_range, cone_angle, 
                   lowProb, midProb,   highProb);
            }
            
              
        } else {
            ROS_DEBUG("Robot is reporting to be at origin ... %2.2f, %2.2f",x,y);

        }
        
       }
       
     // Start lowering probability over time
     if (numDetections==1)
         nextTimeDecay();

    }
    
    void rfid_gridMap::saveMapCallback(const ros::TimerEvent&){
        sensor_msgs::Image image;
        cv_bridge::CvImagePtr cv_ptr;
                
      
            
        GridMapRosConverter::toImage(map_, layerName, rosEncoding, image);
        // image to file...
        try
        {
          cv_ptr = cv_bridge::toCvCopy(image, rosEncoding);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
        } 
        
        cv::imwrite( (save_route+gridmap_image_file), cv_ptr->image );
    }
    
    void rfid_gridMap::updateMapCallback(const ros::TimerEvent&){
        publishMap();
     }
     
    void rfid_gridMap::temporalDecayCallback(const ros::TimerEvent&)
    {
     drawSquare(-size_x/2,-size_y/2,size_x/2,size_y/2,-temporalDecayFactor);
     
     // schedule next decay event: so we can change period
    nextTimeDecay();                 
    
     }
     
     void rfid_gridMap::nextTimeDecay(){
         ros::Duration durTimer(temporalDecayPeriod+(numDetections/tagToDecayRate));
         
         decayTimer = nodeHandle_.createTimer(durTimer,  &rfid_gridMap::temporalDecayCallback,this,true);
         
         //time_t rawtime=(ros::Time::now() + durTimer).sec;
         //ROS_DEBUG("Next decay in %3.3f secs : %s", durTimer.toSec(), ctime (&rawtime)  );

         
     }
     
     // see config/FDG3.yaml to see an example of  yaml to be parsed
    std::vector<rfid_gridMap::type_area> rfid_gridMap::loadAreas(){
        std::vector<type_area> mapa;
        XmlRpc::XmlRpcValue regionsMap;

        nodeHandle_.getParam("/Regions/",regionsMap);        
    
        ROS_ASSERT_MSG(regionsMap.getType() == XmlRpc::XmlRpcValue::TypeArray,"YAML Region file has wrong format. Regions param is not an array ");
        
        for(int m=0; m<regionsMap.size(); ++m) {
            string regionName =regionsMap[m]["name"];
            type_area area;  
            area.name=regionName;
            area.prob=0.0;
                                 
            XmlRpc::XmlRpcValue points;            
            points=regionsMap[m]["points"];
            
            ROS_ASSERT_MSG(points.getType() == XmlRpc::XmlRpcValue::TypeArray,"YAML Region file has wrong format. points is not an array ");
            
            for(int j=0; j<points.size(); ++j) {
                        double px=0;
			double py=0;
                        if ( j % 2 == 0 ){
                            px=points[j];
                        }else {                        
                            py=points[j];
                            Position p=Position(px,py);
                            area.polygon.addVertex(p);
                        }
            }
            mapa.push_back(area);
            
            if (regionsMap[m].hasMember("subregions")) {
                std::vector<rfid_gridMap::type_area> mapSubs;
                    
                for(int s=0; s<regionsMap[m]["subregions"].size(); ++s) {                    
                    string subRegionName =regionsMap[m]["subregions"][s]["name"];

                    type_area area;  
                    area.name=subRegionName;
                    area.prob=0.0;
                    XmlRpc::XmlRpcValue points;

                    points=regionsMap[m]["subregions"][s]["points"];

                    ROS_ASSERT_MSG(points.getType() == XmlRpc::XmlRpcValue::TypeArray,"YAML Region file has wrong format. points is not an array ");

                    for(int j=0; j<points.size(); ++j) {
                        double px=0;
                        double py=0;
                                                
                        if ( j % 2 == 0 ){
                            px=points[j];
                        }else {                        
                            py=points[j];
                            Position p=Position(px,py);
                            area.polygon.addVertex(p);
                        } 
                    }
                    mapSubs.push_back(area);

                }               
                mapSubAreas[area.name]=mapSubs;                 
            }
            
        }
        return mapa;
      
      }

    void rfid_gridMap::updateProbs(const ros::TimerEvent&){
		double total=0;

		double val=0;
		std::stringstream sstream;
		int i;
		
		total=0;
        
            
        double min_d=10000.0;
        double d=0.0;
        
        //count weigh in each region
		for (std::size_t i=0;i<mapAreas.size();i++) { 
            

             val=countValuesInArea(mapAreas[i].polygon);
             mapAreas[i].prob=val;
             total+=val;		 
              
             // If region has subregions, count their respecting values (this could be done better...)
             std::map<std::string,std::vector<rfid_gridMap::type_area>>::iterator it = mapSubAreas.find(mapAreas[i].name);
             if (it != mapSubAreas.end()){
                 double sub_area_total=0;
                 
                 //ROS_DEBUG("Region [%s] has [%lu] subregions",mapAreas[i].name.c_str(),it->second.size());
                 
                 for (std::size_t k=0;k<it->second.size();k++) {                     
                     val=countValuesInArea(it->second[k].polygon);
                     it->second[k].prob=val;
                     sub_area_total+=val;	                     	                      
                     
                     //ROS_DEBUG("SUB Region [%s] has [%3.3f] weight",it->second[k].name.c_str(),it->second[k].prob);
                 }                 
                 for (std::size_t k=0;k<it->second.size();k++) { 
                     if (sub_area_total>0.0)
                         it->second[k].prob=it->second[k].prob/sub_area_total;
                     else
                         it->second[k].prob=0;		                     
                 }                                  
             } //else{
                 //std::vector<rfid_gridMap::type_area> subAreasVec=it->second;
                 //ROS_DEBUG("Region [%s] has [0] subregions",mapAreas[i].name.c_str());
                 
                // }    //...................

             // also check if robot is inside this region
             // use polygon method to check points inside
             wasHere(mapAreas[i]);
                          
             // use minimum distance to centroid: works better
             d = (mapAreas[i].polygon.getCentroid() - lastP).norm();     
             if (d<=min_d) {
                 lastRegion=mapAreas[i];
                 min_d=d;
            }
            
 
		}
        

          
       // first element in published probs is latest region with invalid prob.   
       sstream<<lastRegion.name+",-1";
        
        //then, each region with its probability
       for (std::size_t i=0;i<mapAreas.size();i++){
            
            if (total>0.0)
                mapAreas[i].prob=mapAreas[i].prob/total;
            else
                mapAreas[i].prob=0;			
            
            //ROS_DEBUG("Region [%s] has Prob. [%3.3f]",mapAreas[i].name.c_str(),mapAreas[i].prob);
            
            sstream<<","<<mapAreas[i].name<<","<<mapAreas[i].prob;			 
            
            // If region has subregions, also list them...................
             std::map<std::string,std::vector<rfid_gridMap::type_area>>::iterator it = mapSubAreas.find(mapAreas[i].name);
             if (it != mapSubAreas.end()){
                 
                 //std::vector<rfid_gridMap::type_area> subAreasVec=it->second;
                 //ROS_DEBUG("Region [%s] has Prob. [%3.3f]",mapAreas[i].name.c_str(),mapAreas[i].prob);
                 
                 for (std::size_t k=0;k<it->second.size();k++) { 
                     if (it->second[k].prob>0.0)
                         it->second[k].prob=it->second[k].prob * mapAreas[i].prob;
                 
                     //ROS_DEBUG("SUB Region [%s] has ABS prob. [%3.3f]",it->second[k].name.c_str(),it->second[k].prob);
                 
                     sstream<<","<<it->second[k].name<<","<<it->second[k].prob;			 
                 }                                  
             }    
            //...................            
            
        }		
        
        // Publish stream of probs.
        std_msgs::String msg;        
        msg.data=sstream.str();
        prob_pub_.publish(msg);
        
        
            
    }
    
    void rfid_gridMap::updateTransform(){
        // TODO these are NOT frame ids 
        try{
            listener_.waitForTransform(global_frame, robot_frame, ros::Time(0), ros::Duration(0.5) );
            listener_.lookupTransform(global_frame, robot_frame, ros::Time(0), transform_);
            
            
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }    
    }
    
    void rfid_gridMap::publishMap(){
      
      map_.setTimestamp(ros::Time::now().toNSec());
      
      grid_map_msgs::GridMap message;
      grid_map::GridMapRosConverter::toMessage(map_, message);
      gridMapPublisher_.publish(message);
      ////ROS_DEBUG("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
    }

    double rfid_gridMap::countValuesInArea(Polygon pol){ 
      double total=0.0;
      
      for (grid_map::PolygonIterator iterator(map_, pol); !iterator.isPastEnd(); ++iterator) {     
          
// here
            if (!isnan(map_.at(layerName, *iterator)))
            {
                total+=map_.at(layerName, *iterator);
            }
      }
      
      return total;
    }

    void rfid_gridMap::updateLastDetectionPose(double x, double y){
        lastP=Position(x,y);
    }

    void rfid_gridMap::wasHere(type_area area){            
        ////ROS_DEBUG("Point (%3.3f,%3.3f)", lastP.x(),lastP.y());
        if (area.polygon.isInside(lastP)){
            lastRegion=area;
            ////ROS_DEBUG("We are in area %s", area.name.c_str());
        } else {
            ////ROS_DEBUG("We are NOT in area %s", area.name.c_str());
        }
            
    }

    // we get information from our global map
    void rfid_gridMap::mapCallback(const nav_msgs::OccupancyGrid& msg){
        if (!isMapLoaded){
            if ((msg.info.width>0.0)&&(msg.info.height>0.0)){
                isMapLoaded=true;
                mapDesc=msg.info;
                mapFrame=msg.header.frame_id;
                /*ROS_DEBUG("Received a %d X %d map @ %.3f m/pix  Origin X %.3f Y %.3f\n",
                        msg.info.width,
                        msg.info.height,
                        msg.info.resolution,
                        msg.info.origin.position.x,
                        msg.info.origin.position.y);
                */
            } else {           
                ROS_ERROR("Received an INVALID!! %d X %d map @ %.3f m/pix  Origin X %.3f Y %.3f\n",
                        msg.info.width,
                        msg.info.height,
                        msg.info.resolution,
                        msg.info.origin.position.x,
                        msg.info.origin.position.y);
            }  
        } else {
            map_sub_.shutdown();       
        }
    }

    void rfid_gridMap::drawPolygon(const grid_map::Polygon poly,
    double value){      
      for (grid_map::PolygonIterator iterator(map_, poly); !iterator.isPastEnd(); ++iterator) {     
            map_.at(layerName, *iterator) =  value +map_.at(layerName, *iterator);              
            if (isnan(map_.at(layerName, *iterator)))
            {
                map_.at(layerName, *iterator) =  value;
            }
        if (map_.at(layerName, *iterator)<lowerValue)
            map_.at(layerName, *iterator) =  lowerValue;
        if (map_.at(layerName, *iterator)>upperValue)
            map_.at(layerName, *iterator) = upperValue;       
      }
    }

    void rfid_gridMap::drawSquare(double start_x,double start_y,
    double end_x,double end_y,double value){
        grid_map::Polygon poly;
        Position p;
        
        p=Position(start_x,start_y);  
        poly.addVertex(p);
        p=Position(end_x,start_y);  
        poly.addVertex(p);
        p=Position(end_x,end_y);  
        poly.addVertex(p);
        p=Position(start_x,end_y);  
        poly.addVertex(p);
        
        for (grid_map::PolygonIterator iterator(map_, poly); !iterator.isPastEnd(); ++iterator) {
            map_.at(layerName, *iterator) =  value + map_.at(layerName, *iterator);              
            if (isnan(map_.at(layerName, *iterator)))
            {
                map_.at(layerName, *iterator) =  value;
            }
            
            
            if (map_.at(layerName, *iterator)<lowerValue)
                map_.at(layerName, *iterator) =  lowerValue;
            /*
            if (map_.at(layerName, *iterator)>upperValue)
                map_.at(layerName, *iterator) = upperValue;
            */
        }
    }

    // based on demoCircleIterator
    void rfid_gridMap::drawCircle(double x, double y, double radius, 
    double value){
      ////////ROS_INFO("Plotting circle at (%2.2f,%2.2f)",x,y);
      
      Position center(x, y);
      
      for (grid_map::CircleIterator iterator(map_, center, radius);
          !iterator.isPastEnd(); ++iterator) {
        map_.at(layerName, *iterator) =  value +map_.at(layerName, *iterator);
        
        if (isnan(map_.at(layerName, *iterator)))
        {
            map_.at(layerName, *iterator) =  value;
        }    
        if (map_.at(layerName, *iterator)<lowerValue)
            map_.at(layerName, *iterator) =  lowerValue;
        if (map_.at(layerName, *iterator)>upperValue)
            map_.at(layerName, *iterator) = upperValue;
      }
    }




    void rfid_gridMap::drawDetectionShape(double cx,double cy, double rh, 
            double radius,double cone_range,  double cone_heading, 
            double lowProb, double midProb, double highProb){
        ////////////////////////////////////////////////////////////////
      double x;
      double y;
      double tetha;
      double r;

      Position position;
      Position robotPose(cx,cy);
    
      // shape is a circle slighly ahead the antenna      
      double sx=robotPose.x()+(cone_range-radius)*std::cos(rh);
      double sy=robotPose.y()+(cone_range-radius)*std::sin(rh);
      Position center(sx, sy);
    
      //inside this circle we have high and mid probs
      for (grid_map::CircleIterator iterator(map_, center, radius);
          !iterator.isPastEnd(); ++iterator) {
        
        if (isnan(map_.at(layerName, *iterator)))
        {
            map_.at(layerName, *iterator) =  0;
        }
        
        map_.getPosition(*iterator, position);
        
        
        // cell position respect robot center
        x=position.x()-robotPose.x();
        y=position.y()-robotPose.y();
        // angle between robot heading and cell position
        tetha=constrainAnglePI(std::atan2(y,x)-rh);   

        
        if (std::abs(tetha)>(cone_heading/2)){
                map_.at(layerName, *iterator) += midProb;
        } else {
            map_.at(layerName, *iterator) += highProb;
        }  
      }
      
      // cells just ahead robot pose would never be updated otherwise
      //position=Position(cx+(resolution/2)*std::cos(rh),cy+(resolution/2)*std::cos(rh));
      //map_.atPosition(layerName, position)+=highProb;

      //position=Position(cx+(resolution/2)*std::cos(rh),cy-(resolution/2)*std::cos(rh));
      //map_.atPosition(layerName, position)+=highProb;
        
    }
    
    //0 to 2pi
    double rfid_gridMap::constrainAngle2PI(double x){
    x = fmod(x,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x;
    }

    //-pi,pi
    double  rfid_gridMap::constrainAnglePI(double x){
        x = fmod(x + M_PI,2*M_PI);
        if (x < 0)
            x += 2*M_PI;
        return x - M_PI;
    }

    
} // end of namespace rfid_grid_map






/*
void rfid_gridMap::drawSquare0(double start_x,double start_y,double end_x,double end_y,double value)
{
  Index submapStartIndex;
  Index submapEndIndex;
  Index submapBufferSize;
  
  Index lowerCorner(0,0);
  Index upperCorner(0,0);
    
  Position position;
  
  Position submapStartPosition(end_x, end_y);
  Position submapEndPosition(start_x, start_y);  
  
  Size mapSize=map_.getSize();
  ROS_DEBUG("Map is (%d w %d h)",mapSize(0),mapSize(1));
  upperCorner(0)=  mapSize(0)-1;
  upperCorner(1)=  mapSize(1)-1;  
  
  if (!map_.isInside(submapStartPosition)){
      ROS_DEBUG("Start Position is out of map");
      map_.getPosition( lowerCorner, position ) ;
        
      //rounding
      if (submapStartPosition(0)<position(0))
            submapStartPosition(0)=position(0);
      if (submapStartPosition(1)<position(1))
            submapStartPosition(1)=position(1);
      
      ROS_DEBUG("Accessing  from position (%2.2f,%2.2f) ",submapStartPosition(0),submapStartPosition(1) );    
  }
  if (!map_.isInside(submapEndPosition)){
      ROS_DEBUG("End Position is out of map");
       map_.getPosition( upperCorner, position ) ;
        
      //rounding
      if (submapEndPosition(0)<position(0))
            submapEndPosition(0)=position(0);
      if (submapEndPosition(1)<position(1))
            submapEndPosition(1)=position(1);
      
      ROS_DEBUG("Accessing up to position (%2.2f,%2.2f)",submapEndPosition(0),submapEndPosition(1) );    
  }
  
  map_.getIndex(submapStartPosition,submapStartIndex);
  map_.getIndex(submapEndPosition,submapEndIndex);
 ROS_DEBUG("Accessing  from index (%d,%d) to (%d,%d)",submapStartIndex(0),submapStartIndex(1),submapEndIndex(0),submapEndIndex(1) );    
      
  submapBufferSize=abs(submapEndIndex-submapStartIndex);
  
 ROS_DEBUG("Accessing (%d width x %d high) indexes",submapBufferSize(0),submapBufferSize(1));    
  for (grid_map::SubmapIterator iterator(map_, submapStartIndex, submapBufferSize);
      !iterator.isPastEnd(); ++iterator) {     
        
        map_.at(layerName, *iterator) =  value +map_.at(layerName, *iterator);              
        if (isnan(map_.at(layerName, *iterator)))
        {
            map_.at(layerName, *iterator) =  value;
        }
        if (map_.at(layerName, *iterator)<lowerValue)
            map_.at(layerName, *iterator) =  lowerValue;
        if (map_.at(layerName, *iterator)>upperValue)
            map_.at(layerName, *iterator) = upperValue;
  }

}


    void rfid_gridMap::drawDetectionShape(double cx,double cy, double rh, 
            double radius,double cone_range,  double cone_heading, 
            double lowProb, double midProb, double highProb){
        ////////////////////////////////////////////////////////////////
      double x;
      double y;
      double tetha;
      double r;

      Position position;
      Position center(cx, cy);


      for (grid_map::CircleIterator iterator(map_, center, radius);
          !iterator.isPastEnd(); ++iterator) {
        
        if (isnan(map_.at(layerName, *iterator)))
        {
            map_.at(layerName, *iterator) =  0;
        }
        
        
        
        map_.getPosition(*iterator, position);
        x=position.x()-center.x();
        y=position.y()-center.y();
        tetha=std::abs(std::atan2(y,x)-rh);   
        r=std::sqrt(x*x+y*y);


        if ((tetha-cone_heading)>=0){
            if (r<cone_range){
                map_.at(layerName, *iterator) += midProb;
            } else {
                map_.at(layerName, *iterator) += lowProb;			
            }
        } else {
            map_.at(layerName, *iterator) += highProb;
        }
                
    
        
        }
        
    }
*/
