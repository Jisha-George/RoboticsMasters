
/**
 * ToDo: parametrize decay time, prob publish time, intensity
 *
 *
 * */


#include "rfid_grid_map/rfid_gridMap2.hpp"


namespace rfid_grid_map2 {


    rfid_gridMap2::rfid_gridMap2(ros::NodeHandle& n)
    : nodeHandle_(n),
    // hardcoding layer Name as 'type' and creating a gridmap with that name. Could be improved...
    layerName("type"),
    map_(vector<string>({std::string("type")}))
    {

      loadROSParams();

    prev_x = 0.0;
    prev_y = 0.0;
    prev_h = 0.0;
    min_d = 0.05;
    min_a = M_PI/1800.0; // about 0.1 degree

      getMapDimensions();
      //these parameters are hardcoded...
      rosEncoding="mono16";
      lowerValue=0.0;
      upperValue=10.0;

      // map Size (m.)
      size_x=mapDesc.width*mapDesc.resolution;
      size_y=mapDesc.height*mapDesc.resolution;

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
      showROSParams();

       //for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
       //     map_.at(layerName, *iterator)=0;
       //}


      lastRegion.name=std::string(" ");

      gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(grid_map_name, 1, true);
      //ROS_ERROR("1.- Grid map number of layers is: %lu", map_.getLayers().size());
      publishMap();

      prob_pub_ = nodeHandle_.advertise<std_msgs::String>(prob_pub_name, 1000);

      // get tag readings
      ros::Subscriber sub_ = nodeHandle_.subscribe(rfid_readings_topic_name, 1000, &rfid_gridMap2::tagCallback, this);

      // Update displayed map periodically
      ros::Timer timer = nodeHandle_.createTimer(ros::Duration(mapUpdatePeriod),  &rfid_gridMap2::updateMapCallback,this);

      // publish updated probabilities every reasonable time.
      ros::Timer timer2 = nodeHandle_.createTimer(ros::Duration(probUpdatePeriod),  &rfid_gridMap2::updateProbs,this);

      // publish updated probabilities every reasonable time.
      ros::Timer timer3 = nodeHandle_.createTimer(ros::Duration(saveTime),  &rfid_gridMap2::saveMapCallback,this);

      ROS_DEBUG("Ready...");

      //ROS_ERROR("3.- Grid map number of layers is: %lu", map_.getLayers().size());
      ros::spin();

    }

    void rfid_gridMap2::showROSParams(){

      //...........................................
      cout<<"Current logger is:["<< ROSCONSOLE_DEFAULT_NAME <<"]"<< endl;
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

      ROS_DEBUG("map_topic_name: %s", map_topic_name.c_str());
      ROS_DEBUG("map_service_name: %s", map_service_name.c_str());

      ROS_DEBUG("global_frame: %s", global_frame.c_str());
      ROS_DEBUG("robot_frame: %s", robot_frame.c_str());
      ROS_DEBUG("tagID: %s", tagID.c_str());
      ROS_DEBUG("rfid readings topic: %s", rfid_readings_topic_name.c_str());

      ROS_DEBUG("grid_map_name: %s", grid_map_name.c_str());
      ROS_DEBUG("load/save_route is [%s]",save_route.c_str());
      if (loadGrids){
        ROS_DEBUG("Load Saved data [TRUE]");
      }else{
        ROS_DEBUG("Load Saved data [FALSE]");
      }

  }


    void rfid_gridMap2::getMapDimensions(){
        isMapLoaded=false;
        bool failed_once=false;
        // connect to map server and get dimensions and resolution (meters)
        while (!isMapLoaded){
            // Try getting map properties from topic
            boost::shared_ptr<nav_msgs::OccupancyGrid const> mapPointer;


            mapPointer=ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic_name,nodeHandle_,ros::Duration(0.5));

            if(mapPointer==NULL){
                ROS_ERROR("Topic \"map\" not available, trying service ");
                failed_once=true;
            }else{
                mapCallback(*mapPointer);
            }


            if (!isMapLoaded){
                // Try getting map properties from service...
                if (ros::service::waitForService(map_service_name,500)) {
                    nav_msgs::GetMap::Request  req;
                    nav_msgs::GetMap::Response resp;
                    ros::service::call(map_service_name, req, resp);
                    if(failed_once){
                        ROS_ERROR("Service replied!");
                    }
                    mapCallback(resp.map);
                } else {
                    ROS_ERROR("Service at \"%s\" unavailable too, trying topic again ...", map_service_name.c_str());
                }
            }
        }
    }





    void rfid_gridMap2::loadROSParams(){
              ros::NodeHandle private_node_handle("~");

      // LOAD ROS PARAMETERS ....................................
      std::string temp;

      private_node_handle.param("rfid_readings_topic_name", rfid_readings_topic_name, std::string("/lastTag"));
      private_node_handle.param("map_topic_name", map_topic_name, std::string("/map"));
      private_node_handle.param("map_service_name", map_service_name, std::string("/static_map"));

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

      // load ROS parameters describing zones of interest
      loadZois();


  }

    rfid_gridMap2::~rfid_gridMap2(){
        ros::TimerEvent ev;
        rfid_gridMap2::saveMapCallback(ev);
    }

    bool rfid_gridMap2::isUpdatePose(){
        bool ans;
         // robot position
        double x,y,h;
        double dist;
        double ang;
        double roll, pitch, yaw;
        tf::Quaternion q;

        x=transform_.getOrigin().x();
        y=transform_.getOrigin().y();
        q=transform_.getRotation();
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        h=yaw;

        dist = sqrt( pow(x-prev_x, 2) + pow(y-prev_y, 2));
        ang = fabs(remainder( h-prev_h, 2.0 * M_PI) );

        // if robot has moved OR turned over a threshold...
        ans = (ang > min_a);
        ans |= (dist > min_d);

        prev_x = x;
        prev_y = y;
        prev_h = h;

        //ROS_ERROR("Dist, ang incs. (%2.2f, %2.2f)",dist,ang);
        return ans;
    }

    void rfid_gridMap2::tagCallback(const rfid_node::TagReading::ConstPtr& msg){

        ROS_DEBUG_THROTTLE(2,"Received tag!");
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

        if (isUpdatePose())//((x!=0.0)&&(y!=0.0))
        {
            ROS_DEBUG_THROTTLE(2,"My tag!");
            // count as a good one
            numDetections+=1;

            tf::Quaternion  q=transform_.getRotation();
            tf::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            rh=yaw;

            if (msg->txP>0){
                txPower=msg->txP/100.0;
            } else {
                // comes from simulation or a weirder place
                txPower=20.0/100.0;
            }
            txLoss=std::pow( 10.0 , 11+(msg->rssi -30-txPower) /10.0   ); // 9+ is for received power in nanoWatts

            txLoss= (100 - msg->rssi) /100.0;
            if (txLoss<0.0){
                txLoss=0.0;
            }

            lowProb  =       txLoss * weight_dec;
            midProb  = 0.5 * txLoss * weight_inc;
            highProb =       txLoss * weight_inc;

            //ROS_DEBUG(".");
            //ROS_INFO("rssi (%d) | txPower(%d) -> txLoss (%3.6f)-> Probs %3.6f, %3.6f, %3.6f \n",msg->rssi,msg->txP, txLoss,lowProb,midProb,highProb);

            //weight_dec=0.02*(weight_inc+0.1);

            updateLastDetectionPose(x,y);


            //We decrease probability outside the confidence region
            //drawSquare(-size_x/2,-size_y/2,size_x/2,size_y/2,-weight_dec);

            //And increase inside
            if (oldMode){
                drawCircle( x,  y,  detectRadius, weight_inc);
            } else {                
                /*drawDetectionShape(x,y, rh,
                   detectRadius, cone_range, cone_angle,
                   lowProb, midProb,   highProb);*/
                // drawSimilarityShape(x,y, rh,
                //    detectRadius, cone_range, cone_angle,
                //    lowProb, midProb,   highProb,msg->rssi);

            double a, b;
            // ellipses follow:
            // a^2 = b^2 + c^2 
            // we define:
            // cone_range = a 
            // detectRadius = b
            // c = sqrt(a^2 - b^2)

                
                a =  cone_range;
                b = detectRadius;

                drawEllipticSimilarityShape(x,  y, rh,
                    a,  b, cone_angle, 
                    lowProb, midProb, highProb,  msg->rssi );
            }


        } else {
        //ROS_DEBUG("Robot pose (%2.2f, %2.2f) is almost not changing...",x,y);
        }

       }

     // Start lowering probability over time
     if (numDetections==1)
         nextTimeDecay();

    }

    void rfid_gridMap2::saveMapCallback(const ros::TimerEvent&){
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

    void rfid_gridMap2::updateMapCallback(const ros::TimerEvent&){
        publishMap();
     }

    void rfid_gridMap2::temporalDecayCallback(const ros::TimerEvent&){
        if (isUpdatePose())//((x!=0.0)&&(y!=0.0))
        {
            drawSquare(-size_x/2,-size_y/2,size_x/2,size_y/2,-temporalDecayFactor);
        }
     // schedule next decay event: so we can change period
    nextTimeDecay();

     }

    void rfid_gridMap2::nextTimeDecay(){
         ros::Duration durTimer(temporalDecayPeriod+(numDetections/tagToDecayRate));

         decayTimer = nodeHandle_.createTimer(durTimer,  &rfid_gridMap2::temporalDecayCallback,this,true);

         //time_t rawtime=(ros::Time::now() + durTimer).sec;
         //ROS_DEBUG("Next decay in %3.3f secs : %s", durTimer.toSec(), ctime (&rawtime)  );


     }

/*
 *  Loads zois as poligons.
 *  A subzoi will be identified by having a '_' in the middle of its name: [zoiName]_[subZoiName]
 * */
    void rfid_gridMap2::loadZois(){
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
        Position p;

        std::map<std::string,rfid_gridMap2::type_area>::iterator map_it;

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
                //std::cout<< "point name: " << zoi_point_name<< "\n";

                // zoi point number is last thing after '_'
                slashPos = zoiPointName.find_last_of("_");
                if (slashPos!=std::string::npos)
                {
                    //std::cout << " zoi name: " << zoiPointName.substr(0,slashPos) << '\n';
                    //std::cout << " point num: " << zoiPointName.substr(slashPos+1) << '\n';
                    zoiPoint_num=std::stoi(zoiPointName.substr(slashPos+1));

                    //region
                    zoiName = static_cast<std::string>(itr->second[1]);
                    px =  itr->second[2];
                    py =  itr->second[3];

                    map_it = mapAreas.find(zoiName);
                    if (map_it == mapAreas.end())
                    {
                        type_area area;
                        area.name=zoiName;
                        area.prob=0.0;
                        mapAreas[zoiName]=area;
                    }

                    p=Position(px,py);
                    mapAreas[zoiName].polygon.addVertex(p);

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
                std::cout <<  "ERROR:" << e.getMessage ()<<" .... \n";
            }

        }

        //ROS_DEBUG("Loaded [%lu] zones:", mapAreas.size());
        //for (std::map<std::string,rfid_gridMap2::type_area>::iterator mapIt=mapAreas.begin(); mapIt!=mapAreas.end(); ++mapIt)
        //{
        //    ROS_DEBUG("- [%s]: %lu points", mapIt->first.c_str(),mapIt->second.polygon.nVertices() );
        //}

      }

    bool rfid_gridMap2::isSubregion(std::string zoiName,std::string &parent){
        std::string posibleParent;
        for (std::map<std::string,rfid_gridMap2::type_area>::iterator mapIt=mapAreas.begin(); mapIt!=mapAreas.end(); ++mapIt)
        {
            posibleParent = mapIt->first;
            std::size_t found = zoiName.find(posibleParent);
            if ((found!=std::string::npos)&&( posibleParent.compare(zoiName) != 0 ))
            {
                parent = posibleParent;
                //ROS_DEBUG("[%s] is subregion of [%s], at pos %lu",zoiName.c_str(),parent.c_str(),found);
                return true;
            }
        }
        return false;
    }

    void rfid_gridMap2::updateProbs(const ros::TimerEvent&){
        double total=0;

        double val=0;
        std::stringstream sstream;
        int i;
        std::string father;
        total=0;

        // subzois indexed by zoi name
        std::map<std::string,double>::iterator reg_it;
        std::map<std::string,double> regionsCount;

        double min_d=10000.0;
        double d=0.0;

        //ROS_DEBUG("Region weights:  " );
        //count weigh in each region
        for (std::map<std::string,rfid_gridMap2::type_area>::iterator mapIt=mapAreas.begin(); mapIt!=mapAreas.end(); ++mapIt)
        {
             val=countValuesInArea(mapIt->second.polygon);
             mapIt->second.prob=val;

             // if it's not a subregion, cumulated weight goes to total
             if (!isSubregion(mapIt->first,father))
             {
                total+=val;
                //ROS_DEBUG("- [%s]: %3.3f  ", mapIt->first.c_str(),val );

             //if it's a subregion, cumulated weight goes to its sublist
             }
             else
             {
                //ROS_DEBUG("x [%s]: %3.3f  ", mapIt->first.c_str(),val );

                reg_it = regionsCount.find(father);
                if (reg_it == regionsCount.end())
                {
                    regionsCount[father]=0.0;
                }
                regionsCount[father]+=val;

            }

             // also check if robot is inside this region
             // use polygon method to check points inside
             wasHere(mapIt->second);

             // use minimum distance to centroid: works better
             d = (mapIt->second.polygon.getCentroid() - lastP).norm();
             if (d<=min_d) {
                 lastRegion=mapIt->second;
                 min_d=d;
            }
        }

       // first element in published probs is latest region with invalid prob.
       sstream<<lastRegion.name+",-1";
        //ROS_DEBUG("Total weight: %3.3f  ", total );
        //ROS_DEBUG("Region probs:  " );

        //then, each region with its probability
        //for (std::size_t i=0;i<mapAreas.size();i++)
       for (std::map<std::string,rfid_gridMap2::type_area>::iterator mapIt=mapAreas.begin(); mapIt!=mapAreas.end(); ++mapIt)
       {

            if (total>0.0)
            {
                if (!isSubregion(mapIt->first,father)){
                    mapIt->second.prob=mapIt->second.prob/total;
                } else {
                    // this would be relative probability inside its region
                    mapIt->second.prob=mapIt->second.prob/regionsCount[father];
                    // now we turn this into global probability...
                    mapIt->second.prob*= mapAreas[father].prob;
                }
            }else{
                mapIt->second.prob=0;
            }
            //ROS_DEBUG("- [%s]: %3.3f  ", mapIt->first.c_str(),mapIt->second.prob );
            sstream<<","<<mapIt->second.name<<","<<mapIt->second.prob;

        }

        // Publish stream of probs.
        std_msgs::String msg;
        msg.data=sstream.str();
        prob_pub_.publish(msg);


        //ROS_DEBUG("- [total]: %3.3f ", total );


    }

    void rfid_gridMap2::updateTransform(){
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

    void rfid_gridMap2::publishMap(){

      map_.setTimestamp(ros::Time::now().toNSec());

      grid_map_msgs::GridMap message;
      grid_map::GridMapRosConverter::toMessage(map_, message);
      //ROS_ERROR("2.- Grid map number of layers is: %lu", map_.getLayers().size());
      gridMapPublisher_.publish(message);
      //ROS_ERROR("Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
      //ROS_ERROR("Grid map message layer name is: %s", message.layers[0].c_str());

    }

    double rfid_gridMap2::countValuesInArea(Polygon pol){
      double total=0.0;


      for (grid_map::PolygonIterator iterator(map_, pol); !iterator.isPastEnd(); ++iterator) {
            if (!isnan(map_.at(layerName, *iterator)))
            {
                if (map_.at(layerName, *iterator)>0.01)
                {
                    total+=map_.at(layerName, *iterator);
                }
            }
      }


      return total;
    }

    void rfid_gridMap2::updateLastDetectionPose(double x, double y){
        lastP=Position(x,y);
    }

    void rfid_gridMap2::wasHere(type_area area){
        ////ROS_DEBUG("Point (%3.3f,%3.3f)", lastP.x(),lastP.y());
        if (area.polygon.isInside(lastP)){
            lastRegion=area;
            ////ROS_DEBUG("We are in area %s", area.name.c_str());
        } else {
            ////ROS_DEBUG("We are NOT in area %s", area.name.c_str());
        }

    }

    // we get information from our global map
    void rfid_gridMap2::mapCallback(const nav_msgs::OccupancyGrid& msg){
        if (!isMapLoaded){
            if ((msg.info.width>0.0)&&(msg.info.height>0.0)){
                isMapLoaded=true;
                mapDesc=msg.info;
                mapFrame=msg.header.frame_id;
                ROS_DEBUG("Received a %d X %d map @ %.3f m/pix  Origin X %.3f Y %.3f\n",
                        msg.info.width,
                        msg.info.height,
                        msg.info.resolution,
                        msg.info.origin.position.x,
                        msg.info.origin.position.y);

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

    void rfid_gridMap2::drawPolygon(const grid_map::Polygon poly, double value){

      for (grid_map::PolygonIterator iterator(map_, poly); !iterator.isPastEnd(); ++iterator) {
            map_.at(layerName, *iterator) =  value +map_.at(layerName, *iterator);
            if (isnan(map_.at(layerName, *iterator)))
            {
                map_.at(layerName, *iterator) =  value;
            }
       /*  if (map_.at(layerName, *iterator)<lowerValue)
            map_.at(layerName, *iterator) =  lowerValue;
        if (map_.at(layerName, *iterator)>upperValue)
            map_.at(layerName, *iterator) = upperValue;
       */
      }

    }

    void rfid_gridMap2::drawSquare(double start_x,double start_y, double end_x,double end_y,double value){
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

             /*
            if (map_.at(layerName, *iterator)<lowerValue)
                map_.at(layerName, *iterator) =  lowerValue;
            if (map_.at(layerName, *iterator)>upperValue)
                map_.at(layerName, *iterator) = upperValue;
            */
        }

    }

    // based on demoCircleIterator
    void rfid_gridMap2::drawCircle(double x, double y, double radius, double value){
      ////////ROS_INFO("Plotting circle at (%2.2f,%2.2f)",x,y);

      Position center(x, y);

      for (grid_map::CircleIterator iterator(map_, center, radius);
          !iterator.isPastEnd(); ++iterator) {
        map_.at(layerName, *iterator) =  value +map_.at(layerName, *iterator);

        if (isnan(map_.at(layerName, *iterator)))
        {
            map_.at(layerName, *iterator) =  value;
        }
        /*if (map_.at(layerName, *iterator)<lowerValue)
            map_.at(layerName, *iterator) =  lowerValue;
        if (map_.at(layerName, *iterator)>upperValue)
            map_.at(layerName, *iterator) = upperValue;*/
      }

    }

    void rfid_gridMap2::drawSimilarityShape(double cx,double cy, double rh,
            double radius,double cone_range,  double cone_heading,
            double lowProb, double midProb, double highProb, double rssi){
        ////////////////////////////////////////////////////////////////
      double x;
      double y;
      double tetha;
      double r;
      double rssi_low= -60.0;
      double rssi_hig= -48.0;
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
            //is out of high power cone
            if (rssi<rssi_low){
                // and we received a low power... matches
                map_.at(layerName, *iterator) += midProb;
            } else{
                // and we receive a high power ... not maches, but still high power
                 map_.at(layerName, *iterator) += lowProb;
            }
        } else {
            //is inside high power cone
            if (rssi>rssi_hig){
                // and we receive a high power ... maches
                map_.at(layerName, *iterator) += highProb;
            } else{
                // and we received a low power... does not match
                 map_.at(layerName, *iterator) += lowProb;
            }
        }
      }
    }




    /**
     * @brief Draws an Elliptical similarity shape over map at requested focal point
     * 
     * @param x_f1 Coordinate x of robot position (meters) (focal point 1)
     * @param y_f1 Coordinate y of robot position (meters) (focal point 1)
     * @param orientAngle Robot orientation (rads) (ellipse orientation)
     * @param a Ellipse mayor radius (meters)
     * @param b Ellipse minor radius (meters)
     * @param cone_angle Front lobe (high prob) arc (rads) centered around robot heading
     * @param lowProbInc  prob increase for cells OUTSIDE front lobe but NO low rssi is received OR
     *                                            INSIDE  front lobe but NO high rssi is received
     * @param midProbInc  prob increase for cells OUTSIDE front lobe when low rssi is received
     * @param highProbInc prob increase for cells INSIDE  front lobe when high rssi is received
     * @param rssi        received signal power 
     * @param rssi_low    rssi threshold to consider a low level 
     * @param rssi_hig    rssi threshold to consider a high level 
     */
    void rfid_gridMap2::drawEllipticSimilarityShape(double x_f1,double y_f1, double orientAngle,
            double a, double b, double cone_angle, 
            double lowProbInc, double midProbInc, double highProbInc, 
            double rssi, double rssi_low, double rssi_hig ){
      //////////////////////////////////////////////////////////////////
      Position position;
      double x,y,tetha;
      bool isOutside;

      // DEBUUUUU!!
      //orientAngle = 0;

      // shape is an ellipse, with robot at one focal point
      double c = sqrt((a*a) - (b*b));

      double cx=x_f1 + ( c * std::cos(orientAngle) );
      double cy=y_f1 + ( c * std::sin(orientAngle) );


      Position center(cx, cy);
      Length length(2.0*a, 2.0*b);

      // FIXME: CELLS JUST IN FRONT OF THE ROBOT ARE NOT INSIDE THE HIGH PROB FOV....!!
      //ROS_INFO("Centre (%3.1f, %3.1f) m", cx, cy);
      //ROS_INFO("Orientation (%3.1f) deg", (orientAngle*180.0/3.141592) );
      //ROS_INFO("Focal point 1 (%3.1f, %3.1f) m", x_f1, y_f1);
      //ROS_INFO("Axes  (%3.1f, %3.1f, %3.1f) m", a, b,c);
      //ROS_INFO("FOV/2  (%3.3f) deg\n", (cone_angle*90.0/3.141592));

      //inside this circle we have high and mid probs
      for (grid_map::EllipseIterator iterator(map_, center, length, orientAngle);
          !iterator.isPastEnd(); ++iterator) {

        if (isnan(map_.at(layerName, *iterator)))
        {
            map_.at(layerName, *iterator) =  0;
        }

        map_.getPosition(*iterator, position);

        // cell position respect robot center
        x=position.x()-x_f1;
        y=position.y()-y_f1;

        // angle between robot heading and cell position
        tetha=constrainAnglePI(std::atan2(y,x)-orientAngle);

        // is isOutside of front lobe?
        isOutside = std::abs(tetha)>(cone_angle/2);
        
        if (isOutside){
        //    ROS_INFO("Outside ABS point  (%3.3f, %3.3f) m %3.3f deg", position.x(), position.y(), (std::atan2(y,x)*180.0/3.141592));
         //   ROS_INFO("Outside Rel point  (%3.3f, %3.3f) m %3.3f deg\n", x, y, (tetha*180.0/3.141592));
            //is out of high power cone
            if (rssi<rssi_low){
                // and we received a low power... matches
                map_.at(layerName, *iterator) += midProbInc;
            } else{
                // and we receive a high power ... not maches, but still high power
                 map_.at(layerName, *iterator) += lowProbInc;
            }
        } else {
            //is inside high power cone
            if (rssi>rssi_hig){
                // and we receive a high power ... maches
                map_.at(layerName, *iterator) += highProbInc;
            } else{
                // and we received a low power... does not match
                 map_.at(layerName, *iterator) += lowProbInc;
            }
        }
      }
    }


    void rfid_gridMap2::drawDetectionShape(double cx,double cy, double rh,
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
    double rfid_gridMap2::constrainAngle2PI(double x){
    x = fmod(x,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x;
    }

    //-pi,pi
    double  rfid_gridMap2::constrainAnglePI(double x){
        x = fmod(x + M_PI,2*M_PI);
        if (x < 0)
            x += 2*M_PI;
        return x - M_PI;
    }


} // end of namespace rfid_grid_map

/*
void RFIDSensorLayer::update_cell(double origin_x, double origin_y, double origin_tetha,
                rfid_node::TagReading& rfid_message, double updatePos_x, double updatePos_y)
{
  unsigned int x, y;
  if(worldToMap(updatePos_x, updatePos_y, x, y)){

    double dx = updatePos_x-origin_x;
    double dy = updatePos_y-origin_y;
    double theta = atan2(dy, dx) - origin_tetha;
    theta = angles::normalize_angle(theta);

    //sensor prob: how likely to get (rssi,phase) at (relx,rely,rela)
    double sensor = sensor_model(dx,dy,theta,rfid_message);

    //prior prob: how likely absolute position cell (x,y) to be occupied given the cost.
    double prior = to_prob(getCost(x,y));

    // probability
    double prob_occ = sensor * prior;
    double prob_norigin_tetha = (1 - sensor) * (1 - prior);

    // likelyhood
    double new_prob = prob_occ/(prob_occ+prob_norigin_tetha);

    unsigned char c = to_cost(new_prob);
    setCost(x,y,c);
  }
}
*/
