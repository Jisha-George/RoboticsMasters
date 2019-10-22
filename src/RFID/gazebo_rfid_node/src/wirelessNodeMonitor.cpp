/**
*
* Based on gazebo-contactMonitor.
* This time we forward  a gazebo WirelessNode msg into a ROS fid_node TagReading
* msg.
*
* TODO:
*     propagation model is inherited from gazebo wireless
*     TagReading transmitted power (txP) is not filled
*     TagReading phase  (phase) is not filled
*
* /gazebo/get_model_state
*/

#include <wirelessNodeMonitor/wirelessNodeMonitor.h>

std::string ros_rfid_topic_name;
std::string gazebo_wireless_node_topic_name;
std::string ros_rfid_frame_id;
ros::Publisher ros_rfid_pub;
int seq;

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void rfid_callback(ConstWirelessNodesPtr& _msg)
{
  int numNodes = _msg->node_size();
  rfid_node::TagReading msg;
  gazebo::msgs::WirelessNode wn;
  msg.timestamp = ros::Time::now();


  for (int i = 0; i < numNodes; i++) {

      wn = _msg->node(i);
      // send data
      msg.header.frame_id = ros_rfid_frame_id;
      // make sure is properly filled...
      msg.header.seq = seq++;
      msg.header.stamp = ros::Time::now();
      msg.ID = wn.essid();
      // unit is (100*dBm)
      // TODO!
      msg.txP =-1;

      //msg.timestamp = ros::Time::now();

      // cast db to dbm
      msg.rssi  = wn.signal_level()+30;
      // TODO!
      msg.phase  = -1;

      // cast MHz to KHz
      msg.frequency = (wn.frequency()*1000.0);
      ros_rfid_pub.publish(msg);

  }
}

/////////////////////////////////////////////////
int main(int _argc, char** _argv)
{

  ros::init(_argc, _argv, "wirelessNodeMonitor");
  ros::NodeHandle nh;

  // read configuration
  ros::param::param<std::string>("~ros_rfid_topic_name",
                                 ros_rfid_topic_name, "/lastTag");
  ros::param::param<std::string>("~ros_rfid_frame_id",
                                    ros_rfid_frame_id, "lastTag");

  ros::param::param<std::string>("~gazebo_wireless_node_topic_name",
                                 gazebo_wireless_node_topic_name, "/gazebo/default/thorvald_001/base_link/_head_reader_sensor/transceiver");


  ros_rfid_pub = nh.advertise<rfid_node::TagReading>(ros_rfid_topic_name, 5);

  seq=0;
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Check if the topic to subscribe is present within the list, and
  // create the subscriber only later
  bool topic_found = false;
  bool one_error = false;
  while (topic_found == false)
  {
    std::list<std::string> topics_list =
        gazebo::transport::getAdvertisedTopics("");
    for (std::list<std::string>::iterator topic = topics_list.begin();
         topic != topics_list.end(); topic++)
    {
      if ((*topic).compare(gazebo_wireless_node_topic_name) == 0)
      {
        topic_found = true;
        break;
      }
    }
    ROS_ERROR("[%s] Topic [%s] not found. Waiting 1 sec. for it to be available",ros::this_node::getName().c_str(), gazebo_wireless_node_topic_name.c_str());
    one_error= true;
    ros::Duration(1).sleep();
  }

  gazebo::transport::SubscriberPtr sub =
      node->Subscribe(gazebo_wireless_node_topic_name, rfid_callback);

  if (one_error){
        ROS_ERROR("[%s] Topic [%s] was finally found",ros::this_node::getName().c_str(), gazebo_wireless_node_topic_name.c_str());
  }

  ros::spin();

  // Make sure to shut everything down.
  gazebo::transport::fini();
}
