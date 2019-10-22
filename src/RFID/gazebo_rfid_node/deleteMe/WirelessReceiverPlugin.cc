#include "WirelessReceiverPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(WirelessReceiverPlugin)

/////////////////////////////////////////////////
WirelessReceiverPlugin::WirelessReceiverPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
WirelessReceiverPlugin::~WirelessReceiverPlugin()
{
}

/////////////////////////////////////////////////
void WirelessReceiverPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr &_sdf)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::WirelessReceiverSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "WirelessReceiverPlugin requires a WirelessReceiverSensor.\n";
    return;
  }

  // Get the world name.
  std::string worldName = _sdf->GetWorldName();
  this->world = physics::get_world(worldName);

  this->robot_namespace_ =  GetRobotNamespace(_sensor, _sdf, "Laser");

  //read ros params
  last_tag_pub_name
  last_tag_pub

  // Create ROS PUBLISHER
  last_tag_pub = nodeHandle_.advertise<rfid_node::TagReading>(last_tag_pub_name, 1000);


  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&WirelessReceiverPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void WirelessReceiverPlugin::OnUpdate()
{

    std::string txEssid;
    double rxPower;
    double txFreq;
    rfid_node::TagReading msg;

    // this is like repeating the Receiver callback to publish it into ROS.
    // Why do this? for future change of propagation model from the one embedded
    // in Gazebo.

    ignition::math::Pose3d myPos = this->parentSensor->referencePose;
    Sensor_V sensors = SensorManager::Instance()->GetSensors();
    for (Sensor_V::iterator it = sensors.begin(); it != sensors.end(); ++it)
    {
      if ((*it)->Type() == "wireless_transmitter")
      {
        std::shared_ptr<gazebo::sensors::WirelessTransmitter> transmitter =
            std::static_pointer_cast<WirelessTransmitter>(*it);

        txFreq = transmitter->Freq();
        rxPower = transmitter->SignalStrength(myPos, this->parentSensor->Gain());

        // Discard if the frequency received is out of our frequency range,
        // or if the received signal strengh is lower than the sensivity
        if ((txFreq < this->parentSensor->MinFreqFiltered()) ||
            (txFreq > this->parentSensor->MaxFreqFiltered()) ||
            (rxPower < this->parentSensor->Sensitivity()))
        {
          continue;
        }

        // phase
        double distance = std::max(1.0, myPos.Pos().Distance(transmitter->Pose().Pos()));
        double phase = PHASE_CONSTANT_DEG * txFreq * 1000.0 * distance;
        phase = fmod(phase, 180);

        // send data
        msg.ID = transmitter->ESSID();
        // unit is (100*dBm)
        msg.txP = (transmitter->Power()+30)*100;

        this->simTime  = this->world->GetSimTime();
        msg.timestamp = ros::Time(this->simTime.sec(), this->simTime.nsec());

        // cast db to dbm
        msg.rssi  = rxPower+30;
        msg.phase  = phase;

        // cast MHz to KHz
        msg.frequency = (txFreq*1000.0);
        last_tag_pub.publish(msg);



      }
    }


}
