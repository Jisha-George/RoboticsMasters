#ifndef _GAZEBO_WIRELESSRECEIVER_PLUGIN_HH_
#define _GAZEBO_WIRELESSRECEIVER_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

#include "gazebo/common/Time.hh"
#include <boost/bind.hpp>

#include "gazebo/physics/physics.hh"

namespace gazebo
{
  // 4*pi/c
  const double PHASE_CONSTANT_RAD = 4.192e-8;
  // 720/c
  const double PHASE_CONSTANT_DEG = 2.401e-6;

  /// \brief An example plugin for a WIRELESSRECEIVER sensor.
  class WirelessReceiverPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: WirelessReceiverPlugin();

    /// \brief Destructor.
    public: virtual ~WirelessReceiverPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the wireless receiver sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Pointer to the wireless receiver sensor
    private: sensors::WirelessReceiverSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the wireless receiver sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;


    private: physics::WorldPtr world;

    private: common::Time simTime;



    last_tag_pub_name
    last_tag_pub
  };
}
#endif
