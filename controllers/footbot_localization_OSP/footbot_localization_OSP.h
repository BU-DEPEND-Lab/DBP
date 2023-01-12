/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example flocking controller for the foot-bot.
 *
 * This controller lets a group of foot-bots flock in an hexagonal lattice towards
 * a light source placed in the arena. To flock, it exploits a generalization of the
 * well known Lennard-Jones potential. The parameters of the Lennard-Jones function
 * were chosen through a simple trial-and-error procedure on its graph.
 *
 * This controller is meant to be used with the XML file:
 *    experiments/flocking.argos
 */

#ifndef FOOTBOT_LOCALIZATION_OSP
#define FOOTBOT_LOCALIZATION_OSP

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the omnidirectional camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot-networked/control_interface/ci_footbot_light_sensor.h>
/* Definition of the simple radios actuator and sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_simple_radios_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_simple_radios_sensor.h>
/* positioning */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>
#include <optional>

#include "message_manager.h"
#include "../utility/OSP_messages.h"

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotLocalization_OSP : public CCI_Controller
{

public:
   /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_flocking_controller><parameters><wheel_turning>
    * section.
    */
   struct SWheelTurningParams
   {
      /*
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /*
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;

      void Init(TConfigurationNode &t_tree);
   };

   /*
    * The following variables are used as parameters for
    * flocking interaction. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_flocking_controller><parameters><flocking>
    * section.
    */
   struct SFlockingInteractionParams
   {
      /* Target robot-robot distance in cm */
      Real TargetDistance;
      /* Gain of the Lennard-Jones potential */
      Real Gain;
      /* Exponent of the Lennard-Jones potential */
      Real Exponent;

      void Init(TConfigurationNode &t_node);
      Real GeneralizedLennardJones(Real f_distance);
   };

   struct SObservationSharingProtocolParams
   {
      std::size_t max_hop_count;
      Real max_displacement;
      bool is_byzantine;
      std::optional<Real> byzantine_offset;
      bool is_observer;
      bool mitigation;
      std::optional<UInt32> observation_period;
      std::optional<UInt32> observation_offset;
      void Init(TConfigurationNode &t_node);
      bool is_regular();
   };

public:
   /* Class constructor. */
   CFootBotLocalization_OSP();

   /* Class destructor. */
   virtual ~CFootBotLocalization_OSP() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML file
    * in the <controllers><footbot_flocking_controller> section.
    */
   virtual void Init(TConfigurationNode &t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Destroy() {}

protected:
   virtual void SetNewTarget();
   virtual std::optional<Real> DistanceToTarget();
   virtual std::optional<CVector2> VectorToTarget();
   /*
    * Calculates the flocking interaction vector.
    */
   virtual CVector2 FlockingVector();

   /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    */
   void SetWheelSpeedsFromVector(const CVector2 &c_heading);

   void ObservationSharingProtocol();

public:
   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator *m_pcWheels;
   /* Pointer to the foot-bot light sensor */
   CCI_FootBotLightSensor *m_pcLight;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator *m_pcLEDs;
   /* Pointer to the omnidirectional camera sensor */
   CCI_ColoredBlobOmnidirectionalCameraSensor *m_pcCamera;

   CCI_SimpleRadiosActuator *m_pcRadioTX;
   CCI_SimpleRadiosSensor *m_pcRadioRX;

   CCI_PositioningSensor *m_pcPosition;

   /* The turning parameters. */
   SWheelTurningParams m_sWheelTurningParams;
   /* The flocking interaction parameters. */
   SFlockingInteractionParams m_sFlockingParams;

   SObservationSharingProtocolParams m_sObservationSharingProtocolParams;
   Real local_time;
   std::optional<Location> location;
   LocalizationObservationManager m_ObservationManager;
   LocalizationAccusationManager m_AccusationManager;
   std::unordered_map<MessageKey, OSP_Message> received_messages;
   std::unordered_map<MessageKey, OSP_Message> outbound_messages;

   Real GetRadioRange();
   Real GetObservationRange();
   std::optional<CVector2> GetPosition();
   CVector2 target;
   const Real TARGET_DISTANCE_THRESHOLD;
};

#endif
