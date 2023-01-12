

#ifndef FOOTBOT_TARGET_H
#define FOOTBOT_TARGET_H

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
/* positioning */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotTarget : public CCI_Controller
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

public:
   /* Class constructor. */
   CFootBotTarget();

   /* Class destructor. */
   virtual ~CFootBotTarget() {}

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

   virtual CVector2 GetPosition();

protected:
   virtual void SetNewTarget();
   virtual Real DistanceToTarget();
   virtual CVector2 VectorToTarget();
   /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    */
   void SetWheelSpeedsFromVector(const CVector2 &c_heading);

private:
   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator *m_pcWheels;
   /* Pointer to the foot-bot light sensor */
   CCI_FootBotLightSensor *m_pcLight;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator *m_pcLEDs;
   /* Pointer to the omnidirectional camera sensor */
   CCI_ColoredBlobOmnidirectionalCameraSensor *m_pcCamera;

   CCI_PositioningSensor *m_pcPosition;

   /* The turning parameters. */
   SWheelTurningParams m_sWheelTurningParams;

   CVector2 target;
   const Real TARGET_DISTANCE_THRESHOLD;
};

#endif
