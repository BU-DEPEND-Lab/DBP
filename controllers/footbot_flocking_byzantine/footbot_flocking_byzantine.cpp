/* Include the controller definition */
#include "footbot_flocking_byzantine.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <algorithm>

/****************************************/
/****************************************/

void CFootBotFlockingByzantine::SWheelTurningParams::Init(TConfigurationNode &t_node)
{
   try
   {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CFootBotFlockingByzantine::SFlockingInteractionParams::Init(TConfigurationNode &t_node)
{
   try
   {
      GetNodeAttribute(t_node, "target_distance", TargetDistance);
      GetNodeAttribute(t_node, "gain", Gain);
      GetNodeAttribute(t_node, "exponent", Exponent);
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking parameters.", ex);
   }
}

void CFootBotFlockingByzantine::SByzantineParams::Init(TConfigurationNode &t_node)
{
   try
   {
      Real x, y;
      GetNodeAttribute(t_node, "x", x);
      GetNodeAttribute(t_node, "y", y);
      target = CVector2(x, y);
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing byzantine parameters.", ex);
   }
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential
 */
Real CFootBotFlockingByzantine::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance)
{
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return std::min(0., -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp));
}

/****************************************/
/****************************************/

CFootBotFlockingByzantine::CFootBotFlockingByzantine() : m_pcWheels(NULL),
                                                         m_pcLight(NULL),
                                                         m_pcLEDs(NULL),
                                                         m_pcCamera(NULL),
                                                         m_pcPosition(NULL) {}

/****************************************/
/****************************************/

void CFootBotFlockingByzantine::Init(TConfigurationNode &t_node)
{
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the XML tag of the
    * device whose handle we want to have. For a list of allowed values, type at the
    * command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors internally, on the basis of
    *       the lists provided the configuration file at the
    *       <controllers><footbot_diffusion><actuators> and
    *       <controllers><footbot_diffusion><sensors> sections. If you forgot to
    *       list a device in the XML and then you request it here, an error occurs.
    */
   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcLight = GetSensor<CCI_FootBotLightSensor>("footbot_light");
   m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
   m_pcCamera = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
   m_pcRadioRX = GetSensor<CCI_SimpleRadiosSensor>("simple_radios");
   m_pcRadioTX = GetActuator<CCI_SimpleRadiosActuator>("simple_radios");
   m_pcPosition = GetSensor<CCI_PositioningSensor>("positioning");
   /*
    * Parse the config file
    */
   try
   {
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      /* Flocking-related */
      m_sFlockingParams.Init(GetNode(t_node, "flocking"));
      m_sByzantineParams.Init(GetNode(t_node, "byzantine_target"));
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
   }

   /*
    * Other init stuff
    */
   Reset();
}

/****************************************/
/****************************************/

void CFootBotFlockingByzantine::ControlStep()
{
   SetBroadcastFromLinearUpdate();
   SetWheelSpeedsFromVector(FlockingVector());
}

/****************************************/
/****************************************/

void CFootBotFlockingByzantine::Reset()
{
   /* Enable camera filtering */
   m_pcCamera->Enable();
   /* Set beacon color to all red to be visible for other robots */
   // m_pcLEDs->SetSingleColor(12, CColor::RED);
   m_pcLEDs->SetAllColors(CColor::RED);
   x = m_sByzantineParams.target;
}

/****************************************/
/****************************************/

CVector2 CFootBotFlockingByzantine::VectorToLight()
{
   // /* Get light readings */
   // const CCI_FootBotLightSensor::TReadings &tReadings = m_pcLight->GetReadings();
   // /* Calculate a normalized vector that points to the closest light */
   // CVector2 cAccum;
   // for (size_t i = 0; i < tReadings.size(); ++i)
   // {
   //    cAccum += CVector2(tReadings[i].Value, tReadings[i].Angle);
   // }
   // if (cAccum.Length() > 0.0f)
   // {
   //    /* Make the vector long as 1/4 of the max speed */
   //    cAccum.Normalize();
   //    cAccum *= 0.25f * m_sWheelTurningParams.MaxSpeed;
   // }
   // return cAccum;
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings &sReadings = m_pcCamera->GetReadings();
   CVector2 target;
   for (auto blob : sReadings.BlobList)
   {
      // direct observation, don't use stored x
      if (blob->Color == CColor::YELLOW)
      {
         target = CVector2(blob->Distance, blob->Angle);
         CVector2 p_self;
         m_pcPosition->GetReading().Position.ProjectOntoXY(p_self);
         auto orientation_self = m_pcPosition->GetReading().Orientation;
         CRadians cZAngle, cYAngle, cXAngle;
         orientation_self.ToEulerAngles(cZAngle, cYAngle, cXAngle);
         auto theta_self = cZAngle;
         auto p_target = CVector2(blob->Distance / 100.0, 0.0);
         p_target.Rotate(theta_self).Rotate(blob->Angle);
         p_target += p_self;
         x.emplace(p_target);
         target.Normalize();
         target *= m_sWheelTurningParams.MaxSpeed;
         return target;
      }
   }
   if (x.has_value())
   {
      // use consensus value if no direct observation is available
      CVector2 p_self;
      m_pcPosition->GetReading().Position.ProjectOntoXY(p_self);
      auto orientation_self = m_pcPosition->GetReading().Orientation;
      CRadians cZAngle, cYAngle, cXAngle;
      orientation_self.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      auto theta_self = cZAngle;
      auto delta = (x.value() - p_self).Length();
      auto phi = (p_self - x.value()).Angle() - theta_self;
      target = CVector2(-delta, phi);
      target.Normalize();
      target *= m_sWheelTurningParams.MaxSpeed;
   }
   return target;
}

/****************************************/
/****************************************/

CVector2 CFootBotFlockingByzantine::FlockingVector()
{
   /* Get the camera readings */
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings &sReadings = m_pcCamera->GetReadings();
   /* Go through the camera readings to calculate the flocking interaction vector */
   if (!sReadings.BlobList.empty())
   {
      CVector2 cAccum;
      Real fLJ;
      size_t unBlobsSeen = 0;

      for (size_t i = 0; i < sReadings.BlobList.size(); ++i)
      {

         /*
          * The camera perceives the light as a yellow blob
          * The robots have their red beacon on
          * So, consider only red blobs
          * In addition: consider only the closest neighbors, to avoid
          * attraction to the farthest ones. Taking 180% of the target
          * distance is a good rule of thumb.
          */
         if ((sReadings.BlobList[i]->Color == CColor::RED || sReadings.BlobList[i]->Color == CColor::YELLOW) &&
             sReadings.BlobList[i]->Distance < m_sFlockingParams.TargetDistance * 1.80f)
         {
            /*
             * Take the blob distance and angle
             * With the distance, calculate the Lennard-Jones interaction force
             * Form a 2D vector with the interaction force and the angle
             * Sum such vector to the accumulator
             */
            /* Calculate LJ */
            fLJ = m_sFlockingParams.GeneralizedLennardJones(sReadings.BlobList[i]->Distance);
            /* Sum to accumulator */
            cAccum += CVector2(fLJ,
                               sReadings.BlobList[i]->Angle);
            /* Increment the blobs seen counter */
            ++unBlobsSeen;
         }
      }
      if (unBlobsSeen > 0)
      {
         /* Divide the accumulator by the number of blobs seen */
         cAccum /= unBlobsSeen;
         /* Clamp the length of the vector to the max speed */
         if (cAccum.Length() > m_sWheelTurningParams.MaxSpeed)
         {
            cAccum.Normalize();
            cAccum *= m_sWheelTurningParams.MaxSpeed;
         }
         return cAccum;
      }
      else
         return CVector2();
   }
   else
   {
      return CVector2();
   }
}

/****************************************/
/****************************************/

void CFootBotFlockingByzantine::SetWheelSpeedsFromVector(const CVector2 &c_heading)
{
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if (m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN)
   {
      if (Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold)
      {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if (m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN)
   {
      if (Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold)
      {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if (Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold)
      {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if (m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN)
   {
      if (Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold)
      {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if (Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold)
      {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch (m_sWheelTurningParams.TurningMechanism)
   {
   case SWheelTurningParams::NO_TURN:
   {
      /* Just go straight */
      fSpeed1 = fBaseAngularWheelSpeed;
      fSpeed2 = fBaseAngularWheelSpeed;
      break;
   }
   case SWheelTurningParams::SOFT_TURN:
   {
      /* Both wheels go straight, but one is faster than the other */
      Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
      fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
      fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
      break;
   }
   case SWheelTurningParams::HARD_TURN:
   {
      /* Opposite wheel speeds */
      fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
      fSpeed2 = m_sWheelTurningParams.MaxSpeed;
      break;
   }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if (cHeadingAngle > CRadians::ZERO)
   {
      /* Turn Left */
      fLeftWheelSpeed = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else
   {
      /* Turn Right */
      fLeftWheelSpeed = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

void CFootBotFlockingByzantine::SetBroadcastFromLinearUpdate()
{
   CVector2 p_self;
   m_pcPosition->GetReading().Position.ProjectOntoXY(p_self);
   // Byzantine robots are broadcasting a fake target that is away from the origin
   CVector2 byz_target[] = {CVector2(1.0, 1.0), CVector2(1.0, -1.0), CVector2(-1.0, 1.0), CVector2(-1.0, -1.0)};
   x.emplace(p_self + 0.25 * byz_target[((p_self.GetX() < 0) << 1) + (p_self.GetY() < 0)]);
   CByteArray cMessage;
   cMessage << GetId();
   vec_serialize(&cMessage, x.value());
   m_pcRadioTX->GetInterfaces()[0].Messages.push_back(cMessage);
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotFlockingByzantine, "footbot_flocking_byzantine_controller")
