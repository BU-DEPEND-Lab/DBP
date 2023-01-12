/* Include the controller definition */
#include "footbot_flocking_WMSR.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <algorithm>
#include <numeric>

/****************************************/
/****************************************/

void CFootBotFlocking_WMSR::SWheelTurningParams::Init(TConfigurationNode &t_node)
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

void CFootBotFlocking_WMSR::SFlockingInteractionParams::Init(TConfigurationNode &t_node)
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

void CFootBotFlocking_WMSR::SResilientFlockingParams::Init(TConfigurationNode &t_node)
{
   try
   {
      GetNodeAttribute(t_node, "F", F);
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller resilient flocking parameters.", ex);
   }
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential
 */
Real CFootBotFlocking_WMSR::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance)
{
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return std::min(0., -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp));
}

/****************************************/
/****************************************/

CFootBotFlocking_WMSR::CFootBotFlocking_WMSR() : m_pcWheels(NULL),
                                                 m_pcLight(NULL),
                                                 m_pcLEDs(NULL),
                                                 m_pcCamera(NULL),
                                                 m_pcPosition(NULL) {}

/****************************************/
/****************************************/

void CFootBotFlocking_WMSR::Init(TConfigurationNode &t_node)
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
      m_sResilientFlockingParams.Init(GetNode(t_node, "resilient_flocking"));
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

void CFootBotFlocking_WMSR::ControlStep()
{
   SetBroadcastFromLinearUpdate();
   SetWheelSpeedsFromVector(VectorToLight() + 2 * FlockingVector());
}

/****************************************/
/****************************************/

void CFootBotFlocking_WMSR::Reset()
{
   /* Enable camera filtering */
   m_pcCamera->Enable();
   /* Set beacon color to all red to be visible for other robots */
   m_pcLEDs->SetSingleColor(12, CColor::RED);
   x = std::nullopt;
   obs = std::nullopt;
}

/****************************************/
/****************************************/

CVector2 CFootBotFlocking_WMSR::VectorToLight()
{
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings &sReadings = m_pcCamera->GetReadings();
   CVector2 target;
   obs = std::nullopt;
   for (auto blob : sReadings.BlobList)
   {
      // direct observation, don't use stored x
      if (blob->Color == CColor::YELLOW)
      {
         // target = CVector2(blob->Distance, blob->Angle);
         CVector2 p_self;
         m_pcPosition->GetReading().Position.ProjectOntoXY(p_self);
         auto orientation_self = m_pcPosition->GetReading().Orientation;
         CRadians cZAngle, cYAngle, cXAngle;
         orientation_self.ToEulerAngles(cZAngle, cYAngle, cXAngle);
         auto theta_self = cZAngle;
         auto p_target = CVector2(blob->Distance / 100.0, 0.0);
         p_target.Rotate(theta_self).Rotate(blob->Angle);
         p_target += p_self;
         obs.emplace(p_target);
         // target.Normalize();
         // target *= m_sWheelTurningParams.MaxSpeed;
         // return target;
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

CVector2 CFootBotFlocking_WMSR::FlockingVector()
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

void CFootBotFlocking_WMSR::SetWheelSpeedsFromVector(const CVector2 &c_heading)
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

void CFootBotFlocking_WMSR::SetBroadcastFromLinearUpdate()
{
   Real w = 0;
   std::vector<Real> received_0;
   std::vector<Real> received_1;
   if (obs.has_value())
   {
      w += 1;
      received_0.push_back(obs.value().GetX());
      received_1.push_back(obs.value().GetY());
      CByteArray cMessage;
      cMessage << GetId();
      vec_serialize(&cMessage, obs.value());
      m_pcRadioTX->GetInterfaces()[0].Messages.push_back(cMessage);
   }
   for (auto msg : m_pcRadioRX->GetInterfaces()[0].Messages)
   {
      w += 1;
      std::string sender_id;
      CVector2 y;
      msg >> sender_id;
      vec_deserialize(&msg, y);
      received_0.push_back(y.GetX());
      received_1.push_back(y.GetY());
   }
   std::sort(received_0.begin(), received_0.end());
   std::sort(received_1.begin(), received_1.end());
   if (!obs.has_value() && (w >= 2 * m_sResilientFlockingParams.F + 1))
   {
      CVector2 accum = CVector2(std::accumulate(received_0.begin() + m_sResilientFlockingParams.F, received_0.end() - m_sResilientFlockingParams.F, 0.),
                                std::accumulate(received_1.begin() + m_sResilientFlockingParams.F, received_1.end() - m_sResilientFlockingParams.F, 0.));
      x.emplace(accum / (w - 2 * m_sResilientFlockingParams.F));
   }
   else if (obs.has_value())
   {
      int F = m_sResilientFlockingParams.F;
      while (F > 0 && received_0.at(0) != obs.value().GetX())
      {
         received_0.erase(received_0.begin());
         F--;
      }
      F = m_sResilientFlockingParams.F;
      while (F > 0 && received_0.at(received_0.size() - 1) != obs.value().GetX())
      {
         received_0.erase(received_0.end() - 1);
         F--;
      }
      F = m_sResilientFlockingParams.F;
      while (F > 0 && received_1.at(0) != obs.value().GetY())
      {
         received_1.erase(received_1.begin());
         F--;
      }
      F = m_sResilientFlockingParams.F;
      while (F > 0 && received_1.at(received_1.size() - 1) != obs.value().GetY())
      {
         received_1.erase(received_1.end() - 1);
         F--;
      }
      x.emplace(CVector2(
          std::accumulate(received_0.begin(), received_0.end(), 0.) / received_0.size(),
          std::accumulate(received_1.begin(), received_1.end(), 0.) / received_1.size()));
   }
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
REGISTER_CONTROLLER(CFootBotFlocking_WMSR, "footbot_flocking_WMSR_controller")
