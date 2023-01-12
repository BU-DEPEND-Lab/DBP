/* Include the controller definition */
#include "footbot_time_sync_OSP.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <algorithm>
#include <numeric>

/****************************************/
/****************************************/

void CFootBotTimeSync_OSP::SWheelTurningParams::Init(TConfigurationNode &t_node)
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

void CFootBotTimeSync_OSP::SFlockingInteractionParams::Init(TConfigurationNode &t_node)
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

void CFootBotTimeSync_OSP::SObservationSharingProtocolParams::Init(TConfigurationNode &t_node)
{
   try
   {
      UInt32 byz_offset, obs_p, obs_offset;
      GetNodeAttribute(t_node, "max_hop_count", max_hop_count);
      GetNodeAttribute(t_node, "outbound_obs_limit", outbound_obs_limit);
      GetNodeAttribute(t_node, "byzantine", is_byzantine);
      if (is_byzantine)
      {
         GetNodeAttribute(t_node, "byzantine_offset", byz_offset);
         byzantine_offset = byz_offset;
      }
      GetNodeAttribute(t_node, "observer", is_observer);
      if (is_observer || is_byzantine)
      {
         GetNodeAttribute(t_node, "observation_period", obs_p);
         observation_period = obs_p;
         observation_offset = obs_p * drand48();
      }
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller OSP parameters.", ex);
   }
}

bool CFootBotTimeSync_OSP::SObservationSharingProtocolParams::is_regular()
{
   return !is_byzantine && !is_observer;
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential
 */
Real CFootBotTimeSync_OSP::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance)
{
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return std::min(0., -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp));
}

/****************************************/
/****************************************/

CFootBotTimeSync_OSP::CFootBotTimeSync_OSP() : m_pcWheels(NULL),
                                               m_pcLight(NULL),
                                               m_pcLEDs(NULL),
                                               m_pcCamera(NULL),
                                               m_pcPosition(NULL),
                                               TARGET_DISTANCE_THRESHOLD(0.1) {}

/****************************************/
/****************************************/

void CFootBotTimeSync_OSP::Init(TConfigurationNode &t_node)
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
      m_sObservationSharingProtocolParams.Init(GetNode(t_node, "observation_sharing"));
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

void CFootBotTimeSync_OSP::ControlStep()
{
   m_ObservationManager.prune(local_time > 50 ? local_time - 50 : 0, received_messages);
   if (DistanceToTarget() < TARGET_DISTANCE_THRESHOLD)
      SetNewTarget();
   SetWheelSpeedsFromVector(VectorToTarget() + 2 * FlockingVector());
   if (m_sObservationSharingProtocolParams.is_observer && ((UInt32)local_time % m_sObservationSharingProtocolParams.observation_period.value() == m_sObservationSharingProtocolParams.observation_offset.value()))
   {
      m_ObservationManager.add_observation(MessageKey(get_index<TimeSyncObservationMessage, MessageVariant>{}(), local_time, GetId()), TimeSyncObservationValue(local_time, 0));
      auto o_msg = OSP_Message(TimeSyncObservationMessage(local_time), GetId(), local_time, 0);
      outbound_messages.insert({MessageKey(o_msg), o_msg});
   }
   if (!m_sObservationSharingProtocolParams.is_byzantine)
      ObservationSharingProtocol();
   else
   {
      if ((UInt32)local_time % m_sObservationSharingProtocolParams.observation_period.value() == m_sObservationSharingProtocolParams.observation_offset.value())
      {
         auto o_msg = OSP_Message(TimeSyncObservationMessage(local_time + m_sObservationSharingProtocolParams.byzantine_offset.value()), GetId(), local_time + m_sObservationSharingProtocolParams.byzantine_offset.value(), 0);
         CByteArray cMessage;
         o_msg.serialize(&cMessage);
         m_pcRadioTX->GetInterfaces()[0].Messages.push_back(cMessage);
      }
   }
   if (m_sObservationSharingProtocolParams.is_regular())
      local_time += 1.0 + skew + (0.1 * drand48() - 0.05); // random drift
   else
      local_time += 1.0;
}

/****************************************/
/****************************************/

void CFootBotTimeSync_OSP::Reset()
{
   /* Enable camera filtering */
   m_pcCamera->Enable();
   /* Set beacon color to all red to be visible for other robots */
   m_pcLEDs->SetSingleColor(12, CColor::RED);
   if (m_sObservationSharingProtocolParams.is_byzantine)
      m_pcLEDs->SetAllColors(CColor::RED);
   local_time = 0;
   if (m_sObservationSharingProtocolParams.is_regular())
   {
      local_time = 20 * drand48() - 10;
      skew = 0.02 * drand48() - 0.01;
   }
   m_ObservationManager.clear();
   m_AccusationManager.clear();
   received_messages.clear();
   outbound_messages.clear();
   SetNewTarget();
}
void CFootBotTimeSync_OSP::SetNewTarget()
{
   target = 15. * CVector2(drand48(), drand48()) - 5. * CVector2(1., 1.);
}
Real CFootBotTimeSync_OSP::DistanceToTarget()
{
   CVector2 p_self;
   m_pcPosition->GetReading().Position.ProjectOntoXY(p_self);
   return (target - p_self).Length();
}
CVector2 CFootBotTimeSync_OSP::GetPosition()
{
   CVector2 p_self;
   m_pcPosition->GetReading().Position.ProjectOntoXY(p_self);
   return p_self;
}
CVector2 CFootBotTimeSync_OSP::VectorToTarget()
{
   CVector2 p_self;
   m_pcPosition->GetReading().Position.ProjectOntoXY(p_self);
   auto orientation_self = m_pcPosition->GetReading().Orientation;
   CRadians cZAngle, cYAngle, cXAngle;
   orientation_self.ToEulerAngles(cZAngle, cYAngle, cXAngle);
   auto theta_self = cZAngle;
   auto delta = (target - p_self).Length();
   auto phi = (p_self - target).Angle() - theta_self;
   auto res = CVector2(-delta, phi);
   res.Normalize();
   res *= m_sWheelTurningParams.MaxSpeed;
   return res;
}

/****************************************/
/****************************************/

CVector2 CFootBotTimeSync_OSP::FlockingVector()
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

void CFootBotTimeSync_OSP::SetWheelSpeedsFromVector(const CVector2 &c_heading)
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

void CFootBotTimeSync_OSP::ObservationSharingProtocol()
{
   OSP_Message OSP_msg;
   for (CByteArray c_msg : m_pcRadioRX->GetInterfaces()[0].Messages)
   {
      OSP_msg.deserialize(&c_msg);
      MessageKey key = MessageKey(OSP_msg);
      if (m_AccusationManager.is_accused(key.GetOrigin()) && !std::holds_alternative<AccusationMessage>(OSP_msg.m_msg_variant)) // must forward accusations
         continue;
      if (auto found = received_messages.find(key); found != received_messages.end())
      {
         if (found->second.m_hop_count < m_sObservationSharingProtocolParams.max_hop_count &&
             outbound_messages.count(key) == 0)
         {
            (found->second)++;     // internal repeat counter
            OSP_msg.m_hop_count++; // external hop count
            outbound_messages.insert({key, OSP_msg});
         }
         else
         {
            continue;
         }
      }
      else // first time message has been received
      {
         if (std::holds_alternative<TimeSyncObservationMessage>(OSP_msg.m_msg_variant))
         {
            auto o_msg = std::get<TimeSyncObservationMessage>(OSP_msg.m_msg_variant);
            auto value = TimeSyncObservationValue(o_msg, 0);
            if (m_ObservationManager.is_suspicious(OSP_msg, local_time, m_sObservationSharingProtocolParams.is_observer))
            {
               auto new_accusation = OSP_Message(AccusationMessage(OSP_msg.m_origin), GetId(), local_time, 0);
               outbound_messages.insert({MessageKey(new_accusation), new_accusation});
               m_AccusationManager.add_accused(Accusation(GetId(), std::get<AccusationMessage>(new_accusation.m_msg_variant)),
                                               m_ObservationManager);
               continue;
            }
            else
            {
               m_ObservationManager.add_observation(key, value);
               if (m_sObservationSharingProtocolParams.is_regular())
               {
                  auto update = m_ObservationManager.guess();
                  if (update.has_value())
                     local_time = update.value();
               }
            }
         }
         else if (std::holds_alternative<AccusationMessage>(OSP_msg.m_msg_variant))
         {
            auto accusation = Accusation(OSP_msg.m_origin, std::get<AccusationMessage>(OSP_msg.m_msg_variant));
            m_AccusationManager.add_accused(accusation, m_ObservationManager);
         }
         OSP_msg.m_hop_count++;
         outbound_messages.insert({key, OSP_msg});
         OSP_msg.m_hop_count = 0;
         received_messages.insert({key, OSP_msg});
      }
   }

   std::size_t limit_outbound_obs = m_sObservationSharingProtocolParams.outbound_obs_limit;
   for (auto outbound : outbound_messages)
   {
      if (std::holds_alternative<TimeSyncObservationMessage>(outbound.second.m_msg_variant))
      {
         if (limit_outbound_obs == 0)
            continue;
         limit_outbound_obs--;
      }
      CByteArray c_msg;
      outbound.second.serialize(&c_msg);
      m_pcRadioTX->GetInterfaces()[0].Messages.push_back(c_msg);
   }
   outbound_messages.clear();
}

Real CFootBotTimeSync_OSP::GetRadioRange()
{
   // todo for whatever reason, the range isn't a property of the sensor ??
   return 4;
}
Real CFootBotTimeSync_OSP::GetObservationRange()
{
   // todo same as GetRadioRange()
   CRadians aperture;
   aperture.FromValueInDegrees(72.);
   return 0.288699733f * Tan(aperture);
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
REGISTER_CONTROLLER(CFootBotTimeSync_OSP, "footbot_time_sync_OSP_controller")
