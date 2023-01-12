/* Include the controller definition */
#include "footbot_localization_OSP.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <algorithm>
#include <numeric>

CVector2 center(std::pair<CVector2, CVector2> loc)
{
   return (loc.first + loc.second) / 2;
}
/****************************************/
/****************************************/

void CFootBotLocalization_OSP::SWheelTurningParams::Init(TConfigurationNode &t_node)
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

void CFootBotLocalization_OSP::SFlockingInteractionParams::Init(TConfigurationNode &t_node)
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

void CFootBotLocalization_OSP::SObservationSharingProtocolParams::Init(TConfigurationNode &t_node)
{
   try
   {
      UInt32 obs_p, obs_offset;
      Real byz_offset;
      GetNodeAttribute(t_node, "max_hop_count", max_hop_count);
      GetNodeAttribute(t_node, "max_displacement", max_displacement);
      GetNodeAttribute(t_node, "byzantine", is_byzantine);
      if (is_byzantine)
      {
         GetNodeAttribute(t_node, "byzantine_offset", byz_offset);
         byzantine_offset = byz_offset;
      }
      GetNodeAttribute(t_node, "observer", is_observer);
      if (is_observer)
      {
         GetNodeAttribute(t_node, "observation_period", obs_p);
         observation_period = obs_p;
         observation_offset = obs_p * drand48();
      }
      if (is_observer || is_regular())
      {
         GetNodeAttribute(t_node, "mitigation", mitigation);
      }
   }
   catch (CARGoSException &ex)
   {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller OSP parameters.", ex);
   }
}

bool CFootBotLocalization_OSP::SObservationSharingProtocolParams::is_regular()
{
   return !is_byzantine && !is_observer;
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential
 */
Real CFootBotLocalization_OSP::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance)
{
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return std::min(0., -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp));
}

/****************************************/
/****************************************/

CFootBotLocalization_OSP::CFootBotLocalization_OSP() : m_pcWheels(NULL),
                                                       m_pcLight(NULL),
                                                       m_pcLEDs(NULL),
                                                       m_pcCamera(NULL),
                                                       m_pcPosition(NULL),
                                                       TARGET_DISTANCE_THRESHOLD(1.0) {}

/****************************************/
/****************************************/

void CFootBotLocalization_OSP::Init(TConfigurationNode &t_node)
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

void CFootBotLocalization_OSP::ControlStep()
{
   m_ObservationManager.prune(local_time > 200 ? local_time - 200 : 0, received_messages);
   auto dist = DistanceToTarget();
   if (dist.has_value() && (dist.value() < TARGET_DISTANCE_THRESHOLD))
   {
      SetNewTarget();
      if (auto found = GetId().find("fb"); found != std::string::npos)
         LOG << GetId() << " found target!" << std::endl;
   }
   if (dist.has_value()) // have a localization guess
      SetWheelSpeedsFromVector(VectorToTarget().value() + 2 * FlockingVector());
   else
      SetWheelSpeedsFromVector(FlockingVector());
   if (m_sObservationSharingProtocolParams.is_observer && ((UInt32)local_time % m_sObservationSharingProtocolParams.observation_period.value() == m_sObservationSharingProtocolParams.observation_offset.value()))
   {
      auto o_msg = OSP_Message(LocalizationObservationMessage(GetPosition().value(), GetPosition().value(), true), GetId(), local_time, 0);
      outbound_messages.insert({MessageKey(o_msg), o_msg});
      m_ObservationManager.add_observation(
          MessageKey(get_index<LocalizationObservationMessage, MessageVariant>{}(), local_time, GetId()),
          LocalizationObservationValue(std::get<LocalizationObservationMessage>(o_msg.m_msg_variant), 0));
   }
   if (!m_sObservationSharingProtocolParams.is_byzantine)
      ObservationSharingProtocol();
   else
   {
      // TODO Byzantine control step!
      // auto o_msg = OSP_Message(LocalizationObservationMessage(local_time + m_sObservationSharingProtocolParams.byzantine_offset.value()), GetId(), local_time + m_sObservationSharingProtocolParams.byzantine_offset.value(), 0);
      auto off = m_sObservationSharingProtocolParams.byzantine_offset.value();
      CVector2 dev = 2 * off * CVector2(drand48(), drand48()) - off * CVector2(1., 1.);
      auto o_msg = OSP_Message(LocalizationObservationMessage(GetPosition().value() + dev,
                                                              GetPosition().value() + dev,
                                                              true,
                                                              GetId(),
                                                              local_time - 5,
                                                              GetPosition().value() + dev,
                                                              GetPosition().value() + dev),
                               GetId(), local_time, 0);
      CByteArray cMessage;
      o_msg.serialize(&cMessage);
      m_pcRadioTX->GetInterfaces()[0].Messages.push_back(cMessage);
   }
   local_time += 1.0;
}

/****************************************/
/****************************************/

void CFootBotLocalization_OSP::Reset()
{
   /* Enable camera filtering */
   m_pcCamera->Enable();
   /* Set beacon color to all red to be visible for other robots */
   m_pcLEDs->SetSingleColor(12, CColor::RED);
   if (m_sObservationSharingProtocolParams.is_byzantine)
      m_pcLEDs->SetAllColors(CColor::RED);
   local_time = 0;
   location = std::nullopt;
   m_ObservationManager.clear();
   m_AccusationManager.clear();
   received_messages.clear();
   outbound_messages.clear();
   SetNewTarget();
}
void CFootBotLocalization_OSP::SetNewTarget()
{
   target = 14. * CVector2(drand48(), drand48()) - 7. * CVector2(1., 1.);
}
std::optional<Real> CFootBotLocalization_OSP::DistanceToTarget()
{
   auto loc = GetPosition();
   return loc.has_value() ? std::optional<Real>{(target - loc.value()).Length()} : std::nullopt;
}
std::optional<CVector2> CFootBotLocalization_OSP::GetPosition()
{
   if (m_sObservationSharingProtocolParams.is_observer || m_sObservationSharingProtocolParams.is_byzantine)
   {
      CVector2 p_self;
      m_pcPosition->GetReading().Position.ProjectOntoXY(p_self);
      return p_self;
   }
   else
   {
      if (location.has_value())
      {
         auto err_vec = (location.value().first - location.value().second).Absolute();
         // if (err_vec.GetX() < 8.0 && err_vec.GetY() < 8.0)
         return std::optional<CVector2>{center(location.value())};
      }
      return std::nullopt;
   }
}
std::optional<CVector2> CFootBotLocalization_OSP::VectorToTarget()
{
   if (!GetPosition().has_value())
      return std::nullopt;
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

CVector2 CFootBotLocalization_OSP::FlockingVector()
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

void CFootBotLocalization_OSP::SetWheelSpeedsFromVector(const CVector2 &c_heading)
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

void CFootBotLocalization_OSP::ObservationSharingProtocol()
{
   OSP_Message OSP_msg;
   for (CByteArray c_msg : m_pcRadioRX->GetInterfaces()[0].Messages)
   {
      OSP_msg.deserialize(&c_msg);
      MessageKey key = MessageKey(OSP_msg);
      if (m_AccusationManager.is_accused(key.GetOrigin()) && !std::holds_alternative<AccusationMessage>(OSP_msg.m_msg_variant)) // must forward accusations
         continue;
      if (auto found = received_messages.find(key); found != received_messages.end() && std::holds_alternative<AccusationMessage>(OSP_msg.m_msg_variant))
      {
         if (found->second.m_hop_count < m_sObservationSharingProtocolParams.max_hop_count &&
             outbound_messages.count(key) == 0)
         {
            (found->second)++;
            outbound_messages.insert({key, found->second});
         }
         else
         {
            continue;
         }
      }
      else // first time message has been received
      {
         if (std::holds_alternative<LocalizationObservationMessage>(OSP_msg.m_msg_variant))
         {
            auto o_msg = std::get<LocalizationObservationMessage>(OSP_msg.m_msg_variant);
            auto value = LocalizationObservationValue(o_msg, 0);
            auto acc = m_ObservationManager.is_suspicious(OSP_msg,
                                                          local_time,
                                                          m_sObservationSharingProtocolParams.is_observer,
                                                          m_sObservationSharingProtocolParams.max_displacement,
                                                          GetRadioRange(),
                                                          GetPosition().value_or(CVector2::ZERO));
            if (acc.has_value() && m_sObservationSharingProtocolParams.mitigation)
            {
               auto new_accusation = OSP_Message(AccusationMessage(acc.value()), GetId(), local_time, 0);
               outbound_messages.insert({MessageKey(new_accusation), new_accusation});
               m_AccusationManager.add_accused(Accusation(GetId(), std::get<AccusationMessage>(new_accusation.m_msg_variant)),
                                               m_ObservationManager);
               continue;
            }
            else
            {
               m_ObservationManager.add_observation(key, value);
            }
         }
         else if (std::holds_alternative<AccusationMessage>(OSP_msg.m_msg_variant))
         {
            auto accusation = Accusation(OSP_msg.m_origin, std::get<AccusationMessage>(OSP_msg.m_msg_variant));
            m_AccusationManager.add_accused(accusation, m_ObservationManager);
            outbound_messages.insert({key, OSP_msg});
         }
         OSP_msg.m_hop_count = 0;
         received_messages.insert({key, OSP_msg});
      }
   }
   if (m_sObservationSharingProtocolParams.is_regular())
   {
      auto update = m_ObservationManager.guess(m_sObservationSharingProtocolParams.max_displacement, GetRadioRange());
      if (update.has_value())
      {
         location = update.value().first;
         auto o_msg = OSP_Message(LocalizationObservationMessage(
                                      location.value().first,
                                      location.value().second,
                                      false,
                                      std::get<0>(update.value().second),
                                      std::get<1>(update.value().second),
                                      std::get<2>(update.value().second),
                                      std::get<3>(update.value().second)),
                                  GetId(), local_time, 0);
         outbound_messages.insert({MessageKey(o_msg), o_msg});
      }
   }
   for (auto outbound : outbound_messages)
   {
      CByteArray c_msg;
      outbound.second.serialize(&c_msg);
      m_pcRadioTX->GetInterfaces()[0].Messages.push_back(c_msg);
   }
   outbound_messages.clear();
}

Real CFootBotLocalization_OSP::GetRadioRange()
{
   // todo for whatever reason, the range isn't a property of the sensor ??
   return 4;
}
Real CFootBotLocalization_OSP::GetObservationRange()
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
REGISTER_CONTROLLER(CFootBotLocalization_OSP, "footbot_localization_OSP_controller")
