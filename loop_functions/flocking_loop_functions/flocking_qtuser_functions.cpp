#include "flocking_qtuser_functions.h"
#include "../../controllers/footbot_flocking/footbot_flocking.h"

/****************************************/
/****************************************/

CIDQTUserFunctions::CIDQTUserFunctions()
{
   RegisterUserFunction<CIDQTUserFunctions, CFootBotNetworkedEntity>(&CIDQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CIDQTUserFunctions::Draw(CFootBotNetworkedEntity &c_entity)
{
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */
   std::stringstream x_ss;
   std::string text;
   text.append(c_entity.GetId().c_str());
   text.append(": ");
   auto controller = (CFootBotFlocking *)&c_entity.GetControllableEntity().GetController();
   if (controller->x.has_value())
   {
      x_ss << std::fixed << std::setprecision(2) << "(" << controller->x.value().GetX() << ", " << controller->x.value().GetY() << ")";
   }
   else
   {
      x_ss << "(null)";
   }
   text.append(x_ss.str());
   DrawText(CVector3(0.0, 0.0, 0.3), // position
            text);                   // text
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CIDQTUserFunctions, "flocking_qtuser_functions")