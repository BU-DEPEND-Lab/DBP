#include "id_loop_functions.h"

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

    if (c_entity.GetId().find("byz") != std::string::npos)
        DrawCircle(CVector3(0.0, 0.0, 0.3), CQuaternion(0.0, 0.0, 0.0, 0.0), 0.2);
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CIDQTUserFunctions, "id_qtuser_functions")