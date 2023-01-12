#ifndef ID_QTUSER_FUNCTIONS_H
#define ID_QTUSER_FUNCTIONS_H
 
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot-networked/simulator/footbot_entity.h>
#include <argos3/plugins/robots/generic/control_interface/ci_simple_radios_actuator.h>

 
using namespace argos;
 
class CIDQTUserFunctions : public CQTOpenGLUserFunctions {
 
public:
 
   CIDQTUserFunctions();
 
   virtual ~CIDQTUserFunctions() {}
 
   void Draw(CFootBotNetworkedEntity& c_entity);
 
};
 
#endif