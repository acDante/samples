#include <Controller.h>    
#include <ControllerEvent.h>    
#include <Logger.h>    
#include <ViewImage.h>    
#include <math.h>    
#include <stdio.h>    
#include <stdlib.h>    
    
#define PI 3.141592    
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )    
    
class RobotController : public Controller    
{    
public:    
  void onInit(InitEvent &evt);    
  double onAction(ActionEvent &evt);    
private:     
    
  // velocity of a robot
  double vel;    
  ViewService *m_view;  
};     
    
void RobotController::onInit(InitEvent &evt)    
{    
  m_view = (ViewService*)connectToService("SIGViewer");  
  vel      = 2.0;    
  srand(time(NULL));    
}      
    
//Callback function which is called at fixed intervals.
double RobotController::onAction(ActionEvent &evt)    
{     
  SimObj *my = getObj(myname());    
    
  // Get current self-position
  Vector3d pos;
  my->getPosition(pos);    
    
  // Get current self-orientation
  Rotation rot;
  my->getRotation(rot);
  double qy = rot.qy();    
  double qw = rot.qw();
 
  double tmp = qy*qw;
  if(tmp < 0) qy = -qy;
  
  // Calculate angle from quaternion
  double theta = 2*asin(qy);    

  double dx = 0;    
  double dz = 0;    
    
  // Calculate moving direction
  dx = sin(theta) * vel;    
  dz = cos(theta) * vel;    
    
  // Execution of move
  my->setPosition( pos.x() + dx, pos.y() , pos.z() + dz );    
  
  // Get vertical view angle of the camera (rad)
  double fovy = my->getCamFOV() * PI / 180.0;

  // Calculate aspect ratio
  double ar = my->getCamAS();
  
  // Calculate horizontal view angle phi(deg)
  double fovx = 2 * atan(tan(fovy*0.5)*ar) * 180.0 / PI;

  // Calculate vertical view angle phi(deg)
  fovy = fovy * 180.0 / PI;

  unsigned char distance = 255;  
  if(m_view != NULL) {  

    // Gets the distance matrix
    ViewImage *img = m_view->distanceSensor2D();  
    char *buf = img->getBuffer();  
  
    // Width of the distance matrix
    int width = img->getWidth();

    // Height of the distance matrix
    int height = img->getHeight();

    // Horizontal angle (origin is direction of a camera)
    double phi = 0.0;

    // Vertical angle (origin is direction of a camera)
    double theta = 0.0;
  
    // Calculate of minimum distance in the distance matrix
    for(int i = 0; i < width; i++){  
      for(int j = 0; j < height; j++){
  int index = j *width + i;
  unsigned char tmp_distance = (unsigned char)buf[index];
  if(tmp_distance < distance){

    // Calculate horizontal view angle phi(rad) from a index of pixel.
    phi   = fovx*i/(width-1.0) - fovx/2.0;
    
    // Calculate vertical view angle theta(rad) from a index of pixel.
    theta = fovy*j/(height-1.0) - fovy/2.0;
    
    distance = buf[index];  
  }
      }  
    }  
    LOG_MSG(("phi = %.1f, theta = %.1f, distance = %d",phi, theta, distance));  
        img->saveAsWindowsBMP("distance.bmp");
    // Chage the orientation if the distance is less than 100
    if(distance < -1000){    
      my->setAxisAndAngle(0.0, 1.0, 0.0, (double)rand()/RAND_MAX * 2*PI, true);    
      // Save a range imagery at this moment
      img->saveAsWindowsBMP("distance.bmp");
    }     
    delete img;  
  }  
  
  return 0.2;
}     
    
extern "C" Controller * createController ()    
{    
  return new RobotController;    
}    

