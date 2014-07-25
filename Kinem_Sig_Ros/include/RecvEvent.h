/* $Id: RecvEvent.h,v 1.7 2012-03-27 05:05:09 noma Exp $ */ 
#ifndef RecvEvent_h
#define RecvEvent_h

#include "comm/event/LogMsg.h"
#include "comm/event/RecvEntities.h"
#include "comm/event/RequestAttach.h"
#include "comm/event/RequestGetEntity.h"
#include "comm/event/RequestSimCtrl.h"
#include "comm/event/ResultGetEntity.h"
#include "comm/event/Result.h"
#include "comm/event/RequestCaptureViewImage.h"
#include "comm/event/ResultCaptureViewImage.h"
#include "comm/event/DetectEntities.h"

// added by okamoto@tome (2011/9/9)
#include "comm/event/RequestDistanceSensor.h"
#include "comm/event/ResultDistanceSensor.h"

#include "comm/event/ControllerCommand.h"
#include "comm/event/NameService.h"

#include "comm/event/Attrs.h"
#include "comm/event/JointAngle.h"

// added by okamoto@tome (2011/2/18)
#include "comm/event/JointQuaternion.h"

// added by okamoto@tome (2011/3/3)
#include "comm/event/JointTorque.h"

#include "comm/event/SoundRecog.h"
#include "comm/event/GetJointForce.h"
#include "comm/event/ResultAttach.h"

#include "comm/event/ConnectJoint.h"
#include "comm/event/ReleaseJoint.h"

#include "comm/event/GetObjectNames.h"

// added by okamoto@tome (2011/8/2)
#include "comm/event/DisplayText.h"

// added by noma@tome (2012/02/20)
#include "comm/event/GetPointedObject.h"


#include "comm/event/MoveControl.h"




#endif // RecvEvent_h
