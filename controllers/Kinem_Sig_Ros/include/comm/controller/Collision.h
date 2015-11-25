/* $Id: Collision.h,v 1.4 2012-03-19 07:04:45 okamoto Exp $ */ 
#ifndef CommController_Collision_h
#define CommController_Collision_h

#include <string>
#include <vector>
#include <string.h>

#include "Controller.h"

class CollisionEvent : public ControllerEvent
{
public:
	typedef std::vector<std::string> WithC;
private:
	WithC m_with;
	
	//衝突相手のパーツと衝突した自分のパーツ 2012/3/19
	WithC m_withParts;
	WithC m_myParts;
public:
	bool	set(int packetNum, int seq, char *data, int n);
	//! 衝突した相手の名前を得る
	const WithC & getWith() { return m_with; }

	//added by okamoto@tome 2012/3/19
	//! 衝突した相手のパーツ名を得る(相手がEntityの場合はbodyを取得)
	const WithC & getWithParts() { return m_withParts; }

	//! 衝突した自分のパーツ名を得る
	const WithC & getMyParts() { return m_myParts; }
};


#endif // CommController_Collision_h
 
