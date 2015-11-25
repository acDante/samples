/*
 * Modified by Tetsunari Inamura on 2013-12-29
 *    Add comments
 */

#ifndef NO_ENTITY_ATTRS
DEFINE_ATTR_BOOL(isAgent, "agent")

DEFINE_ATTR_DOUBLE(x, "x")
DEFINE_ATTR_DOUBLE(y, "y")
DEFINE_ATTR_DOUBLE(z, "z")
//
// For example, DEFINE_ATTR_DOUBLE(x, "x") will be expanded as:
// double x() const
//{
//  return getAttr("x").value().getDouble();
//}
//void x(double v)
//{
//  getAttr("x").value().setDouble(v);
//}
//by the definition written in SimObjBase.h
//

DEFINE_ATTR_DOUBLE(fx, "fx")
DEFINE_ATTR_DOUBLE(fy, "fy")
DEFINE_ATTR_DOUBLE(fz, "fz")

DEFINE_ATTR_DOUBLE(tqx, "tqx")
DEFINE_ATTR_DOUBLE(tqy, "tqy")
DEFINE_ATTR_DOUBLE(tqz, "tqz")

// velocity
DEFINE_ATTR_DOUBLE(vx,"vx")
DEFINE_ATTR_DOUBLE(vy,"vy")
DEFINE_ATTR_DOUBLE(vz,"vz")

// angular velocity
DEFINE_ATTR_DOUBLE(avx,"avx")
DEFINE_ATTR_DOUBLE(avy,"avy")
DEFINE_ATTR_DOUBLE(avz,"avz")

DEFINE_ATTR_DOUBLE(qw, "qw")
DEFINE_ATTR_DOUBLE(qx, "qx")
DEFINE_ATTR_DOUBLE(qy, "qy")
DEFINE_ATTR_DOUBLE(qz, "qz")

DEFINE_ATTR_BOOL(dynamics, "dynamics")
DEFINE_ATTR_DOUBLE(mass, "mass")

// scale
DEFINE_ATTR_DOUBLE(scalex, "scalex")
DEFINE_ATTR_DOUBLE(scaley, "scaley")
DEFINE_ATTR_DOUBLE(scalez, "scalez")

// shape file
DEFINE_ATTR_STRING(shapeFile, "shapeFile")

#endif // NO_ENTITI_ATTRS

#ifndef NO_AGENT_ATTRS
DEFINE_ATTR_DOUBLE(vpx, "vpx")
DEFINE_ATTR_DOUBLE(vpy, "vpy")
DEFINE_ATTR_DOUBLE(vpz, "vpz")

DEFINE_ATTR_DOUBLE(vvx, "vvx")
DEFINE_ATTR_DOUBLE(vvy, "vvy")
DEFINE_ATTR_DOUBLE(vvz, "vvz")

// left eye's position
DEFINE_ATTR_DOUBLE(lepx, "lepx")
DEFINE_ATTR_DOUBLE(lepy, "lepy")
DEFINE_ATTR_DOUBLE(lepz, "lepz")

// left eye's direction
DEFINE_ATTR_DOUBLE(levx, "levx")
DEFINE_ATTR_DOUBLE(levy, "levy")
DEFINE_ATTR_DOUBLE(levz, "levz")

// right eye's position
DEFINE_ATTR_DOUBLE(repx, "repx")
DEFINE_ATTR_DOUBLE(repy, "repy")
DEFINE_ATTR_DOUBLE(repz, "repz")

// right eye's direction
DEFINE_ATTR_DOUBLE(revx, "revx")
DEFINE_ATTR_DOUBLE(revy, "revy")
DEFINE_ATTR_DOUBLE(revz, "revz")

// added by yahara@tome (2011/02/08)
// sub camera attachment
DEFINE_ATTR_STRING(elnk1, "elnk1")
DEFINE_ATTR_STRING(elnk2, "elnk2")

// sub camera1's position
DEFINE_ATTR_DOUBLE(epx1, "epx1")
DEFINE_ATTR_DOUBLE(epy1, "epy1")
DEFINE_ATTR_DOUBLE(epz1, "epz1")

// sub camera1's direction
DEFINE_ATTR_DOUBLE(evx1, "evx1")
DEFINE_ATTR_DOUBLE(evy1, "evy1")
DEFINE_ATTR_DOUBLE(evz1, "evz1")

// sub camera2's position
DEFINE_ATTR_DOUBLE(epx2, "epx2")
DEFINE_ATTR_DOUBLE(epy2, "epy2")
DEFINE_ATTR_DOUBLE(epz2, "epz2")

// sub camera2's direction
DEFINE_ATTR_DOUBLE(evx2, "evx2")
DEFINE_ATTR_DOUBLE(evy2, "evy2")
DEFINE_ATTR_DOUBLE(evz2, "evz2")

DEFINE_ATTR_DOUBLE(FOV, "FOV")
DEFINE_ATTR_DOUBLE(aspectRatio, "aspectRatio")

// voice reach radius
DEFINE_ATTR_DOUBLE(voiceReachRadius, "voiceReachRadius")
DEFINE_ATTR_BOOL(voiceReachRadiusEnable, "voiceReachRadiusEnable")

// collision detection
DEFINE_ATTR_BOOL(collision, "collision")

// grasp flag
DEFINE_ATTR_BOOL(graspable, "graspable")

DEFINE_ATTR_DOUBLE(graspRadius, "graspRadius")

#endif // NO_AGENT_ATTRS

