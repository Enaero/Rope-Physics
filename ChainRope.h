#ifndef __CHAINROPE_H__
#define __CHAINROPE_H__

#include "Rope.h"

template <typename pos_t, typename mass_t = pos_t>
class ChainRope : public Rope<pos_t, mass_t>
{
//private:
public:
	pos_t length;
	void collide(PointMass<pos_t, mass_t>& src, PointMass<pos_t, mass_t>& a, PointMass<pos_t, mass_t>& b);
	void collide(PointMass<pos_t, mass_t>& src, PointMass<pos_t, mass_t>& a);
	void fixLength();

public:
	ChainRope(pos_t mass, const Vec3d& start, const Vec3d& end, unsigned num_links);

	virtual void step(int millisecs);
};

#endif