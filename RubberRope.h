#ifndef __RUBBERROPE_H__
#define __RUBBERROPE_H__

#include "Rope.h"

template <typename pos_t, typename mass_t = pos_t>
class RubberRope : public Rope<pos_t, mass_t>
{
protected:
	double k; //spring constant
	double d; //damp constant
	pos_t rest_dist;

	Vec3d spring_force(const Vec3d& x, const Vec3d& reference);

public:
	RubberRope(pos_t mass, const Vec3d& start, const Vec3d& end, unsigned num_links, double spring_k, double damp_k);

	virtual void step(int millisecs);
};
#endif