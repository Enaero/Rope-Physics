#ifndef __ROPE_H_
#define __ROPE_H_

#include <vector>
#include <Eigen/Eigen>

#include "PointMass.h"

template <typename pos_t, typename mass_t = pos_t>
class Rope
{
//protected:
public:
	typedef Matrix<pos_t, 3, 1> Vec3d;
	std::vector<PointMass<pos_t, mass_t> > links;

	static Vec3d project(const Vec3d& a, const Vec3d& b);

public:
	Rope(pos_t mass, const Vec3d& start, const Vec3d& end, unsigned num_links);

	void addPermanentForceAt(Vec3d force, size_t link);

	void addPermanentForce(Vec3d force);

	virtual void addImpulseAt(Vec3d momentum, size_t link);

	virtual void step(int millisecs);
};

#endif