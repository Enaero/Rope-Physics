#include "Rope.h"
#include <string>

template <typename pos_t, typename mass_t>
Matrix<pos_t, 3, 1> Rope<pos_t, mass_t>::project(const Vec3d& a, const Vec3d& b)
{
	pos_t length = a.dot(b)/b.dot(b);
	Vec3d direction = b;
	return direction*length;
}

template <typename pos_t, typename mass_t>
Rope<pos_t, mass_t>::Rope(pos_t mass, const Vec3d& start, const Vec3d& end, unsigned num_links)
{
	mass_t link_mass = mass/num_links;

	if (link_mass == 0)
		throw string("link_mass in Rope::Rope() has a value of zero. This is not allowed.")
		+ string("Check to see of mass is zero or if mass_t is integer and mass < num_links.");
	Vec3d diff = end - start;

	for (unsigned i = 0; i <= num_links; ++i)
	{
		links.push_back(PointMass<pos_t, mass_t>(link_mass, diff*pos_t(i)/num_links + start));
	}
}

template <typename pos_t, typename mass_t>
void Rope<pos_t, mass_t>::addPermanentForceAt(Vec3d force, size_t link)
{
	links[link].accel += force/links[link].mass;
}

template <typename pos_t, typename mass_t>
void Rope<pos_t, mass_t>::addPermanentForce(Vec3d force)
{
	for (size_t i = 0; i < links.size(); ++i)
		links[i].accel += force/links[i].mass;
}

template <typename pos_t, typename mass_t>
void Rope<pos_t, mass_t>::addImpulseAt(Vec3d momentum, size_t link)
{
	links[link].vel_change += momentum/links[link].mass;
}

template <typename pos_t, typename mass_t>
void Rope<pos_t, mass_t>::step(int millisecs)
{
	for (size_t i = 0; i < links.size(); ++i)
	{
		links[i].step(millisecs);
	}
}