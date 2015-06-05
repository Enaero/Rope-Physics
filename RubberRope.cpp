//TODO: REIMPLEMENT!!

#include "RubberRope.h"

template <typename pos_t, typename mass_t>
Matrix<pos_t, 3, 1> RubberRope<pos_t, mass_t>::spring_force(const Vec3d& x, const Vec3d& reference)
{
	Vec3d result(0, 0, 0);
	Vec3d direction = (reference - x).normalized();

	pos_t delta = (x - reference).norm() - rest_dist;

	return delta*k*direction;
}

template <typename pos_t, typename mass_t>
RubberRope<pos_t, mass_t>::RubberRope(pos_t mass, const Vec3d& start, const Vec3d& end, unsigned num_links, double spring_k, double damp_k)
	: Rope(mass, start, end, num_links), k(spring_k), d(damp_k)
{
	rest_dist = (start - end).norm()/double(num_links);
}

template <typename pos_t, typename mass_t>
void RubberRope<pos_t, mass_t>::step(int millisecs)
{
	Vec3d impulse;
	double time = millisecs/1000.0;

	/* Impulse from each spring */
	if (links.size() > 1)
	{
		impulse = spring_force(links[0].pos, links[1].pos)*time;
		addImpulseAt(impulse, 0);
	}
	for (size_t i = 1; i < links.size()-1; ++i)
	{
		impulse = spring_force(links[i].pos, links[i+1].pos)*time;
		impulse += spring_force(links[i].pos, links[i-1].pos)*time;

		addImpulseAt(impulse, i);
	}
	if(links.size() > 1)
	{
		impulse = spring_force(links[links.size()-1].pos, links[links.size()-2].pos)*time;
		addImpulseAt(impulse, links.size()-1);
	}

	Rope::step(millisecs);

	/* Damping each spring */
	if (links.size() > 1)
	{
		Vec3d component = project(links[0].vel, links[1].pos - links[0].pos);
		links[0].vel -= component;
		component *= d;
		links[0].vel += component;
	}
	for (size_t i = 1; i < links.size()-1; ++i)
	{
		Vec3d component = project(links[i].vel, links[i+1].pos - links[i].pos);
		links[i].vel -= component;
		component *= d;
		links[i].vel += component;

		component = project(links[i].vel, links[i-1].pos - links[i].pos);
		links[i].vel -= component;
		component *= d;
		links[i].vel += component;
	}
	if(links.size() > 1)
	{
		size_t i = links.size()-1;
		Vec3d component = project(links[i].vel, links[i-1].pos - links[i].pos);
		links[i].vel -= component;
		component *= d;
		links[i].vel += component;
	}
}