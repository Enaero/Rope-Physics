#include "PointMass.h"

template <typename pos_t, typename mass_t>
Matrix<pos_t, 3, 1> PointMass<pos_t, mass_t>::project(const Vec3d& a, const Vec3d& b)
{
	pos_t length = a.dot(b)/b.dot(b);
	Vec3d direction = b;
	return direction*length;
}

template <typename pos_t, typename mass_t>
PointMass<pos_t, mass_t>::PointMass()
{
	mass = 0;
	pos << 0, 0, 0;
	vel << 0, 0, 0;
	vel_change << 0, 0, 0;
	accel << 0, 0, 0;
}

template <typename pos_t, typename mass_t>
PointMass<pos_t, mass_t>::PointMass(pos_t mass, pos_t x, pos_t y, pos_t z)
{
	pos << x, y, z;
	vel << 0, 0, 0;
	vel_change << 0, 0, 0;
	accel << 0, 0, 0;
	this->mass = mass;
}

template <typename pos_t, typename mass_t>
PointMass<pos_t, mass_t>::PointMass(pos_t mass, Vec3d pos)
{
	this->pos = pos;
	vel << 0, 0, 0;
	vel_change << 0, 0, 0;
	accel << 0, 0, 0;
	this->mass = mass;
}

template <typename pos_t, typename mass_t>
PointMass<pos_t, mass_t>::PointMass(pos_t mass, Vec3d pos, Vec3d vel)
{
	this->pos = pos;
	this->vel = vel;
	vel_change << 0, 0, 0;
	this->mass = mass;
	accel << 0, 0, 0;
}

template <typename pos_t, typename mass_t>
void PointMass<pos_t, mass_t>::step(int millisecs)
{
	double time = millisecs / 1000.0;
	pos += vel*time;
	vel += accel*time + vel_change;
	vel_change << 0, 0, 0;
}