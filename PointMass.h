#ifndef __POINTMASS_H__
#define __POINTMASS_H__

#include <Eigen/Eigen>
#include<iostream>
using namespace std;
using namespace Eigen;

typedef Matrix<double, 3, 1> Vec3d;

template <typename pos_t, typename mass_t = pos_t>
class PointMass
{
//protected:
public:
	typedef Matrix<pos_t, 3, 1> Vec3d;
	Vec3d vel;
	Vec3d pos;
	Vec3d accel;
	Vec3d vel_change;

	mass_t mass;

	static Vec3d project(const Vec3d& a, const Vec3d& b);
public:
	PointMass();
	PointMass(pos_t mass, pos_t x, pos_t y, pos_t z);
	PointMass(pos_t mass, Vec3d pos);
	PointMass(pos_t mass, Vec3d pos, Vec3d vel);

	void step(int millisecs);

	/*void collide(PointMass* other1)
	{
		const int dimension = 3;
		Vec3d A = other1->pos;
		Vec3d S = this->pos;

		pos_t KEa_total = 0.0;

		Vec3d A_dir = (A-S).normalized();
		pos_t KEs = 0.5*this->mass*this->vel.dot(this->vel);

		for(int i = 0; i < dimension; ++i)
		{
			pos_t total = abs(A_dir(i));
			if(total == 0.0)
				continue;
			pos_t KEs = 0.5*this->mass*this->vel(i)*this->vel(i);
			pos_t KEa = abs(A_dir(i)/total*KEs);

			KEa_total += KEa;
		}

		Vec3d vel_changeA = A_dir.dot(this->vel) > 0 ? sqrt(2.0/other1->mass*KEa_total)*A_dir : -sqrt(2.0/other1->mass*KEa_total)*A_dir;

		this->vel_change -= vel_changeA;
		other1->vel_change += vel_changeA;
	}

	void collide(PointMass* other1, PointMass* other2)
	{
		const int dimension = 3;
		Vec3d A = other1->pos;
		Vec3d B = other2->pos;
		Vec3d S = this->pos;

		cout << other1->vel << endl << endl << other2->vel << endl << endl << this->vel << endl << endl;
		pos_t KEa_total = 0.0;
		pos_t KEb_total = 0.0;

		Vec3d A_dir = (A-S).normalized();
		Vec3d B_dir = (B-S).normalized();
		pos_t KEs = 0.5*this->mass*this->vel.dot(this->vel);

		for(int i = 0; i < dimension; ++i)
		{
			pos_t total = abs(A_dir(i)) + abs(B_dir(i));
			if(total == 0.0)
				continue;
			pos_t KEs = 0.5*this->mass*this->vel(i)*this->vel(i);
			pos_t KEa = abs(A_dir(i)/total*KEs);
			pos_t KEb = abs(B_dir(i)/total*KEs);

			KEa_total += KEa;
			KEb_total += KEb;
		}

		cout << KEs << ' ' << KEa_total << ' ' << KEb_total << '\t' << KEa_total + KEb_total << endl;

		//Problem is here, energy is not being conserved
		// DONT FORGET TO UPDATE THE SINGLE COLLISION VERSION
		Vec3d vel_changeA = A_dir.dot(this->vel) > 0 ? sqrt(2.0/other1->mass*KEa_total)*A_dir : -sqrt(2.0/other1->mass*KEa_total)*A_dir;
		Vec3d vel_changeB = B_dir.dot(this->vel) > 0 ? sqrt(2.0/other2->mass*KEb_total)*B_dir : -sqrt(2.0/other2->mass*KEb_total)*B_dir;

		this->vel_change -= vel_changeA + vel_changeB;
		other1->vel_change += vel_changeA;
		other2->vel_change += vel_changeB;
	}*/
};

#endif