//TODO: variable length segments
//TODO: self collision
//TODO: sticky class
#include "ChainRope.h"

template<typename pos_t, typename mass_t>
ChainRope<pos_t, mass_t>::ChainRope(pos_t mass, const Vec3d& start, const Vec3d& end, unsigned num_links) 
	: Rope(mass, start, end, num_links), length((end-start).norm())
{}

template<typename pos_t, typename mass_t>
void ChainRope<pos_t, mass_t>::fixLength()
{
	if (!links.size())
		return;
	pos_t seg_len = length/(links.size()-1);
	Vec3d d_hat;
	for(size_t i = 0; i < links.size()-1; ++i)
	{
		d_hat = (links[i+1].pos - links[i].pos).normalized();
		links[i+1].pos = links[i].pos + d_hat*seg_len;
	}
}

template<typename pos_t, typename mass_t>
void ChainRope<pos_t, mass_t>::collide(PointMass<pos_t, mass_t>& src, PointMass<pos_t, mass_t>& a, PointMass<pos_t, mass_t>& b)
{
	Vec3d p0 = src.vel*src.mass;
	Vec3d A_hat = (a.pos - src.pos).normalized();
	Vec3d B_hat = (b.pos - src.pos).normalized();
	Vec3d i (1, 0, 0);
	Vec3d j (0, 1, 0);
	Vec3d k (0, 0, 1);

	Matrix<pos_t, 3, 3> M;
	M << a.mass/src.mass*A_hat(0)*A_hat.dot(i)+b.mass/src.mass*B_hat(0)*B_hat.dot(i) + i(0), a.mass/src.mass*A_hat(1)*A_hat.dot(i)+b.mass/src.mass*B_hat(1)*B_hat.dot(i) + i(1), a.mass/src.mass*A_hat(2)*A_hat.dot(i)+b.mass/src.mass*B_hat(2)*B_hat.dot(i) + i(2),
		 a.mass/src.mass*A_hat(0)*A_hat.dot(j)+b.mass/src.mass*B_hat(0)*B_hat.dot(j) + j(0), a.mass/src.mass*A_hat(1)*A_hat.dot(j)+b.mass/src.mass*B_hat(1)*B_hat.dot(j) + j(1), a.mass/src.mass*A_hat(2)*A_hat.dot(j)+b.mass/src.mass*B_hat(2)*B_hat.dot(j) + j(2),
		 a.mass/src.mass*A_hat(0)*A_hat.dot(k)+b.mass/src.mass*B_hat(0)*B_hat.dot(k) + k(0), a.mass/src.mass*A_hat(1)*A_hat.dot(k)+b.mass/src.mass*B_hat(1)*B_hat.dot(k) + k(1), a.mass/src.mass*A_hat(2)*A_hat.dot(k)+b.mass/src.mass*B_hat(2)*B_hat.dot(k) + k(2);

	Vec3d pS = M.inverse()*p0;

	src.vel_change += pS/src.mass - src.vel;
	a.vel_change += pS.dot(A_hat)*A_hat/a.mass;
	b.vel_change += pS.dot(B_hat)*B_hat/b.mass;
}

template<typename pos_t, typename mass_t>
void ChainRope<pos_t, mass_t>::collide(PointMass<pos_t, mass_t>& src, PointMass<pos_t, mass_t>& a)
{
	Vec3d p0 = src.vel*src.mass;
	Vec3d A_hat = (a.pos - src.pos).normalized();
	Vec3d i (1, 0, 0);
	Vec3d j (0, 1, 0);
	Vec3d k (0, 0, 1);

	Matrix<pos_t, 3, 3> M;
	M << a.mass/src.mass*A_hat(0)*A_hat.dot(i) + i(0), a.mass/src.mass*A_hat(1)*A_hat.dot(i) + i(1), a.mass/src.mass*A_hat(2)*A_hat.dot(i) + i(2),
		 a.mass/src.mass*A_hat(0)*A_hat.dot(j) + j(0), a.mass/src.mass*A_hat(1)*A_hat.dot(j) + j(1), a.mass/src.mass*A_hat(2)*A_hat.dot(j) + j(2),
		 a.mass/src.mass*A_hat(0)*A_hat.dot(k) + k(0), a.mass/src.mass*A_hat(1)*A_hat.dot(k) + k(1), a.mass/src.mass*A_hat(2)*A_hat.dot(k) + k(2);

	Vec3d pS = M.inverse()*p0;

	src.vel_change += pS/src.mass - src.vel;
	a.vel_change += pS.dot(A_hat)*A_hat/a.mass;

	/*Vec3d A_hat = (a.pos - src.pos).normalized();
	Vec3d p0 = src.mass*src.vel;

	Vec3d vA = (p0.dot(A_hat)*A_hat)/(src.mass + a.mass);
	Vec3d vS = (p0-vA*a.mass)/src.mass;

	src.vel_change += vS - src.vel;
	a.vel_change += vA;*/
}

template<typename pos_t, typename mass_t>
void ChainRope<pos_t, mass_t>::step(int millisecs)
{
	if (links.size() > 1)
	{
		collide(links[0], links[1]);
	}

	for (size_t i = 1; i < links.size()-1; ++i)
	{
		collide(links[i], links[i-1], links[i+1]);
	}
	
	
	if(links.size() > 1)
	{
		collide(links[links.size()-1], links[links.size()-2]);
	}

	Rope::step(millisecs);
	fixLength();
}