
#include "MATHCLASS/mathclass.h"
#include "qmGeodesic.h"

using namespace jhm;

quater
QmGeodesic::getValue( m_real t )
{
	return exp( t*axis/2.0 ) * orientation;
}

quater
QmGeodesic::nearest( quater const& q, m_real& t )
{
	m_real ws = q.real();
	vector vs = q.imaginaries();
	m_real w0 = orientation.real();
	vector v0 = orientation.imaginaries();

	m_real a = ws*w0 + vs%v0;
	m_real b = w0*(axis % vs) - ws*(axis % v0) + vs%(axis * v0);

	m_real alpha = atan2( a,b );

	m_real t1 = -2*alpha + M_PI;
	m_real t2 = -2*alpha - M_PI;

	if ( q % getValue(t1) > q % getValue(t2) )
	{
		t = t1;
		return getValue(t1);
	}

	t = t2;
	return getValue(t2);
}

quater
QmGeodesic::farthest( quater const& q, m_real& t )
{
	m_real ws = q.real();
	vector vs = q.imaginaries();
	m_real w0 = orientation.real();
	vector v0 = orientation.imaginaries();

	m_real a = ws*w0 + vs%v0;
	m_real b = w0*(axis % vs) - ws*(axis % v0) + vs%(axis * v0);

	m_real alpha = atan2( a,b );

	m_real t1 = -alpha + M_PI/2.0;
	m_real t2 = -alpha - M_PI/2.0;

	if ( q % getValue(t1) > q % getValue(t2) )
	{
		t = t2;
		return getValue(t2);
	}

	t = t1;
	return getValue(t1);
}

quater
QmGeodesic::nearest( quater const& q )
{
	m_real t;
	return this->nearest( q, t );
}

quater
QmGeodesic::farthest( quater const& q )
{
	m_real t;
	return this->farthest( q, t );
}

transf
PlaneProject( transf const& t )
{
	QmGeodesic g( quater(1,0,0,0), y_axis );
	vector v = t.translation();

	return transf( g.nearest( t.getRotation() ),
				   vector(v[0], 0, v[2]) );
}
