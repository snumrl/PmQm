
#include "pm.h"


using namespace jhm;

void
PmPosture::ik_right_hand( transf const& palm, m_real roll )
{
	ik_limb( palm, PmHuman::RIGHT_PALM, roll );
}

void
PmPosture::ik_left_hand( transf const& palm, m_real roll )
{
	ik_limb( palm, PmHuman::LEFT_PALM, roll );
}

void
PmPosture::ik_right_foot( transf const& palm, m_real roll )
{
	ik_limb( palm, PmHuman::RIGHT_FOOT, roll );
}

void
PmPosture::ik_left_foot( transf const& palm, m_real roll )
{
	ik_limb( palm, PmHuman::LEFT_FOOT, roll );
}

void
PmPosture::ik_limb( transf const& end, int END, m_real roll )
{
	int LOWER = body->getParent(  END  );
	int UPPER = body->getParent( LOWER );
	int BASE  = body->getParent( UPPER );

	transf reference = getGlobalTransf( BASE ).inverse();

	quater   upper_old = rotate[UPPER];

	//
	// Length information
	//
	vector j1 = getScale() * body->getJointPosition(END);
	vector j2 = getScale() * body->getJointPosition(LOWER);

	m_real l1 = j1.length();
	m_real l2 = j2.length();
	m_real l3 = (j1+j2).length();

	vector endp1  = (end * reference).getTranslation();
	vector startp = (getGlobalTransf(UPPER) * reference).getTranslation();

	vector v1 = endp1 - startp;

	//
	// Adjust Lower
	//
	m_real r1 = sqrt( SQR(j1.y()) + SQR(j1.z()) );
	m_real r2 = sqrt( SQR(j2.y()) + SQR(j2.z()) );

	m_real s1 = sqrt( l1*l1 - r1*r1 );
	m_real s2 = sqrt( l2*l2 - r2*r2 );

	m_real c = ( j1%j1 + j2%j2 + 2.0*s1*s2 - v1%v1 ) / ( 2.0 * r1 * r2 );

	m_real angle1 = M_PI - ASIN(c) - atan2(j2.y(),j2.z());
	m_real angle2 = ASIN(c) - atan2(j2.y(),j2.z());
	vector v4 = normalize(vector(0,j1.y(),j1.z()))*y_axis;

	quater q1 = exp( angle1/2.0 * x_axis - v4/2.0 );
	quater q2 = exp( angle2/2.0 * x_axis - v4/2.0 );

	QmComplex* bound = body->getAngleBound(LOWER);
	//mingle
	
	if ( bound )
	{
		if ( bound->distance(q1) < bound->distance(q2) )
			rotate[LOWER] = q1;
		else
			rotate[LOWER] = q2;
	}
	else
		rotate[LOWER] = q1; //rotate[LOWER] = q1;
	/*
	if ( bound )
	{
		if ( bound->distance(q1) < bound->distance(q2) )
			rotate[LOWER] = q2;
		else
			rotate[LOWER] = q1;
	}
	else
		rotate[LOWER] = q2; //rotate[LOWER] = q1;
	*/

	//
	// Adjust Upper
	//
	vector endp2  = (getGlobalTransf(END) * reference).getTranslation();
	vector v2 = endp2 - startp;
	vector v3 = normalize(v2 * v1);

	c = (v1%v2) / (v1.length()*v2.length());
	m_real angle3 = ACOS( c );

	rotate[UPPER] = exp( (angle3/2) * v3 ) * rotate[UPPER];

	//
	// Adjust Rolling
	//
	QmGeodesic g( rotate[UPPER], normalize(v1) );
	rotate[UPPER] = exp(roll/2.0 * normalize(v1)) *
					g.nearest( upper_old );

	//
	// Adjust END
	//
	transf t = end * getBaseTransf(END).inverse();
	rotate[END] = t.getRotation();
}
