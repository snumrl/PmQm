
#include "pm.h"

using namespace jhm;

void
PmPosture::correct_313_to_322( int END )
{
	int LOWER = body->getParent( END );

	vector v1 = z_axis * Quater2Matrix(rotate[END]);
	vector v2 = v1 - (v1 % y_axis) * y_axis;

	m_real angle1 = atan( (v1 * z_axis)%y_axis / (v1 % z_axis) );

	rotate[END]   = exp(  y_axis * angle1/2.0 ) * rotate[END];
	rotate[LOWER] = rotate[LOWER] * exp( -y_axis * angle1/2.0 );
}

void
PmLinearMotion::correct()
{
	for( int i=0; i<this->getSize(); i++ )
	{
		PmPosture &p = getPosture(i);

		int end = PmHuman::RIGHT_PALM;
		p.ik_limb( p.getGlobalTransf(end), end, 0.0);

		end = PmHuman::LEFT_PALM;
		p.ik_limb( p.getGlobalTransf(end), end, 0.0);

		end = PmHuman::RIGHT_FOOT;
		p.ik_limb( p.getGlobalTransf(end), end, 0.0);

		end = PmHuman::LEFT_FOOT;
		p.ik_limb( p.getGlobalTransf(end), end, 0.0);
	}
}

