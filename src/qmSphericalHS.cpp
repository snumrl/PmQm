
#include "MATHCLASS/mathclass.h"
#include "qmSphericalHS.h"

using namespace jhm;

int
QmSphericalHS::inclusion( quater const& q )
{
	m_real d = 2.0*ln(q * orientation.inverse()).length();

	return d << angle_bound;
}

int
QmSphericalHS::intersection( QmGeodesic const&, quater* )
{
	return -1;
}

vector
QmSphericalHS::gradient( quater const& q )
{
	return vector(0,0,0);
}

m_real
QmSphericalHS::distance( quater const& q )
{
	m_real d = 2*ln(q * orientation.inverse()).length();

	return angle_bound.distance( d );
}

quater
QmSphericalHS::nearest( quater const& q )
{
	vector v = 2*ln(q * orientation.inverse());

	if ( v.length() << angle_bound ) return q;

	return exp( angle_bound.end_pt() * normalize(v) / 2. ) * orientation;
}

void
QmSphericalHS::write( std::ostream& os )
{
	os << (*this);
}

std::ostream&
operator<<( std::ostream& os, QmSphericalHS const& h )
{
    os << "#S( " << h.getOrientation() << " , " << h.getAngleBound() << " )";
    return os;
}

std::istream&
operator>>( std::istream& is, QmSphericalHS& h )
{
	quater   q;
	interval i;
	char	 buf[100];

    is >> buf >> q >> buf >> i >> buf;
    
    h.setOrientation( q );
    h.setAngleBound( i );
    
    return is;
}

