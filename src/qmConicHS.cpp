
#include "MATHCLASS/mathclass.h"
#include "qmConicHS.h"

using namespace jhm;

int
QmConicHS::inclusion( quater const& q )
{
	vector w = rotate( q*orientation.inverse(), axis );
	m_real psi = ACOS( w % axis );

	return psi << angle_bound;
}

int
QmConicHS::intersection( QmGeodesic const&, quater* )
{
	return -1;
}

vector
QmConicHS::gradient( quater const& q )
{
	return vector(0,0,0);
}

m_real
QmConicHS::distance( quater const& q )
{
	vector w = rotate( q*orientation.inverse(), axis );
	m_real psi = ACOS( w % axis );

	return angle_bound.distance( psi );
}

quater
QmConicHS::nearest( quater const& q )
{
	quater p = q*orientation.inverse();
	vector w = rotate( p, axis );
    m_real psi = ACOS( w % axis );

    vector v = normalize(axis * w);
    m_real phi = 2*ln( exp(-psi*v/2.0)*p ) % axis;

    if ( psi << angle_bound ) return q;

    return exp( angle_bound.end_pt()*v/2. )
         * exp( phi*axis/2. ) * orientation;
}

void
QmConicHS::write( std::ostream& os )
{
	os << (*this);
}

std::ostream&
operator<<( std::ostream& os, QmConicHS const& h )
{
    os << "#C( " << h.getOrientation() << " , "
    		   << h.getAxis() << " , "
    		   << h.getAngleBound() << " )";
    return os;
}

std::istream&
operator>>( std::istream& is, QmConicHS& h )
{
	quater   	q;
	unit_vector v;
	interval 	i;
	char		buf[100];

    is >> buf >> q >> buf >> v >> buf >> i >> buf;
    
    h.setOrientation( q );
    h.setAxis( v );
    h.setAngleBound( i );
    
    return is;
}

