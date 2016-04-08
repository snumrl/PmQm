
#include "MATHCLASS/mathclass.h"
#include "qmAxialHS.h"

using namespace jhm;

int
QmAxialHS::inclusion( quater const& q )
{
	quater p = q*orientation.inverse();
	vector w = rotate( p, axis );
    m_real psi = ACOS( w % axis );

	vector v = normalize(axis * w);
	m_real phi = 2*ln( exp(-psi*v/2.0)*p )%axis;

	return phi << angle_bound;
}

int
QmAxialHS::intersection( QmGeodesic const&, quater* )
{
	return -1;
}

vector
QmAxialHS::gradient( quater const& q )
{
	return vector(0,0,0);
}

m_real
QmAxialHS::distance( quater const& q )
{
	quater p = q*orientation.inverse();
	vector w = rotate( p, axis );
    m_real psi = ACOS( w % axis );

	vector v = normalize(axis * w);
	m_real phi = 2*ln( exp(-psi*v/2.0)*p ) % axis;

	return angle_bound.distance( phi );
}

quater
QmAxialHS::nearest( quater const& q )
{
	quater p = q*orientation.inverse();
	vector w = rotate( p, axis );
    m_real psi = ACOS( w % axis );

	vector v = normalize(axis * w);
	m_real phi = 2*ln( exp(-psi*v/2.0)*p ) % axis;

	if ( phi << angle_bound ) return q;
	
	return exp( psi*v/2. )
		 * exp( angle_bound.project( phi )*axis/2. )
		 * orientation;
}

void
QmAxialHS::write( std::ostream& os )
{
	os << (*this);
}

std::ostream&
operator<<( std::ostream& os, QmAxialHS const& h )
{
    os << "#A( " << h.getOrientation() << " , "
    		   << h.getAxis() << " , "
    		   << h.getAngleBound() << " )";
    return os;
}

std::istream&
operator>>( std::istream& is, QmAxialHS& h )
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

