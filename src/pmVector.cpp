
#include "pm.h"

using namespace jhm;

void
PmVector::reset( PmMaskType m, vector const& v )
{
	setLinearVector( v );
	for( int i=1; i<PM_HUMAN_NUM_LINKS; i++ )
		setAngularVector( i, v );

	setMask( m );
}

void
PmVector::setTransq( int i, transq const& t )
{
    assert( i>=0 && i<PM_HUMAN_NUM_LINKS );

    if ( i==PmHuman::PELVIS ) linear = t.translation;

    angular[i] = ln(t.rotation);

    mask = mask | MaskBit(i);
}

transq
PmVector::getTransq( int i ) const
{
    assert( i>=0 && i<PM_HUMAN_NUM_LINKS );

    if ( i==PmHuman::PELVIS )
        return transq( exp(angular[i]), linear );
    else
        return transq( exp(angular[i]), vector(0,0,0) );
}

void
PmVector::getDisplacement( vectorN &d ) const
{
	vector v;

	for( int i=0, j=0; i<PM_HUMAN_NUM_LINKS; i++ )
		if ( getMask() & MaskBit(i) )
		{
			if ( getDOF(i)==6 )
			{
				v = getLinearVector();
				d[j++] = v[0];
				d[j++] = v[1];
				d[j++] = v[2];

				v = getAngularVector( i );
				d[j++] = v[0];
				d[j++] = v[1];
				d[j++] = v[2];
			}
			else if ( getDOF(i)==3 )
			{
				v = getAngularVector( i );
				d[j++] = v[0];
				d[j++] = v[1];
				d[j++] = v[2];
			}
			else if ( getDOF(i)==1 )
			{
				v = getAngularVector( i );
				d[j++] = v[0];
			}
		}
}

void
PmVector::setDisplacement( vectorN const& d )
{
	for( int i=0, j=0; i<PM_HUMAN_NUM_LINKS; i++ )
		if ( getMask() & MaskBit(i) )
		{
			if ( getDOF(i)==6 )
			{
				setLinearVector(     vector( d[j++], d[j++], d[j++] ) );
				setAngularVector( i, vector( d[j++], d[j++], d[j++] ) );
			}
			else if ( getDOF(i)==3 )
			{
				setAngularVector( i, vector( d[j++], d[j++], d[j++] ) );
			}
			else if ( getDOF(i)==1 )
			{
				setAngularVector( i, vector( d[j++], 0, 0 ) );
			}
		}
}

void
PmVector::addDisplacement( vectorN const& d )
{
	for( int i=0, j=0; i<PM_HUMAN_NUM_LINKS; i++ )
		if ( getMask() & MaskBit(i) )
		{
			if ( getDOF(i)==6 )
			{
				addLinearVector(     vector( d[j++], d[j++], d[j++] ) );
				addAngularVector( i, vector( d[j++], d[j++], d[j++] ) );
			}
			else if ( getDOF(i)==3 )
			{
				addAngularVector( i, vector( d[j++], d[j++], d[j++] ) );
			}
			else if ( getDOF(i)==1 )
			{
				addAngularVector( i, vector( d[j++], 0, 0 ) );
			}
		}
}

vector
PmVector::addLinearVector( vector const& v )
{
	linear += v;
	mask = mask | MaskBit(0);

	return v;
}

vector
PmVector::setLinearVector( vector const& v )
{
	linear = v;
	mask = mask | MaskBit(0);

	return v;
}

vector
PmVector::getLinearVector() const
{
	return linear;
}

vector
PmVector::addAngularVector( int i, vector const& v )
{
	assert( i>=0 && i<PM_HUMAN_NUM_LINKS );

	angular[i] += v;
	mask = mask | MaskBit(i);

	return v;
}

vector
PmVector::setAngularVector( int i, vector const& v )
{
	assert( i>=0 && i<PM_HUMAN_NUM_LINKS );

	angular[i] = v;
	mask = mask | MaskBit(i);

	return v;
}

vector
PmVector::getAngularVector( int i ) const
{
	assert( i>=0 && i<PM_HUMAN_NUM_LINKS );
	return angular[i];
}

PmVector&
PmVector::operator=(  PmVector const& v )
{
	this->mask = v.mask;
	this->linear = v.linear;
	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
    	this->angular[i] = v.angular[i];

	return (*this);
}

PmVector&
PmVector::operator+=( PmVector const& v )
{
	PmMaskType m = this->mask & v.getMask();

	if ( m & MaskBit(PmHuman::PELVIS) )
	{
		transq t = transq( exp(this->getAngularVector(0)),
						   this->getLinearVector() ) *
				   transq( exp(v.getAngularVector(0)),
						    v.getLinearVector() );
		this->linear = t.translation;
		this->angular[0] = ln(t.rotation);
	}

	for( int i=1; i<PM_HUMAN_NUM_LINKS; i++ )
	if ( m & MaskBit(i) )
	{
		this->angular[i] = ln( exp(this->getAngularVector(i)) *
							   exp(v.getAngularVector(i)) );
	}
	
	return (*this);
}


PmVector&
PmVector::operator*=( m_real d )
{
	this->linear *= d;
	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
    	this->angular[i] *= d;

	return (*this);
}

PmVector&
PmVector::operator/=( m_real d )
{
	this->linear /= d;
	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
    	this->angular[i] /= d;

	return (*this);
}

PmVector&
PmVector::positionDifference( PmPosture const& p1, PmPosture const& p2 )
{
	this->mask = p1.getMask() & p2.getMask();

	transf t1 = PlaneProject( p1.getTransf(0) );
	transf t2 = PlaneProject( p2.getTransf(0) );

	transf calib = t2.inverse() * t1;

	this->setLinearVector( vector(0,0,0) );

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	if ( this->getMask() & MaskBit(i) )
	{
		position a1 = (position(0,0,0) + p1.getGlobalTranslation(i));
		position a2 = (position(0,0,0) + p2.getGlobalTranslation(i)) * calib;

		this->setAngularVector( i, a2 - a1 );
	}

	return (*this);
}

PmVector&
PmVector::difference( PmPosture const& p1, PmPosture const& p2 )
{
	this->mask = p1.getMask() & p2.getMask();

#ifdef __EULER_ANGLES__

	if ( this->getMask() & MaskBit(PmHuman::PELVIS) )
		setLinearVector( p1.getTranslation() - p2.getTranslation() );

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	if ( this->getMask() & MaskBit(i) )
	{
		vector v = Quater2EulerAngle(p1.getRotation(i)) -
				   Quater2EulerAngle(p2.getRotation(i));
		setAngularVector( i, v );
	}

#else

	if ( this->getMask() & MaskBit(PmHuman::PELVIS) )
	{
		transq q1 = p1.getTransq(0);
		transq q2 = p2.getTransq(0);

		if ( q1.rotation % q2.rotation < 0 )
			q2.rotation = -q2.rotation;

		transq t = q2.inverse() * q1;

		this->setLinearVector( t.translation );
		this->setAngularVector( 0, ln(t.rotation) );
	}

	for( int i=1; i<PM_HUMAN_NUM_LINKS; i++ )
	if ( this->getMask() & MaskBit(i) )
	{
		if ( p2.getRotation(i) % p1.getRotation(i) < 0 )
			this->setAngularVector( i,
				ln( p2.getRotation(i).inverse() * (-p1.getRotation(i)) ) );
		else
			this->setAngularVector( i,
				ln( p2.getRotation(i).inverse() * p1.getRotation(i) ) );
	}

#endif
	
	return (*this);
}

PmVector&
PmVector::difference( PmVector const& v1, PmVector const& v2 )
{
	this->mask = v1.getMask() & v2.getMask();

	if ( this->getMask() & MaskBit(PmHuman::PELVIS) )
	{
		this->setLinearVector(
			v1.getLinearVector() - v2.getLinearVector() );
	}

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	if ( this->getMask() & MaskBit(i) )
	{
		this->setAngularVector( i,
			v1.getAngularVector(i) - v2.getAngularVector(i) );
	}
	
	return (*this);
}

m_real
PmVector::magnitude()
{
	m_real m = 0.0;

	PmMaskType mask = getMask();

	if ( mask & MaskBit(PmHuman::PELVIS) )
		m += getLinearVector().length();

	for ( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
		if ( mask & MaskBit(i) )
			m += getAngularVector(i).length();

	return m;
}

m_real
PmVector::squaredMagnitude()
{
	m_real m = 0.0;
	m_real t;

	PmMaskType mask = getMask();

	if ( mask & MaskBit(PmHuman::PELVIS) )
	{
		t = getLinearVector().length();
		m += t*t;
	}

	for ( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
		if ( mask & MaskBit(i) )
		{
			t = getAngularVector(i).length();
			m += t*t;
		}

	return m;
}

int
PmVector::getDOF( int joint ) const
{
    return GetDOF( joint );
}

int
PmVector::getDOF() const
{
    return GetDOF( this->getMask() );
}

PmVector&
PmVector::interpolate( m_real d, PmVector const& v1,
                                 PmVector const& v2 )
{
    PmMaskType mask1 = v1.getMask();
    PmMaskType mask2 = v2.getMask();

    if ( mask1 & mask2 & MaskBit(PmHuman::PELVIS) )
        setLinearVector( ::interpolate(d, v1.linear, v2.linear ) );

    for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
        if ( mask1 & mask2 & MaskBit(i) )
            setAngularVector( i, ::interpolate(d, v1.angular[i], v2.angular[i]) );

	return (*this);
}

