
#include "pm.h"

using namespace jhm;

void
PmPosture::setScale( m_real s )
{
	scale = s;
	mask = mask | PM_MASK_SCALE;
}

m_real
PmPosture::getScale() const
{
	return scale;
}

void
PmPosture::setTranslation( vector const& v )
{
	trans = v;
	mask = mask | MaskBit(PmHuman::PELVIS);
}

void
PmPosture::addTranslation( vector const& d )
{
	trans += d;
}

vector
PmPosture::getTranslation() const
{
	return trans;
}

void
PmPosture::addRotation( int i, vector const& d )
{
	assert( i>=0 && i<PM_HUMAN_NUM_LINKS );

	rotate[i] = rotate[i] * exp(d);
}

void
PmPosture::setRotation( int i, quater const& q )
{
	assert( i>=0 && i<PM_HUMAN_NUM_LINKS );

	rotate[i] = q;
	mask = mask | MaskBit(i);
}

void
PmPosture::setRotation( int i, vector const& v )
{
	assert( i>=0 && i<PM_HUMAN_NUM_LINKS );

	rotate[i] = EulerAngle2Quater(v);
	mask = mask | MaskBit(i);
}

void
PmPosture::setRotation( int i, matrix const& m )
{
	assert( i>=0 && i<PM_HUMAN_NUM_LINKS );

	rotate[i] = Matrix2Quater( m );
	mask = mask | MaskBit(i);
}

quater
PmPosture::getRotation( int i ) const
{
	assert( i>=0 && i<PM_HUMAN_NUM_LINKS );

	return rotate[i];
}

void
PmPosture::setTransq( int i, transq const& t )
{
	assert( i>=0 && i<PM_HUMAN_NUM_LINKS );

	if ( i==PmHuman::PELVIS ) trans = t.translation;

	rotate[i] = t.rotation;

	mask = mask | MaskBit(i);
}

transq
PmPosture::getTransq( int i ) const
{
	assert( i>=0 && i<PM_HUMAN_NUM_LINKS );

	if ( i==PmHuman::PELVIS )
		return transq( rotate[i], trans );
	else
		return transq( rotate[i], vector(0,0,0) );
}

void
PmPosture::setTransf( int i, transf const& t )
{
	assert( i>=0 && i<PM_HUMAN_NUM_LINKS );

	if ( i==PmHuman::PELVIS ) trans = t.translation();

	rotate[i] = Matrix2Quater( t.affine() );

	mask = mask | MaskBit(i);
}

transf
PmPosture::getTransf( int i ) const
{
	assert( i>=0 && i<PM_HUMAN_NUM_LINKS );

	if ( i==PmHuman::PELVIS )
		return transf( Quater2Matrix(rotate[i]), trans );
	else
		return transf( Quater2Matrix(rotate[i]), vector(0,0,0) );
}

PmPosture&
PmPosture::interpolate( m_real d, PmPosture const& v1,
								  PmPosture const& v2 )
{
    PmMaskType mask1 = v1.getMask();
    PmMaskType mask2 = v2.getMask();

	if ( mask1 & mask2 & PM_MASK_SCALE )
		this->scale = (1.0 - d)*v1.scale + d*v2.scale;

	if ( mask1 & mask2 & MaskBit(PmHuman::PELVIS) )
		setTranslation( ::interpolate(d, v1.trans, v2.trans ) );

   	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
		if ( mask1 & mask2 & MaskBit(i) )
			setRotation( i, ::interpolate(d, v1.rotate[i], v2.rotate[i]) );

	return (*this);
}

PmPosture&
PmPosture::copyOver( PmPosture const& v )
{
	if ( v.mask & PM_MASK_SCALE ) this->scale = v.scale;

	if ( v.mask & MaskBit(PmHuman::PELVIS) ) this->trans = v.trans;

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
		if ( v.mask & MaskBit(i) ) this->rotate[i] = v.rotate[i];

	this->mask |= v.mask;

	return (*this);
}

PmPosture&
PmPosture::copyUnder( PmPosture const& v )
{
	if ( v.mask & PM_MASK_SCALE & this->mask )
		this->scale = v.scale;

	if ( v.mask & MaskBit(PmHuman::PELVIS) & this->mask )
		this->trans = v.trans;

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
		if ( v.mask & MaskBit(i) & this->mask )
			this->rotate[i] = v.rotate[i];

	return (*this);
}

PmPosture&
PmPosture::blend( m_real t, PmPosture const& v )
{
	if ( v.mask & !PM_MASK_SCALE )
		this->scale = v.scale;
	else if ( v.mask & PM_MASK_SCALE )
		this->scale = (1.0 - t)*this->scale + t*v.scale;

	if ( v.mask & !MaskBit(PmHuman::PELVIS) )
		this->trans = v.trans;
	else if ( v.mask & MaskBit(PmHuman::PELVIS) )
		this->trans = ::interpolate(t, this->trans, v.trans);

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	{
		if ( v.mask & !MaskBit(i) )
			this->rotate[i] = v.rotate[i];
		else if ( v.mask & MaskBit(i) )
		{
			if ( this->rotate[i] % v.rotate[i] > 0 )
				this->rotate[i] = ::interpolate(t, this->rotate[i], v.rotate[i]);
			else
				this->rotate[i] = ::interpolate(t, this->rotate[i], -v.rotate[i]);
		}
	}

	this->mask |= v.mask;

	return (*this);
}

transf
PmPosture::getGlobalTransf( int i ) const
{
    transf t = getTransf(i);

    while( body->getParent(i) != -1 )
    {
		m_real s = getScale();
        t *= scale_transf(s,s,s) * body->getJointTransf(i);
//        t *= translate_transf( getScale() * body->getJointPosition(i) );
        i  = body->getParent(i);
		t *= getTransf(i);
    }

    return t;
}

transf
PmPosture::getBaseTransf( int i ) const
{
    transf t = identity_transf;

    while( body->getParent(i) != -1 )
    {
		m_real s = getScale();
        t *= scale_transf(s,s,s) * body->getJointTransf(i);
//        t *= translate_transf( getScale() * body->getJointPosition(i) );
        i  = body->getParent(i);
		t *= getTransf(i);
    }

    return t;
}

vector
PmPosture::getGlobalTranslation( int i ) const
{
    transf t = getTransf(i);

    while( body->getParent(i) != -1 )
    {
		m_real s = getScale();
        t *= scale_transf(s,s,s) * body->getJointTransf(i);
        i  = body->getParent(i);
		t *= getTransf(i);
    }

    return t.translation();
}

position
PmPosture::getGlobalPosition( int i ) const
{
    transf t = getTransf(i);

    while( body->getParent(i) != -1 )
    {
		m_real s = getScale();
        t *= scale_transf(s,s,s) * body->getJointTransf(i);
        i  = body->getParent(i);
		t *= getTransf(i);
    }

    return t.getPosition();
}

quater
PmPosture::getGlobalRotation( int i ) const
{
    transf t = getTransf(i);

    while( body->getParent(i) != -1 )
    {
		m_real s = getScale();
        t *= scale_transf(s,s,s) * body->getJointTransf(i);
        i  = body->getParent(i);
		t *= getTransf(i);
    }

    return Matrix2Quater(t.affine());
}

//----------------------------------------------------------------------------//

PmPosture&
PmPosture::operator=(  PmPosture const& v )
{
	this->body = v.body;
	this->mask = v.mask;
	this->scale = v.scale;
	this->trans = v.trans;
    for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
        this->rotate[i] = v.rotate[i];

    return (*this);
}

PmPosture&
PmPosture::operator+=(  PmVector const& v )
{
#ifdef __EULER_ANGLES__
	
	if ( this->mask & v.getMask() & MaskBit(PmHuman::PELVIS) )
		setTranslation( getTranslation() + v.getLinearVector() );
	
    for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
    if ( this->mask & v.getMask() & MaskBit(i) )
	{
		setRotation( i, Quater2EulerAngle(getRotation(i)) +
						v.getAngularVector(i) );
	}

#else

	if ( this->mask & v.getMask() & MaskBit(PmHuman::PELVIS) )
	{
		transq t = transq( exp(v.getAngularVector(0)),
						       v.getLinearVector() );
		this->setTransq( 0, getTransq(0) * t );
	};
	
    for( int i=1; i<PM_HUMAN_NUM_LINKS; i++ )
    if ( this->mask & v.getMask() & MaskBit(i) )
    	this->rotate[i] = this->rotate[i] * exp(v.getAngularVector(i));

#endif

    return (*this);
}

void
PmPosture::addDisplacement(  PmVector const& v, m_real f )
{
#ifdef __EULER_ANGLES__
	
	if ( this->mask & v.getMask() & MaskBit(PmHuman::PELVIS) )
		setTranslation( getTranslation() + f * v.getLinearVector() );
	
    for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
    if ( this->mask & v.getMask() & MaskBit(i) )
	{
		setRotation( i, Quater2EulerAngle(getRotation(i)) +
						f * v.getAngularVector(i) );
	}

#else

	if ( this->mask & v.getMask() & MaskBit(PmHuman::PELVIS) )
	{
		transq t = transq( exp(f * v.getAngularVector(0)),
							   f * v.getLinearVector() );
		this->setTransq( 0, getTransq(0) * t );
	};
	
    for( int i=1; i<PM_HUMAN_NUM_LINKS; i++ )
    if ( this->mask & v.getMask() & MaskBit(i) )
    	this->rotate[i] = this->rotate[i] * exp(f * v.getAngularVector(i));

#endif
}

//----------------------------------------------------------------------------//

void
PmPosture::quater2euler()
{
	vector v;

	rotate[0] = EulerAngle2Quater( Quater2EulerAngle( rotate[0] ) );
    for( int i=1; i<PM_HUMAN_NUM_LINKS; i++ )
    if ( this->mask & MaskBit(i) )
	{
		v = Quater2EulerAngle( getRotation(i) );
		setRotation( i, quater(0,v[0],v[1],v[2]) );
	}
}

void
PmPosture::euler2quater()
{
	quater q;
	vector v;

    for( int i=1; i<PM_HUMAN_NUM_LINKS; i++ )
    if ( this->mask & MaskBit(i) )
	{
		q = getRotation(i);
		v = vector( q.x(), q.y(), q.z() );
		setRotation( i, EulerAngle2Quater(v) );
	}
}

//----------------------------------------------------------------------------//

PmPosture
PmPosture::posture_stand()
{
	PmPosture h;

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
        h.rotate[i] = quater( 1,0,0,0 );

    h.rotate[PmHuman::LOWER_RIGHT_ARM] = exp(-M_PI/48 * x_axis );
    h.rotate[PmHuman::LOWER_LEFT_ARM]  = exp(-M_PI/48 * x_axis );

	h.rotate[PmHuman::UPPER_RIGHT_LEG] = exp(-M_PI/64 * x_axis );
	h.rotate[PmHuman::UPPER_LEFT_LEG]  = exp(-M_PI/64 * x_axis );

	h.rotate[PmHuman::LOWER_RIGHT_LEG] = exp(M_PI/48 * x_axis );
	h.rotate[PmHuman::LOWER_LEFT_LEG]  = exp(M_PI/48 * x_axis );

    h.copyOver( posture_hand_relax_right() );
    h.copyOver( posture_hand_relax_left() );


//	h.setMask( PM_MASK_ALL_JOINT );
	return h;
}

PmPosture
PmPosture::posture_stand( vector const& v )
{
	PmPosture h;

    h.trans = v;
    for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
        h.rotate[i] = quater( 1,0,0,0 );

    h.rotate[PmHuman::LOWER_RIGHT_ARM] = exp(-M_PI/48 * x_axis );
    h.rotate[PmHuman::LOWER_LEFT_ARM]  = exp(-M_PI/48 * x_axis );

    h.copyOver( posture_hand_relax_right() );
    h.copyOver( posture_hand_relax_left() );

//	h.setMask( PM_MASK_ALL_JOINT );
	return h;
}

PmPosture
PmPosture::posture_hand_fist_right()
{
	PmPosture h;

    h.rotate[PmHuman::RIGHT_PALM]      = quater(1,0,0,0);
    h.rotate[PmHuman::RIGHT_FINGER_11] = Matrix2Quater( EulerAngle2Matrix(vector(-1.41437,-0.865949,2.30966)) );
    h.rotate[PmHuman::RIGHT_FINGER_12] = exp( -1.01914/2 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_13] = exp( -1.09799/2 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_21] = exp( M_PI/4 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_22] = exp( M_PI/4 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_23] = exp( M_PI/4 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_31] = exp( M_PI/4 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_32] = exp( M_PI/4 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_33] = exp( M_PI/4 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_41] = exp( M_PI/4 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_42] = exp( M_PI/4 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_43] = exp( M_PI/4 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_51] = exp( M_PI/4 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_52] = exp( M_PI/4 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_53] = exp( M_PI/4 * z_axis );

	h.setMask( PM_MASK_RIGHT_HAND );
	return h;
}

PmPosture
PmPosture::posture_hand_fist_left()
{
	PmPosture h;

    h.rotate[PmHuman::LEFT_PALM]      = quater(1,0,0,0);
    h.rotate[PmHuman::LEFT_FINGER_11] = Matrix2Quater( EulerAngle2Matrix(vector(-1.41437,0.865949,-2.30966)) );
    h.rotate[PmHuman::LEFT_FINGER_12] = exp( 1.01914/2 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_13] = exp( 1.09799/2 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_21] = exp( -M_PI/4 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_22] = exp( -M_PI/4 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_23] = exp( -M_PI/4 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_31] = exp( -M_PI/4 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_32] = exp( -M_PI/4 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_33] = exp( -M_PI/4 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_41] = exp( -M_PI/4 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_42] = exp( -M_PI/4 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_43] = exp( -M_PI/4 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_51] = exp( -M_PI/4 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_52] = exp( -M_PI/4 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_53] = exp( -M_PI/4 * z_axis );

	h.setMask( PM_MASK_LEFT_HAND );
	return h;
}

PmPosture
PmPosture::posture_hand_relax_right()
{
	PmPosture h;

    h.rotate[PmHuman::RIGHT_PALM]      = quater(1,0,0,0);
    h.rotate[PmHuman::RIGHT_FINGER_11] = Matrix2Quater( EulerAngle2Matrix(vector(-1.637,-0.911,1.962)) );
    h.rotate[PmHuman::RIGHT_FINGER_12] = exp(-M_PI/12 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_13] = exp(-M_PI/12 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_21] = exp( M_PI/12 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_22] = exp( M_PI/6  * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_23] = exp( M_PI/12 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_31] = exp( M_PI/12 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_32] = exp( M_PI/5  * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_33] = exp( M_PI/12 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_41] = exp( M_PI/10 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_42] = exp( M_PI/5  * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_43] = exp( M_PI/12 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_51] = exp( M_PI/8  * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_52] = exp( M_PI/4  * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_53] = exp( M_PI/12 * z_axis );

	h.setMask( PM_MASK_RIGHT_HAND );
	return h;
}

PmPosture
PmPosture::posture_hand_relax_left()
{
	PmPosture h;

    h.rotate[PmHuman::LEFT_PALM]      = quater(1,0,0,0);
    h.rotate[PmHuman::LEFT_FINGER_11] = Matrix2Quater( EulerAngle2Matrix(vector(-1.637,0.911,-1.962)) );
    h.rotate[PmHuman::LEFT_FINGER_12] = exp( M_PI/12 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_13] = exp( M_PI/12 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_21] = exp(-M_PI/12 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_22] = exp(-M_PI/6  * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_23] = exp(-M_PI/12 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_31] = exp(-M_PI/12 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_32] = exp(-M_PI/5  * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_33] = exp(-M_PI/12 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_41] = exp(-M_PI/10 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_42] = exp(-M_PI/5  * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_43] = exp(-M_PI/12 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_51] = exp(-M_PI/8  * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_52] = exp(-M_PI/4  * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_53] = exp(-M_PI/12 * z_axis );

	h.setMask( PM_MASK_LEFT_HAND );
	return h;
}

PmPosture
PmPosture::posture_hand_stretch_right()
{
	PmPosture h;

    h.rotate[PmHuman::RIGHT_PALM]      = quater(1,0,0,0);
    h.rotate[PmHuman::RIGHT_FINGER_11] = Matrix2Quater( EulerAngle2Matrix(vector(-1.6392,-0.729895,2.00465)) );
    h.rotate[PmHuman::RIGHT_FINGER_12] = exp(-M_PI/24 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_13] = exp(-M_PI/24 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_21] = quater(1,0,0,0);
    h.rotate[PmHuman::RIGHT_FINGER_22] = exp( M_PI/24 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_23] = exp( M_PI/24 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_31] = quater(1,0,0,0);
    h.rotate[PmHuman::RIGHT_FINGER_32] = exp( M_PI/24 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_33] = exp( M_PI/24 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_41] = quater(1,0,0,0);
    h.rotate[PmHuman::RIGHT_FINGER_42] = exp( M_PI/24 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_43] = exp( M_PI/24 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_51] = quater(1,0,0,0);
    h.rotate[PmHuman::RIGHT_FINGER_52] = exp( M_PI/24 * z_axis );
    h.rotate[PmHuman::RIGHT_FINGER_53] = exp( M_PI/24 * z_axis );

	h.setMask( PM_MASK_RIGHT_HAND );
	return h;
}

PmPosture
PmPosture::posture_hand_stretch_left()
{
	PmPosture h;

    h.rotate[PmHuman::LEFT_PALM]      = quater(1,0,0,0);
    h.rotate[PmHuman::LEFT_FINGER_11] = Matrix2Quater( EulerAngle2Matrix(vector(-1.6392,0.729895,-2.00465)) );
    h.rotate[PmHuman::LEFT_FINGER_12] = exp( M_PI/24 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_13] = exp( M_PI/24 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_21] = quater(1,0,0,0);
    h.rotate[PmHuman::LEFT_FINGER_22] = exp(-M_PI/24 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_23] = exp(-M_PI/24 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_31] = quater(1,0,0,0);
    h.rotate[PmHuman::LEFT_FINGER_32] = exp(-M_PI/24 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_33] = exp(-M_PI/24 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_41] = quater(1,0,0,0);
    h.rotate[PmHuman::LEFT_FINGER_42] = exp(-M_PI/24 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_43] = exp(-M_PI/24 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_51] = quater(1,0,0,0);
    h.rotate[PmHuman::LEFT_FINGER_52] = exp(-M_PI/24 * z_axis );
    h.rotate[PmHuman::LEFT_FINGER_53] = exp(-M_PI/24 * z_axis );

	h.setMask( PM_MASK_LEFT_HAND );
	return h;
}

//----------------------------------------------------------------------------//

vector
PmPosture::getRollingAxis( int BASE, int UPPER, int LOWER, int END )
{
	transf reference = getGlobalTransf( BASE ).inverse();

	vector end   = (getGlobalTransf(END)   * reference).translation();
	vector upper = (getGlobalTransf(UPPER) * reference).translation();

	return upper - end;
}

PmPosture&
PmPosture::applyTransf( transf const& t )
{
	setTransf( 0, getTransf(0) * t );

	return (*this);
}

//----------------------------------------------------------------------------//

int 
PmPosture::getNumSpineLinks() const
{
	if ( body ) return body->getNumSpineLinks();
	return 0;
}

int 
PmPosture::getDOF( int joint ) const
{
	return GetDOF( joint );
}

int 
PmPosture::getDOF() const
{
	return GetDOF( this->getMask() );
}

void
PmPosture::addDisplacement( vectorN const& disp )
{
	int i, j = 0;
	vector p, v;

	PmMaskType mask = getMask();
	for( i=0; i<PM_HUMAN_NUM_LINKS; i++ )
		if ( mask & MaskBit(i) )
		{
			if ( getDOF(i)==6 )
			{
				p = vector( disp[j  ], disp[j+1], disp[j+2] );
				v = vector( disp[j+3], disp[j+4], disp[j+5] );
				setRotation( i, getRotation(i) * exp(v) );
				setTranslation( getTranslation() + p );
				j += 6;
			}
			else if ( getDOF(i)==3 )
			{
				v = vector( disp[j], disp[j+1], disp[j+2] );
				setRotation( i, getRotation(i) * exp(v) );
				j += 3;
			}
			else if ( getDOF(i)==1 )
			{
				v = vector( disp[j], 0, 0 );
				setRotation( i, getRotation(i) * exp(v) );
				j ++;
			}
		}

	if ( mask & MaskBit(PmHuman::CHEST) )
	{
		quater q = getRotation( PmHuman::CHEST );

		for( i=0; i<getNumSpineLinks(); i++ )
        	setRotation( PmHuman::CHEST-i-1, q );
	}
}



//----------------------------------------------------------------------------//
//							Physical Properties
//----------------------------------------------------------------------------//

vector
PmPosture::getCOG( int i ) const
{
	return getGlobalTranslation(i);
}

vector
PmPosture::getCOG() const
{
	assert( getBody() );

	m_real	mass = 0.0;
	vector	center(0,0,0);

	PmMaskType mask = getMask();

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
		if ( mask & MaskBit(i) )
		{
			center += getBody()->getMass(i) * getCOG(i);
			mass   += getBody()->getMass(i);
		}

	return center / mass;
}

void
PmPosture::getCOGlist( m_real* mass, vector* cog ) const
{
	assert( getBody() );
	PmMaskType mask = getMask();

	int i;
	for( i=0; i<PM_HUMAN_NUM_LINKS; i++ )
		if ( mask & MaskBit(i) )
		{
			mass[i] = getBody()->getMass(i);
			cog[i]  = mass[i] * getCOG(i);
		}

	for( i=PM_HUMAN_NUM_LINKS-1; i>=0; i-- )
		if ( mask & MaskBit(i) )
		{
			int parent = getBody()->getParent( i );

			if ( parent != -1 )
			{
				mass[parent] += mass[i];
				cog[parent]  += cog[i];
			}
		}

	for( i=0; i<PM_HUMAN_NUM_LINKS; i++ )
		if ( mask & MaskBit(i) )
		{
			cog[i] /= mass[i];
		}
}

void
PmPosture::getSupportPolygon( PmConstraint const& c, QmPolygon2D& sp ) const
{
	assert( getBody() );

	int		 pn=0;
	position points[30];

	/*
	//------------------------------------------------
	//   Viewpoint model
	transf	 t;
	m_real h = -getBody()->getAnkleHeight();

	if ( c.isConstrained( PmHuman::RIGHT_FOOT ) )
	{
		t = getGlobalTransf( PmHuman::RIGHT_FOOT );
		points[pn++] = position( 1.5,h, 7) * t;
		points[pn++] = position( 1.5,h,-1) * t;
		points[pn++] = position(-1.5,h,-1) * t;
		points[pn++] = position(-1.5,h, 7) * t;
	}

	if ( c.isConstrained( PmHuman::LEFT_FOOT ) )
	{
		t = getGlobalTransf( PmHuman::LEFT_FOOT );
		points[pn++] = position( 1.5,h, 7) * t;
		points[pn++] = position( 1.5,h,-1) * t;
		points[pn++] = position(-1.5,h,-1) * t;
		points[pn++] = position(-1.5,h, 7) * t;
	}
	//   Viewpoint model
	//------------------------------------------------
	*/

	//------------------------------------------------
	//   ASF/AMC model
	position	p1, p2;
	m_real		d = 3.0;

	if ( c.isConstrained( PmHuman::RIGHT_FOOT ) )
	{
		p1 = vector2position( getGlobalTranslation( PmHuman::RIGHT_FOOT ) );
		p2 = vector2position( getGlobalTranslation( PmHuman::RIGHT_TOE ) );

		points[pn++] = p1 + vector( d,0, d);
		points[pn++] = p1 + vector(-d,0, d);
		points[pn++] = p1 + vector( d,0,-d);
		points[pn++] = p1 + vector(-d,0,-d);

		points[pn++] = p2 + vector( d,0, d);
		points[pn++] = p2 + vector(-d,0, d);
		points[pn++] = p2 + vector( d,0,-d);
		points[pn++] = p2 + vector(-d,0,-d);
	}

	if ( c.isConstrained( PmHuman::LEFT_FOOT ) )
	{
		p1 = vector2position( getGlobalTranslation( PmHuman::LEFT_FOOT ) );
		p2 = vector2position( getGlobalTranslation( PmHuman::LEFT_TOE ) );

		points[pn++] = p1 + vector( d,0, d);
		points[pn++] = p1 + vector(-d,0, d);
		points[pn++] = p1 + vector( d,0,-d);
		points[pn++] = p1 + vector(-d,0,-d);

		points[pn++] = p2 + vector( d,0, d);
		points[pn++] = p2 + vector(-d,0, d);
		points[pn++] = p2 + vector( d,0,-d);
		points[pn++] = p2 + vector(-d,0,-d);
	}
	//   ASF/AMC model
	//------------------------------------------------

	sp.convexHull( pn, points );
}
