
#include "pm.h"
#include "qmApproximate.h"

using namespace jhm;


PmLinearMotion::PmLinearMotion() : PmMotion()
{
	postures = NULL;
	knots    = NULL;
}

PmLinearMotion::PmLinearMotion( PmHuman* h ) : PmMotion()
{
	postures = NULL;
	knots    = NULL;
	setBody( h );
}

PmLinearMotion::~PmLinearMotion()
{
	if ( oSize>0 )
	{
		delete[] postures;
		delete[] knots;
	}
}

int
PmLinearMotion::getDOF() const
{
    int dof = 0;

    for( int i=0; i<getSize(); i++ )
        dof += getPosture(i).getDOF();

    return dof;
}

PmMaskType
PmLinearMotion::getMask() const
{
    PmMaskType mask = 0x00;
    for( int i=0; i<getSize(); i++ )
        mask |= getPosture(i).getMask();

    return mask;
}

void
PmLinearMotion::setBody( PmHuman* h )
{
    PmMotion::setBody( h );

    for( int i=0; i<getSize(); i++ )
        postures[i].setBody( h );
}

void
PmLinearMotion::setSize( int n )
{
	assert( n>=0 );

	if ( oSize < n )
	{
		if ( oSize>0 )
		{
			delete[] postures;
			delete[] knots;
		}

		postures = new PmPosture[n];
		knots    = new m_real[n];
	}

	for( int i=0; i<n; i++ ) knots[i] = i;

	PmMotion::setSize( n );
}

void
PmLinearMotion::extend( int n )
{
	if ( n>0 && n>oSize )
	{
	    PmPosture *temp_p = new PmPosture[n];
		m_real    *temp_k = new m_real[n];

		int i;
		for( i=0; i<n; i++ ) temp_k[i] = i;

		for( i=0; i<getSize() && i<n; i++ )
		{
			temp_p[i] = postures[i];
			temp_k[i] = knots[i];
		}

		if ( getSize()>0 )
		{
			for( i=getSize(); i<n; i++ )
			{
				temp_p[i] = temp_p[getSize()-1];
				temp_k[i] = temp_k[getSize()-1];
			}

			delete[] postures;
			delete[] knots;
		}

		postures = temp_p;
		knots    = temp_k;

		PmMotion::extend( n );
	}
	else
		setSize( n );
}

void
PmLinearMotion::setScale( m_real s )
{
    for( int i=0; i<getSize(); i++ )
		postures[i].setScale( s );
}

void
PmLinearMotion::getTranslation( vector* p ) const
{
    for( int i=0; i<getSize(); i++ )
        p[i] = getPosture(i).getTranslation();
}

void
PmLinearMotion::getRotation( int link, quater* q ) const
{
    for( int i=0; i<getSize(); i++ )
        q[i] = getPosture(i).getRotation(link);
}

void
PmLinearMotion::setTranslation( vector* p )
{
    for( int i=0; i<getSize(); i++ )
        postures[i].setTranslation( p[i] );
}

void
PmLinearMotion::setRotation( int link, quater* q )
{
    for( int i=0; i<getSize(); i++ )
        postures[i].setRotation( link, q[i] );
}

void
PmLinearMotion::addTranslation( vector* d )
{
    for( int i=0; i<getSize(); i++ )
        postures[i].addTranslation( d[i] );
}

void
PmLinearMotion::addRotation( int link, vector* d )
{
    for( int i=0; i<getSize(); i++ )
        postures[i].addRotation( link, d[i] );
}

void
PmLinearMotion::setPosture( int i, PmPosture const& v )
{
	assert( i>=0 && i<getSize() );

	postures[i] = v;
	postures[i].setBody( this->getBody() );
}

void
PmLinearMotion::getKnots( m_real *k )
{
	for( int i=0; i<getSize(); i++ )
		k[i] = knots[i];
}

void
PmLinearMotion::setKnots( m_real *k )
{
	for( int i=0; i<getSize(); i++ )
		knots[i] = k[i];
}

PmPosture&
PmLinearMotion::getPosture( int i ) const
{
	assert( i>=0 && i<getSize() );

	return postures[i];
}

PmLinearMotion&
PmLinearMotion::operator=(  PmLinearMotion const& m )
{
	(*(PmMotion*)this) = m;
	setSize( m.getSize() );

    for( int i=0; i<this->getSize(); i++ )
	{
        this->postures[i]    = m.postures[i];
		this->knots[i]       = m.knots[i];
	}

    return (*this);
}

PmLinearMotion&
PmLinearMotion::operator+=(  PmVectorArray const& va )
{
	assert( this->getSize() == va.getSize() );

    for( int i=0; i<this->getSize(); i++ )
        this->postures[i] += va.getVector(i);

    return (*this);
}

void
PmLinearMotion::addDisplacement(  PmVectorArray const& va, m_real f )
{
	assert( this->getSize() == va.getSize() );

    for( int i=0; i<this->getSize(); i++ )
        this->postures[i].addDisplacement( va.getVector(i), f );
}

void
PmLinearMotion::crop( PmLinearMotion const& m, int s, int l )
{
	assert( 0<=s && s<m.getSize() );
	assert( 0<l && l<m.getSize()-s+1 );

	setSize( l );
	for( int i=0; i<getSize(); i++ )
	{
		this->postures   [i] = m.postures	[i+s];
		this->constraints[i] = m.constraints[i+s];
		this->knots      [i] = m.knots		[i+s];
	}
}

void
PmLinearMotion::alignment()
{
	quater q1;
	quater q2;

    PmMaskType mask = this->getMask();

   	for( int j=0; j<PM_HUMAN_NUM_LINKS; j++ )
    if ( mask & MaskBit(j) )
	{
		for( int i=1; i<this->getSize(); i++ )
		{
				q1 = postures[i-1].getRotation(j);
				q2 = postures[i].getRotation(j);
				if ( q1%q2 < 0 ) postures[i].setRotation( j, -q2 );
		}
	}
}

void
PmLinearMotion::concat( PmLinearMotion const& m )
{
	int s = this->getSize();
	this->extend( s + m.getSize() );

	for( int i=0; i<m.getSize(); i++ )
	{
		this->postures   [i+s] = m.postures   [i];
		this->constraints[i+s] = m.constraints[i];
		this->knots      [i+s] = m.knots      [i];
	}
}

transf
PmLinearMotion::stitch( PmLinearMotion const& m )
{
	int s = this->getSize();
	if ( s==0 ) { (*this) = m; return identity_transf; }

	m_real k = this->knots[s-1] - m.knots[0];

	//  compute calibration transf
	transf t1 = this->postures[s-1].getGlobalTransf( PmHuman::PELVIS );
	QmGeodesic g1( quater(1,0,0,0), y_axis );
	vector v = t1.getTranslation();
	t1 = transf( g1.nearest(t1.getRotation()), vector(v[0],0,v[2]) );

	transf t2 = m.postures[0].getGlobalTransf( PmHuman::PELVIS );
	QmGeodesic g2( quater(1,0,0,0), y_axis );
	v = t2.getTranslation();
	t2 = transf( g2.nearest(t2.getRotation()), vector(v[0],0,v[2]) );

	transf calib = t2.inverse() * t1;

	//  blend overlapping
	PmPosture p = m.postures[0];
	p.applyTransf( calib );

	this->postures[s-1].interpolate( 0.5, this->postures[s-1], p );

	//  stitch non-overlapping
	this->extend( s + m.getSize() - 1 );

	for( int i=1; i<m.getSize(); i++ )
	{
		this->postures[i+s-1] = m.postures[i];
		this->postures[i+s-1].applyTransf( calib );

		this->constraints[i+s-1] = m.constraints[i];
		this->constraints[i+s-1].applyTransf( calib );

		this->knots[i+s-1] = m.knots[i] + k;
	}

	return calib;
}

transf
PmLinearMotion::stitchWithWarp( PmLinearMotion& m )
{
	int s = this->getSize();
	if ( s==0 ) { (*this) = m; return identity_transf; }

	m_real k = this->knots[s-1] - m.knots[0];

	//  compute calibration transf
	transf t1 = this->postures[s-1].getGlobalTransf( PmHuman::PELVIS );
	QmGeodesic g1( quater(1,0,0,0), y_axis );
	vector v = t1.getTranslation();
	t1 = transf( g1.nearest(t1.getRotation()), vector(v[0],0,v[2]) );

	transf t2 = m.postures[0].getGlobalTransf( PmHuman::PELVIS );
	QmGeodesic g2( quater(1,0,0,0), y_axis );
	v = t2.getTranslation();
	t2 = transf( g2.nearest(t2.getRotation()), vector(v[0],0,v[2]) );

	transf calib = t2.inverse() * t1;

	//  stitch
	m.applyTransf( calib );
	m.directManip( 0, this->postures[s-1], NULL, MIN(10, m.getSize()/2), 1, FALSE, FALSE );

	for( int j=0; j<PM_HUMAN_NUM_LINKS; j++ )
	for( int i=0; i<m.getSize(); i++ )
		if ( m.constraints[i].isConstrained(j) )
			m.constraints[i].setTransf(j, m.getPosture(i).getGlobalTransf(j));

	this->extend( s + m.getSize() - 1 );

	for( int i=1; i<m.getSize(); i++ )
	{
		this->postures[i+s-1] = m.postures[i];
		this->constraints[i+s-1] = m.constraints[i];
		this->knots[i+s-1] = m.knots[i] + k;
	}

	return calib;
}

transf
PmLinearMotion::connect( PmLinearMotion& m )
{
	if ( this->getSize()==0 ) return identity_transf;

	//
	//		move `m' such that its start meets the end of `this'
	//
	vector v1 = this->getPosture(this->getSize()-1).getTranslation();
	vector v2 = this->getPosture(this->getSize()-2).getTranslation();
	vector v3 = m.getPosture(0).getTranslation();

	quater q1 = this->getPosture(this->getSize()-1).getRotation(0);
	quater q2 = this->getPosture(this->getSize()-2).getRotation(0);
	quater q3 = m.getPosture(0).getRotation(0);

	transf t1 = transf( Quater2Matrix(q1), v1 );
	transf t2 = transf( Quater2Matrix(q2), v2 );
	transf t3 = transf( Quater2Matrix(q3), v3 );
	transf calib = t3.inverse() * t1 * t2.inverse() * t1;

	m.applyTransf( calib );

	//
	//		reestablish the knots of `m'
	//
	m_real d = (knots[getSize()-1] - knots[getSize()-2]) +
			   (m.knots[1] - m.knots[0] );
	d = d/2 + knots[getSize()-1] - m.knots[0];

	for( int i=0; i<m.getSize(); i++ )
		m.knots[i] += d;

	return calib;
}

void
PmLinearMotion::applyTransf( const transf& t )
{
	for( int i=0; i<getSize(); i++ )
	{
		postures[i].applyTransf( t );
		constraints[i].applyTransf( t );
	}
}

void
PmLinearMotion::centering()
{
	vector center(0,0,0);

	for( int i=0; i<getSize(); i++ )
		center -= getPosture(i).getTranslation();

	center /= getSize();
	center.set_y( 0 );

	applyTransf( translate_transf(center) );
}

extern QmScatteredData	displacement[PM_MAX_FRAMES];
extern vector			p_buf[PM_MAX_FRAMES];
extern vector			p1_buf[PM_MAX_FRAMES];

void
PmLinearMotion::flatTune( int height )
{
	if ( getSize() < 32 ) return;

	int nknots = getSize() / 32;

	QmApproximate appr;
	appr.setSize( nknots );

	vector	v1, v2;
	int	i, k;

	for( i=0; i<getSize(); i++ )
	{
		PmPosture &p = getPosture(i);
		p.setTranslation( p.getTranslation() - vector(0,10,0) );
	}

	for( k=0; k<5; k++ )
	{
		int count = 0;
		for( i=0; i<getSize(); i++ )
		{
			PmPosture &p = postures[i];
			v1 = p.getGlobalTranslation( PmHuman::RIGHT_FOOT );
			v2 = p.getGlobalTranslation( PmHuman::LEFT_FOOT );

			if ( v1.y()<height || v2.y()<height )
			{
				displacement[count].position = i;
				displacement[count].value = vector(0, height - MIN(v1.y(),v2.y()), 0);
				count++;
			}
		}

		appr.approximate( getSize(), count, displacement );
		appr.discretize( getSize(), p1_buf );

		getTranslation( p_buf );
		for( i=0; i<getSize(); i++ )
			p_buf[i] += p1_buf[i];
		setTranslation( p_buf );
	}

	for( i=0; i<getSize(); i++ )
	{
		for( k=0; k<constraints[i].getNumEntity(); k++ )
		{
			PmConstraintEntity &c = constraints[i].entity[k];
			c.value = postures[i].getGlobalTransf( c.link );
		}
	}
}


//----------------------------------------------------------------------
//			Physical properties
//----------------------------------------------------------------------

vector
PmLinearMotion::getLinearVelocity( int segment, int frame ) const
{
	vector v1, v2;

	if ( frame == 0 )
	{
		v1 = getPosture( frame+1 ).getGlobalTranslation( segment );
		v2 = getPosture( frame   ).getGlobalTranslation( segment );

		return v1 - v2;
	}
	
	if ( frame == getSize()-1 )
	{
		v1 = getPosture( frame   ).getGlobalTranslation( segment );
		v2 = getPosture( frame-1 ).getGlobalTranslation( segment );

		return v1 - v2;
	}

	v1 = getPosture( frame+1 ).getGlobalTranslation( segment );
	v2 = getPosture( frame-1 ).getGlobalTranslation( segment );

	return (v1 - v2)/2.0;
}

vector
PmLinearMotion::getAngularVelocity( int segment, int frame ) const
{
	quater q1, q2;

	if ( frame == 0 )
	{
		q1 = getPosture( frame+1 ).getGlobalRotation( segment );
		q2 = getPosture( frame   ).getGlobalRotation( segment );

		if ( q1%q2<0 ) q2 = -q2;

		return ln( q2.inverse() * q1 );
	}

	if ( frame == getSize()-1 )
	{
		q1 = getPosture( frame   ).getGlobalRotation( segment );
		q2 = getPosture( frame-1 ).getGlobalRotation( segment );

		if ( q1%q2<0 ) q2 = -q2;

		return ln( q2.inverse() * q1 );
	}

	q1 = getPosture( frame+1 ).getGlobalRotation( segment );
	q2 = getPosture( frame-1 ).getGlobalRotation( segment );

	if ( q1%q2<0 ) q2 = -q2;

	return ln( q2.inverse() * q1 );
}

vector
PmLinearMotion::getLinearAcceleration( int segment, int frame ) const
{
	vector v1, v2;

	if ( frame == 0 )
	{
		v1 = getLinearVelocity( frame+1, segment );
		v2 = getLinearVelocity( frame,   segment );

		return v1 - v2;
	}
	
	if ( frame == getSize()-1 )
	{
		v1 = getLinearVelocity( frame,   segment );
		v2 = getLinearVelocity( frame-1, segment );

		return v1 - v2;
	}
	
	v1 = getLinearVelocity( frame+1, segment );
	v2 = getLinearVelocity( frame-1, segment );

	return (v1 - v2)/2.0;
}

vector
PmLinearMotion::getAngularAcceleration( int segment, int frame ) const
{
	vector v1, v2;

	if ( frame == 0 )
	{
		v1 = getAngularVelocity( frame+1, segment );
		v2 = getAngularVelocity( frame,   segment );

		return v1 - v2;
	}
	
	if ( frame == getSize()-1 )
	{
		v1 = getAngularVelocity( frame,   segment );
		v2 = getAngularVelocity( frame-1, segment );

		return v1 - v2;
	}
	
	v1 = getAngularVelocity( frame+1, segment );
	v2 = getAngularVelocity( frame-1, segment );

	return (v1 - v2)/2.0;
}

void
PmLinearMotion::getPositionVelocity( int index, PmVector& v )
{
	if ( index==0  )
	{
		v.positionDifference( getPosture(index+1), getPosture(index) );
	}
	else if ( index==getSize()-1 )
	{
		v.positionDifference( getPosture(index), getPosture(index-1) );
	}
	else
	{
		v.positionDifference( getPosture(index+1), getPosture(index-1) );
		v /= 2.0;
	}
}

void
PmLinearMotion::getVelocity( int index, PmVector& v )
{
	if ( index==0  )
	{
		v.difference( getPosture(index+1), getPosture(index) );
	}
	else if ( index==getSize()-1 )
	{
		v.difference( getPosture(index), getPosture(index-1) );
	}
	else
	{
		v.difference( getPosture(index+1), getPosture(index-1) );
		v /= 2.0;
	}
}

void
PmLinearMotion::getAcceleration( int index, PmVector& v )
{
	PmMaskType mask = getMask();

	if ( index==0 || index==getSize()-1 )
	{
		v.reset( mask, vector(0,0,0) );
	}
	else
	{
		static PmVector v1, v2;
		v1.difference( getPosture(index+1), getPosture(index) );
		v2.difference( getPosture(index), getPosture(index-1) );
		v.difference( v1, v2 );

//		transf t = getPosture(index).getTransf( PmHuman::PELVIS ).inverse();
//		v.setLinearVector( v.getLinearVector() );
//		v.setAngularVector( 0, v.getAngularVector(0) );
	}
}

m_real
PmLinearMotion::maxLinearVelocity( int link )
{
	m_real s, max_s;

	for( int i=0; i<getSize(); i++ )
	{
		s = getLinearVelocity( link, i ).length();

		if ( i==0 || s>max_s ) max_s = s;
	}

	return s;
}

m_real
PmLinearMotion::maxAngularVelocity( int link )
{
	m_real s, max_s;

	for( int i=0; i<getSize(); i++ )
	{
		m_real s = getAngularVelocity( link, i ).length();

		if ( i==0 || s>max_s ) max_s = s;
	}

	return max_s;
}

vector
PmLinearMotion::getZMP( int frame ) const
{
	vector a, p;
	vector g(0, 0.98, 0);

	m_real x1=0, x2=0;
	m_real z1=0, z2=0;
	m_real deno=0;

	PmMaskType mask = getMask();

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	if ( mask & MaskBit(i) )
	{
		PmPosture &posture = getPosture( frame );
		p = posture.getGlobalTranslation( i );
		a = getLinearAcceleration( frame, i );
		m_real mass = posture.getBody()->getMass( i );

		x1 += mass * (a[1] - g[1]) * p[0];
		x2 += mass * (a[0] - g[0]) * p[1];

		z1 += mass * (a[1] - g[1]) * p[2];
		z2 += mass * (a[2] - g[2]) * p[1];

		deno += mass * (a[1] - g[1]);
	}
	
	if ( -EPS_jhm<deno && deno<EPS_jhm ) return vector(0,0,0);

	return vector( (x1-x2)/deno, 0, (z1-z2)/deno );
}

m_real
PmLinearMotion::getKineticEnergy( int frame ) const
{
	m_real	e = 0;
	vector	v;

	PmMaskType mask = getMask();

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	if ( mask & MaskBit(i) )
	{
		v = getLinearVelocity( i, frame );

		e += 0.5 * (v % v);
	}

	return e;
}

void
PmLinearMotion::setPelvisConstraints( m_real speed )
{
	vector	l0, l1, l2;
	vector	r0, r1, r2;
	vector	c0, c1, c2;
	vector	vl, vr, vc;

	for( int i=0; i<getSize(); i++ )
	{
		if ( i==0 )
		{
			l0 = getPosture(i  ).getGlobalTranslation( PmHuman::UPPER_LEFT_LEG );
			r0 = getPosture(i  ).getGlobalTranslation( PmHuman::UPPER_RIGHT_LEG );
			c0 = getPosture(i  ).getGlobalTranslation( PmHuman::SPINE_1 );

			l2 = getPosture(i+1).getGlobalTranslation( PmHuman::UPPER_LEFT_LEG );
			r2 = getPosture(i+1).getGlobalTranslation( PmHuman::UPPER_RIGHT_LEG );
			c2 = getPosture(i+1).getGlobalTranslation( PmHuman::SPINE_1 );

			vl = l2 - l0;
			vr = r2 - r0;
			vc = c2 - c0;
		}
		else if ( i==getSize()-1 )
		{
			l0 = getPosture(i  ).getGlobalTranslation( PmHuman::UPPER_LEFT_LEG );
			r0 = getPosture(i  ).getGlobalTranslation( PmHuman::UPPER_RIGHT_LEG );
			c0 = getPosture(i  ).getGlobalTranslation( PmHuman::SPINE_1 );

			l1 = getPosture(i-1).getGlobalTranslation( PmHuman::UPPER_LEFT_LEG );
			r1 = getPosture(i-1).getGlobalTranslation( PmHuman::UPPER_RIGHT_LEG );
			c1 = getPosture(i-1).getGlobalTranslation( PmHuman::SPINE_1 );

			vl = l0 - l1;
			vr = r0 - r1;
			vc = c0 - c1;
		}
		else
		{
			l1 = getPosture(i-1).getGlobalTranslation( PmHuman::UPPER_LEFT_LEG );
			r1 = getPosture(i-1).getGlobalTranslation( PmHuman::UPPER_RIGHT_LEG );
			c1 = getPosture(i-1).getGlobalTranslation( PmHuman::SPINE_1 );

			l2 = getPosture(i+1).getGlobalTranslation( PmHuman::UPPER_LEFT_LEG );
			r2 = getPosture(i+1).getGlobalTranslation( PmHuman::UPPER_RIGHT_LEG );
			c2 = getPosture(i+1).getGlobalTranslation( PmHuman::SPINE_1 );

			vl = (l2 - l1) / 2;
			vr = (r2 - r1) / 2;
			vc = (c2 - c1) / 2;
		}

		if ( vl.length() < speed && vr.length() < speed && vc.length() < speed )
			constraints[i].push( PmHuman::PELVIS,
				getPosture(i).getGlobalTransf( PmHuman::PELVIS ) );
	}
}


void
PmLinearMotion::setFootConstraints(
		m_real   toe_speed, m_real   toe_height,
		m_real ankle_speed, m_real ankle_height )
{
	vector	al0, al1, al2;
	vector	tl0, tl1, tl2;
	vector	ar0, ar1, ar2;
	vector	tr0, tr1, tr2;
	vector	val, var, vtl, vtr;

	int left_touch  = TRUE;
	int right_touch = TRUE;

	for( int i=0; i<getSize(); i++ )
	{
		al0 = getPosture(i  ).getGlobalTranslation( PmHuman::LEFT_FOOT );
		tl0 = getPosture(i  ).getGlobalTranslation( PmHuman::LEFT_TOE );
		ar0 = getPosture(i  ).getGlobalTranslation( PmHuman::RIGHT_FOOT );
		tr0 = getPosture(i  ).getGlobalTranslation( PmHuman::RIGHT_TOE );

		if ( i==0 )
		{
			al2 = getPosture(i+1).getGlobalTranslation( PmHuman::LEFT_FOOT );
			tl2 = getPosture(i+1).getGlobalTranslation( PmHuman::LEFT_TOE );
			ar2 = getPosture(i+1).getGlobalTranslation( PmHuman::RIGHT_FOOT );
			tr2 = getPosture(i+1).getGlobalTranslation( PmHuman::RIGHT_TOE );

			val = al2 - al0;
			vtl = tl2 - tl0;
			var = ar2 - ar0;
			vtr = tr2 - tr0;
		}
		else if ( i==getSize()-1 )
		{
			al1 = getPosture(i-1).getGlobalTranslation( PmHuman::LEFT_FOOT );
			tl1 = getPosture(i-1).getGlobalTranslation( PmHuman::LEFT_TOE );
			ar1 = getPosture(i-1).getGlobalTranslation( PmHuman::RIGHT_FOOT );
			tr1 = getPosture(i-1).getGlobalTranslation( PmHuman::RIGHT_TOE );

			val = al0 - al1;
			vtl = tl0 - tl1;
			var = ar0 - ar1;
			vtr = tr0 - tr1;
		}
		else
		{
			al1 = getPosture(i-1).getGlobalTranslation( PmHuman::LEFT_FOOT );
			tl1 = getPosture(i-1).getGlobalTranslation( PmHuman::LEFT_TOE );
			ar1 = getPosture(i-1).getGlobalTranslation( PmHuman::RIGHT_FOOT );
			tr1 = getPosture(i-1).getGlobalTranslation( PmHuman::RIGHT_TOE );

			al2 = getPosture(i+1).getGlobalTranslation( PmHuman::LEFT_FOOT );
			tl2 = getPosture(i+1).getGlobalTranslation( PmHuman::LEFT_TOE );
			ar2 = getPosture(i+1).getGlobalTranslation( PmHuman::RIGHT_FOOT );
			tr2 = getPosture(i+1).getGlobalTranslation( PmHuman::RIGHT_TOE );

			val = (al2 - al1) / 2;
			vtl = (tl2 - tl1) / 2;
			var = (ar2 - ar1) / 2;
			vtr = (tr2 - tr1) / 2;
		}

		bool mtl = getMask() & MaskBit(PmHuman::LEFT_TOE);
		bool mtr = getMask() & MaskBit(PmHuman::RIGHT_TOE);
		bool mal = getMask() & MaskBit(PmHuman::LEFT_FOOT);
		bool mar = getMask() & MaskBit(PmHuman::RIGHT_FOOT);

		left_touch  = ( (mtl && vtl.length() < toe_speed   && tl0.y() < toe_height   ) ||
						(mal && val.length() < ankle_speed && al0.y() < ankle_height ) );

		right_touch = ( (mtr && vtr.length() < toe_speed   && tr0.y() < toe_height   ) ||
						(mar && var.length() < ankle_speed && ar0.y() < ankle_height ) );

		if ( left_touch )
			constraints[i].push(
				PmHuman::LEFT_FOOT,
				getPosture(i).getGlobalTransf( PmHuman::LEFT_FOOT ) );

		if ( right_touch )
			constraints[i].push(
				PmHuman::RIGHT_FOOT,
				getPosture(i).getGlobalTransf( PmHuman::RIGHT_FOOT ) );
	}
}

void
PmLinearMotion::setHandConstraints( m_real palm_speed, m_real hand_speed )
{
	vector	al0, al1, al2;
	vector	tl0, tl1, tl2;
	vector	ar0, ar1, ar2;
	vector	tr0, tr1, tr2;
	vector	val, var, vtl, vtr;

	int left_touch  = TRUE;
	int right_touch = TRUE;

	for( int i=0; i<getSize(); i++ )
	{
		if ( i==0 )
		{
			al0 = getPosture(i  ).getGlobalTranslation( PmHuman::LEFT_PALM );
			tl0 = getPosture(i  ).getGlobalTranslation( PmHuman::LEFT_FINGER_11 );
			ar0 = getPosture(i  ).getGlobalTranslation( PmHuman::RIGHT_PALM );
			tr0 = getPosture(i  ).getGlobalTranslation( PmHuman::RIGHT_FINGER_11 );

			al2 = getPosture(i+1).getGlobalTranslation( PmHuman::LEFT_PALM );
			tl2 = getPosture(i+1).getGlobalTranslation( PmHuman::LEFT_FINGER_11 );
			ar2 = getPosture(i+1).getGlobalTranslation( PmHuman::RIGHT_PALM );
			tr2 = getPosture(i+1).getGlobalTranslation( PmHuman::RIGHT_FINGER_11 );

			val = al2 - al0;
			vtl = tl2 - tl0;
			var = ar2 - ar0;
			vtr = tr2 - tr0;
		}
		else if ( i==getSize()-1 )
		{
			al0 = getPosture(i  ).getGlobalTranslation( PmHuman::LEFT_PALM );
			tl0 = getPosture(i  ).getGlobalTranslation( PmHuman::LEFT_FINGER_11 );
			ar0 = getPosture(i  ).getGlobalTranslation( PmHuman::RIGHT_PALM );
			tr0 = getPosture(i  ).getGlobalTranslation( PmHuman::RIGHT_FINGER_11 );

			al1 = getPosture(i-1).getGlobalTranslation( PmHuman::LEFT_PALM );
			tl1 = getPosture(i-1).getGlobalTranslation( PmHuman::LEFT_FINGER_11 );
			ar1 = getPosture(i-1).getGlobalTranslation( PmHuman::RIGHT_PALM );
			tr1 = getPosture(i-1).getGlobalTranslation( PmHuman::RIGHT_FINGER_11 );

			val = al0 - al1;
			vtl = tl0 - tl1;
			var = ar0 - ar1;
			vtr = tr0 - tr1;
		}
		else
		{
			al1 = getPosture(i-1).getGlobalTranslation( PmHuman::LEFT_PALM );
			tl1 = getPosture(i-1).getGlobalTranslation( PmHuman::LEFT_FINGER_11 );
			ar1 = getPosture(i-1).getGlobalTranslation( PmHuman::RIGHT_PALM );
			tr1 = getPosture(i-1).getGlobalTranslation( PmHuman::RIGHT_FINGER_11 );

			al2 = getPosture(i+1).getGlobalTranslation( PmHuman::LEFT_PALM );
			tl2 = getPosture(i+1).getGlobalTranslation( PmHuman::LEFT_FINGER_11 );
			ar2 = getPosture(i+1).getGlobalTranslation( PmHuman::RIGHT_PALM );
			tr2 = getPosture(i+1).getGlobalTranslation( PmHuman::RIGHT_FINGER_11 );

			val = (al2 - al1) / 2;
			vtl = (tl2 - tl1) / 2;
			var = (ar2 - ar1) / 2;
			vtr = (tr2 - tr1) / 2;
		}

		left_touch  = ( (vtl.length() < hand_speed ) ||
						(val.length() < palm_speed ) );

		right_touch = ( (vtr.length() < hand_speed ) ||
						(var.length() < palm_speed ) );

		if ( left_touch )
			constraints[i].push(
				PmHuman::LEFT_PALM,
				getPosture(i).getGlobalTransf( PmHuman::LEFT_PALM ) );

		if ( right_touch )
			constraints[i].push(
				PmHuman::RIGHT_PALM,
				getPosture(i).getGlobalTransf( PmHuman::RIGHT_PALM ) );
	}
}


void
PmLinearMotion::mirrorReflect( PmLinearMotion const& m )
{
	this->setSize( m.getSize() );

	vector		v;
	quater		q;
	vector		y_hat;
	m_real		psi, phi;

	int			i, j;
	PmPosture	p;
	PmMaskType	mask = m.getMask();

	for( i=0; i<m.getSize(); i++ )
	{
		v = m.getPosture(i).getTranslation();
		p.setTranslation( vector(-v[0],v[1],v[2]) );

		for( j=0; j<PM_HUMAN_NUM_LINKS; j++ )
		if ( mask & MaskBit(j) )
		{
			q     = m.getPosture(i).getRotation(j);

			//  Axial decomposition of q
			y_hat = rotate( q, y_axis );
			psi   = ACOS( y_axis % y_hat );
			v     = normalize( y_axis * y_hat );
			phi   = 2 * ln( exp(-psi*v/2.0)*q ) % y_axis;

			//  mirror reflection
			y_hat = vector( -y_hat[0], y_hat[1], y_hat[2] );
			psi   = ACOS( y_axis % y_hat );
			v     = normalize( y_axis * y_hat );
			q     = exp(psi*v/2.0) * exp(-phi*y_axis/2.0);

			p.setRotation( PmHuman::getSymmetricPart(j), q );
		}
		this->setPosture( i, p );

		//  constraints
		this->constraints[i].reset();

		for( j=0; j<PM_HUMAN_NUM_LINKS; j++ )
		if ( mask & MaskBit(j) )
		{
			int k = PmHuman::getSymmetricPart(j);

			if ( m.constraints[i].isConstrained(j) )
				this->constraints[i].push( k,
					this->postures[i].getGlobalTransf(k) );
		}
	}

	this->alignment();
}

bool
PmLinearMotion::isLeftToeOff( int i )
{
	return	 constraints[i  ].isConstrained(PmHuman::LEFT_FOOT) &&
			!constraints[i+1].isConstrained(PmHuman::LEFT_FOOT);
}

bool
PmLinearMotion::isRightToeOff( int i )
{
	return	 constraints[i  ].isConstrained(PmHuman::RIGHT_FOOT) &&
			!constraints[i+1].isConstrained(PmHuman::RIGHT_FOOT);
}

bool
PmLinearMotion::isToeOff( int i )
{
	return isLeftToeOff(i) || isRightToeOff(i);
}

void
PmLinearMotion::cleanupConstraints( int gap_size, int con_size )
{
	int			i, j, k;
	PmMaskType	mask = this->getMask();

	for( j=0; j<PM_HUMAN_NUM_LINKS; j++ )
	if ( mask & MaskBit(j) )
	{
		//
		//	remove small gaps
		//
		int flag  = constraints[0].isConstrained(j);
		int count = 0;

		for( i=1; i<this->getSize()-1; i++ )
		{
			if ( ! constraints[i].isConstrained(j) )
			{
				count++;
			}
			else if ( ! flag && constraints[i].isConstrained(j) )
			{
				if ( count <= gap_size )
				{
					for( k=i-count-1; k<i; k++ )
						constraints[k].push( j, getPosture(k).getGlobalTransf(j) );
				}

				count = 0;
			}

			flag = constraints[i].isConstrained(j);
		}

		//
		//	remove constraints of short periods
		//
		flag  = ! constraints[0].isConstrained(j);
		count = 0;

		for( i=1; i<this->getSize()-1; i++ )
		{
			if ( constraints[i].isConstrained(j) )
			{
				count++;
			}
			else if ( ! flag && ! constraints[i].isConstrained(j) )
			{
				if ( count <= con_size )
				{
					for( k=i-count-1; k<i; k++ )
						constraints[k].remove( j );
				}

				count = 0;
			}

			flag = constraints[i].isConstrained(j);
		}
	}
}

void
PmLinearMotion::updateConstraintsAfterWarp()
{
	int i, j, k;
	transf t, t0, t1;

	for( j=0; j<PM_HUMAN_NUM_LINKS; j++ )
		for( i=0; i<this->getSize(); i++ )
		if ( this->constraints[i].isConstrained(j) )
		{
			for( k=i; k<this->getSize(); k++ )
				if ( ! this->constraints[k].isConstrained(j) ) break;

			if ( k<this->getSize() && i>0 )
			{
				t0 = this->constraints[i].getTransf(j);
				t1 = this->getPosture(i).getGlobalTransf(j);
				
				t = PlaneProject( t0.inverse() * t1 );
			}
			else
			{
				t0 = this->constraints[i].getTransf(j);
				t1 = this->getPosture(i).getGlobalTransf(j);
				
				t = PlaneProject( t0.inverse() * t1 );
			}

			this->constraints[i].applyTransf( j, t );
			for( k=i; k<this->getSize(); k++ )
			{
				if ( ! this->constraints[k].isConstrained(j) ) break;

				this->constraints[k].applyTransf( j, t );
			}

			i = k;
		}
}

void
PmLinearMotion::steerLocomotion( m_real angle )
{
	vector* ld = new vector[this->getSize()-1]; // Linear displacements
	quater* ad = new quater[this->getSize()-1]; // Angular displacement

	transf	t, t0, t1;
	vector	v;
	quater	q;
	int		i;
	
	angle /= (this->getSize() - 1);

	//
	//	Rotate linear and angular displacements
	//
	for( i=0; i<this->getSize()-1; i++ )
	{
		t0 = this->getPosture(i  ).getTransf( PmHuman::PELVIS );
		t1 = this->getPosture(i+1).getTransf( PmHuman::PELVIS );
		t  = t1 * t0.inverse();  // Displacement between i+1 and i measured in the body-fixed frame

		v = vector(0,1,0) * t0.inverse(); // Y-axis measured in the body-fixed frame
		q = exp( angle * v / 2.0 );

		ld[i] = rotate( q, t.getTranslation() ); // rotate linear displacements
		ad[i] = t.getRotation() * q; // rotate angular displacements
	}

	//
	//	Integrate the displacements
	//
	for( i=0; i<this->getSize()-1; i++ )
	{
		t.setTranslation( ld[i] );
		t.setRotation( ad[i] );

		t0 = this->getPosture(i).getTransf( PmHuman::PELVIS );
		this->getPosture(i+1).setTransf( PmHuman::PELVIS, t * t0 );
	}

	delete[] ld;
	delete[] ad;
}

void
PmLinearMotion::steerLocomotion( m_real angle, int start, int end )
{
	vector* ld = new vector[this->getSize()-1]; // Linear displacements
	quater* ad = new quater[this->getSize()-1]; // Angular displacement

	transf	t, t0, t1, t_end;
	vector	v;
	quater	q;
	int		i;
	
	angle /= (end-start-1);

	m_real scale = 1.0 - 0.7*angle/M_PI;

	//
	//	Rotate linear and angular displacements
	//
	for( i=start; i<end; i++ )
	{
		t0 = this->getPosture(i  ).getTransf( PmHuman::PELVIS );
		t1 = this->getPosture(i+1).getTransf( PmHuman::PELVIS );
		t  = t1 * t0.inverse();  // Displacement between i+1 and i measured in the body-fixed frame

		v = vector(0,1,0) * t0.inverse(); // Y-axis measured in the body-fixed frame
		q = exp( angle * v / 2.0 );

		ld[i] = rotate( q, scale * t.getTranslation() ); // rotate linear displacements
		ad[i] = t.getRotation() * q; // rotate angular displacements
	}

	//
	//	Integrate the displacements
	//
	t_end = this->getPosture( end ).getGlobalTransf( PmHuman::PELVIS );

	for( i=start; i<end; i++ )
	{
		t.setTranslation( ld[i] );
		t.setRotation( ad[i] );

		t0 = this->getPosture(i).getTransf( PmHuman::PELVIS );
		this->getPosture(i+1).setTransf( PmHuman::PELVIS, t * t0 );
	}

	t = t_end.inverse() * this->getPosture( end ).getGlobalTransf( PmHuman::PELVIS );

	for( i=end+1; i<this->getSize(); i++ )
	{
		t0 = this->getPosture(i).getTransf( PmHuman::PELVIS );
		this->getPosture(i).setTransf( PmHuman::PELVIS, t0 * t );
	}

	delete[] ld;
	delete[] ad;
}
