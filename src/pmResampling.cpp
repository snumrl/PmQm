
#include "pm.h"


using namespace jhm;

//--------------------------------------------------------------------------//
//								Evaluation									//
//--------------------------------------------------------------------------//

extern m_real QmCardinalFunction( int, int, m_real*, m_real );

int
PmLinearMotion::evaluate( m_real time, PmPosture& p,
						  PmConstraint& c, int start ) const
{
	int		n = getSize();
	m_real	t, t1, t2;
	
	static PmPosture	p1, p2;
	static PmConstraint	c1, c2;
	
	if ( time <= knots[0] )
	{
		p = postures[0];
		c = constraints[0];
		
		return 0;
	}
	else if ( time <= knots[1] )
	{
		t = (time-knots[0]) / (knots[1]-knots[0]);
		p.interpolate( t, postures   [0], postures   [1] );
		c.interpolate( t, constraints[0], constraints[1] );
		
		return 0;
	}
	else if ( time <= knots[n-2] )
	{
		for( int i=MAX(1,start); i<n-2; i++ )
		if ( time <= knots[i+1] )
		{
			if ( time < knots[i]+EPS_jhm )
			{
				p = postures[i];
				c = constraints[i];

				return i;
			}
			else if ( time > knots[i+1]-EPS_jhm )
			{
				p = postures[i+1];
				c = constraints[i+1];

				return i;
			}

			m_real	a[4], k[4];
			
			k[0] = knots[i-1];
			k[1] = knots[i  ];
			k[2] = knots[i+1];
			k[3] = knots[i+2];
			
			a[0] = QmCardinalFunction( 4, 0, k, time );
			a[1] = QmCardinalFunction( 4, 1, k, time );
			a[2] = QmCardinalFunction( 4, 2, k, time );
			a[3] = QmCardinalFunction( 4, 3, k, time );
			
			t1 = a[1]/(a[0]+a[1]);
			t2 = a[3]/(a[2]+a[3]);
			t  = (a[2]+a[3])/(a[0]+a[1]+a[2]+a[3]);

			p1.interpolate( t1, postures[i-1], postures[i  ] );
			p2.interpolate( t2, postures[i+1], postures[i+2] );
			p.interpolate( t, p1, p2 );

			t = (time-knots[i]) / (knots[i+1]-knots[i]);
			c.interpolate( t, constraints[i], constraints[i+1], PM_C_AND );
			
			return i;
		}
	}
	else if ( time <= knots[n-1] )
	{
		t = (time-knots[n-2]) / (knots[n-1]-knots[n-2]);
		p.interpolate( t, postures   [n-2], postures   [n-1] );
		c.interpolate( t, constraints[n-2], constraints[n-1] );
		
		return n-2;
	}

	p = postures   [n-1];
	c = constraints[n-1];
	
	return n-1;
}


//--------------------------------------------------------------------------//
//								Resampling									//
//--------------------------------------------------------------------------//


void
PmLinearMotion::resampling( int n, PmLinearMotion const& m )
{
	this->setSize( n );

	m_real		 t;
	PmPosture	 p;
	PmConstraint c;

	for( int i=0, start=0; i<n; i++ )
	{
		t = i;
		t = t / (n-1);
		t = t * (m.knots[m.getSize()-1] - m.knots[0]) + m.knots[0];

		start = m.evaluate( t, p, c, start );
		
		this->setPosture( i, p );
		this->setConstraint( i, c );
	}

	this->setBody( m.getBody() );
}

//--------------------------------------------------------------------------//

#define H0(t) (  2.0*(t)*(t)*(t) - 3.0*(t)*(t) + 1.0 )
#define H1(t) ( -2.0*(t)*(t)*(t) + 3.0*(t)*(t) )
#define H2(t) (  1.0*(t)*(t)*(t) - 2.0*(t)*(t) + 1.0*(t) )
#define H3(t) (  1.0*(t)*(t)*(t) - 1.0*(t)*(t) )

void
PmLinearMotion::resampling( int n, int n1, int n2, PmLinearMotion const& m )
{
	this->setSize( n );

	m_real		 t;
	PmPosture 	 p;
	PmConstraint c;

	for( int i=0, start=0; i<n; i++ )
	{
		t = i;
		t = t / (n-1);
		t = H1(t) + (H2(t)*n2)/n + (H3(t)*n1)/n;
		t = t * (m.knots[m.getSize()-1] - m.knots[0]) + m.knots[0];
		
		start = m.evaluate( t, p, c, start );
		
		this->setPosture( i, p );
		this->setConstraint( i, c );
	}

	this->setBody( m.getBody() );
}

//--------------------------------------------------------------------------//

#define MAX_MARKERS		50

void
PmLinearMotion::resampling( PmLinearMotion const& m,
							PmMomentType moment,
							PmMaskType link, int max_block )
{
	int i, j;
	int	num_markers = 0;
	int	markers[MAX_MARKERS];

	//
	//	Find Markers
	//
	if ( moment & PM_HEEL_STRIKE )
	{
		for( i=1; i<m.getSize(); i++ )
		for( j=0; j<PM_HUMAN_NUM_LINKS; j++ )
		if ( link & MaskBit(j) )
		{
			if ( ! m.constraints[i-1].isConstrained(j) &&
				   m.constraints[i  ].isConstrained(j) )
			{
				assert( num_markers < MAX_MARKERS );
				markers[num_markers++] = i;
			}
		}
	}
	if ( moment & PM_TOE_OFF )
	{
		for( i=0; i<m.getSize()-1; i++ )
		for( j=0; j<PM_HUMAN_NUM_LINKS; j++ )
		if ( link & MaskBit(j) )
		{
			if (   m.constraints[i  ].isConstrained(j) &&
				 ! m.constraints[i+1].isConstrained(j) )
			{
				assert( num_markers < MAX_MARKERS );
				markers[num_markers++] = i;
			}
		}
	}

	//
	//	Determine the Length of the Motion
	//
	int max_i = 0;
	for( i=1; i<num_markers; i++ )
		max_i = MAX( max_i, markers[i] - markers[i-1] );

	int temp = 1;
	while( temp<max_i ) temp *= 2;
	if ( max_block==0 ) max_i = temp;
				   else max_i = MIN( max_block, temp );

	int n = max_i*(num_markers-1) + 1;

	this->setSize( n );

	//
	//	Resampling
	//
	m_real		 t;
	PmPosture	 p;
	PmConstraint c;

	int k = 0;
	int start = 0;

	for( j=0; j<num_markers-1; j++ )
	{
		m_real a = m.knots[markers[j+1]] - m.knots[markers[j]];
		m_real b = m.knots[markers[j]];
		
		for( i=0; i<max_i; i++ )
		{
			t = i;
			t = t / max_i;
			t = a * t + b;

			start = m.evaluate( t, p, c, start );

			this->postures   [k] = p;
			this->constraints[k] = c;
			this->knots      [k] = t;
			k++;
		}
	}
	
	this->postures   [k] = m.postures   [markers[num_markers-1]];
	this->constraints[k] = m.constraints[markers[num_markers-1]];
	this->knots      [k] = m.knots      [markers[num_markers-1]];
	k++;

	this->setBody( m.getBody() );
}

