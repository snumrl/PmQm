
#include <stdlib.h>

#include "pm.h"
#include "qmSubdivide.h"
#include "qmFilter.h"

using namespace jhm;

vector p_buf[PM_MAX_FRAMES];
quater q_buf[PM_MAX_FRAMES];
vector p1_buf[PM_MAX_FRAMES];
quater q1_buf[PM_MAX_FRAMES];


//------------------------------------------------------------------//
//						Up Sampling by Subdivision					//
//------------------------------------------------------------------//

void
PmLinearMotion::subdivide( PmLinearMotion const& lm, int n,
						   char* mask_list, m_real* knot_seq )
{
	PmMaskType mask = lm.getMask();

	this->setSize( n );

	if ( PenvMRAdownSamplingType == PENV_NON_UNIFORM_DOWN_SAMPLING )
		QmSubdivideWeights( n, knot_seq );

	if ( mask & PM_MASK_PELVIS )
	{
		lm.getTranslation( p_buf );

		if ( PenvMRAdownSamplingType==PENV_NON_UNIFORM_DOWN_SAMPLING )
			QmInterpolatorySubdivide( n, p_buf, p1_buf, mask_list, knot_seq );
		else
			QmInterpolatorySubdivide( n, p_buf, p1_buf, mask_list );

		this->setTranslation( p1_buf );
	}

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	if ( mask & MaskBit(i) )
	{
		lm.getRotation( i, q_buf );

		if ( PenvMRAdownSamplingType==PENV_NON_UNIFORM_DOWN_SAMPLING )
			QmInterpolatorySubdivide( n, q_buf, q1_buf, mask_list, knot_seq );
		else
			QmInterpolatorySubdivide( n, q_buf, q1_buf, mask_list );

		this->setRotation( i, q1_buf );
	}
	
	setKnots( knot_seq );
	this->setBody( lm.getBody() );
}

//------------------------------------------------------------------//

void
PmLinearMotion::subdivide( PmLinearMotion const& lm )
{
	PmMaskType mask = lm.getMask();

	int n = 2*lm.getSize() - 1;

	this->setSize( n );

	static char   mask_list[PM_MAX_FRAMES];

	for( int j=0; j<n; j++ )
	{
		if ( j%2 ) mask_list[j] = 0;
			  else mask_list[j] = 1;
	}

	if ( mask & PM_MASK_PELVIS )
	{
		lm.getTranslation( p_buf );
		QmInterpolatorySubdivide( n, p_buf, p1_buf, mask_list );
		this->setTranslation( p1_buf );
	}

	int i;
	for( i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	if ( mask & MaskBit(i) )
	{
		lm.getRotation( i, q_buf );
		QmInterpolatorySubdivide( n, q_buf, q1_buf, mask_list );
		this->setRotation( i, q1_buf );
	}
	
	for( i=0; i<getSize(); i++ )
	{
		if ( i%2 == 0 )
			knots[i] = lm.knots[i/2];
		else
			knots[i] = (lm.knots[i/2] + lm.knots[i/2+1])/2.0;
	}

	this->setBody( lm.getBody() );
}


//------------------------------------------------------------------//
//				Down Sampling (Uniform & Non-uniform)				//
//------------------------------------------------------------------//


void
PmLinearMotion::uniformDownSampling( PmLinearMotion const& lm, char* mask_list )
{
	int i, j;

	for( i=0; i<lm.getSize(); i++ )
		mask_list[i] = TRUE;

	for( i=1, j=0; i<lm.getSize(); i+=2 )
	{
		mask_list[i] = FALSE;
		j++;
	}

	this->setSize( lm.getSize() - j );

	for( i=0, j=0; i<lm.getSize(); i++ )
		if ( mask_list[i] )
		{
			this->knots[j]    = lm.knots[i];
			this->postures[j] = lm.postures[i];
			j++;
		}

	this->setBody( lm.getBody() );
}

void
PmLinearMotion::downSampling( int l )
{
	for( int j=0; j<l; j++ )
		if ( getSize()>1 )
		{
			for( int i=0; i<getSize(); i+=2 )
			{
				knots[i/2]    = knots[i];
				postures[i/2] = postures[i];
			}

			setSize( getSize()/2 );
		}
}

//------------------------------------------------------------------//

#define VERY_LARGE 1000000

static int		index[PM_MAX_FRAMES];
static m_real	value[PM_MAX_FRAMES];

int
compar( const void* a, const void* b )
{
	if ( value[*(int*)a] == value[*(int*)b] ) return 0;
	if ( value[*(int*)a] >  value[*(int*)b] ) return 1;
	return -1;
}

void
PmLinearMotion::nonuniformDownSampling( PmLinearMotion &lm,
								char* mask_list, m_real *knot_seq )
{
	int					i, j;
	m_real				t;
	m_real				a[4];
	m_real				k[4];
	static PmPosture	p1, p2, p3;
	static PmVector		v;

	//
	//		Initialize variables
	//
	for( i=0; i<lm.getSize(); i++ )
	{
		mask_list[i] = TRUE;
		index[i] = i;
		value[i] = 0;
	}

	//
	//		Compute weight value for each frame
	//
	value[0] = value[lm.getSize()-1] = VERY_LARGE;

	for( i=1; i<lm.getSize()-1; i++ )
	{
		if ( 2<i && i<lm.getSize()-3 )
		{
			k[0] = knot_seq[i-2];
			k[1] = knot_seq[i-1];
			k[2] = knot_seq[i+1];
			k[3] = knot_seq[i+2];

			a[0] = QmCardinalFunction( 4, 0, k, knot_seq[i] );
			a[1] = QmCardinalFunction( 4, 1, k, knot_seq[i] );
			a[2] = QmCardinalFunction( 4, 2, k, knot_seq[i] );
			a[3] = QmCardinalFunction( 4, 3, k, knot_seq[i] );

			p1.interpolate( a[1]/(a[0]+a[1]), lm.getPosture(i-2),
											  lm.getPosture(i-1) );
			p2.interpolate( a[2]/(a[2]+a[3]), lm.getPosture(i+2),
											  lm.getPosture(i+1) );

			t  = (a[0]+a[1])/(a[0]+a[1]+a[2]+a[3]);
			p3.interpolate( 1-t, p1, p2 );

			v.difference( p3, lm.getPosture(i) );
		}
		else
		{
			t = (knot_seq[i] - knot_seq[i-1]) / (knot_seq[i+1] - knot_seq[i-1]);
			p1.interpolate( t, lm.getPosture(i-1), lm.getPosture(i+1) );
			v.difference( p1, lm.getPosture(i) );
		}

		value[i] = v.magnitude();
		assert( value[i] < VERY_LARGE );
	}

	qsort( index, lm.getSize(), sizeof(int), compar );

	for( i=0, j=0; i<lm.getSize()-2; i++ )
		if ( mask_list[index[i]-1] && mask_list[index[i]+1] )
		{
			mask_list[index[i]] = FALSE;
			j++;
		}

	//
	//		Down-sampling
	//
	this->setSize( lm.getSize() - j );

	for( i=0, j=0; i<lm.getSize(); i++ )
		if ( mask_list[i] )
		{
			this->knots[j]    = lm.knots[i];
			this->postures[j] = lm.postures[i];
			j++;
		}

	this->setBody( lm.getBody() );
}


//------------------------------------------------------------------//
//						Filtering for Motion						//
//------------------------------------------------------------------//


void
PmLinearMotion::smoothing( int numIter, PmMaskType const& iMask,
						   int start, int end )
{
	int n = this->getSize();
	PmMaskType mask = this->getMask();

	QmSubdivideWeights( n, knots );

	if ( mask & iMask & PM_MASK_PELVIS )
	{
		this->getTranslation( p_buf );
		QmSmoothing( n, p_buf, knots, numIter, start, end );
		this->setTranslation( p_buf );
	}

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	if ( mask & iMask & MaskBit(i) )
	{
		this->getRotation( i, q_buf );
		QmSmoothing( n, q_buf, knots, numIter, start, end );
		this->setRotation( i, q_buf );
	}
}

void
PmLinearMotion::LTIfilter( int numIter, PmMaskType const& iMask,
						   int mask_size1, m_real *mask1,
						   int mask_size2, m_real *mask2 )
{
	int n = this->getSize();
	PmMaskType mask = this->getMask();

	if ( mask & iMask & PM_MASK_PELVIS )
	{
		this->getTranslation( p_buf );
		QmLTIfilter( n, p_buf, mask_size1, mask1, numIter, 0, n-1 );
		this->setTranslation( p_buf );
	}

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	if ( mask & iMask & MaskBit(i) )
	{
		this->getRotation( i, q_buf );
		QmLTIfilter( n, q_buf, mask_size2, mask2, numIter, 0, n-1 );
		this->setRotation( i, q_buf );
	}
}

void
PmLinearMotion::LTIfilter( int numIter, PmMaskType const& iMask,
						   int mask_size1, m_real *mask1,
						   int mask_size2, m_real *mask2,
						   int start, int end )
{
	int n = this->getSize();
	PmMaskType mask = this->getMask();

	if ( mask & iMask & PM_MASK_PELVIS )
	{
		this->getTranslation( p_buf );
		QmLTIfilter( n, p_buf, mask_size1, mask1, numIter, start, end );
		this->setTranslation( p_buf );
	}

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	if ( mask & iMask & MaskBit(i) )
	{
		this->getRotation( i, q_buf );
		QmLTIfilter( n, q_buf, mask_size2, mask2, numIter, start, end );
		this->setRotation( i, q_buf );
	}
}

void
PmLinearMotion::LTIfilter( void (*filter1)(int,vector*,int),
						   void (*filter2)(int, quater*,int),
						   int numIter, PmMaskType const& iMask )
{
	int n = this->getSize();
	PmMaskType mask = this->getMask();

	if ( mask & iMask & PM_MASK_PELVIS )
	{
		this->getTranslation( p_buf );
		filter1( n, p_buf, numIter );
		this->setTranslation( p_buf );
	}

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	if ( mask & iMask & MaskBit(i) )
	{
		this->getRotation( i, q_buf );
		filter2( n, q_buf, numIter );
		this->setRotation( i, q_buf );
	}
}

void
PmLinearMotion::LTIfilter( void (*filter1)(int,vector*,int,int,int),
						   void (*filter2)(int, quater*,int,int,int),
						   int numIter, PmMaskType const& iMask,
						   int start, int end )
{
	int n = this->getSize();
	PmMaskType mask = this->getMask();

	if ( mask & iMask & PM_MASK_PELVIS )
	{
		this->getTranslation( p_buf );
		filter1( n, p_buf, numIter, start, end );
		this->setTranslation( p_buf );
	}

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	if ( mask & iMask & MaskBit(i) )
	{
		this->getRotation( i, q_buf );
		filter2( n, q_buf, numIter, start, end );
		this->setRotation( i, q_buf );
	}
}
