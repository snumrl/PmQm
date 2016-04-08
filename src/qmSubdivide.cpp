
#include "MATHCLASS/mathclass.h"



using namespace jhm;

#define OMEGA(x) ln(data[(x)].inverse() * data[(x)+1]);

#define MAX_FRAMES	500
#define NUM_ITER	3


//------------------------------------------------------------------//
//	  Pre-compute Weights for Non-Uniform Subdivision & Smooting	//
//------------------------------------------------------------------//


#define C1(x1,x2,x3,x4,x5) (1/(((x1)-(x5))*((x1)-(x4))*((x1)-(x3))*((x1)-(x2))))
#define C2(x1,x2,x3,x4,x5) (1/(((x2)-(x5))*((x2)-(x4))*((x2)-(x3))*((x2)-(x1))))
#define C3(x1,x2,x3,x4,x5) (1/(((x1)-(x3))*((x2)-(x3))*((x3)-(x5))*((x3)-(x4))))
#define C4(x1,x2,x3,x4,x5) (1/(((x1)-(x4))*((x2)-(x4))*((x3)-(x4))*((x5)-(x4))))
#define C5(x1,x2,x3,x4,x5) (1/(((x1)-(x5))*((x2)-(x5))*((x3)-(x5))*((x4)-(x5))))

static m_real weights[MAX_FRAMES][5];

void
QmSubdivideWeights( int n, m_real *kn )
{
	assert( n<MAX_FRAMES );

	m_real s = n-1;
	s = (kn[n-1] - kn[0]) / s;
	s = s*s*s*s;

	weights[0][0] = 0;
	weights[0][1] = 0;
	weights[0][2] = 0;
	weights[0][3] = 0;
	weights[0][4] = 0;

	weights[1][0] = s * C1(2*kn[0]-kn[1],kn[0],kn[1],kn[2],kn[3]);
	weights[1][1] = s * C2(2*kn[0]-kn[1],kn[0],kn[1],kn[2],kn[3]);
	weights[1][2] = s * C3(2*kn[0]-kn[1],kn[0],kn[1],kn[2],kn[3]);
	weights[1][3] = s * C4(2*kn[0]-kn[1],kn[0],kn[1],kn[2],kn[3]);
	weights[1][4] = s * C5(2*kn[0]-kn[1],kn[0],kn[1],kn[2],kn[3]);

	for( int i=2; i<n-2; i++ )
	{
	weights[i][0] = s * C1(kn[i-2],kn[i-1],kn[i],kn[i+1],kn[i+2]);
	weights[i][1] = s * C2(kn[i-2],kn[i-1],kn[i],kn[i+1],kn[i+2]);
	weights[i][2] = s * C3(kn[i-2],kn[i-1],kn[i],kn[i+1],kn[i+2]);
	weights[i][3] = s * C4(kn[i-2],kn[i-1],kn[i],kn[i+1],kn[i+2]);
	weights[i][4] = s * C5(kn[i-2],kn[i-1],kn[i],kn[i+1],kn[i+2]);
	}

	weights[n-2][0] = s * C1(kn[n-4],kn[n-3],kn[n-2],kn[n-1],2*kn[n-1]-kn[n-2]);
	weights[n-2][1] = s * C2(kn[n-4],kn[n-3],kn[n-2],kn[n-1],2*kn[n-1]-kn[n-2]);
	weights[n-2][2] = s * C3(kn[n-4],kn[n-3],kn[n-2],kn[n-1],2*kn[n-1]-kn[n-2]);
	weights[n-2][3] = s * C4(kn[n-4],kn[n-3],kn[n-2],kn[n-1],2*kn[n-1]-kn[n-2]);
	weights[n-2][4] = s * C5(kn[n-4],kn[n-3],kn[n-2],kn[n-1],2*kn[n-1]-kn[n-2]);

	weights[n-1][0] = 0;
	weights[n-1][1] = 0;
	weights[n-1][2] = 0;
	weights[n-1][3] = 0;
	weights[n-1][4] = 0;
}


//------------------------------------------------------------------//
//						Non-uniform smoothing						//
//------------------------------------------------------------------//


void
QmSmoothing( int n, vector *data, m_real *knots, int numIter, int start, int end )
{
	for( int j=0; j<numIter; j++ )
	{
		if ( start<=1 && 1<=end )
			data[1] -= weights[1][0] * (2*data[0] - data[1]) +
					   weights[1][1] * data[0] +
					   weights[1][2] * data[1] +
					   weights[1][3] * data[2] +
					   weights[1][4] * data[3];

		for( int i=MAX(2,start); i<MIN(n-2,end+1); i++ )
			data[i] -= weights[i][0] * data[i-2] +
					   weights[i][1] * data[i-1] +
					   weights[i][2] * data[i  ] +
					   weights[i][3] * data[i+1] +
					   weights[i][4] * data[i+2];

		if ( start<=n-2 && n-2<=end )
			data[n-2] -= weights[n-2][0] * data[n-4] +
						 weights[n-2][1] * data[n-3] +
						 weights[n-2][2] * data[n-2] +
						 weights[n-2][3] * data[n-1] +
						 weights[n-2][4] * (2*data[n-1] - data[n-2]);
	}
}

void
QmSmoothing( int n, quater *data, m_real *knots, int numIter, int start, int end )
{
	vector v1, v2, v3, v4;

	for( int j=0; j<numIter; j++ )
	{
		if ( start<=1 && 1<=end )
		{
			v1 =    weights[1][0]				   * OMEGA(1);
			v2 =   (weights[1][0] + weights[1][1]) * OMEGA(0);
			v3 = - (weights[1][3] + weights[1][4]) * OMEGA(1);
			v4 = -  weights[1][4]				   * OMEGA(2);

			data[1] = data[1] * exp( v1 + v2 + v3 + v4 );
		}

		for( int i=MAX(2,start); i<MIN(n-2,end+1); i++ )
		{
			v1 =    weights[i][0]				   * OMEGA(i-2);
			v2 =   (weights[i][0] + weights[i][1]) * OMEGA(i-1);
			v3 = - (weights[i][3] + weights[i][4]) * OMEGA(i);
			v4 = -  weights[i][4]				   * OMEGA(i+1);

			data[i] = data[i] * exp( v1 + v2 + v3 + v4 );
		}

		if ( start<=n-2 && n-2<=end )
		{
			v1 =    weights[n-2][0]				  	   * OMEGA(n-4);
			v2 =   (weights[n-2][0] + weights[n-2][1]) * OMEGA(n-3);
			v3 = - (weights[n-2][3] + weights[n-2][4]) * OMEGA(n-2);
			v4 = -  weights[n-2][4]					   * OMEGA(n-3);

			data[n-2] = data[n-2] * exp( v1 + v2 + v3 + v4 );
		}
	}
}


//------------------------------------------------------------------//
//					Uniform interpolatory subdivision				//
//------------------------------------------------------------------//


void
QmInterpolatorySubdivide( int n, vector *source, vector *target, char *mask_list )
{
	int i, j;
	for( i=0, j=0; i<n; i++ )
		if ( mask_list[i] ) target[i] = source[j++];

	for( i=1; i<n-1; i++ )
		if ( ! mask_list[i] )
		{
			if ( 2<i && i<n-3 )
				target[i] = ( 9*target[i-1] + 9*target[i+1]
							  - target[i-3] -   target[i+3] ) / 16;
			else
				target[i] = ( target[i-1] + target[i+1] ) / 2;
		}

	if ( ! mask_list[0] )
		target[0] = ::interpolate( -1.0, target[1], target[2] );

	if ( ! mask_list[n-1] )
		target[n-1] = ::interpolate( 2.0, target[n-3], target[n-2] );
}

void QmInterpolatorySubdivide( int n, quater *source, quater *target, char *mask_list )
{
	int i, j;
	for( i=0, j=0; i<n; i++ )
		if ( mask_list[i] ) target[i] = source[j++];

	quater q1, q2;

	for( i=1; i<n-1; i++ )
		if ( ! mask_list[i] )
		{
			if ( 2<i && i<n-3 )
			{
				q1 = interpolate( 0.5, target[i-1], target[i+1] );
				q2 = interpolate( 0.5, target[i-3], target[i+3] );
				target[i] = interpolate( 1.125, q2, q1 );
			}
			else
				target[i] = interpolate( 0.5, target[i-1], target[i+1] );
		}

	if ( ! mask_list[0] )
		target[0] = interpolate( -1.0, target[1], target[2] );

	if ( ! mask_list[n-1] )
		target[n-1] = interpolate( 2.0, target[n-3], target[n-2] );
}


//------------------------------------------------------------------//
//				Non-uniform interpolatory subdivision				//
//------------------------------------------------------------------//


m_real
QmCardinalFunction( int n, int index, m_real *knots, m_real x )
{
	m_real value = 1;

	for( int i=0; i<n; i++ )
		if ( i != index )
			value *= (x - knots[i]) / (knots[index] - knots[i]);

	return value;
}

void
QmInterpolatorySubdivide( int n, vector *source, vector *target, char *mask_list, m_real *knots )
{
	int i, j;
	for( i=0, j=0; i<n; i++ )
		if ( mask_list[i] ) target[i] = source[j++];

	int		i1, i2;
	m_real	t;
	m_real	a[4];
	m_real	k[4];

	for( i=1; i<n-1; i++ )
		if ( ! mask_list[i] )
		{
			if ( 2<i && i<n-3 )
			{
				if ( mask_list[i-2] ) i1 = i-2;
								else  i1 = i-3;
				if ( mask_list[i+2] ) i2 = i+2;
								else  i2 = i+3;
				
				k[0] = knots[i1];
				k[1] = knots[i-1];
				k[2] = knots[i+1];
				k[3] = knots[i2];

				a[0] = QmCardinalFunction( 4, 0, k, knots[i] );
				a[1] = QmCardinalFunction( 4, 1, k, knots[i] );
				a[2] = QmCardinalFunction( 4, 2, k, knots[i] );
				a[3] = QmCardinalFunction( 4, 3, k, knots[i] );

				target[i] = a[0]*target[i1] +
							a[1]*target[i-1] +
							a[2]*target[i+1] +
							a[3]*target[i2];
			}
			else
			{
				t = (knots[i] - knots[i-1]) / (knots[i+1] - knots[i-1]);
				target[i] = ::interpolate( t, target[i-1], target[i+1] );
			}
		}

	if ( ! mask_list[0] )
	{
		t = (knots[0] - knots[2]) / (knots[1] - knots[2]);
		target[0] = ::interpolate( t, target[2], target[1] );
	}

	if ( ! mask_list[n-1] )
	{
		t = (knots[n-1] - knots[n-3]) / (knots[n-2] - knots[n-3]);
		target[n-1] = ::interpolate( t, target[n-3], target[n-2] );
	}
}

void QmInterpolatorySubdivide( int n, quater *source, quater *target, char *mask_list, m_real *knots )
{
	int i, j;
	for( i=0, j=0; i<n; i++ )
		if ( mask_list[i] ) target[i] = source[j++];

	m_real	t;
	m_real	a[4], k[4];
	int		i1, i2;
	quater	q1, q2;

	for( i=1; i<n-1; i++ )
		if ( ! mask_list[i] )
		{
			if ( mask_list[i-2] ) i1 = i-2;
							else  i1 = i-3;
			if ( mask_list[i+2] ) i2 = i+2;
							else  i2 = i+3;

			if ( 0<=i1 && i2<=n-1 )
			{
				k[0] = knots[i1];
				k[1] = knots[i-1];
				k[2] = knots[i+1];
				k[3] = knots[i2];

				a[0] = QmCardinalFunction( 4, 0, k, knots[i] );
				a[1] = QmCardinalFunction( 4, 1, k, knots[i] );
				a[2] = QmCardinalFunction( 4, 2, k, knots[i] );
				a[3] = QmCardinalFunction( 4, 3, k, knots[i] );

				q1 = interpolate( a[1]/(a[0]+a[1]), target[i1], target[i-1] );
				q2 = interpolate( a[2]/(a[2]+a[3]), target[i2], target[i+1] );

				t  = (a[0]+a[1])/(a[0]+a[1]+a[2]+a[3]);
				target[i] = interpolate( 1-t, q1, q2 );
			}
			else
			{
				t = (knots[i] - knots[i-1]) / (knots[i+1] - knots[i-1]);
				target[i] = interpolate( t, target[i-1], target[i+1] );
			}
		}

	if ( ! mask_list[0] )
	{
		t = (knots[0] - knots[2]) / (knots[1] - knots[2]);
		target[0] = interpolate( t, target[1], target[2] );
	}

	if ( ! mask_list[n-1] )
	{
		t = (knots[n-1] - knots[n-3]) / (knots[n-2] - knots[n-3]);
		target[n-1] = interpolate( t, target[n-3], target[n-2] );
	}
}


//------------------------------------------------------------------//
//					Cubic B-spline subdivision 						//
//------------------------------------------------------------------//


void
QmBsplineSubdivide( int n, vector *source, vector *target)
{
	target[0]     = source[0];
	target[2*n-2] = source[n-1];

	int i;
	for( i=0; i<n-1; i++ )
		target[2*i+1] = ::interpolate( 0.5, source[i], source[i+1] );

	vector p1, p2;

	for( i=1; i<n-1; i++ )
	{
		p1 = ::interpolate( 0.75, source[i-1], source[i] );
		p2 = ::interpolate( 0.25, source[i], source[i+1] );
		target[2*i] = ::interpolate( 0.5, p1, p2 );
	}
}

void
QmBsplineSubdivide( int n, quater *source, quater *target )
{
	target[0]     = source[0];
	target[2*n-2] = source[n-1];

	int i;
	for( i=0; i<n-1; i++ )
		target[2*i+1] = interpolate( 0.5, source[i], source[i+1] );

	quater p1, p2;

	for( i=1; i<n-1; i++ )
	{
		p1 = interpolate( 0.75, source[i-1], source[i] );
		p2 = interpolate( 0.25, source[i], source[i+1] );
		target[2*i] = interpolate( 0.5, p1, p2 );
	}
}

