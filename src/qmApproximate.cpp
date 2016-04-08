
#include "MATHCLASS/mathclass.h"
#include "qmApproximate.h"

using namespace jhm;

void
QmApproximate::setSize( int n )
{
	if ( n==num ) return;

	delete[] cp; cp = NULL;

	num = n;
	if ( n>0 ) cp = new vector[n+3];
}

void
QmApproximate::smoothBoundary()
{
	assert( num > 3 );

	cp[0] =
	cp[1] = 
	cp[2] =
	cp[num-1] = 
	cp[num-2] =
	cp[num-3] = vector(0,0,0);
}

vector
QmApproximate::evaluate( m_real t ) const
{
	int i = int(t);
	if ( i < 0     ) i = 0;
	if ( i > num-1 ) i = num-1;

	m_real f = t - i;

	return B0(f)*cp[i] + B1(f)*cp[i+1] + B2(f)*cp[i+2] + B3(f)*cp[i+3];
}

void
QmApproximate::discretize( int num_frames, vector *data )
{
    for( int i=0; i<num_frames; i++ )
    {
        m_real t = num * i;
        data[i] = evaluate( t/num_frames );
    }
}

void
QmApproximate::approximate( int num_frames, int num_data, QmScatteredData *data )
{
	static vectorN weight; weight.setSize( num+3 );

	int i, j;
    for( i=0; i<num+3; i++ )
	{
		cp[i] = vector(0,0,0);
		weight[i] = 0.0;
	}

    for( j=0; j<num_data; j++ )
    {
        m_real t = num * data[j].position;
        t /= num_frames;

        i = int(t);
        if ( i < 0     ) i = 0;
        if ( i > num-1 ) i = num-1;

        m_real f = t - i;

		m_real r0 = B0(f); m_real s0 = r0*r0;
		m_real r1 = B1(f); m_real s1 = r1*r1;
		m_real r2 = B2(f); m_real s2 = r2*r2;
		m_real r3 = B3(f); m_real s3 = r3*r3;

        m_real sum = s0 + s1 + s2 + s3;

        cp[i  ] += s0 * data[j].value * r0 / sum;
        cp[i+1] += s1 * data[j].value * r1 / sum;
        cp[i+2] += s2 * data[j].value * r2 / sum;
        cp[i+3] += s3 * data[j].value * r3 / sum;

        weight[i  ] += s0;
        weight[i+1] += s1;
        weight[i+2] += s2;
        weight[i+3] += s3;
    }

    for( i=0; i<num+3; i++ )
        if ( weight[i]>EPS_jhm ) cp[i] /= weight[i];
                        else cp[i] = vector(0,0,0);
}

void
QmApproximate::approximateLeastSquare( int num_frames, int num_data, QmScatteredData *data )
{
	static matrixN M;  M.setSize( num+3, num_data );
	static vectorN bx; bx.setSize( num_data );
	static vectorN by; by.setSize( num_data );
	static vectorN bz; bz.setSize( num_data );

	int i, j;

	for( i=0; i<num+3; i++ )
	for( j=0; j<num_data; j++ )
		M[i][j] = 0.0;

    for( j=0; j<num_data; j++ )
    {
        m_real t = num * data[j].position;
        t /= num_frames;

        i = int(t);
        if ( i < 0     ) i = 0;
        if ( i > num-1 ) i = num-1;

        m_real f = t - i;

		M[i  ][j] = B0(f);
		M[i+1][j] = B1(f);
		M[i+2][j] = B2(f);
		M[i+3][j] = B3(f);

		bx[j] = data[j].value[0];
		by[j] = data[j].value[1];
		bz[j] = data[j].value[2];
    }

	static matrixN Mt;  Mt.transpose( M );
	static matrixN Mp;  Mp.mult( Mt, M );
	static vectorN bxp; bxp.mult( Mt, bx );
	static vectorN byp; byp.mult( Mt, by );
	static vectorN bzp; bzp.mult( Mt, bz );

	static vectorN cx;	cx.solve( Mp, bxp );
	static vectorN cy;	cy.solve( Mp, byp );
	static vectorN cz;	cz.solve( Mp, bzp );

	/*
	static vectorN cx;	cx.solve( M, bx, 1e-3 );
	static vectorN cy;	cy.solve( M, by, 1e-3 );
	static vectorN cz;	cz.solve( M, bz, 1e-3 );
	*/

	for( i=0; i<num+3; i++ )
		cp[i] = vector( cx[i], cy[i], cz[i] );
}

