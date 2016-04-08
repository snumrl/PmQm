

#include "MATHCLASS/mathclass.h"

using namespace jhm;


void
QmAlignUnitQuaternions( int n, quater *data )
{
    for( int i=1; i<n; i++ )
        if ( data[i-1]%data[i]<0 ) data[i] = -data[i];
}

void
QmLTIfilter( int n, vector *data, int mask_size, m_real *mask,
			 int numIter, int start, int end )
{
	assert( mask_size%2 == 1 ); // the size of the mask must be odd

	int		i, j, k;
	vector	v;
	int		r = mask_size/2;

    for( j=0; j<numIter; j++ )
    {
		for( i=MAX(0,start); i<MIN(r,end+1); i++ )
		{
			v = mask[r] * data[i];

			for( k=1; k<r+1; k++ )
			{
				v += mask[r+k] * data[i+k];

				if ( i-k >= 0 ) v += mask[r-k] * data[i-k];
						   else v += mask[r-k] * (2*data[0] - data[k-i]);
			}

			data[i] = v;
		}

        for( i=MAX(r,start); i<MIN(n-r,end+1); i++ )
		{
			data[i] = mask[r] * data[i];

			for( k=1; k<r+1; k++ )
			{
	            data[i] += mask[r-k] * data[i-k];
	            data[i] += mask[r+k] * data[i+k];
			}
		}

		for( i=MAX(n-r,start); i<MIN(n,end+1); i++ )
		{
			v = mask[r] * data[i];

			for( k=1; k<r+1; k++ )
			{
				v += mask[r-k] * data[i-k];

				if ( i+k < n ) v += mask[r+k] * data[i+k];
						  else v += mask[r+k] * (2*data[n-1] - data[2*n-2-i-k]);
			}

			data[i] = v;
		}
	}
}

void
QmLTIfilter( int n, quater *data, int mask_size, m_real *mask,
			 int numIter, int start, int end )
{
	assert( mask_size%2 == 0 ); // the size of the mask must be even

	int		i, j, k;
	vector	v;
	int		r = mask_size/2;

    for( j=0; j<numIter; j++ )
    {
		for( i=MAX(0,start); i<MIN(r,end+1); i++ )
		{
			v = vector(0,0,0);

			for( k=1; k<r+1; k++ )
			{
				v += mask[r+k-1] * ln( data[i+k-1].inverse() * data[i+k] );

				if ( i-k >= 0 )
					v += mask[r-k] * ln( data[i-k].inverse() * data[i-k+1] );
				else
					v += mask[r-k] * ln( data[k-i].inverse() * data[k-i+1] );
			}

			data[i] = data[i] * exp( v );
		}

        for( i=MAX(r,start); i<MIN(n-r,end+1); i++ )
		{
			v = vector(0,0,0);

			for( k=1; k<r+1; k++ )
			{
				v += mask[r+k-1] * ln( data[i+k-1].inverse() * data[i+k] );
				v += mask[r-k]   * ln( data[i-k].inverse() * data[i-k+1] );
			}

			data[i] = data[i] * exp( v );
		}

		for( i=MAX(n-r,start); i<MIN(n,end+1); i++ )
		{
			v = vector(0,0,0);

			for( k=1; k<r+1; k++ )
			{
				v += mask[r-k] * ln( data[i-k].inverse() * data[i-k+1] );

				if ( i+k < n )
					v += mask[r+k-1] * ln( data[i+k-1].inverse() * data[i+k] );
				else
					v += mask[r+k-1] * ln( data[2*n-1-i-k].inverse() * data[2*n-i-k] );
			}

			data[i] = data[i] * exp( v );
		}
	}
}

