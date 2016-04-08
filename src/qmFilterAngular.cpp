

#include "MATHCLASS/mathclass.h"



using namespace jhm;

//--------------------------------------------------------------//
//				New Versions ( Smoothing )						//
//--------------------------------------------------------------//

void
QmSmoothing( int n, quater *data, vector *disp )
{
    vector v1, v2, v3, v4;

	disp[0] = disp[n-1] = vector(0,0,0);

    v1 = ln( data[1].inverse() * data[2] );
    v2 = ln( data[0].inverse() * data[1] );
    v3 = ln( data[1].inverse() * data[2] );
    v4 = ln( data[2].inverse() * data[3] );
    disp[1] = (v1 - 3*v2 + 3*v3 - v4)/16;

    for( int i=2; i<n-2; i++ )
    {
        v1 = ln( data[i-2].inverse() * data[i-1] );
        v2 = ln( data[i-1].inverse() * data[i  ] );
        v3 = ln( data[i  ].inverse() * data[i+1] );
        v4 = ln( data[i+1].inverse() * data[i+2] );
        disp[i] = (v1 - 3*v2 + 3*v3 - v4)/16;
    }

    v1 = -ln( data[n-3].inverse() * data[n-2] );
    v2 = -ln( data[n-2].inverse() * data[n-1] );
	v3 = -ln( data[n-3].inverse() * data[n-2] );
    v4 = -ln( data[n-4].inverse() * data[n-3] );
    disp[n-2] = (v1 - 3*v2 + 3*v3 - v4)/16;
}

void
QmSmoothing( int n, quater *data, int numIter )
{
    vector v1, v2, v3, v4;

	int j;
    for( j=0; j<numIter; j++ )
    {
        v1 = ln( data[1].inverse() * data[2] );
        v2 = ln( data[0].inverse() * data[1] );
        v3 = ln( data[1].inverse() * data[2] );
        v4 = ln( data[2].inverse() * data[3] );
        data[1] = data[1] * exp( (v1 - 3*v2 + 3*v3 - v4)/16  );

        for( int i=2; i<n-2; i++ )
        {
            v1 = ln( data[i-2].inverse() * data[i-1] );
            v2 = ln( data[i-1].inverse() * data[i  ] );
            v3 = ln( data[i  ].inverse() * data[i+1] );
            v4 = ln( data[i+1].inverse() * data[i+2] );
            data[i] = data[i] * exp( (v1 - 3*v2 + 3*v3 - v4)/16  );
        }

        v1 = -ln( data[n-3].inverse() * data[n-2] );
        v2 = -ln( data[n-2].inverse() * data[n-1] );
		v3 = -ln( data[n-3].inverse() * data[n-2] );
        v4 = -ln( data[n-4].inverse() * data[n-3] );
        data[n-2] = data[n-2] * exp( (v1 - 3*v2 + 3*v3 - v4)/16  );
    }

    for( j=0; j<n; j++ ) data[j] = data[j].normalize();
}

void
QmSmoothing( int n, quater *data, int numIter, int start, int end )
{
    vector v1, v2, v3, v4;

	int j;
    for( j=0; j<numIter; j++ )
    {
		if ( start <= 1 && end >= 1 )
		{
	        v1 = ln( data[1].inverse() * data[2] );
			v2 = ln( data[0].inverse() * data[1] );
			v3 = ln( data[1].inverse() * data[2] );
			v4 = ln( data[2].inverse() * data[3] );
			data[1] = data[1] * exp( (v1 - 3*v2 + 3*v3 - v4)/16  );
		}

        for( int i=MAX(2,start); i<MIN(n-2,end+1); i++ )
        {
            v1 = ln( data[i-2].inverse() * data[i-1] );
            v2 = ln( data[i-1].inverse() * data[i  ] );
            v3 = ln( data[i  ].inverse() * data[i+1] );
            v4 = ln( data[i+1].inverse() * data[i+2] );
            data[i] = data[i] * exp( (v1 - 3*v2 + 3*v3 - v4)/16  );
        }

		if ( start <= n-2 && end >= n-2 )
		{
        	v1 = -ln( data[n-3].inverse() * data[n-2] );
        	v2 = -ln( data[n-2].inverse() * data[n-1] );
			v3 = -ln( data[n-3].inverse() * data[n-2] );
        	v4 = -ln( data[n-4].inverse() * data[n-3] );
        	data[n-2] = data[n-2] * exp( (v1 - 3*v2 + 3*v3 - v4)/16  );
		}
    }

    for( j=0; j<n; j++ ) data[j] = data[j].normalize();
}

//--------------------------------------------------------------//
//				New Versions ( Sharpening )						//
//--------------------------------------------------------------//

void
QmSharpening( int n, quater *data, vector *disp )
{
    vector v1, v2, v3, v4;

	disp[0] = disp[n-1] = vector(0,0,0);

    v1 = ln( data[1].inverse() * data[2] );
    v2 = ln( data[0].inverse() * data[1] );
    v3 = ln( data[1].inverse() * data[2] );
    v4 = ln( data[2].inverse() * data[3] );
    disp[1] = (v1 + 5*v2 - 5*v3 - v4)/16;

    for( int i=2; i<n-2; i++ )
    {
        v1 = ln( data[i-2].inverse() * data[i-1] );
        v2 = ln( data[i-1].inverse() * data[i  ] );
        v3 = ln( data[i  ].inverse() * data[i+1] );
        v4 = ln( data[i+1].inverse() * data[i+2] );
        disp[i] = (v1 + 5*v2 - 5*v3 - v4)/16;
    }

    v1 = -ln( data[n-3].inverse() * data[n-2] );
    v2 = -ln( data[n-2].inverse() * data[n-1] );
	v3 = -ln( data[n-3].inverse() * data[n-2] );
    v4 = -ln( data[n-4].inverse() * data[n-3] );
    disp[n-2] = (v1 + 5*v2 - 5*v3 - v4)/16;
}

void
QmSharpening( int n, quater *data, int numIter )
{
    vector v1, v2, v3, v4;

	int j;
    for( j=0; j<numIter; j++ )
    {
        v1 = ln( data[1].inverse() * data[2] );
        v2 = ln( data[0].inverse() * data[1] );
        v3 = ln( data[1].inverse() * data[2] );
        v4 = ln( data[2].inverse() * data[3] );
        data[1] = data[1] * exp( (v1 + 5*v2 - 5*v3 - v4)/16  );

        for( int i=2; i<n-2; i++ )
        {
            v1 = ln( data[i-2].inverse() * data[i-1] );
            v2 = ln( data[i-1].inverse() * data[i  ] );
            v3 = ln( data[i  ].inverse() * data[i+1] );
            v4 = ln( data[i+1].inverse() * data[i+2] );
            data[i] = data[i] * exp( (v1 + 5*v2 - 5*v3 - v4)/16 );
        }

        v1 = -ln( data[n-3].inverse() * data[n-2] );
        v2 = -ln( data[n-2].inverse() * data[n-1] );
		v3 = -ln( data[n-3].inverse() * data[n-2] );
        v4 = -ln( data[n-4].inverse() * data[n-3] );
        data[n-2] = data[n-2] * exp( (v1 + 5*v2 - 5*v3 - v4)/16  );
    }

    for( j=0; j<n; j++ ) data[j] = data[j].normalize();
}

void
QmSharpening( int n, quater *data, int numIter, int start, int end )
{
    vector v1, v2, v3, v4;

	int j;
    for( j=0; j<numIter; j++ )
    {
		if ( start <= 1 && end >= 1 )
		{
	        v1 = ln( data[1].inverse() * data[2] );
	        v2 = ln( data[0].inverse() * data[1] );
	        v3 = ln( data[1].inverse() * data[2] );
	        v4 = ln( data[2].inverse() * data[3] );
	        data[1] = data[1] * exp( (v1 + 5*v2 - 5*v3 - v4)/16  );
		}

        for( int i=MAX(2,start); i<MIN(n-2,end+1); i++ )
        {
            v1 = ln( data[i-2].inverse() * data[i-1] );
            v2 = ln( data[i-1].inverse() * data[i  ] );
            v3 = ln( data[i  ].inverse() * data[i+1] );
            v4 = ln( data[i+1].inverse() * data[i+2] );
            data[i] = data[i] * exp( (v1 + 5*v2 - 5*v3 - v4)/16 );
        }

		if ( start <= n-2 && end >= n-2 )
		{
        	v1 = -ln( data[n-3].inverse() * data[n-2] );
        	v2 = -ln( data[n-2].inverse() * data[n-1] );
			v3 = -ln( data[n-3].inverse() * data[n-2] );
        	v4 = -ln( data[n-4].inverse() * data[n-3] );
        	data[n-2] = data[n-2] * exp( (v1 + 5*v2 - 5*v3 - v4)/16  );
		}
    }

    for( j=0; j<n; j++ ) data[j] = data[j].normalize();
}

//--------------------------------------------------------------//
//				New Versions ( Binomial )						//
//--------------------------------------------------------------//

void
QmBinomial( int n, quater *data, vector *disp )
{
    vector v1, v2, v3, v4;

	disp[0] = disp[n-1] = vector(0,0,0);

    v1 = ln( data[1].inverse() * data[2] );
    v2 = ln( data[0].inverse() * data[1] );
    v3 = ln( data[1].inverse() * data[2] );
    v4 = ln( data[2].inverse() * data[3] );
    disp[1] = (-v1 - 5*v2 + 5*v3 + v4)/16;

    for( int i=2; i<n-2; i++ )
    {
        v1 = ln( data[i-2].inverse() * data[i-1] );
        v2 = ln( data[i-1].inverse() * data[i  ] );
        v3 = ln( data[i  ].inverse() * data[i+1] );
        v4 = ln( data[i+1].inverse() * data[i+2] );
    	disp[i] = (-v1 - 5*v2 + 5*v3 + v4)/16;
    }

    v1 = -ln( data[n-3].inverse() * data[n-2] );
    v2 = -ln( data[n-2].inverse() * data[n-1] );
	v3 = -ln( data[n-3].inverse() * data[n-2] );
    v4 = -ln( data[n-4].inverse() * data[n-3] );
    disp[n-2] = (-v1 - 5*v2 + 5*v3 + v4)/16;
}

void
QmBinomial( int n, quater *data, int numIter )
{
    vector v1, v2, v3, v4;

	int j;
    for( j=0; j<numIter; j++ )
    {
        v1 = ln( data[1].inverse() * data[2] );
        v2 = ln( data[0].inverse() * data[1] );
        v3 = ln( data[1].inverse() * data[2] );
        v4 = ln( data[2].inverse() * data[3] );
        data[1] = data[1] * exp( (-v1 - 5*v2 + 5*v3 + v4)/16  );

        for( int i=2; i<n-2; i++ )
        {
            v1 = ln( data[i-2].inverse() * data[i-1] );
            v2 = ln( data[i-1].inverse() * data[i  ] );
            v3 = ln( data[i  ].inverse() * data[i+1] );
            v4 = ln( data[i+1].inverse() * data[i+2] );
            data[i] = data[i] * exp( (-v1 - 5*v2 + 5*v3 + v4)/16 );
        }

        v1 = -ln( data[n-3].inverse() * data[n-2] );
        v2 = -ln( data[n-2].inverse() * data[n-1] );
		v3 = -ln( data[n-3].inverse() * data[n-2] );
        v4 = -ln( data[n-4].inverse() * data[n-3] );
        data[n-2] = data[n-2] * exp( (-v1 - 5*v2 + 5*v3 + v4)/16  );
    }

    for( j=0; j<n; j++ ) data[j] = data[j].normalize();
}

void
QmBinomial( int n, quater *data, int numIter, int start, int end )
{
    vector v1, v2, v3, v4;

	int j;
    for( j=0; j<numIter; j++ )
    {
		if ( start <= 1 && end >= 1 )
		{
	        v1 = ln( data[1].inverse() * data[2] );
	        v2 = ln( data[0].inverse() * data[1] );
	        v3 = ln( data[1].inverse() * data[2] );
	        v4 = ln( data[2].inverse() * data[3] );
	        data[1] = data[1] * exp( (-v1 - 5*v2 + 5*v3 + v4)/16  );
		}

        for( int i=MAX(2,start); i<MIN(n-2,end+1); i++ )
        {
            v1 = ln( data[i-2].inverse() * data[i-1] );
            v2 = ln( data[i-1].inverse() * data[i  ] );
            v3 = ln( data[i  ].inverse() * data[i+1] );
            v4 = ln( data[i+1].inverse() * data[i+2] );
            data[i] = data[i] * exp( (-v1 - 5*v2 + 5*v3 + v4)/16 );
        }

		if ( start <= n-2 && end >= n-2 )
		{
	        v1 = -ln( data[n-3].inverse() * data[n-2] );
	        v2 = -ln( data[n-2].inverse() * data[n-1] );
			v3 = -ln( data[n-3].inverse() * data[n-2] );
	        v4 = -ln( data[n-4].inverse() * data[n-3] );
	        data[n-2] = data[n-2] * exp( (-v1 - 5*v2 + 5*v3 + v4)/16  );
		}
    }

    for( j=0; j<n; j++ ) data[j] = data[j].normalize();
}

