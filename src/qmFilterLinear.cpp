

#include "MATHCLASS/mathclass.h"



using namespace jhm;

//--------------------------------------------------------------//
//              New Versions ( Smoothing )                      //
//--------------------------------------------------------------//

void
QmSmoothing( int n, vector *data, vector *disp )
{
	disp[0] = disp[n-1] = vector(0,0,0);

    disp[1] = (2*data[0] + 11*data[1] + 4*data[2] - data[3])/16.0 - data[1];

    for( int i=2; i<n-2; i++ )
        disp[i] = (- data[i-2] + 4*data[i-1] + 10*data[i] +
					4*data[i+1] - data[i+2])/16.0 - data[i];

    disp[n-2] = (2*data[n-1] + 11*data[n-2] + 4*data[n-3] -
					data[n-4])/16.0 - data[n-2];
}

void
QmSmoothing( int n, vector *data, int numIter )
{
    for( int j=0; j<numIter; j++ )
    {
        data[1] = (2*data[0] + 11*data[1] + 4*data[2] - data[3])/16.0;

        for( int i=2; i<n-2; i++ )
            data[i] = (- data[i-2] + 4*data[i-1]
					   + 10*data[i] + 4*data[i+1] - data[i+2])/16.0;

        data[n-2] = (2*data[n-1] + 11*data[n-2] + 4*data[n-3] - data[n-4])/16.0;
    }
}

void
QmSmoothing( int n, vector *data, int numIter, int start, int end )
{
    for( int j=0; j<numIter; j++ )
    {
		if ( start <= 1 && end >= 1 )
			data[1] = (2*data[0] + 11*data[1] + 4*data[2] - data[3])/16.0;

		for( int i=MAX(2,start); i<MIN(n-2,end+1); i++ )
            data[i] = (- data[i-2] + 4*data[i-1]
					   + 10*data[i] + 4*data[i+1] - data[i+2])/16.0;

        if ( start <= n-2 && end >= n-2 )
			data[n-2] = (2*data[n-1] + 11*data[n-2] + 4*data[n-3] - data[n-4])/16.0;
    }
}

//--------------------------------------------------------------//
//              New Versions ( Binomial )                       //
//--------------------------------------------------------------//

void
QmBinomial( int n, vector *data, vector *disp )
{
	disp[0] = disp[n-1] = vector(0,0,0);

    disp[1] = (6*data[0] + 5*data[1] + 4*data[2] + data[3])/16.0 - data[1];

    for( int i=2; i<n-2; i++ )
        disp[i] = (data[i-2] + 4*data[i-1] + 6*data[i] +
					4*data[i+1] + data[i+2])/16.0 - data[i];

    disp[n-2] = (6*data[n-1] + 5*data[n-2] + 4*data[n-3] +
					data[n-4])/16.0 - data[n-2];
}

void
QmBinomial( int n, vector *data, int numIter )
{
    for( int j=0; j<numIter; j++ )
    {
        data[1] = (6*data[0] + 5*data[1] + 4*data[2] + data[3])/16.0;

        for( int i=2; i<n-2; i++ )
            data[i] = (data[i-2] + 4*data[i-1]
					+ 6*data[i] + 4*data[i+1] + data[i+2])/16.0;

        data[n-2] = (6*data[n-1] + 5*data[n-2] + 4*data[n-3] + data[n-4])/16.0;
    }
}

void
QmBinomial( int n, vector *data, int numIter, int start, int end )
{
    for( int j=0; j<numIter; j++ )
    {
        if ( start <= 1 && end >= 1 )
			data[1] = (6*data[0] + 5*data[1] + 4*data[2] + data[3])/16.0;

		for( int i=MAX(2,start); i<MIN(n-2,end+1); i++ )
            data[i] = (data[i-2] + 4*data[i-1]
					+ 6*data[i] + 4*data[i+1] + data[i+2])/16.0;

        if ( start <= n-2 && end >= n-2 )
			data[n-2] = (6*data[n-1] + 5*data[n-2] + 4*data[n-3] + data[n-4])/16.0;
    }
}

//--------------------------------------------------------------//
//              New Versions ( Sharpening )                     //
//--------------------------------------------------------------//

void
QmSharpening( int n, vector *data, vector *disp )
{
	disp[0] = disp[n-1] = vector(0,0,0);

    disp[1] = (-6*data[0] + 27*data[1] - 4*data[2] - data[3])/16.0 - data[1];

    for( int i=2; i<n-2; i++ )
        disp[i] = (- data[i-2] - 4*data[i-1] + 26*data[i] -
					4*data[i+1] - data[i+2])/16.0 - data[i];

    disp[n-2] = (-6*data[n-1] + 27*data[n-2] - 4*data[n-3] -
					data[n-4])/16.0 - data[n-2];
}

void
QmSharpening( int n, vector *data, int numIter )
{
    for( int j=0; j<numIter; j++ )
    {
        data[1] = (-6*data[0] + 27*data[1] - 4*data[2] - data[3])/16.0;

        for( int i=2; i<n-2; i++ )
            data[i] = (- data[i-2] - 4*data[i-1]
					   + 26*data[i] - 4*data[i+1] - data[i+2])/16.0;

        data[n-2] = (-6*data[n-1] + 27*data[n-2] - 4*data[n-3] - data[n-4])/16.0;
    }
}

void
QmSharpening( int n, vector *data, int numIter, int start, int end )
{
    for( int j=0; j<numIter; j++ )
    {
        if ( start <= 1 && end >= 1 )
			data[1] = (-6*data[0] + 27*data[1] - 4*data[2] - data[3])/16.0;

		for( int i=MAX(2,start); i<MIN(n-2,end+1); i++ )
            data[i] = (- data[i-2] - 4*data[i-1]
					   + 26*data[i] - 4*data[i+1] - data[i+2])/16.0;

        if ( start <= n-2 && end >= n-2 )
			data[n-2] = (-6*data[n-1] + 27*data[n-2] - 4*data[n-3] - data[n-4])/16.0;
    }
}

