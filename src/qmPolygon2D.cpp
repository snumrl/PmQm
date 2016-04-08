

#include <stdlib.h>
#include "MATHCLASS/mathclass.h"
#include "qmPolygon2D.h"

using namespace jhm;

QmPolygon2D::QmPolygon2D()
{
	numVertices = 0;
	numStorage = 0;
	vertices = NULL;
}

QmPolygon2D::QmPolygon2D( int n )
{
	numVertices = n;
	numStorage = n;
	vertices = new position[n];
}

QmPolygon2D::~QmPolygon2D()
{
	if ( numVertices>0 ) delete[] vertices;
}

void
QmPolygon2D::setNumVertices( int n )
{
	if ( numStorage < n )
	{
		if ( numStorage>0 ) delete[] vertices;
		numStorage = n;
		if ( numStorage>0 ) vertices = new position[numStorage];
	}
	numVertices = n;
}

static int
compare( const void* a, const void* b )
{
	if ( ((position*)a)->x() < ((position*)b)->x() ) return -1;
	if ( ((position*)a)->x() > ((position*)b)->x() ) return 1;

	if ( ((position*)a)->z() < ((position*)b)->z() ) return -1;
	return 1;
}

void
QmPolygon2D::convexHull( int input_n, position* input )
{
	setNumVertices( input_n + 1 );

	qsort( input, input_n, sizeof(position), compare );

	int n = 0;
	vertices[n++] = input[0];
	vertices[n++] = input[1];

	int i;
	for( i=2; i<input_n; i++ )
	{
		vertices[n++] = input[i];

		while( n>2 &&
			   ((vertices[n-2]-vertices[n-3]) * (vertices[n-1]-vertices[n-2])).y()>0 )
		{
			n--;
			vertices[n-1] = vertices[n];
		}
	}

	int temp_n = n-1;
	vertices[n++] = input[input_n-2];

	for( i=input_n-3; i>=0; i-- )
	{
		vertices[n++] = input[i];

		while( (n-temp_n) > 2 &&
			   ((vertices[n-2]-vertices[n-3]) * (vertices[n-1]-vertices[n-2])).y()>0 )
		{
			n--;
			vertices[n-1] = vertices[n];
		}
	}

	setNumVertices( n );
}

int
QmPolygon2D::inclusion( position const& p ) const
{
	for( int i=0; i<numVertices-1; i++ )
	{
		if ( ((vertices[i+1]-vertices[i]) * (p - vertices[i])).y() > 0 )
			return FALSE;
	}

	return TRUE;
}

m_real
QmPolygon2D::distance( position const& p ) const
{
	vector v1, v2;
	m_real dist = 0.0;

	for( int i=0; i<numVertices-1; i++ )
	{
		v1 = vertices[i+1]-vertices[i];
		v2 = p - vertices[i];

		if ( (v1 * v2).y() > 0 )
			dist = MAX( dist, (v1*(v1%v2)/(v1%v1) - v2).length() );
	}

	return dist;
}

vector
QmPolygon2D::gradient( position const& p ) const
{
	vector v1, v2;
	vector grad(0,0,0);

	for( int i=0; i<numVertices-1; i++ )
	{
		v1 = vertices[i+1]-vertices[i];
		v2 = p - vertices[i];

		if ( (v1 * v2).y() > 0 )
			grad += (v1*(v1%v2)/(v1%v1)) - v2;
	}

	return grad;
}
