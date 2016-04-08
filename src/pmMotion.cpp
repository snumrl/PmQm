
#include "pm.h"

using namespace jhm;

PmMotion::PmMotion()
{
	body = NULL;
	size = oSize = 0;
}

PmMotion::~PmMotion()
{
	if ( oSize>0 ) delete[] constraints;
}

void
PmMotion::setSize( int n )
{
	assert( n>=0 && n<PM_MAX_FRAMES );

	if ( oSize < n )
	{
		if ( oSize>0 ) delete[] constraints;
		constraints = new PmConstraint[n];

		oSize = n;
	}

	size = n;
}

void
PmMotion::extend( int n )
{
	if ( n>0 && n>oSize )
	{
		PmConstraint *temp = new PmConstraint[n];

		int i;
		for( i=0; i<getSize() && i<n; i++ )
			temp[i] = constraints[i];

		if ( getSize()>0 )
		{
			for( i=getSize(); i<n; i++ )
				temp[i] = temp[getSize()-1];

			delete[] constraints;
		}

		constraints = temp;

		size = oSize = n;
	}
	else
		setSize( n );
}

PmMotion&
PmMotion::operator=( PmMotion const& m )
{
	this->setBody( m.getBody() );
	this->setSize( m.getSize() );

	for( int i=0; i<this->getSize(); i++ )
		this->constraints[i] = m.constraints[i];

	return (*this);
}

