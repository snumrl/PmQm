
#include "pm.h"

using namespace jhm;

PmVectorArray::PmVectorArray()
{
	size = oSize = 0;
	vectors = NULL;
}

PmVectorArray::~PmVectorArray()
{
	if ( oSize>0 ) delete[] vectors;
}

void
PmVectorArray::setSize( int n )
{
    assert( n>=0 );

    if ( oSize < n )
    {
        if ( oSize>0 ) delete[] vectors;

		oSize = n;
		vectors = new PmVector[n];
    }

	size = n;
}

PmMaskType
PmVectorArray::getMask() const
{
    PmMaskType mask = 0x00;
    for( int i=0; i<size; i++ )
        mask |= getVector(i).getMask();

    return mask;
}


PmVectorArray&
PmVectorArray::operator=(  PmVectorArray const& v )
{
	this->setSize( v.getSize() );
		
	for( int i=0; i<this->getSize(); i++ )
    	this->vectors[i] = v.vectors[i];

	return (*this);
}

PmVectorArray&
PmVectorArray::operator+=( PmVectorArray const& v )
{
	assert( this->getSize()==v.getSize() );

	for( int i=0; i<this->getSize(); i++ )
		this->vectors[i] += v.getVector(i);
	
	return (*this);
}

PmVectorArray&
PmVectorArray::operator*=( m_real d )
{
	for( int i=0; i<this->getSize(); i++ )
    	this->vectors[i] *= d;

	return (*this);
}

PmVectorArray&
PmVectorArray::operator/=( m_real d )
{
	for( int i=0; i<this->getSize(); i++ )
    	this->vectors[i] /= d;

	return (*this);
}

PmVectorArray&
PmVectorArray::difference( PmLinearMotion const& m1, PmLinearMotion const& m2 )
{
	assert( m1.getSize()==m2.getSize() );

	setSize( m1.getSize() );

	for( int i=0; i<this->getSize(); i++ )
		this->vectors[i].difference( m1.getPosture(i), m2.getPosture(i) );
	
	return (*this);
}

PmVectorArray&
PmVectorArray::difference( PmVectorArray const& v1, PmVectorArray const& v2 )
{
	assert( v1.getSize()==v2.getSize() );

	setSize( v1.getSize() );

	for( int i=0; i<this->getSize(); i++ )
		this->vectors[i].difference( v1.getVector(i), v2.getVector(i) );
	
	return (*this);
}

void
PmVectorArray::getLinearVectors( vector* p ) const
{
	for( int i=0; i<getSize(); i++ )
		p[i] = this->vectors[i].getLinearVector();
}

void
PmVectorArray::getAngularVectors( int link, vector* p ) const
{
	for( int i=0; i<getSize(); i++ )
		p[i] = this->vectors[i].getAngularVector( link );
}

void
PmVectorArray::setLinearVectors( vector* p )
{
	for( int i=0; i<getSize(); i++ )
		this->vectors[i].setLinearVector( p[i] );
}

void
PmVectorArray::setAngularVectors( int link, vector* p )
{
	for( int i=0; i<getSize(); i++ )
		this->vectors[i].setAngularVector( link, p[i] );
}

void
PmVectorArray::amplify( m_real factor, PmMaskType m )
{
	for( int i=0; i<getSize(); i++ )
	{
		PmVector &v = vectors[i];

		if ( m & MaskBit(PmHuman::PELVIS) )
			v.setLinearVector( v.getLinearVector() * factor );

		for( int j=0; j<PM_HUMAN_NUM_LINKS; j++ )
		if ( m & MaskBit(j) )
			v.setAngularVector( j, v.getAngularVector(j) * factor );
	}
}

