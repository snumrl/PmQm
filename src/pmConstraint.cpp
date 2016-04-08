
#include "MATHCLASS/mathclass.h"
#include <assert.h>
#include "pm.h"

using namespace jhm;

void
PmConstraint::applyTransf( transf const& t )
{
	for( int i=0; i<num_entity; i++ )
		entity[i].applyTransf( t );
}

void
PmConstraint::applyTransf( int j, transf const& t )
{
	int i;
	for( i=0; i<num_entity; i++ )
		if ( entity[i].link == j ) break;

	if ( i<num_entity )
		entity[i].applyTransf( t );
}

void
PmConstraint::remove( int j )
{
	int i;
	for( i=0; i<num_entity; i++ )
		if ( entity[i].link == j ) break;

	if ( i<num_entity )
	{
		entity[i] = entity[num_entity-1];
		num_entity--;
	}
}

void
PmConstraint::push( int i, char m, transf t )
{
	assert( num_entity < MAX_CONSTRAINT_ENTITY-1 );

	entity[num_entity].mask  = m;
	entity[num_entity].link  = i;
	entity[num_entity].value = t;
	num_entity++;
}

void
PmConstraint::push( int i, transf t )
{
	assert( num_entity < MAX_CONSTRAINT_ENTITY-1 );

	entity[num_entity].mask  = PM_C_TRANSF;
	entity[num_entity].link  = i;
	entity[num_entity].value = t;
	num_entity++;
}

void
PmConstraint::push( int i, vector v )
{
	assert( num_entity < MAX_CONSTRAINT_ENTITY-1 );

	entity[num_entity].mask  = PM_C_POSITION;
	entity[num_entity].link  = i;
	entity[num_entity].value = transf( scaling(1), v );
	num_entity++;
}

void
PmConstraint::push( int i, quater q )
{
	assert( num_entity < MAX_CONSTRAINT_ENTITY-1 );

	entity[num_entity].mask  = PM_C_ORIENTATION;
	entity[num_entity].link  = i;
	entity[num_entity].value = transf( Quater2Matrix(q), vector(0,0,0) );
	num_entity++;
}

void
PmConstraint::push( int i, matrix m )
{
	assert( num_entity < MAX_CONSTRAINT_ENTITY-1 );

	entity[num_entity].mask  = PM_C_ORIENTATION;
	entity[num_entity].link  = i;
	entity[num_entity].value = transf( m, vector(0,0,0) );
	num_entity++;
}

void
PmConstraint::push_cog( vector v )
{
	assert( num_entity < MAX_CONSTRAINT_ENTITY-1 );

	entity[num_entity].mask  = PM_C_COG;
	entity[num_entity].link  = -1;
	entity[num_entity].value = transf( scaling(1), v );
	num_entity++;
}

void
PmConstraint::push_cog( m_real x, m_real z )
{
	assert( num_entity < MAX_CONSTRAINT_ENTITY-1 );

	entity[num_entity].mask  = PM_C_BALANCE;
	entity[num_entity].link  = -1;
	entity[num_entity].value = transf( scaling(1), vector(x,0,z) );
	num_entity++;
}

void
PmConstraint::operator=( PmConstraint const& c )
{
	this->num_entity = c.num_entity;

	for( int i=0; i<num_entity; i++ )
		this->entity[i] = c.entity[i];
}

transf
PmConstraint::get( int j ) const
{
	for( int i=0; i<num_entity; i++ )
		if ( entity[i].link == j ) return entity[i].value;

	return identity_transf;
}

transf
PmConstraint::getTransf( int j ) const
{
	for( int i=0; i<num_entity; i++ )
		if ( entity[i].link==j )
		{
			return entity[i].value;
		}

	return identity_transf;
}

vector
PmConstraint::getPosition( int j ) const
{
	for( int i=0; i<num_entity; i++ )
		if ( entity[i].link==j )
		{
			return entity[i].value.getTranslation();
		}

	return vector(0,0,0);
}

quater
PmConstraint::getOrientation( int j ) const
{
	for( int i=0; i<num_entity; i++ )
		if ( entity[i].link==j )
		{
			return entity[i].value.getRotation();
		}

	return quater(1,0,0,0);
}

char
PmConstraint::getMask( int j ) const
{
	for( int i=0; i<num_entity; i++ )
		if ( entity[i].link==j ) return entity[i].mask;

	return PM_UNCONSTRAINED;
}

void
PmConstraint::setMask( int j, char m )
{
	for( int i=0; i<num_entity; i++ )
		if ( entity[i].link==j ) entity[i].mask = m;
}

char
PmConstraint::isConstrained( int j ) const
{
	for( int i=0; i<num_entity; i++ )
		if ( entity[i].link==j ) return TRUE;

	return FALSE;
}

void
PmConstraint::set( int j, transf const& t )
{
	for( int i=0; i<num_entity; i++ )
		if ( entity[i].link==j ) entity[i].value = t;
}

void
PmConstraint::setTransf( int j, transf const& t )
{
	for( int i=0; i<num_entity; i++ )
		if ( entity[i].link==j )
		{
			entity[i].value = t;
		}
}

void
PmConstraint::setPosition( int j, vector const& v )
{
	for( int i=0; i<num_entity; i++ )
		if ( entity[i].link==j )
		{
			entity[i].value = transf( entity[i].value.getAffine(), v );
		}
}

void
PmConstraint::setOrientation( int j, quater const& q )
{
	for( int i=0; i<num_entity; i++ )
		if ( entity[i].link==j )
		{
			entity[i].value = transf( Quater2Matrix(q), entity[i].value.getTranslation() );
		}
}

void
PmConstraint::setOrientation( int j, matrix const& m )
{
	for( int i=0; i<num_entity; i++ )
		if ( entity[i].link==j )
		{
			entity[i].value = transf( m, entity[i].value.getTranslation() );
		}
}

int
PmConstraint::getDOC()
{
	int doc=0;

	for( int i=0; i<num_entity; i++ )
		doc += entity[i].getDOC();

	return doc;
}

PmMaskType
PmConstraint::getJointMask( PmHuman *h ) const
{
	int j;
	PmMaskType m = 0x00;

	for( int i=0; i<num_entity; i++ )
	{
		j = entity[i].link;
		m |= MaskBit( j );

		while( h->getParent( j ) != -1 )
		{
			j = h->getParent( j );
			m |= MaskBit( j );
		}
	}

	return m;	
}

void
PmConstraint::interpolate( m_real t, PmConstraint const& c1,
									 PmConstraint const& c2, int type )
{
	reset();

	int i, j;
	for( i=0; i<c1.getNumEntity(); i++ )
	for( j=0; j<c2.getNumEntity(); j++ )
	{
		PmConstraintEntity const& e1 = c1.entity[i];
		PmConstraintEntity const& e2 = c2.entity[j];

		if ( type == PM_C_AND && e1.link == e2.link )
		{
			if ( e1.mask & e2.mask )
				push( e1.link, e1.mask & e2.mask,
					  ::interpolate( t, e1.value, e2.value ) );
		}
	}

	if ( type == PM_C_OR )
	{
		for( i=0; i<c1.getNumEntity(); i++ )
		{
			PmConstraintEntity const& e = c1.entity[i];

			if ( ! isConstrained(e.link) )
				push( e.link, e.mask, e.value );
		}

		for( i=0; i<c2.getNumEntity(); i++ )
		{
			PmConstraintEntity const& e = c2.entity[i];

			if ( ! isConstrained(e.link) )
				push( e.link, e.mask, e.value );
		}
	}
}

