
#include "pm.h"

using namespace jhm;


PmMRmotion::PmMRmotion() : PmMotion()
{
	num_levels = 0;
}

PmMRmotion::PmMRmotion( PmHuman *h ) : PmMotion()
{
	num_levels = 0;
	setBody( h );
}

PmMRmotion::~PmMRmotion()
{
	if ( num_levels > 0 )
		for( int i=0; i<num_levels; i++ )
		{
			delete[] mask_list[i];
			delete[] knot_seqs[i];
		}
}

void
PmMRmotion::setBody( PmHuman* h )
{
	PmMotion::setBody( h );
	base_level.setBody( h );
}

PmMRmotion&
PmMRmotion::operator=( PmMRmotion const& m )
{
	if ( num_levels > 0 )
		for( int i=0; i<num_levels; i++ )
		{
			delete[] mask_list[i];
			delete[] knot_seqs[i];
		}

	(*(PmMotion*)this) = m;
    setSize( m.getSize() );

	num_levels	= m.num_levels;
	num_dof		= m.num_dof;
	mask		= m.mask;
	base_level	= m.base_level;

	for( int i=0; i<num_levels; i++ )
	{
		length[i]	 = m.length[i];
		displace[i]  = m.displace[i];
		mask_list[i] = new char[length[i]];
		knot_seqs[i] = new m_real[length[i]];

		for( int j=0; j<length[i]; j++ )
		{
			mask_list[i][j] = m.mask_list[i][j];
			knot_seqs[i][j] = m.knot_seqs[i][j];
		}
	}

	return (*this);
}


//------------------------------------------------------------------//
//				Convert PmMRmotion to PmLinearMotion				//
//------------------------------------------------------------------//


void
PmMRmotion::buildHierarchy( int level )
{
	assert( level <= PM_MOTION_MAX_LEVEL );

	int i, j, n, l;

	if ( num_levels > 0 )
		for( i=0; i<num_levels; i++ )
		{
			delete[] mask_list[i];
			delete[] knot_seqs[i];
		}

	num_levels = level;
	assert( base_level.getSize()>=4 );

	for( i=0; i<num_levels; i++ )
	{
		l = num_levels - i - 1;
		n = int(pow(2.,i+1) * (base_level.getSize()-1) + 1);

		length   [l] = n;
		mask_list[l] = new char[n];
		knot_seqs[l] = new m_real[n];
		displace [l].setSize( n );

		for( j=0; j<n; j++ )
		{
			if ( j%2 == 0 ) mask_list[l][j] = 1;
					   else mask_list[l][j] = 0;

			knot_seqs[l][j] = pow(2.,i+1) * j;
		}
	}

	setSize( length[0] );
}

//------------------------------------------------------------------//

void
PmMRmotion::buildHierarchy( PmLinearMotion const& lm, int max_level )
{
	assert( max_level <= PM_MOTION_MAX_LEVEL );

	static PmLinearMotion temp1;
	static PmLinearMotion temp2;
	static PmLinearMotion temp3;
	static PmLinearMotion temp4;

	int i;
	
	if ( num_levels > 0 )
		for( i=0; i<num_levels; i++ )
		{
			delete[] mask_list[i];
			delete[] knot_seqs[i];
		}

	mask = lm.getMask();
	num_dof = lm.getDOF();

	(*(PmMotion*)this) = lm;

	temp1 = lm;
	temp1.alignment();
	
	for ( i=0; i<max_level; i++ )
	{
		length   [i] = temp1.getSize();
		mask_list[i] = new char[ length[i] ];
		knot_seqs[i] = new m_real[ length[i] ];
		displace [i].setSize( length[i] );

		temp1.getKnots( knot_seqs[i] );

		temp2 = temp1;
		if ( PenvPresmoothing > 0 )
			temp2.smoothing( PenvPresmoothing, mask, 0, temp2.getSize() );

		if ( PenvMRAdownSamplingType == PENV_UNIFORM_DOWN_SAMPLING )
			temp3.uniformDownSampling( temp2, mask_list[i] );
		else
			temp3.nonuniformDownSampling( temp2, mask_list[i], knot_seqs[i] );

		if ( temp2.getSize()==temp3.getSize() ||
			 temp3.getSize() < 4 ) break;

		temp4.subdivide( temp3, length[i], mask_list[i], knot_seqs[i] );

		displace[i].difference( temp1, temp4 );
		temp1 = temp3;
	}

	num_levels = i;
	base_level = temp1;
}


//------------------------------------------------------------------//
//				Convert PmMRmotion to PmLinearMotion				//
//------------------------------------------------------------------//


static PmLinearMotion temp;

void
PmMRmotion::linearize( PmLinearMotion& lm ) const
{
	lm = base_level;

	int i;
	for( i=this->getNumLevels()-1; i>=0; i-- )
	{
		temp.subdivide( lm, length[i], mask_list[i], knot_seqs[i] );

		lm = temp;
		lm += displace[i];
	}

	for( i=0; i<lm.getSize(); i++ )
		lm.constraints[i] = this->constraints[i];
}

void
PmMRmotion::linearize( PmLinearMotion& lm, int n, float* factors ) const
{
	lm = base_level;

	int i, j;
	for( i=this->getNumLevels()-1, j=0; i>=0; i--, j++ )
	{
		temp.subdivide( lm, length[i], mask_list[i], knot_seqs[i] );

		lm = temp;

		if ( j<n )
			lm.addDisplacement( displace[i], factors[j] );
		else
			lm += displace[i];
	}

	for( i=0; i<lm.getSize(); i++ )
		lm.constraints[i] = this->constraints[i];
}

void
PmMRmotion::linearize( PmLinearMotion& lm, int offset ) const
{
	lm = base_level;

	int i;
	for( i=this->getNumLevels()-1; i>=offset; i-- )
	{
		temp.subdivide( lm, length[i], mask_list[i], knot_seqs[i] );

		lm = temp;
		lm += displace[i];
	}

	if ( offset==0.0 )
		for( i=0; i<lm.getSize(); i++ )
			lm.constraints[i] = this->constraints[i];
}

void
PmMRmotion::linearize( PmLinearMotion& lm, m_real offset ) const
{
	if ( offset > this->getNumLevels() )
	 	 offset = this->getNumLevels();

	lm = base_level;

	int i;
	for( i=this->getNumLevels()-1; i>=ceil(offset); i-- )
	{
		temp.subdivide( lm, length[i], mask_list[i], knot_seqs[i] );

		lm = temp;
		lm += displace[i];
	}

	if ( i>0 )
	{
		temp.subdivide( lm, length[i], mask_list[i], knot_seqs[i] );
		lm = temp;
		lm.addDisplacement( displace[i], ceil(offset) - offset );
		i--;
	}

	for( ; i>=0; i-- )
	{
		temp.subdivide( lm, length[i], mask_list[i], knot_seqs[i] );

		lm = temp;
	}

	if ( offset<1.0 )
		for( i=0; i<lm.getSize(); i++ )
			lm.constraints[i] = this->constraints[i];
}

//------------------------------------------------------------------//
//				Stitching Multiresolution Motions					//
//------------------------------------------------------------------//

transf
PmMRmotion::stitch( PmMRmotion const& m1, PmMRmotion const& m2 )
{
	if ( m1.getNumLevels()!=m2.getNumLevels() ) return identity_transf;

	this->base_level = m1.base_level;
	transf calib = this->base_level.stitch( m2.base_level );

	int i, j;

	if ( num_levels > 0 )
		for( i=0; i<num_levels; i++ )
		{
			delete[] mask_list[i];
			delete[] knot_seqs[i];
		}
	this->num_levels = m1.getNumLevels();

	for( i=0; i<m1.getNumLevels(); i++ )
	{
		int n = m1.length[i] + m2.length[i] - 1;
		this->length[i]    = n;
		this->mask_list[i] = new char[n];
		this->knot_seqs[i] = new m_real[n];
		this->displace[i].setSize(n);

		for( j=0; j<m1.length[i]; j++ )
		{
			this->mask_list[i][j] = m1.mask_list[i][j];
			this->knot_seqs[i][j] = m1.knot_seqs[i][j];
			this->displace[i].vectors[j] = m1.displace[i].vectors[j];
		}

		int d = m1.length[i]-1;
		m_real offset = m1.knot_seqs[i][d] - m2.knot_seqs[i][0];

		this->displace[i].vectors[d]
			.interpolate( 0.5, m1.displace[i].vectors[d],
							   m2.displace[i].vectors[0] );

		for( j=1; j<m2.length[i]; j++ )
		{
			this->mask_list[i][d+j] = m2.mask_list[i][j];
			this->knot_seqs[i][d+j] = m2.knot_seqs[i][j] + offset;
			this->displace[i].vectors[d+j] = m2.displace[i].vectors[j];
		}
	}

	setSize( this->length[0] );

	return calib;
}
