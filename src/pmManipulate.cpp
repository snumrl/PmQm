
#include "pm.h"
#include "qmFilter.h"
#include "qmApproximate.h"

using namespace jhm;

//------------------------------------------------------//
//														//
//			Direct Manipulation for Motion Data			//
//														//
//------------------------------------------------------//

#define ERROR_TOLERANCE 0.01

QmApproximate	appr;
QmScatteredData	displacement[PM_MAX_FRAMES];
PmPosture		pos_buf[PM_MAX_FRAMES];
extern vector	p_buf[PM_MAX_FRAMES];
extern quater	q_buf[PM_MAX_FRAMES];
extern vector	p1_buf[PM_MAX_FRAMES];

//------------------------------------------------------//

void
PmLinearMotion::directManipScale( int time, m_real scale, int range )
{
	int offset = time - range;

	int min_t = MAX( 0, time - range );
	int max_t = MIN( this->getSize(), time + range + 1 );

	displacement[0].position = time - offset;
	displacement[0].value = vector( scale - getPosture(time).getScale(), 0, 0 );

	appr.setSize( 6 );
	appr.approximate( 2*range + 1, 1, displacement );
	appr.discretize( 2*range + 1, p1_buf );

	for( int j=min_t; j<max_t; j++ )
	{
		PmPosture &p = getPosture(j);
		p.setScale( p.getScale() + p1_buf[j - offset].x() );
	}
}

//------------------------------------------------------//

void
PmLinearMotion::adjustRootTrajectory(
		PmConstraint* constraint, int min_knots, int level )
{
	PmMaskType mask = this->getMask();

	for( int nknots=min_knots, l=0; l<level; nknots*=2, l++)
	{
		vector d;
		int count = 0;

		int j;
		for( j=0; j<this->getSize(); j++ )
		{
			d = vector(0,0,0);
			int flag = 0;
			
			for( int k=0; k<constraint[j].getNumEntity(); k++ )
			{
				flag++;
				PmConstraintEntity& e = constraint[j].entity[k];
				d += e.value.getTranslation() -
					 getPosture(j).getGlobalTranslation( e.link );
			}

			if ( flag>0 )
			{
				displacement[count].position = j;
				displacement[count].value = d/flag;
				count++;
			}
		}

		appr.setSize( nknots );
		appr.approximate( this->getSize(), count, displacement );
		appr.discretize( this->getSize(), p1_buf );

		this->getTranslation( p_buf );
		for( j=0; j<this->getSize(); j++ )
			p_buf[j] += p1_buf[j];
		this->setTranslation( p_buf );
	}
}

void
PmLinearMotion::adjustRootTrajectory( int time,
		PmConstraint* constraint, int min_knots, int level )
{
	PmMaskType mask = this->getMask();

	for( int nknots=min_knots, l=0; l<level; nknots*=2, l++)
	{
		int j;
		vector d;

		int count = 0;
		displacement[count].position = time;
		displacement[count].value = vector(0,0,0);
		count++;

		for( j=0; j<this->getSize(); j++ )
		{
			d = vector(0,0,0);
			int flag = 0;
			
			for( int k=0; k<constraint[j].getNumEntity(); k++ )
			{
				flag++;
				PmConstraintEntity& e = constraint[j].entity[k];
				d += e.value.getTranslation() -
					 getPosture(j).getGlobalTranslation( e.link );
			}

			if ( flag>0 && ABS(j-time)>5 )
			{
				displacement[count].position = j;
				displacement[count].value = d/flag;
				count++;
			}
		}

		appr.setSize( nknots );
		appr.approximate( this->getSize(), count, displacement );
		appr.discretize( this->getSize(), p1_buf );

		this->getTranslation( p_buf );
		for( j=0; j<this->getSize(); j++ )
			p_buf[j] += p1_buf[j];
		this->setTranslation( p_buf );
	}
}

//------------------------------------------------------//

void
PmLinearMotion::retarget( PmConstraint* constraint,
		int min_knots, int level, char fine_adjustment,
		char optimize )
{
	PmMaskType mask = this->getMask();

	int		i, j;
	int		count;
	vector	d;
	quater	q1, q2;

	assert( constraint );

	for( int nknots=min_knots, l=0; l<level; nknots*=2, l++)
	{
		//
		// Compute error
		//
		for( j=0; j<this->getSize(); j++ )
		{
			pos_buf[j] = getPosture(j);

			if ( PenvIKsolverType == PENV_IK_NUMERICAL )
				pos_buf[j].ik_body( constraint[j] );
			else
				pos_buf[j].ik_torso( constraint[j], optimize );
		}

		//
		// Translation handling
		//
		if ( mask & MaskBit(PmHuman::PELVIS) )
		{
			count = 0;
			for( j = 0; j < this->getSize(); j++ )
			{
				d = pos_buf[j].getTranslation() -
					getPosture(j).getTranslation();

				if ( d.length() > ERROR_TOLERANCE )
				{
					displacement[count].position = j;
					displacement[count].value = d;
					count++;
				}
			}

			appr.setSize( nknots );
			appr.approximate( this->getSize(), count, displacement );
			appr.discretize( this->getSize(), p1_buf );

			this->getTranslation( p_buf );
			for( j = 0; j < this->getSize(); j++ )
				p_buf[j] += p1_buf[j];
			this->setTranslation( p_buf );
		}

		//
		// Rotation handling
		//
		for( i=0; i<PM_HUMAN_NUM_LINKS; i++ )
		if ( mask & MaskBit(i) )
		{
			count = 0;
			for( j=0; j<this->getSize(); j++ )
			{
				q1 = pos_buf[j].getRotation(i);
				q2 = getPosture(j).getRotation(i);

				if ( q1%q2<0 )
					d = difference( q1,-q2 );
				else
					d = difference( q1, q2 );

				if ( d.length() > ERROR_TOLERANCE )
				{
					displacement[count].position = j;
					displacement[count].value = d;
					count++;
				}
			}

			appr.setSize( nknots );
			appr.approximate( this->getSize(), count, displacement );
			appr.discretize( this->getSize(), p1_buf );

			this->getRotation( i, q_buf );
			for( j = 0; j < this->getSize(); j++ )
				q_buf[j] = q_buf[j] * exp(p1_buf[j]);
			this->setRotation( i, q_buf );
		}
	}

	if ( fine_adjustment )
		for( j = 0; j < this->getSize(); j++ )
		{
			if ( PenvIKsolverType == PENV_IK_NUMERICAL )
				this->getPosture(j).ik_body( constraint[j] );
			else
				this->getPosture(j).ik_torso( constraint[j], optimize );
		}
}

//------------------------------------------------------//

void
PmLinearMotion::directManip( int time, PmPosture const& target,
		PmConstraint* constraint, int range, int level,
		char fine_adjustment, char optimize, int repeat )
{
	int offset = time - range;

	int min_t = MAX( 0, time - range );
	int max_t = MIN( this->getSize(), time + range + 1 );

	PmMaskType mask = this->getMask();

	int		i, j;
	int		count;
	vector	d;
	quater	q1, q2;

	for( int nknots=6, l=0; l<level; nknots*=2, l++)
	for( int k=0; k<repeat; k++ )
	{
		//
		// Compute Key postures
		//
		for( j=min_t; j<max_t; j++ )
		if ( constraint && l>0  )
		{
			pos_buf[j] = getPosture(j);
			pos_buf[j].ik_torso( constraint[j], optimize );
//			pos_buf[j].ik_body( constraint[j] );
		}
		else
			pos_buf[j] = getPosture(j);

		pos_buf[time] = target;

		//
		// Translation handling
		//
		if ( mask & MaskBit(PmHuman::PELVIS) )
		{
			count = 0;

			for( j=min_t; j<max_t; j++ )
			{
				d = pos_buf[j].getTranslation() -
					getPosture(j).getTranslation();

				if ( d.length() > ERROR_TOLERANCE )
				{
					displacement[count].position = j - offset;
					displacement[count].value = d;
					count++;
				}
			}

			appr.setSize( nknots );
			appr.approximate( 2*range + 1, count, displacement );
//			appr.approximateLeastSquare( 2*range + 1, count, displacement );
			appr.discretize( 2*range + 1, p1_buf );

			this->getTranslation( p_buf );
			for( j=min_t; j<max_t; j++ )
				p_buf[j] += p1_buf[j - offset];
			this->setTranslation( p_buf );
		}

		//
		// Rotation handling
		//
		for( i=0; i<PM_HUMAN_NUM_LINKS; i++ )
		if ( mask & MaskBit(i) )
		{
			count = 0;
			for( j=min_t; j<max_t; j++ )
			{
				q1 = pos_buf[j].getRotation(i);
				q2 = getPosture(j).getRotation(i);

				if ( q1%q2<0 )
					d = difference( q1,-q2 );
				else
					d = difference( q1, q2 );

				if ( d.length() > ERROR_TOLERANCE )
				{
					displacement[count].position = j - offset;
					displacement[count].value = d;
					count++;
				}
			}

			appr.setSize( nknots );
			appr.approximate( 2*range + 1, count, displacement );
//			appr.approximateLeastSquare( 2*range + 1, count, displacement );
			appr.discretize( 2*range + 1, p1_buf );

			this->getRotation( i, q_buf );
			for( j=min_t; j<max_t; j++ )
				q_buf[j] = q_buf[j] * exp(p1_buf[j-offset]);
			this->setRotation( i, q_buf );
		}
	}

	if ( fine_adjustment && constraint )
		for( j=min_t; j<max_t; j++ )
			this->getPosture(j).ik_torso( constraint[j], optimize );
}

//------------------------------------------------------//

//  Bernstein Basis
static m_real B0( m_real t ) { return (1-t)*(1-t)*(1-t); }
static m_real B1( m_real t ) { return 3.0 * t * (1-t) * (1-t); }
static m_real B2( m_real t ) { return 3.0 * t * t * (1-t); }
static m_real B3( m_real t ) { return t*t*t; }

void
PmLinearMotion::headWarp( PmPosture const& targetP,
						  PmVector  const& targetV )
{
	PmMaskType mask = this->getMask();

	int		i, j;
	vector	d, dp, dv;
	m_real	t;
	quater	q1, q2;

	//
	// Translation handling
	//
	if ( mask & MaskBit(PmHuman::PELVIS) )
	{
		dp = targetP.getTranslation() -
			 getPosture(0).getTranslation();

		dv = targetV.getLinearVector() -
			 getLinearVelocity( PmHuman::PELVIS, 0 );

		for( j=0; j<this->getSize(); j++ )
		{
			PmPosture &p = getPosture(j);

			t = j;
			t /= this->getSize()-1;

			d = dp * B0(t) + (dp + dv/3) * B1(t);

			p.setTranslation( p.getTranslation() + d );
		}
	}

	//
	// Rotation handling
	//
	for( i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	if ( mask & MaskBit(i) )
	{
		q1 = targetP.getRotation(i);
		q2 = getPosture(0).getRotation(i);

		if ( q1%q2<0 )
			dp = difference( q1,-q2 );
		else
			dp = difference( q1, q2 );

		dv = targetV.getAngularVector(i) -
			 getAngularVelocity( i, 0 );

		for( j=0; j<this->getSize(); j++ )
		{
			PmPosture &p = getPosture(j);

			t = j;
			t /= this->getSize()-1;

			d = dp * B0(t) + (dp + dv/3) * B1(t);

			p.setRotation( i, p.getRotation(i) * exp(d) );
		}
	}
}


void
PmLinearMotion::tailWarp( PmPosture const& targetP,
						  PmVector  const& targetV )
{
	PmMaskType mask = this->getMask();

	int		i, j;
	vector	d, dp, dv;
	m_real	t;
	quater	q1, q2;

	//
	// Translation handling
	//
	if ( mask & MaskBit(PmHuman::PELVIS) )
	{
		dp = targetP.getTranslation() -
			 getPosture(this->getSize()-1).getTranslation();

		dv = targetV.getLinearVector() -
			 getLinearVelocity( PmHuman::PELVIS, this->getSize()-1 );

		for( j=0; j<this->getSize(); j++ )
		{
			PmPosture &p = getPosture(j);

			t = j;
			t /= this->getSize()-1;

			d = dp * B3(t) + (dp - dv/3) * B2(t);

			p.setTranslation( p.getTranslation() + d );
		}
	}

	//
	// Rotation handling
	//
	for( i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	if ( mask & MaskBit(i) )
	{
		q1 = targetP.getRotation(i);
		q2 = getPosture(this->getSize()-1).getRotation(i);

		if ( q1%q2<0 )
			dp = difference( q1,-q2 );
		else
			dp = difference( q1, q2 );

		dv = targetV.getAngularVector(i) -
			 getAngularVelocity( i, this->getSize()-1 );

		for( j=0; j<this->getSize(); j++ )
		{
			PmPosture &p = getPosture(j);

			t = j;
			t /= this->getSize()-1;

			d = dp * B3(t) + (dp - dv/3) * B2(t);

			p.setRotation( i, p.getRotation(i) * exp(d) );
		}
	}
}

