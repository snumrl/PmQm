
#include "pm.h"

using namespace jhm;

//----------------------------------------------------------//
//															//
//			  Inverse Kinematics with Balance				//
//															//
//----------------------------------------------------------//

#define DAMPING			1.0
#define	BALANCE			10.0

static m_real  energyFunc( vectorN const& );
static m_real  gradientFuncLS( vectorN const&, vectorN& );
static m_real  gradientFunc( vectorN const&, vectorN& );
static m_real  computeJacobian();

static int				num_dof,
						num_equations,
						num_unknowns;
static PmMaskType		link_mask;
static PmHuman*			oBody;
static PmPosture		oPosture;
static PmPosture		tPosture;
static PmConstraint		constraint;
static QmPolygon2D		support_polygon;

static matrixN			J;
static vectorN			b;

int
PmPosture::ik_balance( PmConstraint const& c )
{
	if ( this->getBody() )
		return ik_body( c, this->getMask() & c.getJointMask(this->getBody()) );

	return ik_balance( c, this->getMask() );
}

int
PmPosture::ik_balance( PmConstraint const& c, PmMaskType m )
{
	constraint = c;
	link_mask = this->getMask() & m;
	oBody = getBody();

	oPosture = (*this);
	tPosture = (*this);

	oPosture.setMask( link_mask );
	tPosture.setMask( link_mask );

	num_dof = oPosture.getDOF();

	int i;
	static vectorN d; d.setSize(num_dof);
	for( i=0; i<num_dof; i++ ) d[i] = 0.0;

	oPosture.getSupportPolygon( constraint, support_polygon );

	m_real f;
	int iter = 0;

	if ( energyFunc(d) > PenvIKerrorBound )
	{
		if ( PenvIKsolverMethod == PENV_IK_CG_METHOD )
			frprmn( d, num_dof, PenvIKtolerance, iter, f, energyFunc, gradientFunc );
		else
			gradient_descent( d, num_dof, PenvIKtolerance, iter, f, energyFunc, gradientFuncLS );
	}

	oPosture.addDisplacement( d );
	this->copyUnder( oPosture );

	transf t;
	for( i=0; i<c.getNumEntity(); i++ )
	{
		t = c.entity[i].value * getBaseTransf(c.entity[i].link).inverse();
		rotate[c.entity[i].link] = t.getRotation();
	}

	if (c.getMask(PmHuman::RIGHT_PALM)) ik_right_hand( c.get(PmHuman::RIGHT_PALM), 0 );
	if (c.getMask(PmHuman::LEFT_PALM))  ik_left_hand( c.get(PmHuman::LEFT_PALM), 0 );
	if (c.getMask(PmHuman::RIGHT_FOOT)) ik_right_foot( c.get(PmHuman::RIGHT_FOOT), 0 );
	if (c.getMask(PmHuman::LEFT_FOOT))  ik_left_foot( c.get(PmHuman::LEFT_FOOT), 0 );

	return iter;
}


//------------------------------------------------------//
//														//
//	 			Computing Energy Function				//
//														//
//------------------------------------------------------//


static m_real
energyFunc( vectorN const&d )
{
	tPosture = oPosture;
	tPosture.addDisplacement( d );

	m_real	dist = 0.0;
	vector	dv, dq;
	quater	q;
	int		i, j, jj;

	for( i=0; i<constraint.getNumEntity(); i++ )
	{
		PmConstraintEntity &c = constraint.entity[i];

		if ( c.mask & PM_C_POSITION )
		{
			dv = tPosture.getGlobalTranslation( c.link ) -
				 c.value.getTranslation();
			dist += dv%dv;
		}

		if ( c.mask & PM_C_ORIENTATION )
		{
			dq = difference( tPosture.getGlobalRotation( c.link ),
							 c.value.getRotation() );
			dist += dq%dq;
		}
	}

	vector cog = tPosture.getCOG();
	vector grad = BALANCE * support_polygon.gradient( position(cog[0],0,cog[2]) );

	dist += grad%grad;

	if ( PenvDampingEnabled )
		for( j=0, jj=0; j<PM_HUMAN_NUM_LINKS; j++ )
		if ( link_mask & MaskBit(j) )
		{
			m_real c1 = PenvDampingCoeff[j+1];

			if ( tPosture.getDOF( j ) == 6 )
			{
				dist += c1*d[jj]*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; jj++;

				dist += c1*d[jj]*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; jj++;
			}
			else if ( tPosture.getDOF( j ) == 3 )
			{
				dist += c1*d[jj]*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; jj++;
			}
			else if ( tPosture.getDOF( j ) == 1 )
			{
				dist += c1*d[jj]*d[jj]; jj++;
			}
		}

	if ( PenvJointLimitEnabled )
		for( j=1; j<PM_HUMAN_NUM_LINKS; j++ )
		if ( link_mask & MaskBit(j) )
		{
			QmComplex *angle_bound = oBody->getAngleBound(j);

			if ( angle_bound )
			{
				q = tPosture.getRotation( j );
				m_real a = angle_bound->distance(q);
				dist += PenvJointLimitCoeff[j+1] * a * a;
			}
		}

	return dist;
}


//------------------------------------------------------//
//														//
// 				Computing Jacobian matrix				//
//														//
//------------------------------------------------------//

static m_real
computeJacobian()
{
	m_real	dist = 0.0;
	vector	dv, dq;
	transf	t;
	vector	endTrans;
	quater	endRot;

	int		i, j;
	vector	w1, w2, w3;

	num_equations = MAX( constraint.getDOC()+2, num_dof );
	num_unknowns  = num_dof;

	J.setSize( num_equations, num_unknowns );
	b.setSize( num_equations );

	for( i=0; i<num_equations; i++ )
	{
		b[i] = 0.0;
		for( j=0; j<num_unknowns; j++ )
			J[i][j] = 0.0;
	}

	int	ii, iii, jj;

	//
	//	Handling Kinematic Constraints
	//
	for( i=0, ii=0; i<constraint.getNumEntity(); i++ )
	{
		PmConstraintEntity &c = constraint.entity[i];

		endTrans = tPosture.getGlobalTranslation( c.link );
		endRot   = tPosture.getGlobalRotation( c.link );

		int temp = ii;

		if ( c.mask & PM_C_POSITION )
		{
			dv = endTrans - c.value.getTranslation();
			dist += dv%dv;

			b[ii++] = dv.x();
			b[ii++] = dv.y();
			b[ii++] = dv.z();
		}

		if ( c.mask & PM_C_ORIENTATION )
		{
			dq = difference( endRot, c.value.getRotation() );
			dist += dq%dq;

			b[ii++] = dq.x();
			b[ii++] = dq.y();
			b[ii++] = dq.z();
		}

		for( j=0, jj=0; j<PM_HUMAN_NUM_LINKS; j++ )
		if ( link_mask & MaskBit(j) )
		{
			if ( oBody->isAncestor( j, c.link ) )
			{
				iii = temp;

				if ( tPosture.getDOF( j ) == 6 )
				{
					if ( c.mask & PM_C_POSITION )
					{
						J[iii  ][jj  ] = 1;
						J[iii+1][jj+1] = 1;
						J[iii+2][jj+2] = 1;

						t  = tPosture.getGlobalTransf(j);
						dv = endTrans - t.getTranslation();

						w1 = vector(1,0,0) * t * dv;
						w2 = vector(0,1,0) * t * dv;
						w3 = vector(0,0,1) * t * dv;

						J[iii  ][jj+3] = w1[0];
						J[iii  ][jj+4] = w2[0];
						J[iii  ][jj+5] = w3[0];

						J[iii+1][jj+3] = w1[1];
						J[iii+1][jj+4] = w2[1];
						J[iii+1][jj+5] = w3[1];

						J[iii+2][jj+3] = w1[2];
						J[iii+2][jj+4] = w2[2];
						J[iii+2][jj+5] = w3[2];

						iii += 3;
					}

					if ( c.mask & PM_C_ORIENTATION )
					{
						J[iii  ][jj+3] = 1;
						J[iii+1][jj+4] = 1;
						J[iii+2][jj+5] = 1;
					}
				}
				else if ( tPosture.getDOF( j ) == 3 )
				{
					if ( c.mask & PM_C_POSITION )
					{
						t  = tPosture.getGlobalTransf(j);
						dv = endTrans - t.getTranslation();

						w1 = vector(1,0,0) * t * dv;
						w2 = vector(0,1,0) * t * dv;
						w3 = vector(0,0,1) * t * dv;

						J[iii  ][jj  ] = w1[0];
						J[iii  ][jj+1] = w2[0];
						J[iii  ][jj+2] = w3[0];

						J[iii+1][jj  ] = w1[1];
						J[iii+1][jj+1] = w2[1];
						J[iii+1][jj+2] = w3[1];

						J[iii+2][jj  ] = w1[2];
						J[iii+2][jj+1] = w2[2];
						J[iii+2][jj+2] = w3[2];

						iii += 3;
					}

					if ( c.mask & PM_C_ORIENTATION )
					{
						J[iii  ][jj  ] = 1;
						J[iii+1][jj+1] = 1;
						J[iii+2][jj+2] = 1;
					}
				}
				else if ( tPosture.getDOF( j ) == 1 )
				{
					if ( c.mask & PM_C_POSITION )
					{
						t  = tPosture.getGlobalTransf(j);
						dv = vector(1,0,0) * t * (endTrans - t.getTranslation());

						J[iii  ][jj] = dv[0];
						J[iii+1][jj] = dv[1];
						J[iii+2][jj] = dv[2];

						iii += 3;
					}

					if ( c.mask & PM_C_ORIENTATION )
					{
						J[iii  ][jj] = 1;
						J[iii+1][jj] = 0;
						J[iii+2][jj] = 0;
					}
				}
			}

			jj += tPosture.getDOF( j );
		}
	}

	//
	//	Handling Static Balance Constraints
	//
	static m_real mass[PM_HUMAN_NUM_LINKS];
	static vector cog[PM_HUMAN_NUM_LINKS];

	tPosture.getCOGlist( mass, cog );
	vector grad = BALANCE * support_polygon.gradient( position(cog[0][0],0,cog[0][2]) );

	if ( grad%grad > EPS_jhm )
	{
		b[ii  ] = grad.x();
		b[ii+1] = grad.z();

		dist += grad%grad;

		for( j=0, jj=0; j<PM_HUMAN_NUM_LINKS; j++ )
		if ( link_mask & MaskBit(j) )
		{
			if ( tPosture.getDOF( j ) == 6 )
			{
				J[ii  ][jj  ] = 1;
				J[ii+1][jj+2] = 1;

				t  = tPosture.getGlobalTransf(j);
				dv = cog[j] - t.getTranslation();

				m_real m = mass[j] / mass[0];

				w1 = m * vector(1,0,0) * t * dv;
				w2 = m * vector(0,1,0) * t * dv;
				w3 = m * vector(0,0,1) * t * dv;

				J[ii  ][jj+3] = w1[0];
				J[ii  ][jj+4] = w2[0];
				J[ii  ][jj+5] = w3[0];

				J[ii+1][jj+3] = w1[2];
				J[ii+1][jj+4] = w2[2];
				J[ii+1][jj+5] = w3[2];
			}
			else if ( tPosture.getDOF( j ) == 3 )
			{
				t  = tPosture.getGlobalTransf(j);
				dv = cog[j] - t.getTranslation();

				m_real m = mass[j] / mass[0];

				w1 = m * vector(1,0,0) * t * dv;
				w2 = m * vector(0,1,0) * t * dv;
				w3 = m * vector(0,0,1) * t * dv;

				J[ii  ][jj  ] = w1[0];
				J[ii  ][jj+1] = w2[0];
				J[ii  ][jj+2] = w3[0];

				J[ii+1][jj  ] = w1[2];
				J[ii+1][jj+1] = w2[2];
				J[ii+1][jj+2] = w3[2];
			}
			else if ( tPosture.getDOF( j ) == 1 )
			{
				t  = tPosture.getGlobalTransf(j);
				dv = cog[j] - t.getTranslation();

				m_real m = mass[j] / mass[0];

				w1 = m * vector(1,0,0) * t * dv;

				J[ii  ][jj  ] = w1[0];
				J[ii+1][jj  ] = w1[2];
			}

			jj += tPosture.getDOF( j );
		}
	}

	return dist;
}

//------------------------------------------------------//
//														//
// 		Computing Gradient of Energy Function			//
//														//
//------------------------------------------------------//

static void
scalingJacobian( matrixN &J )
{
	int i, j, jj;
	m_real  c;

	for( j=0, jj=0; j<PM_HUMAN_NUM_LINKS; j++ )
		if ( link_mask & MaskBit(j) )
		{
			if ( tPosture.getDOF(j)==6 )
			{
				c = PenvRigidityCoeff[j];
				for( i=0; i<num_equations; i++ )
				{
					J[i][jj  ] *= c;
					J[i][jj+1] *= c;
					J[i][jj+2] *= c;
				}

				c = PenvRigidityCoeff[j+1];
				for( i=0; i<num_equations; i++ )
				{
					J[i][jj+3] *= c;
					J[i][jj+4] *= c;
					J[i][jj+5] *= c;
				}
			}
			else if ( tPosture.getDOF(j)==3 )
			{
				c = PenvRigidityCoeff[j+1];
				for( i=0; i<num_equations; i++ )
				{
					J[i][jj  ] *= c;
					J[i][jj+1] *= c;
					J[i][jj+2] *= c;
				}
			}
			else if ( tPosture.getDOF(j)==1 )
			{
				c = PenvRigidityCoeff[j+1];
				for( i=0; i<num_equations; i++ )
				{
					J[i][jj  ] *= c;
				}
			}

			jj += tPosture.getDOF(j);
		}
}

static void
scalingCoefficients( vectorN &x )
{
	m_real c;

	for( int j=0, jj=0; j<PM_HUMAN_NUM_LINKS; j++ )
		if ( link_mask & MaskBit(j) )
		{
			if ( tPosture.getDOF(j)==6 )
			{
				c = PenvRigidityCoeff[j];
				x[jj++] *= c;
				x[jj++] *= c;
				x[jj++] *= c;

				c = PenvRigidityCoeff[j+1];
				x[jj++] *= c;
				x[jj++] *= c;
				x[jj++] *= c;
			}
			else if ( tPosture.getDOF(j)==3 )
			{
				c = PenvRigidityCoeff[j+1];
				x[jj++] *= c;
				x[jj++] *= c;
				x[jj++] *= c;
			}
			else if ( tPosture.getDOF(j)==1 )
			{
				c = PenvRigidityCoeff[j+1];
				x[jj++] *= c;
			}
		}
}

static m_real
incorporateDamping( vectorN const& d, vectorN& dp )
{
	m_real dist = 0;
	m_real c1, c2;

	for( int j=0, jj=0; j<PM_HUMAN_NUM_LINKS; j++ )
		if ( link_mask & MaskBit(j) )
		{
			c1 = PenvDampingCoeff[j+1];
			c2 = 2.0 * c1;

			if ( tPosture.getDOF( j ) == 6 )
			{
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;

				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
			}
			else if ( tPosture.getDOF( j ) == 3 )
			{
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
			}
			else if ( tPosture.getDOF( j ) == 1 )
			{
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
			}
		}

	return dist;
}

static m_real
incorporateJointLimit( vectorN const& d, vectorN& dp )
{
	m_real dist = 0.0;
	m_real a, c;
	quater q;
	vector v;

	for( int j=0, jj=0; j<PM_HUMAN_NUM_LINKS; j++ )
		if ( link_mask & MaskBit(j) )
		{
			if ( tPosture.getDOF( j ) == 3 ||
				 tPosture.getDOF( j ) == 1 )
			{
				QmComplex *angle_bound = oBody->getAngleBound(j);

				if ( angle_bound )
				{
					c = PenvJointLimitCoeff[j+1];

					q = tPosture.getRotation( j );
					a = angle_bound->distance(q);
					v = angle_bound->gradient(q);

					dist += c * (a*a);
				}
			}

			if ( tPosture.getDOF( j ) == 3 )
			{
				dp[jj  ] -= 2.0 * c * v[0];
				dp[jj+1] -= 2.0 * c * v[1];
				dp[jj+2] -= 2.0 * c * v[2];
			}
			else if ( tPosture.getDOF( j ) == 1 )
			{
				dp[jj] -= 2.0 * c * v.length();
			}

			jj += tPosture.getDOF( j );
		}

	return dist;
}

static m_real
gradientFuncLS( vectorN const&d, vectorN& dp )
{
	tPosture = oPosture;
	tPosture.addDisplacement( d );

	m_real	dist = computeJacobian();
	scalingJacobian( J );

	if ( PenvIKsolverMethod == PENV_IK_LS_SVD_METHOD )
		dp.solve( J, b, 1.0e-2 );
	else
	{
		static matrixN Jt; Jt.transpose( J );
		static matrixN Jp; Jp.mult( Jt, J );
		static vectorN bp; bp.mult( Jt, b );

		for( int k=0; k<Jp.row(); k++ ) Jp[k][k] += DAMPING;

		dp.solve( Jp, bp );
	}

	scalingCoefficients( dp );

	if ( PenvDampingEnabled )	 dist += incorporateDamping( d, dp );
	if ( PenvJointLimitEnabled ) dist += incorporateJointLimit( d, dp );

	return dist;
}

static m_real
gradientFunc( vectorN const& d, vectorN& dp )
{
	int i, j;

	tPosture = oPosture;
	tPosture.addDisplacement( d );

	m_real dist = computeJacobian();

	for( i=0; i<num_dof; i++ )
	{
		dp[i] = 0;

		for( j=0; j<constraint.getDOC(); j++ )
			dp[i] -= 2.0 * J[j][i] * b[j];
	}

	if ( PenvDampingEnabled )	 dist += incorporateDamping( d, dp );
	if ( PenvJointLimitEnabled ) dist += incorporateJointLimit( d, dp );

	return dist;
}

