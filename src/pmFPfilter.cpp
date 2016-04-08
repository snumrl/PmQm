
#include "pm.h"
#include "qmFilter.h"

using namespace jhm;

#define DAMPING  0.5

extern vector p_buf[PM_MAX_FRAMES];
extern quater q_buf[PM_MAX_FRAMES];
extern vector p1_buf[PM_MAX_FRAMES];

static matrixN		J;
static vectorN		b;
static int			num_dof,
					num_equations;
static PmMaskType	Mask;
static PmHuman*		Body;

static void computeJacobian( PmConstraint&, PmPosture const& );

void
PmLinearMotion::FPfilter( void (*filter1)(int,vector*,vector*),
                          void (*filter2)(int, quater*,vector*),
                          int numIter, PmConstraint* constraint )
{
	static PmVectorArray va;
	va.setSize( this->getSize() );

	int i, j, k;

	int	n = this->getSize();
	Mask = this->getMask();
	Body = this->getBody();

  	num_dof	= GetDOF( Mask );
	num_equations = 0;

	for( i=0; i<n; i++ )
	{
		j = getConstraint(i).getDOC();
		if ( j > num_equations ) num_equations = j;
	}
	num_equations = MAX( num_equations, num_dof );

	J.setSize( num_equations, num_dof );
	b.setSize( num_equations );

	static vectorN x1; x1.setSize( num_dof );
	static vectorN x2; x2.setSize( num_dof );
	static vectorN x3; x3.setSize( num_dof );

	for( j=0; j<numIter; j++ )
	{
		if ( Mask & PM_MASK_PELVIS )
		{
			this->getTranslation( p_buf );
			filter1( n, p_buf, p1_buf );
			va.setLinearVectors( p1_buf );
		}

		for( i=0; i<PM_HUMAN_NUM_LINKS; i++ )
		if ( Mask & MaskBit(i) )
		{
			this->getRotation( i, q_buf );
			filter2( n, q_buf, p1_buf );
			va.setAngularVectors( i, p1_buf );
		}

		for( i=0; i<n; i++ )
		{
			computeJacobian( getConstraint(i), this->getPosture(i) );
			static matrixN Jt; Jt.transpose( J );
			static matrixN Jp; Jp.mult( Jt, J );


			static matrixN Jn; Jn.assign( Jp );
			static matrixN Ji; Jn.SVinverse( Ji );

			static matrixN J1; J1.mult( Ji, Jt );
			static matrixN J2; J2.mult( J1, J );

			x3.mult( J1, b );

			for( k=0; k<J2.row(); k++ ) J2[k][k] = 1.0 - J2[k][k];

			va.getVector(i).getDisplacement( x1 );
			x2.mult( J2, x1 );
			va.getVector(i).setDisplacement( x2 );
//			va.getVector(i).addDisplacement( x3 );
		}

		(*this) += va;
	}
}

static void
computeJacobian( PmConstraint& constraint, PmPosture const& posture )
{
	vector	dv, dq;
	transf	t;
	vector	endTrans;
	quater	endRot;

	int		i, ii, iii;
	int		j, jj;
	vector	w1, w2, w3;

	num_equations = MAX( constraint.getDOC(), num_dof );

	J.setSize( num_equations, num_dof );
	b.setSize( num_equations );

	for( i=0; i<num_equations; i++ )
	{
		b[i] = 0.0;
		for( j=0; j<num_dof; j++ )
			J[i][j] = 0.0;
	}

	for( i=0, ii=0; i<constraint.getNumEntity(); i++ )
	{
		PmConstraintEntity &c = constraint.entity[i];

		endTrans = posture.getGlobalTranslation( c.link );
		endRot   = posture.getGlobalRotation( c.link );

		int temp = ii;

		if ( c.mask & PM_C_POSITION )
		{
			dv = endTrans - c.value.getTranslation();

			b[ii++] = dv.x();
			b[ii++] = dv.y();
			b[ii++] = dv.z();
		}

		if ( c.mask & PM_C_ORIENTATION )
		{
			dq = difference( endRot, Matrix2Quater(c.value.getAffine()) );

			b[ii++] = dq.x();
			b[ii++] = dq.y();
			b[ii++] = dq.z();
		}

		for( j=0, jj=0; j<PM_HUMAN_NUM_LINKS; j++ )
		if ( Mask & MaskBit(j) )
		{
			if ( Body->isAncestor( j, c.link ) )
			{
				iii = temp;

				if ( posture.getDOF( j ) == 6 )
				{
					if ( c.mask & PM_C_POSITION )
					{
						J[iii  ][jj  ] = 1;
						J[iii+1][jj+1] = 1;
						J[iii+2][jj+2] = 1;

						t  = posture.getGlobalTransf(j);
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

						iii += 3;
					}
				}
				else if ( posture.getDOF( j ) == 3 )
				{
					if ( c.mask & PM_C_POSITION )
					{
						t  = posture.getGlobalTransf(j);
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

						iii += 3;
					}
				}
				else if ( posture.getDOF( j ) == 1 )
				{
					if ( c.mask & PM_C_POSITION )
					{
						t  = posture.getGlobalTransf(j);
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

						iii += 3;
					}
				}
			}

			jj += posture.getDOF( j );
		}
	}
}
