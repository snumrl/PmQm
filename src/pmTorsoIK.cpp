
#include "pm.h"

//------------------------------------------------------//
//														//
// Inverse kinematics for pelvis and flexible spine		//
//														//
//------------------------------------------------------//

using namespace jhm;

#define NUM_DATA				13
#define TOLERANCE				1e-3
#define ESTIMATION_STEP_SIZE	1e-3


static QmComplex		*bound_rh, *bound_lh,
						*bound_rf, *bound_lf,
						*bound_spine;
static PmPosture		tPosture;
static int		  		RHMask,   LHMask;
static int		  		RFMask,   LFMask;
static int		  		BDMask;
static transf			RHTarget, LHTarget;
static transf			RFTarget, LFTarget;
static transf			BDTarget;
static quater			RHTargetRot, LHTargetRot;
static quater			RFTargetRot, LFTargetRot;
static quater			BDTargetRot;
static vector			RHTargetTrans, LHTargetTrans;
static vector			RFTargetTrans, LFTargetTrans;
static vector			BDTargetTrans;
static quater			RHBack, LHBack;
static quater			RFBack, LFBack;
static vector			orig_v;
static quater			orig_q1;
static quater			orig_q2;
static m_real			orig_l1, orig_l2, orig_l3, orig_l4;
static m_real			k1, k2, k3,
						k4, k5, k6, k7,
						k8, k9, k10, k11,
						c1, c2, c3, c4, c5;
static matrixN			J;
static vectorN			x;
static vectorN			b;

static m_real  energyFunc( vectorN const& );
static m_real  gradientFunc( vectorN const&, vectorN& );
static m_real  gradientFuncLS( vectorN const&, vectorN& );
static m_real  computeJacobian();


int
PmPosture::ik_torso( PmConstraint const& c, int optimize )
{
	return ik_torso(
			c.getMask(PmHuman::RIGHT_PALM), c.get(PmHuman::RIGHT_PALM),
			c.getMask(PmHuman::LEFT_PALM ), c.get(PmHuman::LEFT_PALM ),
			c.getMask(PmHuman::RIGHT_FOOT), c.get(PmHuman::RIGHT_FOOT),
			c.getMask(PmHuman::LEFT_FOOT ), c.get(PmHuman::LEFT_FOOT ),
			c.getMask(PmHuman::PELVIS), c.get(PmHuman::PELVIS), optimize );
}

int
PmPosture::ik_torso( int RH_mask, transf const& RH_target,
					 int LH_mask, transf const& LH_target,
					 int RF_mask, transf const& RF_target,
					 int LF_mask, transf const& LF_target,
					 int BD_mask, transf const& BD_target,
					 int optimize )
{
	if ( !optimize )
	{
		if ( RH_mask ) ik_right_hand( RH_target );
		if ( LH_mask ) ik_left_hand ( LH_target );
		if ( RF_mask ) ik_right_foot( RF_target );
		if ( LF_mask ) ik_left_foot ( LF_target );
		return 0;
	}

	//
	// Set global variables
	//
	tPosture = (*this);

	PmHuman *human = this->body;
	bound_rh = human->getAngleBound(PmHuman::UPPER_RIGHT_ARM);
	bound_lh = human->getAngleBound(PmHuman::UPPER_LEFT_ARM);
	bound_rf = human->getAngleBound(PmHuman::UPPER_RIGHT_LEG);
	bound_lf = human->getAngleBound(PmHuman::UPPER_LEFT_LEG);
	bound_spine = human->getAngleBound(PmHuman::CHEST);

	RHMask   = RH_mask;
	LHMask   = LH_mask;
	RFMask   = RF_mask;
	LFMask   = LF_mask;
	BDMask   = BD_mask;

	RHTarget = RH_target;
	LHTarget = LH_target;
	RFTarget = RF_target;
	LFTarget = LF_target;
	BDTarget = BD_target;

	RHTargetRot = RH_target.getRotation();
	LHTargetRot = LH_target.getRotation();
	RFTargetRot = RF_target.getRotation();
	LFTargetRot = LF_target.getRotation();
	BDTargetRot = BD_target.getRotation();

	RHTargetTrans = RH_target.getTranslation();
	LHTargetTrans = LH_target.getTranslation();
	RFTargetTrans = RF_target.getTranslation();
	LFTargetTrans = LF_target.getTranslation();
	BDTargetTrans = BD_target.getTranslation();

	RHBack = tPosture.getRotation( PmHuman::UPPER_RIGHT_ARM );
	LHBack = tPosture.getRotation( PmHuman::UPPER_LEFT_ARM );
	RFBack = tPosture.getRotation( PmHuman::UPPER_RIGHT_LEG );
	LFBack = tPosture.getRotation( PmHuman::UPPER_LEFT_LEG );

	k1  = PenvDampingCoeff[PmHuman::PELVIS];
	k2  = PenvDampingCoeff[PmHuman::PELVIS + 1];
	k3  = PenvDampingCoeff[PmHuman::CHEST  + 1];
	k4  = PenvDampingCoeff[PmHuman::UPPER_RIGHT_ARM + 1];
	k5  = PenvDampingCoeff[PmHuman::UPPER_LEFT_ARM  + 1];
	k6  = PenvDampingCoeff[PmHuman::UPPER_RIGHT_LEG + 1];
	k7  = PenvDampingCoeff[PmHuman::UPPER_LEFT_LEG  + 1];
	k8  = PenvDampingCoeff[PmHuman::LOWER_RIGHT_ARM + 1];
	k9  = PenvDampingCoeff[PmHuman::LOWER_LEFT_ARM  + 1];
	k10 = PenvDampingCoeff[PmHuman::LOWER_RIGHT_LEG + 1];
	k11 = PenvDampingCoeff[PmHuman::LOWER_LEFT_LEG  + 1];

	c1 = PenvJointLimitCoeff[PmHuman::CHEST + 1];
	c2 = PenvJointLimitCoeff[PmHuman::UPPER_RIGHT_ARM + 1];
	c3 = PenvJointLimitCoeff[PmHuman::UPPER_LEFT_ARM  + 1];
	c4 = PenvJointLimitCoeff[PmHuman::UPPER_RIGHT_LEG + 1];
	c5 = PenvJointLimitCoeff[PmHuman::UPPER_LEFT_LEG  + 1];

	//
	// Optimization
	//
	orig_l1 = (	getGlobalTranslation( PmHuman::UPPER_RIGHT_ARM ) -
				getGlobalTranslation( PmHuman::RIGHT_PALM ) ).length();
	orig_l2 = (	getGlobalTranslation( PmHuman::UPPER_LEFT_ARM ) -
				getGlobalTranslation( PmHuman::LEFT_PALM ) ).length();
	orig_l3 = (	getGlobalTranslation( PmHuman::UPPER_RIGHT_LEG ) -
				getGlobalTranslation( PmHuman::RIGHT_FOOT ) ).length();
	orig_l4 = (	getGlobalTranslation( PmHuman::UPPER_LEFT_LEG ) -
				getGlobalTranslation( PmHuman::LEFT_FOOT ) ).length();

	if ( BDMask )
	{
		orig_v  = BDTargetTrans;
		orig_q1 = BDTargetRot;
		orig_q2 = rotate[PmHuman::CHEST];
	}
	else
	{
		orig_v  = this->trans;
		orig_q1 = rotate[PmHuman::PELVIS];
		orig_q2 = rotate[PmHuman::CHEST];
	}

	static vectorN d; d.setSize(NUM_DATA);

	int i;
	for( i=0; i<NUM_DATA; i++ ) d[i] = 0.0;

	m_real f;
	int iter = 0;

	if ( energyFunc(d) > TOLERANCE )
	{
		if ( PenvIKsolverMethod == PENV_IK_CG_METHOD )
			frprmn( d, NUM_DATA, 0.1, iter, f, energyFunc, gradientFunc );
		else
			gradient_descent( d, NUM_DATA, 0.1, iter, f, energyFunc, gradientFuncLS );
	}

	vector v  = orig_v  + vector(d[0],d[1],d[2]);
	quater q1 = orig_q1 * exp(vector(d[3],d[4],d[5]));
	quater q2 = orig_q2 * exp(vector(d[6],d[7],d[8]));

	this->trans = v;
	rotate[PmHuman::PELVIS]  = q1;
	rotate[PmHuman::CHEST]   = q2;
	for( i=0; i<getNumSpineLinks(); i++ )
		rotate[PmHuman::CHEST-i-1] = q2;

	if ( RHMask ) ik_right_hand( RHTarget, d[ 9] );
	if ( LHMask ) ik_left_hand ( LHTarget, d[10] );
	if ( RFMask ) ik_right_foot( RFTarget, d[11] );
	if ( LFMask ) ik_left_foot ( LFTarget, d[12] );

	return iter;
}


//------------------------------------------------------//
//														//
//	 			Computing Energy Function				//
//														//
//------------------------------------------------------//


static m_real
energyFunc( vectorN const& d )
{
	vector v  = orig_v  + vector(d[0],d[1],d[2]);
	quater q1 = orig_q1 * exp(vector(d[3],d[4],d[5]));
	quater q2 = orig_q2 * exp(vector(d[6],d[7],d[8]));

	tPosture.setTranslation( v );
	tPosture.setRotation( PmHuman::PELVIS , q1 );
	tPosture.setRotation( PmHuman::CHEST  , q2 );

	int i;
	for( i=0; i<tPosture.getNumSpineLinks(); i++ )
		tPosture.setRotation( PmHuman::CHEST-i-1, q2 );

	//
	// Goal specification
	//
	m_real  dist = 0.0;
	vector  end;
	vector	dv, dq;

	if ( BDMask )
	{
		dv = BDTargetTrans - v;
		dq = difference( BDTargetRot, q1 );
		dist += dv%dv + dq%dq;
	}

	if ( RHMask )
	{
		tPosture.setRotation( PmHuman::UPPER_RIGHT_ARM, RHBack );
		tPosture.ik_right_hand( RHTarget, d[9] );

		end = tPosture.getGlobalTranslation( PmHuman::RIGHT_PALM );
		dv = RHTargetTrans - end;
		dist += dv%dv;
	}

	if ( LHMask )
	{
		tPosture.setRotation( PmHuman::UPPER_LEFT_ARM, LHBack );
		tPosture.ik_left_hand( LHTarget, d[10] );

		end = tPosture.getGlobalTranslation( PmHuman::LEFT_PALM );
		dv = LHTargetTrans - end;
		dist += dv%dv;
	}

	if ( RFMask )
	{
		tPosture.setRotation( PmHuman::UPPER_RIGHT_LEG, RFBack );
		tPosture.ik_right_foot( RFTarget, d[11] );

		end = tPosture.getGlobalTranslation( PmHuman::RIGHT_FOOT );
		dv = RFTargetTrans - end;
		dist += dv%dv;
	}

	if ( LFMask )
	{
		tPosture.setRotation( PmHuman::UPPER_LEFT_LEG, LFBack );
		tPosture.ik_left_foot( LFTarget, d[12] );

		end = tPosture.getGlobalTranslation( PmHuman::LEFT_FOOT );
		dv = LFTargetTrans - end;
		dist += dv%dv;
	}

	//
	// Damping and Spring terms
	//
	i=0;

	dist += k1*d[i]*d[i]; i++;
	dist += k1*d[i]*d[i]; i++;
	dist += k1*d[i]*d[i]; i++;

	dist += k2*d[i]*d[i]; i++;
	dist += k2*d[i]*d[i]; i++;
	dist += k2*d[i]*d[i]; i++;

	dist += k3*d[i]*d[i]; i++;
	dist += k3*d[i]*d[i]; i++;
	dist += k3*d[i]*d[i]; i++;

	dist += k4*d[i]*d[i]; i++;
	dist += k5*d[i]*d[i]; i++;
	dist += k6*d[i]*d[i]; i++;
	dist += k7*d[i]*d[i]; i++;

	if ( PenvIKsolverMethod == PENV_IK_CG_METHOD )
	{
		m_real l1 = (tPosture.getGlobalTranslation(PmHuman::UPPER_RIGHT_ARM) -
				tPosture.getGlobalTranslation(PmHuman::RIGHT_PALM) ).length();
		m_real l2 = (tPosture.getGlobalTranslation(PmHuman::UPPER_LEFT_ARM) -
				tPosture.getGlobalTranslation(PmHuman::LEFT_PALM) ).length();
		m_real l3 = (tPosture.getGlobalTranslation(PmHuman::UPPER_RIGHT_LEG) -
				tPosture.getGlobalTranslation(PmHuman::RIGHT_FOOT) ).length();
		m_real l4 = (tPosture.getGlobalTranslation(PmHuman::UPPER_LEFT_LEG) -
				tPosture.getGlobalTranslation(PmHuman::LEFT_FOOT) ).length();

		dist += k8  * SQR( orig_l1 - l1 );
		dist += k9  * SQR( orig_l2 - l2 );
		dist += k10 * SQR( orig_l3 - l3 );
		dist += k11 * SQR( orig_l4 - l4 );
	}

	//
	// Joint limit
	//
	quater rhq = tPosture.getRotation( PmHuman::UPPER_RIGHT_ARM );
	quater lhq = tPosture.getRotation( PmHuman::UPPER_LEFT_ARM );
	quater rfq = tPosture.getRotation( PmHuman::UPPER_RIGHT_LEG );
	quater lfq = tPosture.getRotation( PmHuman::UPPER_LEFT_LEG );

	if ( bound_rh )    dist += c2 * SQR( bound_rh->distance(rhq) );
	if ( bound_lh )    dist += c3 * SQR( bound_lh->distance(lhq) );
	if ( bound_rf )    dist += c4 * SQR( bound_rf->distance(rfq) );
	if ( bound_lf )    dist += c5 * SQR( bound_lf->distance(lfq) );
	if ( bound_spine ) dist += c1 * SQR( bound_spine->distance(q2) );

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
	vector  v  = tPosture.getTranslation();
	quater  q1 = tPosture.getRotation( PmHuman::PELVIS );
	quater  q2 = tPosture.getRotation( PmHuman::CHEST );

	m_real  dist = 0.0;
	vector	dv, dq, pv;
	vector	w1, w2, w3;
	vector	u1, u2, u3;

	transf	t = tPosture.getGlobalTransf( PmHuman::PELVIS );
	transf  t1;

	vector	endRH, endLH, endRF, endLF;
	vector	endBD = t.getTranslation();

	int		num_equations = 18;
	int		num_unknowns  = 9;

	J.setSize( num_equations, num_unknowns );
	x.setSize( num_unknowns );
	b.setSize( num_equations );

	for( int i=0; i<num_equations; i++ )
	{
		b[i] = 0.0;
		for( int j=0; j<num_unknowns; j++ )
			J[i][j] = 0.0;
	}

	if ( BDMask )
	{
		dv = BDTargetTrans - v;
		dq = difference( BDTargetRot, q1 );
		dist += dv%dv + dq%dq;

		b[0] = dv.x();
		b[1] = dv.y();
		b[2] = dv.z();

		b[3] = dq.x();
		b[4] = dq.y();
		b[5] = dq.z();
	}

	if ( RHMask )
	{
		endRH = tPosture.getGlobalTranslation( PmHuman::RIGHT_PALM );
		dv = RHTargetTrans - endRH;
		dist += dv%dv;

		b[6] = dv.x();
		b[7] = dv.y();
		b[8] = dv.z();
	}

	if ( LHMask )
	{
		endLH = tPosture.getGlobalTranslation( PmHuman::LEFT_PALM );
		dv = LHTargetTrans - endLH;
		dist += dv%dv;

		b[ 9] = dv.x();
		b[10] = dv.y();
		b[11] = dv.z();
	}

	if ( RFMask )
	{
		endRF = tPosture.getGlobalTranslation( PmHuman::RIGHT_FOOT );
		dv = RFTargetTrans - endRF;
		dist += dv%dv;

		b[12] = dv.x();
		b[13] = dv.y();
		b[14] = dv.z();
	}

	if ( LFMask )
	{
		endLF = tPosture.getGlobalTranslation( PmHuman::LEFT_FOOT );
		dv = LFTargetTrans - endLF;
		dist += dv%dv;

		b[15] = dv.x();
		b[16] = dv.y();
		b[17] = dv.z();
	}


	//
	// Compute Jacobian
	//
	if ( BDMask )
	{
		J[ 0][0] = 1; J[ 0][1] = 0; J[ 0][2] = 0;
		J[ 1][0] = 0; J[ 1][1] = 1; J[ 1][2] = 0;
		J[ 2][0] = 0; J[ 2][1] = 0; J[ 2][2] = 1;

		J[ 3][0] = 0; J[ 3][1] = 0; J[ 3][2] = 0;
		J[ 4][0] = 0; J[ 4][1] = 0; J[ 4][2] = 0;
		J[ 5][0] = 0; J[ 5][1] = 0; J[ 5][2] = 0;

		J[ 0][3] = 0; J[ 0][4] = 0; J[ 0][5] = 0;
		J[ 1][3] = 0; J[ 1][4] = 0; J[ 1][5] = 0;
		J[ 2][3] = 0; J[ 2][4] = 0; J[ 2][5] = 0;

		J[ 3][3] = 1; J[ 3][4] = 0; J[ 3][5] = 0;
		J[ 4][3] = 0; J[ 4][4] = 1; J[ 4][5] = 0;
		J[ 5][3] = 0; J[ 5][4] = 0; J[ 5][5] = 1;
	}

	if ( RHMask )
	{
		dv = endRH - endBD;
		w1 = vector(1,0,0) * t * dv;
		w2 = vector(0,1,0) * t * dv;
		w3 = vector(0,0,1) * t * dv;

		J[ 6][0] = 1; J[ 6][1] = 0; J[ 6][2] = 0;
		J[ 7][0] = 0; J[ 7][1] = 1; J[ 7][2] = 0;
		J[ 8][0] = 0; J[ 8][1] = 0; J[ 8][2] = 1;

		J[ 6][3] = w1[0]; J[ 6][4] = w2[0]; J[ 6][5] = w3[0];
		J[ 7][3] = w1[1]; J[ 7][4] = w2[1]; J[ 7][5] = w3[1];
		J[ 8][3] = w1[2]; J[ 8][4] = w2[2]; J[ 8][5] = w3[2];
	}

	if ( LHMask )
	{
		dv = endLH - endBD;
		w1 = vector(1,0,0) * t * dv;
		w2 = vector(0,1,0) * t * dv;
		w3 = vector(0,0,1) * t * dv;

		J[ 9][0] = 1; J[ 9][1] = 0; J[ 9][2] = 0;
		J[10][0] = 0; J[10][1] = 1; J[10][2] = 0;
		J[11][0] = 0; J[11][1] = 0; J[11][2] = 1;

		J[ 9][3] = w1[0]; J[ 9][4] = w2[0]; J[ 9][5] = w3[0];
		J[10][3] = w1[1]; J[10][4] = w2[1]; J[10][5] = w3[1];
		J[11][3] = w1[2]; J[11][4] = w2[2]; J[11][5] = w3[2];
	}

	if ( RHMask || LHMask )
	{
		t1 = tPosture.getGlobalTransf( PmHuman::CHEST );
		quater q = tPosture.getRotation( PmHuman::CHEST );
		vector u1(1,0,0);
		vector u2(0,1,0);
		vector u3(0,0,1);

		for( int i=0; i<tPosture.getNumSpineLinks()+1; i++ )
		{
			pv = t1.getTranslation();

			if ( RHMask )
			{
				dv = endRH - pv;
				w1 = u1 * t1 * dv;
				w2 = u2 * t1 * dv;
				w3 = u3 * t1 * dv;

				J[ 6][6] += w1[0]; J[ 6][7] += w2[0]; J[ 6][8] += w3[0];
				J[ 7][6] += w1[1]; J[ 7][7] += w2[1]; J[ 7][8] += w3[1];
				J[ 8][6] += w1[2]; J[ 8][7] += w2[2]; J[ 8][8] += w3[2];
			}

			if ( LHMask )
			{
				dv = endLH - pv;
				w1 = u1 * t1 * dv;
				w2 = u2 * t1 * dv;
				w3 = u3 * t1 * dv;

				J[ 9][6] += w1[0]; J[ 9][7] += w2[0]; J[ 9][8] += w3[0];
				J[10][6] += w1[1]; J[10][7] += w2[1]; J[10][8] += w3[1];
				J[11][6] += w1[2]; J[11][7] += w2[2]; J[11][8] += w3[2];
			}
		}

		u1 = rotate( q, u1 );
		u2 = rotate( q, u2 );
		u3 = rotate( q, u3 );
	}

	if ( RFMask )
	{
		dv = endRF - endBD;
		w1 = vector(1,0,0) * t * dv;
		w2 = vector(0,1,0) * t * dv;
		w3 = vector(0,0,1) * t * dv;

		J[12][0] = 1; J[12][1] = 0; J[12][2] = 0;
		J[13][0] = 0; J[13][1] = 1; J[13][2] = 0;
		J[14][0] = 0; J[14][1] = 0; J[14][2] = 1;

		J[12][3] = w1[0]; J[12][4] = w2[0]; J[12][5] = w3[0];
		J[13][3] = w1[1]; J[13][4] = w2[1]; J[13][5] = w3[1];
		J[14][3] = w1[2]; J[14][4] = w2[2]; J[14][5] = w3[2];
	}

	if ( LFMask )
	{
		dv = endLF - endBD;
		w1 = vector(1,0,0) * t * dv;
		w2 = vector(0,1,0) * t * dv;
		w3 = vector(0,0,1) * t * dv;

		J[15][0] = 1; J[15][1] = 0; J[15][2] = 0;
		J[16][0] = 0; J[16][1] = 1; J[16][2] = 0;
		J[17][0] = 0; J[17][1] = 0; J[17][2] = 1;

		J[15][3] = w1[0]; J[15][4] = w2[0]; J[15][5] = w3[0];
		J[16][3] = w1[1]; J[16][4] = w2[1]; J[16][5] = w3[1];
		J[17][3] = w1[2]; J[17][4] = w2[2]; J[17][5] = w3[2];
	}

	return dist;
}


//------------------------------------------------------//
//														//
// 		Computing Gradient of Energy Function			//
//														//
//------------------------------------------------------//

static m_real
gradientFuncLS( vectorN const& d, vectorN& dp )
{
	//
	//  Reconstruct configuration of a figure
	//
	vector v  = orig_v  + vector(d[0],d[1],d[2]);
	quater q1 = orig_q1 * exp(vector(d[3],d[4],d[5]));
	quater q2 = orig_q2 * exp(vector(d[6],d[7],d[8]));

	tPosture.setTranslation( v );
	tPosture.setRotation( PmHuman::PELVIS , q1 );
	tPosture.setRotation( PmHuman::CHEST  , q2 );

	int i;
	for( i=0; i<tPosture.getNumSpineLinks(); i++ )
		tPosture.setRotation( PmHuman::CHEST+i+1, q2 );

	if ( RHMask )
	{
		tPosture.setRotation( PmHuman::UPPER_RIGHT_ARM, RHBack );
		tPosture.ik_right_hand( RHTarget, d[9] );
	}

	if ( LHMask )
	{
		tPosture.setRotation( PmHuman::UPPER_LEFT_ARM, LHBack );
		tPosture.ik_left_hand( LHTarget, d[10] );
	}

	if ( RFMask )
	{
		tPosture.setRotation( PmHuman::UPPER_RIGHT_LEG, RFBack );
		tPosture.ik_right_foot( RFTarget, d[11] );
	}

	if ( LFMask )
	{
		tPosture.setRotation( PmHuman::UPPER_LEFT_LEG, LFBack );
		tPosture.ik_left_foot( LFTarget, d[12] );
	}


	//
	//  Jacobian matrix and its pseudo inverse
	//
	m_real  dist = computeJacobian();

	if ( PenvIKsolverMethod == PENV_IK_LS_SVD_METHOD )
		x.solve( J, b, 1.0e-4 );
	else
	{
		static matrixN Jt; Jt.transpose( J );
		static matrixN Jp; Jp.mult( Jt, J );
		static vectorN bp; bp.mult( Jt, b );

		for( int k=0; k<Jp.row(); k++ ) Jp[k][k] += 1.0e-2;

//		x.solve( Jp, bp );
		x.solve( Jp, bp, 50, 1e-4, 1.0 );
	}

	dp[0] = x[0]; dp[1] = x[1]; dp[2] = x[2];
	dp[3] = x[3]; dp[4] = x[4]; dp[5] = x[5];
	dp[6] = x[6]; dp[7] = x[7]; dp[8] = x[8];
	dp[9] = dp[10] = dp[11] = dp[12] = 0.0;


	//
	//  Damping and Spring terms
	//
	i=0;
	dist += k1*d[i]*d[i]; dp[i] -= 2.0*k1*d[i]; i++;
	dist += k1*d[i]*d[i]; dp[i] -= 2.0*k1*d[i]; i++;
	dist += k1*d[i]*d[i]; dp[i] -= 2.0*k1*d[i]; i++;

	dist += k2*d[i]*d[i]; dp[i] -= 2.0*k2*d[i]; i++;
	dist += k2*d[i]*d[i]; dp[i] -= 2.0*k2*d[i]; i++;
	dist += k2*d[i]*d[i]; dp[i] -= 2.0*k2*d[i]; i++;

	dist += k3*d[i]*d[i]; dp[i] -= 2.0*k3*d[i]; i++;
	dist += k3*d[i]*d[i]; dp[i] -= 2.0*k3*d[i]; i++;
	dist += k3*d[i]*d[i]; dp[i] -= 2.0*k3*d[i]; i++;

	dist += k4*d[i]*d[i]; dp[i] -= 2.0*k4*d[i]; i++;
	dist += k5*d[i]*d[i]; dp[i] -= 2.0*k5*d[i]; i++;
	dist += k6*d[i]*d[i]; dp[i] -= 2.0*k6*d[i]; i++;
	dist += k7*d[i]*d[i]; dp[i] -= 2.0*k7*d[i]; i++;


	//
	// Joint limit
	//
	quater rhq = tPosture.getRotation( PmHuman::UPPER_RIGHT_ARM );
	quater lhq = tPosture.getRotation( PmHuman::UPPER_LEFT_ARM );
	quater rfq = tPosture.getRotation( PmHuman::UPPER_RIGHT_LEG );
	quater lfq = tPosture.getRotation( PmHuman::UPPER_LEFT_LEG );

	if ( bound_rh )    dist += c2 * SQR( bound_rh->distance(rhq) );
	if ( bound_lh )    dist += c3 * SQR( bound_lh->distance(lhq) );
	if ( bound_rf )    dist += c4 * SQR( bound_rf->distance(rfq) );
	if ( bound_lf )    dist += c5 * SQR( bound_lf->distance(lfq) );
	if ( bound_spine ) dist += c1 * SQR( bound_spine->distance(q2) );

	if ( bound_spine )
	{
		vector dv = bound_spine->gradient(q2);
		dp[6] -= 2.0 * c1 * dv[0];
		dp[7] -= 2.0 * c1 * dv[1];
		dp[8] -= 2.0 * c1 * dv[2];
	}

	return dist;
}

//------------------------------------------------------//

static m_real
gradientFunc( vectorN const& d, vectorN& dp )
{
	int i;
	m_real e1, e2;

	for( i=0; i<NUM_DATA; i++ ) dp[i] = 0.0;

	if ( RHMask || LHMask || RFMask || LFMask || BDMask )
		for( i=0; i<6; i++ )
		{
			d[i] += ESTIMATION_STEP_SIZE;
			e1 = energyFunc( d );
			d[i] -= 2*ESTIMATION_STEP_SIZE;
			e2 = energyFunc( d );
			d[i] += ESTIMATION_STEP_SIZE;

			dp[i] = 2.0 * (e1 - e2) / ESTIMATION_STEP_SIZE;
		}

	if ( RHMask || LHMask )
		for( i=6; i<9; i++ )
		{
			d[i] += ESTIMATION_STEP_SIZE;
			e1 = energyFunc( d );
			d[i] -= 2*ESTIMATION_STEP_SIZE;
			e2 = energyFunc( d );
			d[i] += ESTIMATION_STEP_SIZE;

			dp[i] = 2.0 * (e1 - e2) / ESTIMATION_STEP_SIZE;
		}

	if ( RHMask )
		{
			i = 9;
			d[i] += ESTIMATION_STEP_SIZE;
			e1 = energyFunc( d );
			d[i] -= 2*ESTIMATION_STEP_SIZE;
			e2 = energyFunc( d );
			d[i] += ESTIMATION_STEP_SIZE;

			dp[i] = 2.0 * (e1 - e2) / ESTIMATION_STEP_SIZE;
		}

	if ( LHMask )
		{
			i = 10;
			d[i] += ESTIMATION_STEP_SIZE;
			e1 = energyFunc( d );
			d[i] -= 2*ESTIMATION_STEP_SIZE;
			e2 = energyFunc( d );
			d[i] += ESTIMATION_STEP_SIZE;

			dp[i] = 2.0 * (e1 - e2) / ESTIMATION_STEP_SIZE;
		}

	if ( RFMask )
		{
			i = 11;
			d[i] += ESTIMATION_STEP_SIZE;
			e1 = energyFunc( d );
			d[i] -= 2*ESTIMATION_STEP_SIZE;
			e2 = energyFunc( d );
			d[i] += ESTIMATION_STEP_SIZE;

			dp[i] = 2.0 * (e1 - e2) / ESTIMATION_STEP_SIZE;
		}

	if ( LFMask )
		{
			i = 12;
			d[i] += ESTIMATION_STEP_SIZE;
			e1 = energyFunc( d );
			d[i] -= 2*ESTIMATION_STEP_SIZE;
			e2 = energyFunc( d );
			d[i] += ESTIMATION_STEP_SIZE;

			dp[i] = 2.0 * (e1 - e2) / ESTIMATION_STEP_SIZE;
		}

	for(    ; i<6; i++ )		dp[i] *= 0.001;
	for(    ; i<9; i++ )		dp[i] *= 0.01;
	for(    ; i<NUM_DATA; i++ )	dp[i] *= 0.001;

	return energyFunc( d );
}

