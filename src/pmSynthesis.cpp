
#include "pm.h"
#include <fstream>

using namespace jhm;

#define VERY_LARGE		10e+10
#define MAX_EXAMPLES	5


static m_real featureCompare( int, int, int, int );

static PmLinearMotion ori_m[MAX_EXAMPLES][PM_MOTION_MAX_LEVEL];
static PmVectorArray  ori_v[MAX_EXAMPLES][PM_MOTION_MAX_LEVEL];

static PmLinearMotion syn_m[PM_MOTION_MAX_LEVEL];
static PmVectorArray  syn_v[PM_MOTION_MAX_LEVEL];

static int map_k[PM_MAX_FRAMES];
static int map_e[PM_MAX_FRAMES];



void
PmMRmotion::reconstructDetail( int num_examples, PmMRmotion **exam )
{
	assert( num_examples <= MAX_EXAMPLES );
	
	PmMaskType mask = this->getMask();

	int						i, j, k, e, l;
	static PmVector			v;
	static PmLinearMotion	temp;

	//
	//	Reconstruct Detail Coefficients
	//
	int		b, h;
	int		min_k[MAX_EXAMPLES];
	m_real	min_v[MAX_EXAMPLES];
	m_real	value;

	for( i=0; i<this->getNumLevels(); i++ )
	{
		l = this->getNumLevels() - i - 1;

		//
		//	Precompute Accelerations at each layer
		//
		this->linearize( syn_m[i], l+1 );
		syn_v[i].setSize( syn_m[i].getSize() );

		for( j=0; j<syn_m[i].getSize(); j++ )
		{
			syn_m[i].getAcceleration( j, v );
			syn_v[i].setVector( j, v );
		}

		for( e=0; e<num_examples; e++ )
		{
			exam[e]->linearize( ori_m[e][i], l+1 );
			ori_v[e][i].setSize( ori_m[e][i].getSize() );

			for( j=0; j<ori_m[e][i].getSize(); j++ )
			{
				ori_m[e][i].getAcceleration( j, v );
				ori_v[e][i].setVector( j, v );
			}
		}
		
		for( j=0; j<this->getLevelLength(l); j+=2 )
		{
			displace[l].vectors[j].reset( mask, vector(0,0,0) );

			if ( i==0 )
				knot_seqs[l][j] = base_level.knots[j/2];
			else
				knot_seqs[l][j] = knot_seqs[l+1][j/2];
		}

		//
		//	Multiresolution Sampling
		//
		for( j=1; j<this->getLevelLength(l); j+=2 )
		{
			int step = pow(2.0,i+1);

			b = j / step;
			h = j - step*b; 

			for( e=0; e<num_examples; e++ )
			{
				min_k[e] = -1;
				min_v[e] = VERY_LARGE;
				
				for( k=h; k<exam[e]->getLevelLength(l); k+=step )
				{
					value = featureCompare( i, j, k, e );
					value *= value;

					if ( value < min_v[e] )
					{
						min_k[e] = k;
						min_v[e] = value;
					}
				}
			}

			if ( num_examples==2 )
			{
				m_real t = min_v[0] / (min_v[0] + min_v[1]);
				v.interpolate( t, exam[0]->displace[l].getVector(min_k[0]),
								  exam[1]->displace[l].getVector(min_k[1]) );

				displace[l].setVector( j, v );
			}
			else
				displace[l].setVector( j,
					exam[0]->displace[l].getVector(min_k[0]) );

			knot_seqs[l][j] = (knot_seqs[l][j-1] + knot_seqs[l][j+1]) / 2;

			if ( i==getNumLevels()-1 )
			{
				map_e[j] = 0;
				map_k[j] = min_k[0];
				
				if ( num_examples==2 && min_v[1]<min_v[0] )
				{
					map_e[j] = 1;
					map_k[j] = min_k[1];
				}
			}
		}
	}

	//
	//	Reconstruct Constraints
	//
	int n = pow(2.,getNumLevels());

	for( j=0; j<getSize(); j++ )
	{
		if ( j%2 == 0 )
		{
			if ( j == 0 )
			{
				map_k[j] = map_k[j+1]-1;

				if ( map_k[j] < 0 ) map_k[j] = 0;
			}
			else if ( j == getLevelLength(0)-1 )
			{
				map_k[j] = map_k[j-1]+1;

				if ( map_k[j] >= exam[map_e[j]]->getSize()-1 )
					map_k[j] = exam[map_e[j]]->getSize()-1;
			}
			else if ( int(map_k[j-1]/n) == int(map_k[j+1]/n) )
				map_k[j] = (map_k[j-1] + map_k[j+1])/2;
			else
				map_k[j] = map_k[j+1]-1;
			
			if ( j == 0 )
				map_e[j] = map_e[j+1];
			else
				map_e[j] = map_e[j-1];
		}

		constraints[j] = exam[map_e[j]]->constraints[map_k[j]];
	}

	//  Debugging
	std::ofstream file( "output", std::ios::out );
 	for( j=0; j<getSize(); j++ )
 	{
 		file << j << "\t" << map_e[j] << "\t" << map_k[j] << std::endl;
 	}
	file.close();

/*
	//  Remove isolated constraints
	for( i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	for( j=0; j<getSize(); j++ )
		if ( constraints[j].isConstrained(i) )
		{
			if ((j==0           && !constraints[j+1].isConstrained(i)) ||
			    (j==getSize()-1 && !constraints[j-1].isConstrained(i)) ||
				(j>0 && j<getSize()-1) &&
				(!constraints[j+1].isConstrained(i) &&
				 !constraints[j-1].isConstrained(i)) ) 
					constraints[j].remove(i);
		}
*/

	transf t;
	static PmLinearMotion lm;
	this->linearize( lm );

	for( i=0; i<PM_HUMAN_NUM_LINKS; i++ )
	for( j=0; j<getSize(); j++ )
		if ( constraints[j].isConstrained(i) )
		{
			int kc = map_k[j];
			t = constraints[j].getTransf(i).inverse() *
				lm.getPosture(j).getGlobalTransf(i);

			constraints[j].setTransf( i, lm.getPosture(j).getGlobalTransf(i) );
			j++;
			kc++;

			for( ; j<getSize(); j++, kc++ )
			{
				if ( ! constraints[j].isConstrained(i) ) break;

//				t = exam[map_e[j]]->constraints[kc-1].getTransf(i).inverse() *
//					exam[map_e[j]]->constraints[kc  ].getTransf(i);
				t = exam[map_e[j]]->constraints[kc  ].getTransf(i) *
					exam[map_e[j]]->constraints[kc-1].getTransf(i).inverse();

				constraints[j].setTransf( i,
					 t * constraints[j-1].getTransf(i)  );

//				constraints[j].setTransf( i,
//					exam[map_e[j]]->constraints[kc].getTransf(i) * t );
			}
		}
}


//----------------------------------------------------------------------//


// static m_real
// featureCompare( int level, int index_s, int index_o, int exam )
// {
// 	m_real f=0;
// 
// 	static PmVector  d1, d2;
// 	static PmVector  vs1, vs2,
// 					 vo1, vo2;
// 
// 	for( int i=0; i<=level; i++ )
// 	{
// 		int d = pow( 2, level - i + 1 );
// 
// 		m_real n = 1.0;
// 		if ( index_s/d == 0 || index_o/d == 0 ) n = 2.0;
// 		if ( index_s/d == syn_m[i].getSize()-1 ||
// 			 index_o/d == ori_m[exam][i].getSize()-1 ) n = 2.0;
// 
// 		//
// 		//		Estimate the change of velocity
// 		//
// 		vs1 = syn_v[i].getVector( index_s/d );
// 		vs2 = syn_v[i].getVector( index_s/d + 1 );
// 		vo1 = ori_v[exam][i].getVector( index_o/d );
// 		vo2 = ori_v[exam][i].getVector( index_o/d + 1 );
// 		d1.difference( vs1, vo1 );
// 		d2.difference( vs2, vo2 );
// 
// 		f += (d1.magnitude() + d2.magnitude()) * n;
// 	}
// 
// 	return f;
// }

static m_real
featureCompare( int level, int index_s, int index_o, int exam )
{
	m_real f=0;
	
	static PmVector d1, d2;
	static PmVector vs1, vs2,
					vo1, vo2;

	for( int i=0; i<=level; i++ )
	{
		int d = pow( 2., level - i + 1 );

		m_real n = 1.0;
		if ( index_s/d == 0 || index_o/d == 0 ) n = 2.0;
		if ( index_s/d == syn_m[i].getSize()-1 ||
			 index_o/d == ori_m[exam][i].getSize()-1 ) n = 2.0;

		//
		//		Estimate the change of velocity
		//
		vs1 = syn_v[i].getVector( index_s/d );
		vs2 = syn_v[i].getVector( index_s/d + 1 );
		vo1 = ori_v[exam][i].getVector( index_o/d );
		vo2 = ori_v[exam][i].getVector( index_o/d + 1 );
		
		double dd = d;
		double index_ds = index_s;
		double t = index_ds / dd - (index_s/d);
		
		d1.difference( vs1, vo1 );
		d2.difference( vs2, vo2 );

		f += ((1.0-t)*d1.magnitude() + t*d2.magnitude()) * n;
	}

	return f;
}

