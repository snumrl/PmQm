
#include "pm.h"
#include <stdlib.h>
#include <string.h>
#include <fstream>

using namespace jhm;

static int symmetric_part[PM_HUMAN_NUM_LINKS] =
{
	PmHuman::PELVIS,
	PmHuman::SPINE_1,
	PmHuman::SPINE_2,
	PmHuman::SPINE_3,
	PmHuman::SPINE_4,
	PmHuman::CHEST, 
	PmHuman::NECK, PmHuman::HEAD,
	PmHuman::LEFT_SHOULDER, PmHuman::RIGHT_SHOULDER,
	PmHuman::LEFT_COLLAR,   PmHuman::RIGHT_COLLAR,
	PmHuman::UPPER_LEFT_ARM, PmHuman::UPPER_RIGHT_ARM,
	PmHuman::LOWER_LEFT_ARM, PmHuman::LOWER_RIGHT_ARM,
	PmHuman::UPPER_LEFT_LEG, PmHuman::UPPER_RIGHT_LEG, 
	PmHuman::LOWER_LEFT_LEG, PmHuman::LOWER_RIGHT_LEG,
	PmHuman::LEFT_FOOT, PmHuman::RIGHT_FOOT,
	PmHuman::LEFT_TOE, PmHuman::RIGHT_TOE,
	PmHuman::LEFT_PALM,	PmHuman::RIGHT_PALM,
	PmHuman::LEFT_HEEL, PmHuman::RIGHT_HEEL,
	PmHuman::LEFT_FINGER_11, PmHuman::LEFT_FINGER_12, PmHuman::LEFT_FINGER_13,
	PmHuman::LEFT_FINGER_21, PmHuman::LEFT_FINGER_22, PmHuman::LEFT_FINGER_23,
	PmHuman::LEFT_FINGER_31, PmHuman::LEFT_FINGER_32, PmHuman::LEFT_FINGER_33,
	PmHuman::LEFT_FINGER_41, PmHuman::LEFT_FINGER_42, PmHuman::LEFT_FINGER_43,
	PmHuman::LEFT_FINGER_51, PmHuman::LEFT_FINGER_52, PmHuman::LEFT_FINGER_53,
	PmHuman::RIGHT_FINGER_11, PmHuman::RIGHT_FINGER_12, PmHuman::RIGHT_FINGER_13,
	PmHuman::RIGHT_FINGER_21, PmHuman::RIGHT_FINGER_22, PmHuman::RIGHT_FINGER_23,
	PmHuman::RIGHT_FINGER_31, PmHuman::RIGHT_FINGER_32, PmHuman::RIGHT_FINGER_33,
	PmHuman::RIGHT_FINGER_41, PmHuman::RIGHT_FINGER_42, PmHuman::RIGHT_FINGER_43,
	PmHuman::RIGHT_FINGER_51, PmHuman::RIGHT_FINGER_52, PmHuman::RIGHT_FINGER_53
};

static char* human_part_name[PM_HUMAN_NUM_LINKS] =
{
	  "pelvis",
	  "spine_1", "spine_2", "spine_3", "spine_4", "chest",
      "neck", "head",
	  "right_shoulder", "left_shoulder",
 	  "right_collar", "left_collar",
      "upper_right_arm", "upper_left_arm", "lower_right_arm", "lower_left_arm",
      "upper_right_leg", "upper_left_leg", "lower_right_leg", "lower_left_leg",
      "right_foot", "left_foot", "right_toe", "left_toe",
      "right_palm", "left_palm",
	  "right_heel", "left_heel",
      "right_finger_11", "right_finger_12", "right_finger_13",
      "right_finger_21", "right_finger_22", "right_finger_23",
      "right_finger_31", "right_finger_32", "right_finger_33",
      "right_finger_41", "right_finger_42", "right_finger_43",
      "right_finger_51", "right_finger_52", "right_finger_53",
      "left_finger_11", "left_finger_12", "left_finger_13",
      "left_finger_21", "left_finger_22", "left_finger_23",
      "left_finger_31", "left_finger_32", "left_finger_33",
      "left_finger_41", "left_finger_42", "left_finger_43",
      "left_finger_51", "left_finger_52", "left_finger_53"
};

static m_real mass_list[PM_HUMAN_NUM_LINKS] =
{
	1.0, // pelvis
	0.0, // spine_1
	0.0, // spine_2
	0.0, // spine_3
	0.0, // spine_4
	1.5, // chest
	0.5, // neck
	1.5, // head
	0.0, 0.0,  // shoulders
	0.0, 0.0,  // clavicles
	0.6, 0.6, // upper arms
	0.4, 0.4, // lower arms
	1.0, 1.0, // upper legs
	0.7, 0.7, // lower legs
	0.3, 0.3, // feet
	0.0, 0.0, // toes
	0.2, 0.2,
	0.1, 0.1,
	0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, // right hand
	0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0  // left hand
};


PmHuman::PmHuman()
{
	num_links = 0;

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
		angle_bound[i] = NULL;

	num_spine_links = 0;
}

PmHuman::PmHuman( char *file_name, double scale )
{
    num_links = 0;

    for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
		angle_bound[i] = NULL;

	openHuman( file_name, scale );
	setAngleBound();

	num_spine_links = 0;
	if (mask & MaskBit(SPINE_1)) num_spine_links++;
	if (mask & MaskBit(SPINE_2)) num_spine_links++;
	if (mask & MaskBit(SPINE_3)) num_spine_links++;
	if (mask & MaskBit(SPINE_4)) num_spine_links++;
}

PmHuman::~PmHuman()
{
}

void
PmHuman::setAngleBound()
{
	QmComplex* bound;
	unit_vector u;
	quater q;
	quater I(1,0,0,0);

	if ( num_spine_links > 0 )
	{
		q = exp( x_axis*M_PI/48. );
		u = normalize( rotate( q, y_axis ) );
		bound = new QmComplex;
		bound->addHalfspace(new QmConicHS(q,u,interval(0,M_PI/13.)));
		bound->addHalfspace(new QmAxialHS(I,y_axis,interval(-M_PI/18.,M_PI/18.)));

		angle_bound[PmHuman::SPINE_1] = bound;
		angle_bound[PmHuman::SPINE_2] = bound;
		angle_bound[PmHuman::SPINE_3] = bound;
		angle_bound[PmHuman::SPINE_4] = bound;
		angle_bound[CHEST] = bound;
	}
	else
	{
		q = exp( x_axis*M_PI/12. );
		u = normalize( rotate( q, y_axis ) );
		bound = new QmComplex;
		bound->addHalfspace(new QmConicHS(q,u,interval(0,M_PI/3.)));
		bound->addHalfspace(new QmAxialHS(I,y_axis,interval(-M_PI/3.,M_PI/3.)));
		angle_bound[CHEST] = bound;
	}

	bound = new QmComplex;
	bound->addHalfspace(new QmSphericalHS(I,interval(0,M_PI/4.)));
	angle_bound[NECK] = bound;

	bound = new QmComplex;
	bound->addHalfspace(new QmSphericalHS(I,interval(0,M_PI/4.)));
	angle_bound[HEAD] = bound;

	q = exp( -y_axis*M_PI/12.) * exp( -x_axis*M_PI/4. );
	u = normalize( rotate( q, y_axis ) );
	bound = new QmComplex;
	bound->addHalfspace(new QmConicHS(q,u,interval(0,M_PI*2./3.)));
	bound->addHalfspace(new QmAxialHS(I,-y_axis,interval(-M_PI/2.,M_PI/2.)));
	angle_bound[UPPER_RIGHT_ARM] = bound;

	q = exp( y_axis*M_PI/12.) * exp( -x_axis*M_PI/4. );
	u = normalize( rotate( q, y_axis ) );
	bound = new QmComplex;
	bound->addHalfspace(new QmConicHS(q,u,interval(0,M_PI*2./3.)));
	bound->addHalfspace(new QmAxialHS(I,y_axis,interval(-M_PI/2.,M_PI/2.)));
	bound->addHalfspace(new QmAxialHS(I,-x_axis,interval(0,M_PI*6./7.)));
	angle_bound[UPPER_LEFT_ARM] = bound;

	bound = new QmComplex;
	bound->addHalfspace(new QmAxialHS(I, -x_axis,interval(0,M_PI*8./9.)));
	bound->addHalfspace(new QmConicHS(I, x_axis,interval(0,1e-4)));
	angle_bound[LOWER_RIGHT_ARM] = bound;
	angle_bound[LOWER_LEFT_ARM] = bound;

	bound = new QmComplex;
	bound->addHalfspace(new QmAxialHS(I,-x_axis,interval(-M_PI/4.,M_PI/2.)));
	bound->addHalfspace(new QmAxialHS(I,-y_axis,interval(-M_PI/6.,M_PI/2.)));
	bound->addHalfspace(new QmAxialHS(I,-z_axis,interval(-M_PI/6.,M_PI/2.)));
	angle_bound[UPPER_RIGHT_LEG] = bound;

	bound = new QmComplex;
	bound->addHalfspace(new QmAxialHS(I,-x_axis,interval(-M_PI/4.,M_PI/2.)));
	bound->addHalfspace(new QmAxialHS(I, y_axis,interval(-M_PI/6.,M_PI/2.)));
	bound->addHalfspace(new QmAxialHS(I, z_axis,interval(-M_PI/6.,M_PI/2.)));
	angle_bound[UPPER_LEFT_LEG] = bound;

	bound = new QmComplex;
	bound->addHalfspace(new QmAxialHS(I, x_axis,interval(0,M_PI*8./9.)));
	bound->addHalfspace(new QmConicHS(I, x_axis,interval(0,1e-4)));
	angle_bound[LOWER_RIGHT_LEG] = bound;
	angle_bound[LOWER_LEFT_LEG] = bound;

	bound = new QmComplex;
	bound->addHalfspace(new QmAxialHS(I,-x_axis,interval(-M_PI/2.,M_PI/4.)));
	bound->addHalfspace(new QmAxialHS(I, y_axis,interval(-M_PI/6.,M_PI/6.)));
	bound->addHalfspace(new QmAxialHS(I, z_axis,interval(-M_PI/4.,M_PI/4.)));
	angle_bound[RIGHT_FOOT] = bound;

	bound = new QmComplex;
	bound->addHalfspace(new QmAxialHS(I,-x_axis,interval(-M_PI/2.,M_PI/4.)));
	bound->addHalfspace(new QmAxialHS(I, y_axis,interval(-M_PI/6.,M_PI/6.)));
	bound->addHalfspace(new QmAxialHS(I, z_axis,interval(-M_PI/4.,M_PI/4.)));
	angle_bound[LEFT_FOOT] = bound;

	bound = new QmComplex;
	bound->addHalfspace(new QmAxialHS(I,x_axis,interval(-M_PI/2.,0)));
	angle_bound[RIGHT_TOE] = bound;

	bound = new QmComplex;
	bound->addHalfspace(new QmAxialHS(I,x_axis,interval(-M_PI/2.,0)));
	angle_bound[LEFT_TOE] = bound;

	bound = new QmComplex;
	bound->addHalfspace(new QmAxialHS(I, x_axis, interval(-EPS_jhm, EPS_jhm)));
	angle_bound[RIGHT_HEEL] = bound;

	bound = new QmComplex;
	bound->addHalfspace(new QmAxialHS(I, x_axis, interval(-EPS_jhm, EPS_jhm)));
	angle_bound[LEFT_HEEL] = bound;

/*
	q = exp( z_axis*M_PI/12.);
	u = normalize( rotate( q, y_axis ) );
	bound = new QmComplex;
	bound->addHalfspace(new QmConicHS(q,u,interval(0,M_PI/3.)));
	bound->addHalfspace(new QmAxialHS(I,y_axis,interval(-M_PI/2.,M_PI/2.)));
	angle_bound[RIGHT_PALM] = bound;
*/

	bound = new QmComplex;
	bound->addHalfspace(new QmAxialHS(I,z_axis,interval(0,M_PI/2.)));
	angle_bound[RIGHT_FINGER_21] = bound;
	angle_bound[RIGHT_FINGER_22] = bound;
	angle_bound[RIGHT_FINGER_23] = bound;
	angle_bound[RIGHT_FINGER_31] = bound;
	angle_bound[RIGHT_FINGER_32] = bound;
	angle_bound[RIGHT_FINGER_33] = bound;
	angle_bound[RIGHT_FINGER_41] = bound;
	angle_bound[RIGHT_FINGER_42] = bound;
	angle_bound[RIGHT_FINGER_43] = bound;
	angle_bound[RIGHT_FINGER_51] = bound;
	angle_bound[RIGHT_FINGER_52] = bound;
	angle_bound[RIGHT_FINGER_53] = bound;

/*
	q = exp( -z_axis*M_PI/12.);
	u = normalize( rotate( q, y_axis ) );
	bound = new QmComplex;
	bound->addHalfspace(new QmConicHS(q,u,interval(0,M_PI/3.)));
	bound->addHalfspace(new QmAxialHS(I,y_axis,interval(-M_PI/2.,M_PI/2.)));
	angle_bound[LEFT_PALM] = bound;
*/

	bound = new QmComplex;
	bound->addHalfspace(new QmAxialHS(I,-z_axis,interval(0,M_PI/2.)));
	angle_bound[LEFT_FINGER_21] = bound;
	angle_bound[LEFT_FINGER_22] = bound;
	angle_bound[LEFT_FINGER_23] = bound;
	angle_bound[LEFT_FINGER_31] = bound;
	angle_bound[LEFT_FINGER_32] = bound;
	angle_bound[LEFT_FINGER_33] = bound;
	angle_bound[LEFT_FINGER_41] = bound;
	angle_bound[LEFT_FINGER_42] = bound;
	angle_bound[LEFT_FINGER_43] = bound;
	angle_bound[LEFT_FINGER_51] = bound;
	angle_bound[LEFT_FINGER_52] = bound;
	angle_bound[LEFT_FINGER_53] = bound;
}

void
PmHuman::openHuman( char* file_name, double scale )
{
	std::ifstream in_actor( file_name, std::ios::in );

	char			buf[256];
	char			geom_name[256];
	char			parent_name[30];
	vector			displacep;
	quater			displaceq;
	int				coord_flag;
	QmComplex*		bound;

	PmPosture		posture;
	posture.setMask( PM_MASK_NULL );
	posture.setBody( this );
	posture.setScale( 1.0 );

	in_actor >> buf; in_actor >> buf;
	root_height = atof( buf );

	in_actor >> buf; in_actor >> buf;
	ankle_height = atof( buf );

	while( 1 )
	{
		in_actor >> buf;
		int part_num = getLinkNumber( buf );
		if ( part_num==-1 ) break;

		in_actor >> buf;
		if ( strcmp(buf, "begin")!=0 ) break;

		strcpy( geom_name, "" );
		strcpy( parent_name, "" );

		in_actor >> buf;
		while ( 1 )
		{
			if ( strcmp(buf, "geometry")==0 )
			{
				in_actor >> buf;
			}
			else if ( strcmp(buf, "parent")==0 )
			{
				in_actor >> parent_name;
			}
			else if ( strcmp(buf, "displace")==0 )
			{
				coord_flag = 0;
				in_actor >> displacep;
				in_actor >> displaceq;
			}
			else if ( strcmp(buf, "coordinate")==0 )
			{
				coord_flag = 1;
				in_actor >> displacep;
				in_actor >> displaceq;
			}
			else if ( strcmp(buf, "lcoordinate")==0 )
			{
				coord_flag = 2;
				in_actor >> displacep;
				in_actor >> displaceq;
			}
			else if ( strcmp(buf, "angle_bound")==0 )
			{
				bound = new QmComplex;
				in_actor >> (*bound);
			}
			else	{
				if (strlen(geom_name) == 0)	break;

				break;
			}

			in_actor >> buf;
		}

		num_links++;

		int parent_num = getLinkNumber( parent_name );
		parent_list[part_num] = parent_num;

		displacep *= scale;

		if ( parent_num>=0 )
		{
			switch (coord_flag)
			{
			case 0 :
				base_transf[part_num] = transf( displaceq, displacep );
				break;
			case 1 :
			case 2 :
				base_transf[part_num] = 
					transf( displaceq, displacep ) *
					posture.getGlobalTransf(parent_num).inverse();
				break;
			}

			posture.setTransf( part_num, identity_transf );
		}
		else
		{
			posture.setTransf( part_num, transf( displaceq, displacep ) );
		}
	}

	mask = posture.getMask();
}

char*
PmHuman::getLinkName( int i )
{
	return human_part_name[i];
}

int
PmHuman::getLinkNumber( char* s )
{
	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
        if ( strcmp( s, human_part_name[i] )==0 ) return i;

	return -1;
}

char
PmHuman::isAncestor( int a, int c )
{
	while( a <= c && c != -1 )
	{
		if ( a==c ) return TRUE;
		c = getParent(c);
	}

	return FALSE;
}

void
PmHuman::reshape( vector* list )
{
    for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
        base_transf[i] = translate_transf( list[i] );
}

m_real
PmHuman::getMass( int i ) const
{
	return mass_list[i];
}

void
PmHuman::setMass( int i, m_real m )
{
	mass_list[i] = m;
}

int
PmHuman::getSymmetricPart( int i )
{
	return symmetric_part[i];
}

