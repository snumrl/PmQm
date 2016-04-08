
#include <fstream>
#include <string.h>
#include <stdlib.h>

#include "pm.h"

using namespace jhm;


//----------------------------------------------------------------------//
//						File I/O for "*.asf/*.amc"						//
//----------------------------------------------------------------------//

//
//  For acclaim files that are generated from Vicon Bodybuilder
//  using Acclaim_F.mp, Acclaim_F.mod, and Acclaim_F.ast
//
#define	PM_MOTION_NUM_JOINT_IN_ASF_FILE	31
#define PM_MOTION_NUM_DUMMY				 2
#define ASF_SCALE 3.0

enum AsfPart	{
	root=0,
	lhipjoint, lfemur, ltibia, lfoot, ltoes,
	rhipjoint, rfemur, rtibia, rfoot, rtoes,
	lowerback, upperback, thorax, lowerneck, upperneck, head,
	lclavicle, lhumerus, lradius, lwrist, lhand, lfingers, lthumb,
	rclavicle, rhumerus, rradius, rwrist, rhand, rfingers, rthumb
};

static char *AsfPartName[PM_MOTION_NUM_JOINT_IN_ASF_FILE] =
{
	"root",
	"lhipjoint", "lfemur", "ltibia", "lfoot", "ltoes",
	"rhipjoint", "rfemur", "rtibia", "rfoot", "rtoes",
	"lowerback", "upperback", "thorax", "lowerneck", "upperneck", "head",
	"lclavicle", "lhumerus", "lradius", "lwrist", "lhand", "lfingers", "lthumb",
	"rclavicle", "rhumerus", "rradius", "rwrist", "rhand", "rfingers", "rthumb"
};

static int	AsfParent[] =
{
	root,
	root, lhipjoint, lfemur, ltibia, lfoot,
	root, rhipjoint, rfemur, rtibia, rfoot,
	root, lowerback, upperback, thorax, lowerneck, upperneck,
	thorax, lclavicle, lhumerus, lradius, lwrist, lhand, lhand,
	thorax, rclavicle, rhumerus, rradius, rwrist, rhand, rhand
};

static int	AsfJointSequence[] =
{
	PmHuman::PELVIS,
	PmHuman::UNDEFINED, PmHuman::UPPER_LEFT_LEG,  PmHuman::LOWER_LEFT_LEG,  PmHuman::LEFT_FOOT,  PmHuman::LEFT_TOE,  
	PmHuman::UNDEFINED, PmHuman::UPPER_RIGHT_LEG, PmHuman::LOWER_RIGHT_LEG, PmHuman::RIGHT_FOOT, PmHuman::RIGHT_TOE, 
	PmHuman::SPINE_1, PmHuman::SPINE_2, PmHuman::SPINE_3, PmHuman::CHEST, PmHuman::NECK, PmHuman::HEAD,
	PmHuman::LEFT_COLLAR,  PmHuman::UPPER_LEFT_ARM,  PmHuman::LOWER_LEFT_ARM,  PmHuman::LEFT_PALM,  PmHuman::LEFT_FINGER_11,  PmHuman::LEFT_FINGER_21,  PmHuman::LEFT_FINGER_31,
	PmHuman::RIGHT_COLLAR, PmHuman::UPPER_RIGHT_ARM, PmHuman::LOWER_RIGHT_ARM, PmHuman::RIGHT_PALM, PmHuman::RIGHT_FINGER_11, PmHuman::RIGHT_FINGER_21, PmHuman::RIGHT_FINGER_31
};

static vector	AsfBaseTranslation[PM_MOTION_NUM_JOINT_IN_ASF_FILE];
static quater	AsfBaseRotation[PM_MOTION_NUM_JOINT_IN_ASF_FILE];

static int
GetPmHumanParentFromAsf(int i)
{
	while (1)
	{
		if (AsfJointSequence[AsfParent[i]] != PmHuman::UNDEFINED)
			return	AsfParent[i];

		i = AsfParent[i];
	}
}

void
ConvertASF2ACTOR_BB_F(char* asf_filename, char* skeleton_filename, m_real scale)
{
	std::ifstream is(asf_filename, std::ios::in);
	if (is.fail()) return;

	char	str[2048], keyword[256];
	while (1)
	{
		is.getline(str, 2048);	sscanf(str, "%s", keyword);

		if (strcmp(keyword, ":bonedata") == 0)	break;
	}
	
	is.getline(str, 2048);

	AsfBaseTranslation[0] = vector(0,0,0);
	AsfBaseRotation[0] = quater(1,0,0,0);

	int	i;
	float	tx, ty, tz, rx, ry, rz, s;
	for ( i = 1; i < PM_MOTION_NUM_JOINT_IN_ASF_FILE; i++)
	{
		while (1)
		{
			is.getline(str, 2048);	sscanf(str, "%s", keyword);

			if (strcmp(keyword, "direction") == 0)
				sscanf(str, "%s %f %f %f", keyword, &tx, &ty, &tz);
			
			if (strcmp(keyword, "length") == 0)
				sscanf(str, "%s %f", keyword, &s);
			
			if (strcmp(keyword, "axis") == 0)
				sscanf(str, "%s %f %f %f", keyword, &rx, &ry, &rz);
			
			if (strcmp(keyword, "end") == 0)	break;
		}

		tx *= s * scale * ASF_SCALE;
		ty *= s * scale * ASF_SCALE;
		tz *= s * scale * ASF_SCALE;
		rx = M_PI * rx / 180;
		ry = M_PI * ry / 180;
		rz = M_PI * rz / 180;

		quater q = exp(rz*z_axis/2) * exp(ry*y_axis/2) * exp(rx*x_axis/2);

		AsfBaseTranslation[i] = vector(tx, ty, tz) + AsfBaseTranslation[AsfParent[i]];
		AsfBaseRotation[i] = q;
	}
	
	is.close();

	std::ofstream	os(skeleton_filename);
	if (os.fail())	return;

	os << "root_height\t" << AsfBaseTranslation[0].y() << std::endl;
	m_real r_height = AsfBaseTranslation[AsfJointSequence[PmHuman::RIGHT_FOOT]].y();
	m_real l_height = AsfBaseTranslation[AsfJointSequence[PmHuman::LEFT_FOOT]].y();
	os << "ankle_height\t" << (l_height + r_height)/2.0 << std::endl;

	for (i = 0; i < PM_MOTION_NUM_JOINT_IN_ASF_FILE; i++)
	{
		if (AsfJointSequence[i] == PmHuman::UNDEFINED)	continue;

		os << PmHuman::getLinkName(AsfJointSequence[i]) << std::endl;
		os << "begin" << std::endl;

		int	parent = GetPmHumanParentFromAsf(i);
		if (AsfParent[i] != i)
			os << "\tparent\t" << PmHuman::getLinkName(AsfJointSequence[parent]) << std::endl;
		
		os << "\tcoordinate\t"
		   << AsfBaseTranslation[AsfParent[i]] << " "
		   << AsfBaseRotation[i] << std::endl;
		os << "end" << std::endl;
		os << std::endl;
	}

	os.close();
}


int
PmLinearMotion::saveAMC_BB_F( char* name )
{
	std::ofstream file( name, std::ios::out );
	if ( file.fail() ) return -1;

	file.precision( 10 );

	file << "# generated from PmQm by Jehee Lee" << std::endl;
	file << ":FULLY-SPECIFIED" << std::endl;
	file << ":DEGREES" << std::endl;

	PmPosture	h;
	vector		v, u;
	quater		q;

	for( int i=0; i<this->getSize(); i++ )
	{
		file << i+1 << std::endl;

		h = getPosture(i);

		for( int j=0; j<PM_MOTION_NUM_JOINT_IN_ASF_FILE; j++ )
		if ( AsfJointSequence[j] != PmHuman::UNDEFINED )
		{
			file << AsfPartName[j] << " ";

			if ( j == root )           // dof x y z rx ry rz
			{
				q = h.getRotation( AsfJointSequence[j] );
				u = Quater2EulerAngle(q);
				v = h.getTranslation();

				u = 180 * u / M_PI;
				v = v / ASF_SCALE;

				file << v[0] << " " << v[1] << " " << v[2] << " ";
				file << u[0] << " " << u[1] << " " << u[2] << std::endl;
			}
			else if ( j == lowerback ||
					  j == upperback ||
					  j == thorax    ||
					  j == lowerneck ||
					  j == upperneck ||
					  j == head      ||
					  j == rhumerus  ||
					  j == lhumerus  ||
					  j == rfemur    ||
					  j == lfemur )    // dof rx ry rz
			{
				q = h.getRotation( AsfJointSequence[j] );
				u = Quater2EulerAngle(q);

				u = 180 * u / M_PI;

				file << u[0] << " " << u[1] << " " << u[2] << std::endl;
			}
			else if ( j == ltibia   ||
					  j == ltoes    ||
					  j == rtibia   ||
					  j == rtoes    ||
					  j == lfingers ||
					  j == rfingers ||
					  j == lradius  ||
					  j == rradius )   // dof rx
			{
				q = h.getRotation( AsfJointSequence[j] );
				u = Quater2EulerAngle(q);

				u = 180 * u / M_PI;

				file << u[0] << std::endl;
			}
			else if ( j == lhand  ||
					  j == rhand  ||
					  j == lthumb ||
					  j == rthumb ||
					  j == lfoot  ||
					  j == rfoot )     // dof rx rz
			{
				q = h.getRotation( AsfJointSequence[j] );
				u = Quater2EulerAngle(q);

				u = 180 * u / M_PI;

				file << u[0] << " " << u[2] << std::endl;
			}
			else if ( j == lclavicle ||
					  j == rclavicle ) // dof ry rz
			{
				q = h.getRotation( AsfJointSequence[j] );
				u = Quater2EulerAngle(q);

				u = 180 * u / M_PI;

				file << u[1] << " " << u[2] << std::endl;
			}
			else if ( j == lwrist ||
					  j == rwrist )    // dof ry
			{
				q = h.getRotation( AsfJointSequence[j] );
				u = Quater2EulerAngle(q);

				u = 180 * u / M_PI;

				file << u[1] << std::endl;
			}
		}
	}

	return this->getSize();
}

int
PmLinearMotion::openAMC_BB_F( char* name, m_real scale, int subsample )
{
	std::ifstream file( name, std::ios::in );
	if ( file.fail() ) return -1;

	int		n=0;
	char	str[2048];

	while (!file.eof())  // count the number of lines
	{
		file.getline(str, 2048);
		if (file.eof()) break;

		int m = atoi(str);
		if (m > n) n = m;
	}
	file.close();
	file.clear();

	this->setSize( (n+subsample-1) / subsample );

	file.open( name, std::ios::in );

	while (1) // skip the header
	{
		file >> str;
		if (strcmp(str, ":DEGREES") == 0)	break;
	}

	PmPosture hv;

	int		index;
	quater	q;
	m_real	x, y, z, r, u, v;
	int	i, j, k;


	for( i=0; i<n; i++ )
	{
		file >> index;

		for( j=0; j<(PM_MOTION_NUM_JOINT_IN_ASF_FILE-PM_MOTION_NUM_DUMMY); j++ )
		{
			file >> str;

			for( k=0; k<PM_MOTION_NUM_JOINT_IN_ASF_FILE; k++ )
				if ( strcmp( str, AsfPartName[k] )==0 ) break;

			if ( strcmp( str, "root" )==0 ) // dof x y z rx ry rz
			{
				file >> x >> y >> z >> r >> u >> v;
				x = x * scale * ASF_SCALE;
				y = y * scale * ASF_SCALE;
				z = z * scale * ASF_SCALE;
				r = M_PI*r/180;
				u = M_PI*u/180;
				v = M_PI*v/180;

				q = exp(v*z_axis/2) * exp(u*y_axis/2) * exp(r*x_axis/2);
				hv.setTransf( PmHuman::PELVIS, transf(q,vector(x,y,z)) );
			}

			else if ( // dof rx ry rz
				strcmp( str, "lowerback" )==0 ||
				strcmp( str, "upperback" )==0 ||
				strcmp( str, "thorax" )==0 ||
				strcmp( str, "lowerneck" )==0 ||
				strcmp( str, "upperneck" )==0 ||
				strcmp( str, "head" )==0 ||
				strcmp( str, "rhumerus" )==0 ||
				strcmp( str, "lhumerus" )==0 ||
				strcmp( str, "rfemur" )==0 ||
				strcmp( str, "lfemur" )==0 )
			{
				file >> r >> u >> v;
				r = M_PI*r/180;
				u = M_PI*u/180;
				v = M_PI*v/180;

				q = exp(v*z_axis/2) * exp(u*y_axis/2) * exp(r*x_axis/2);
				hv.setRotation( AsfJointSequence[k], q );
			}

			else if ( // dof rx
				strcmp( str, "lfingers" )==0 ||
				strcmp( str, "rfingers" )==0 ||
				strcmp( str, "ltibia" )==0 ||
				strcmp( str, "ltoes" )==0 ||
				strcmp( str, "rtibia" )==0 ||
				strcmp( str, "rtoes" )==0 ||
				strcmp( str, "lradius" )==0 ||
				strcmp( str, "rradius" )==0 )
			{
				file >> r;
				r = M_PI*r/180;

				q = exp(r*x_axis/2);
				hv.setRotation( AsfJointSequence[k], q );
			}

			else if ( // dof rx rz
				strcmp( str, "lthumb" )==0 ||
				strcmp( str, "rthumb" )==0 ||
				strcmp( str, "lhand" )==0 ||
				strcmp( str, "rhand" )==0 ||
				strcmp( str, "lfoot" )==0 ||
				strcmp( str, "rfoot" )==0 )
			{
				file >> r >> v;
				r = M_PI*r/180;
				v = M_PI*v/180;

				q = exp(v*z_axis/2) * exp(r*x_axis/2);
				hv.setRotation( AsfJointSequence[k], q );
			}

			else if ( // dof ry rz
				strcmp( str, "lclavicle" )==0 ||
				strcmp( str, "rclavicle" )==0 )
			{
				file >> u >> v;
				u = M_PI*u/180;
				v = M_PI*v/180;

				q = exp(v*z_axis/2) * exp(u*y_axis/2);
				hv.setRotation( AsfJointSequence[k], q );
			}

			else if ( // dof ry
				strcmp( str, "lwrist" )==0 ||
				strcmp( str, "rwrist" )==0 )
			{
				file >> u;
				u = M_PI*u/180;

				q = exp(u*y_axis/2);
				hv.setRotation( AsfJointSequence[k], q );
			}
		}

		if ( i%subsample == 0 )
			this->setPosture( i/subsample, hv );
		else
			this->getPosture( i/subsample ).blend( 1.0 / (i%subsample + 1), hv );
	}

	this->alignment();

	file.close();

	return n;
}
