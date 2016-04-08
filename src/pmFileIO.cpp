
#include <fstream>
#include <string.h>
#include <stdlib.h>

#include "pm.h"

using namespace jhm;

//#define _NEW_ACCLAIM_FORMAT__

//----------------------------------------------------------------------//
//							File I/O for "*.mov"						//
//----------------------------------------------------------------------//

const int	PM_MOTION_NUM_JOINT_IN_MOV_FILE = 18;
static int joint_sequence[PM_MOTION_NUM_JOINT_IN_MOV_FILE] =
{
	PmHuman::PELVIS,
	PmHuman::CHEST,
	PmHuman::NECK,
	PmHuman::HEAD,
	PmHuman::UNDEFINED,
	PmHuman::UPPER_LEFT_ARM,
	PmHuman::LOWER_LEFT_ARM,
	PmHuman::LEFT_PALM,
	PmHuman::UNDEFINED,
	PmHuman::UPPER_RIGHT_ARM,
	PmHuman::LOWER_RIGHT_ARM,
	PmHuman::RIGHT_PALM,
	PmHuman::UPPER_LEFT_LEG,
	PmHuman::LOWER_LEFT_LEG,
	PmHuman::LEFT_FOOT,
	PmHuman::UPPER_RIGHT_LEG,
	PmHuman::LOWER_RIGHT_LEG,
	PmHuman::RIGHT_FOOT
};

const int	PM_MOTION_NUM_JOINT_NOT_IN_MOV_FILE = 4;
static int aux_joint_sequence[PM_MOTION_NUM_JOINT_NOT_IN_MOV_FILE] =
{
	PmHuman::RIGHT_HEEL,
	PmHuman::LEFT_HEEL,
	PmHuman::RIGHT_TOE,
	PmHuman::LEFT_TOE
};

int
PmLinearMotion::saveMOV( char* name )
{
	std::ofstream file( name, std::ios::out );
	if ( file.fail() ) return -1;

	file.precision( 20 );

	transf t;
	quater q;
	vector e;
	m_real x, y, z;
	
	for( int i=0; i<getSize(); i++ )
	{
		PmPosture& hv = this->getPosture(i);
		
		e = hv.getTranslation();
		x = e.x(); y = e.y(); z = e.z();
		file << x << " " << y << " " << z << " ";

		q = hv.getRotation( PmHuman::PELVIS );
		e = 180.0*Quater2EulerAngle(q)/M_PI;
		x = e.x(); y = e.y(); z = e.z();
		file << x << " " << y << " " << z << " ";

		for( int j=1; j<PM_MOTION_NUM_JOINT_IN_MOV_FILE; j++ )
		{
			if ( joint_sequence[j] == PmHuman::UNDEFINED )
			{
				file << "0 0 0 0 0 0 ";
			}
			else
			{
				q = hv.getRotation(joint_sequence[j]);

				if ( joint_sequence[j] == PmHuman::CHEST )
					q = exp( ln(q) * (this->getBody()->getNumSpineLinks()+1) );

				e = 180.0*Quater2EulerAngle(q)/M_PI;
				x = e.x(); y = e.y(); z = e.z();

				file << "0 0 0 " << x << " " << y << " " << z << " ";
			}
		}

		file << std::endl;
	}

	file.close();

	return getSize();
}

int
PmLinearMotion::openMOV( char* name, m_real scale )
{
	assert( this->getBody() );

	std::ifstream file( name, std::ios::in );
	if ( file.fail() ) return -1;

	int n=0;
	static char str[2048];

	while (!file.eof())  // count the number of lines
	{
		file.getline(str, 2048);
		if (file.eof()) break;
		n++;
	}
	file.close();

	file.open( name );
	this->setSize( n );

	PmPosture hv;
	hv.setBody( this->getBody() );

	m_real x, y, z, r, u, v;

	for( int i=0; i<n; i++ )
	{
		file >> x >> y >> z >> r >> u >> v;

		x = scale * x;
		y = scale * y;
		z = scale * z;

		matrix m = EulerAngle2Matrix( vector(M_PI*r/180,M_PI*u/180,M_PI*v/180) );
		hv.setTransf( joint_sequence[0], transf(m,vector(x,y,z)) );

		int j;
		for( j=1; j<PM_MOTION_NUM_JOINT_IN_MOV_FILE; j++ )
		{
			file >> x >> y >> z >> r >> u >> v;
		
			if ( joint_sequence[j] != PmHuman::UNDEFINED )
			{
				matrix m = EulerAngle2Matrix( vector(M_PI*r/180,M_PI*u/180,M_PI*v/180) );
				hv.setTransf( joint_sequence[j], transf(m, vector(0, 0, 0)) );
			}
			else	{
				static vector	collar[2];

			}
		}

		for (j=0; j<PM_MOTION_NUM_JOINT_NOT_IN_MOV_FILE; j++)
		{
			if (!(hv.getBody()->getMask() & MaskBit(aux_joint_sequence[j])))	continue;
			
			hv.setTransf(aux_joint_sequence[j], identity_transf);
		}
		
		quater q = exp( ln(hv.getRotation(PmHuman::CHEST)) /
						(this->getBody()->getNumSpineLinks() + 1) );

		hv.setRotation( PmHuman::CHEST, q );

		for( j=0; j<this->getBody()->getNumSpineLinks(); j++ )
			hv.setRotation( PmHuman::CHEST-j-1, q );
	
		hv.setScale(1.0);

		this->setPosture( i, hv );
	}

	this->alignment();

	file.close();

	return n;
}

//----------------------------------------------------------------------//
//							File I/O for "*.htr"						//
//----------------------------------------------------------------------//

#define	PM_MOTION_NUM_JOINT_IN_HTR_FILE	23

enum HtrPart	{
	HtrLowerTorso = 0, HtrMidTorso, HtrLPelvis, HtrRPelvis,
	HtrLThigh, HtrRThigh, HtrLLowLeg, HtrRLowLeg,
	HtrLFoot, HtrRFoot, HtrLToe, HtrRToe, 
	HtrUpperTorso, HtrLCollarBone, HtrRCollarBone,
	HtrLUpArm, HtrRUpArm, HtrLLowArm, HtrRLowArm, HtrLHand, HtrRHand,
	HtrNeck, HtrHead
};

static int	HtrParent[] =
{
	HtrLowerTorso, HtrLowerTorso, HtrLowerTorso, HtrLowerTorso,
	HtrLPelvis, HtrRPelvis, HtrLThigh, HtrRThigh,
	HtrLLowLeg, HtrRLowLeg, HtrLFoot, HtrRFoot,
	HtrMidTorso, HtrUpperTorso, HtrUpperTorso,
	HtrLCollarBone, HtrRCollarBone,	HtrLUpArm, HtrRUpArm, HtrLLowArm, HtrRLowArm, 
	HtrUpperTorso, HtrNeck 
};

static int	HtrJointSequence[] =
{
	PmHuman::PELVIS, PmHuman::SPINE_2, PmHuman::UNDEFINED, PmHuman::UNDEFINED,
	PmHuman::UPPER_LEFT_LEG, PmHuman::UPPER_RIGHT_LEG, PmHuman::LOWER_LEFT_LEG, PmHuman::LOWER_RIGHT_LEG,
	PmHuman::LEFT_FOOT,	PmHuman::RIGHT_FOOT, PmHuman::LEFT_TOE, PmHuman::RIGHT_TOE,
	PmHuman::CHEST, PmHuman::LEFT_COLLAR, PmHuman::RIGHT_COLLAR,
	PmHuman::UPPER_LEFT_ARM, PmHuman::UPPER_RIGHT_ARM, PmHuman::LOWER_LEFT_ARM,
	PmHuman::LOWER_RIGHT_ARM, PmHuman::LEFT_PALM, PmHuman::RIGHT_PALM,
	PmHuman::NECK, PmHuman::HEAD	
};

static int
GetPmHumanParentFromHTR(int i)
{
	while (1)
	{
		if (HtrJointSequence[HtrParent[i]] != PmHuman::UNDEFINED)
			return	HtrParent[i];

		i = HtrParent[i];
	}
}

transf	HtrBaseTransf[PM_MOTION_NUM_JOINT_IN_HTR_FILE];
m_real	HtrLength[PM_MOTION_NUM_JOINT_IN_HTR_FILE];
transf	HtrTransf[PM_MAX_FRAMES][PM_MOTION_NUM_JOINT_IN_HTR_FILE];

int
PmLinearMotion::openHTR( char* name, m_real scale )
{
	assert( this->getBody() );

	std::ifstream is( name, std::ios::in );
	if ( is.fail() ) return -1;

	char	str[2048], keyword[256];
	int	n;
	while (1)
	{
		is.getline(str, 2048);	sscanf(str, "%s", keyword);

		if (strcmp(keyword, "NumFrames") == 0)
			sscanf(str, "%s %d", keyword, &n);
		
		if (strcmp(keyword, "[BasePosition]") == 0)	break;
	}

	is.getline(str, 2048);

	char	part[256];
	m_real	tx, ty, tz, rx, ry, rz, length;
	int	i;
	for ( i = 0; i < PM_MOTION_NUM_JOINT_IN_HTR_FILE; i++)
	{
		is >> part >> tx >> ty >> tz >> rx >> ry >> rz >> length;

		HtrBaseTransf[i] = transf(
			EulerAngle2Matrix(vector(M_PI*rx/180, M_PI*ry/180, M_PI*rz/180)),
			vector(tx, ty, tz));
		
	}
	is.getline(str, 2048);

	this->setSize(n);
	this->setBody(this->getBody());
	
	transf t;
	vector v, rv;
	int	number;
	for (i = 0; i < PM_MOTION_NUM_JOINT_IN_HTR_FILE; i++)
	{
		is.getline(str, 2048);
		is.getline(str, 2048);
		is.getline(str, 2048);
		
		for (int j = 0; j < getSize(); j++)
		{
			is >> number >> tx >> ty >> tz >> rx >> ry >> rz >> length;
				
			v  = vector(tx, ty, tz);
			v += HtrBaseTransf[i].getTranslation();
			v *= scale;
			rv = vector(M_PI*rx/180, M_PI*ry/180, M_PI*rz/180);

			HtrTransf[j][i] = transf(EulerAngle2Matrix(rv)*HtrBaseTransf[i].getAffine(), v);

			if ( i!=0 )	HtrTransf[j][i] *=  HtrTransf[j][HtrParent[i]];
			
			if ( HtrJointSequence[i] != PmHuman::UNDEFINED )
			{
				t = getPosture(j).getBaseTransf( HtrJointSequence[i] ).inverse();
				getPosture(j).setTransf(HtrJointSequence[i], HtrTransf[j][i] * t );
			}
		}
	}

	is.close();

	return	n;
}


void
WriteSkeletonFromHTR(char* htr_filename, char* skeleton_filename, m_real scale)
{
	std::ifstream is(htr_filename, std::ios::in);
	if (is.fail()) return;

	int		n;
	char	str[2048], keyword[256];
	while (1)
	{
		is.getline(str, 2048);	sscanf(str, "%s", keyword);

		if (strcmp(keyword, "NumFrames") == 0)
			sscanf(str, "%s %d", keyword, &n);
		
		if (strcmp(keyword, "[BasePosition]") == 0)	break;
	}
	
	is.getline(str, 2048);

	char	part[256];
	m_real	tx, ty, tz, rx, ry, rz;
	int	i;
	for ( i = 0; i < PM_MOTION_NUM_JOINT_IN_HTR_FILE; i++)
	{
		is >> part >> tx >> ty >> tz >> rx >> ry >> rz >> HtrLength[i];

		vector	v(tx, ty, tz);	v *= scale;   
		vector	rv(M_PI*rx/180, M_PI*ry/180, M_PI*rz/180);

		HtrLength[i] *= scale;

		HtrBaseTransf[i] = transf(EulerAngle2Matrix(rv), v);

		if (i != 0)	HtrBaseTransf[i] *= HtrBaseTransf[HtrParent[i]];
	}
	
	is.close();

	std::ofstream	os(skeleton_filename);
	if (os.fail())	return;

	os << "root_height\t" << HtrBaseTransf[0].getTranslation().y() << std::endl;
	m_real r_height = HtrBaseTransf[HtrJointSequence[PmHuman::RIGHT_FOOT]].getTranslation().y();
	m_real l_height = HtrBaseTransf[HtrJointSequence[PmHuman::LEFT_FOOT]].getTranslation().y();
	os << "ankle_height\t" << (l_height + r_height)/2.0 << std::endl;

	for (i = 0; i < PM_MOTION_NUM_JOINT_IN_HTR_FILE; i++)
	{
		if (HtrJointSequence[i] == PmHuman::UNDEFINED)	continue;

		os << PmHuman::getLinkName(HtrJointSequence[i]) << std::endl;
		os << "begin" << std::endl;

		int	parent = GetPmHumanParentFromHTR(i);
		if (HtrParent[i] != i)
			os << "\tparent\t" << PmHuman::getLinkName(HtrJointSequence[parent]) << std::endl;
		
		os << "\tcoordinate\t" 
		   << HtrBaseTransf[i].translation() << " "
		   << Matrix2Quater(HtrBaseTransf[i].getAffine()) << std::endl;
		os << "end" << std::endl;
		os << std::endl;
	}

	os.close();
}



//----------------------------------------------------------------------//
//							File I/O for "*.txt"						//
//----------------------------------------------------------------------//

#define PM_MOTION_NUM_JOINT_IN_TXT_FILE		16

static int joint_sequence_txt[] =
{
	PmHuman::PELVIS, PmHuman::CHEST, PmHuman::NECK,
	PmHuman::HEAD, PmHuman::UPPER_RIGHT_ARM, PmHuman::LOWER_RIGHT_ARM,
	PmHuman::RIGHT_PALM, PmHuman::UPPER_LEFT_ARM, PmHuman::LOWER_LEFT_ARM,
	PmHuman::LEFT_PALM, PmHuman::UPPER_RIGHT_LEG, PmHuman::LOWER_RIGHT_LEG,
	PmHuman::RIGHT_FOOT, PmHuman::UPPER_LEFT_LEG,
	PmHuman::LOWER_LEFT_LEG, PmHuman::LEFT_FOOT,
};
int
PmLinearMotion::openTXT(char* name, m_real scale )
{
	assert( this->getBody() );

	std::ifstream file( name, std::ios::in );
	if ( file.fail() ) return -1;

	int n=0;
	static char str[2048];

	while (!file.eof())  // count the number of lines
	{
		file.getline(str, 2048);
		if (file.eof()) break;
		n++;
	}
	file.close();
	file.clear();

	n /= PM_MOTION_NUM_JOINT_IN_TXT_FILE;
	this->setSize(n);

	m_real tx, ty, tz, rx, ry, rz;
	m_real	model_factor = 1.0;

	PmPosture hv, lhv;
	hv.setBody( this->getBody() );

	file.open(name);
	for( int i=0; i<n; i++ )
	{
		for( int j=0; j<PM_MOTION_NUM_JOINT_IN_TXT_FILE; j++ )
		{
			int	joint = joint_sequence_txt[j];

			file >> tx >> ty >> tz >> rx >> ry >> rz;

			tx = model_factor * scale * tx;
			ty = model_factor * scale * ty;
			tz = model_factor * scale * tz;
			
			matrix m = EulerAngle2Matrix( vector(M_PI*rx/180,
												 M_PI*ry/180,
												 M_PI*rz/180) );

			if (joint != PmHuman::PELVIS)
			{
				hv.setTransf( joint, transf(m, vector(0, 0, 0)) );
			}
			else	{
				hv.setTransf( joint, transf(m, vector(tx, ty, tz)) );
			}
		}
	
		hv.setScale(1.0);
		this->setPosture( i, hv);
	}
	this->alignment();

	file.close();

	return n;
}




//----------------------------------------------------------------------//
//							File I/O for "*.cca"						//
//----------------------------------------------------------------------//

//#define PM_MOTION_NUM_JOINT_IN_CCA_FILE		14
#define PM_MOTION_NUM_JOINT_IN_CCA_FILE		16

static int joint_sequence_cca[] =
{
	/*
	PmHuman::RIGHT_FOOT, PmHuman::LEFT_FOOT, PmHuman::LOWER_RIGHT_LEG,
	PmHuman::LOWER_LEFT_LEG, PmHuman::UPPER_RIGHT_LEG, PmHuman::UPPER_LEFT_LEG,
	PmHuman::PELVIS, PmHuman::CHEST, PmHuman::LOWER_RIGHT_ARM,
	PmHuman::LOWER_LEFT_ARM, PmHuman::UPPER_RIGHT_ARM, PmHuman::UPPER_LEFT_ARM,
	PmHuman::NECK, PmHuman::HEAD,
	*/

	PmHuman::PELVIS, PmHuman::CHEST, PmHuman::NECK,
	PmHuman::HEAD, PmHuman::UPPER_RIGHT_ARM, PmHuman::LOWER_RIGHT_ARM,
	PmHuman::RIGHT_PALM, PmHuman::UPPER_LEFT_ARM, PmHuman::LOWER_LEFT_ARM,
	PmHuman::LEFT_PALM, PmHuman::UPPER_RIGHT_LEG, PmHuman::LOWER_RIGHT_LEG,
	PmHuman::RIGHT_FOOT, PmHuman::UPPER_LEFT_LEG,
	PmHuman::LOWER_LEFT_LEG, PmHuman::LEFT_FOOT,
};


void
GLOBAL2LOCAL(PmPosture& hv, PmPosture& lhv)
{
	lhv.setScale(hv.getScale());

	for (int i = 0; i < PM_HUMAN_NUM_LINKS; i++)
	{
		if (hv.getMask() & MaskBit(i))
		{
			transf	t = hv.getTransf(i);

			if (hv.getBody()->getParent(i) != -1)
			{
				t *= hv.getTransf(hv.getBody()->getParent(i)).inverse();
			}

			lhv.setTransf(i, t);
		}
	}
}

int
PmLinearMotion::saveCCA( char* name )
{
	std::ofstream os( name, std::ios::out );
	if ( os.fail() ) return -1;

	os.precision( 20 );

	transf t;
	quater q;
	vector v;
	
	for( int i=0; i<getSize(); i++ )
	{
		PmPosture& hv = this->getPosture(i);

		for (int j = 0; j < PM_MOTION_NUM_JOINT_IN_CCA_FILE; j++)
		{
			int	part = joint_sequence_cca[j];

			if (part == PmHuman::UNDEFINED)	continue;
			else	{
				if (part == PmHuman::PELVIS)
						v = hv.getTranslation();
				else	v = vector(0, 0, 0);

				os << v.x() << " " << v.y() << " " << v.z() << " ";

				q = hv.getGlobalRotation(part);
				os << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
			}
		}
		os << std::endl;
	}
	os.close();

	return getSize();
}

char	ACTOR_CALIBRATION = false;

int
PmLinearMotion::openCCA( char* name, m_real scale )
{
	assert( this->getBody() );

	std::ifstream file( name, std::ios::in );
	if ( file.fail() ) return -1;

	int n=0;
	static char str[2048];

	while (!file.eof())  // count the number of lines
	{
		file.getline(str, 2048);
		if (file.eof()) break;
		n++;
	}

	file.close();
	file.clear();

	m_real tx, ty, tz, rw, rx, ry, rz;

	//
	// for actor calibration

	m_real	model_factor = 1.0;
	if (ACTOR_CALIBRATION)
	{
		file.open(name);
		vector	pos[PM_HUMAN_NUM_LINKS];
		for (int j = 0; j < PM_MOTION_NUM_JOINT_IN_CCA_FILE; j++)
		{
			int	joint = joint_sequence_cca[j];
			if (joint == PmHuman::UNDEFINED)	continue;

			file >> tx >> ty >> tz >> rw >> rx >> ry >> rz;
			pos[joint] = vector(tx, ty, tz);
		}
		m_real	actual_height = pos[PmHuman::PELVIS].y();
		actual_height -= pos[PmHuman::LEFT_FOOT].y() * 0.5;
		actual_height -= pos[PmHuman::RIGHT_FOOT].y() * 0.5;
		actual_height = fabs(actual_height);

		model_factor = this->getBody()->getRootHeight() / actual_height;

		file.close();
		file.clear();
	}
	else	{
		model_factor = 1.0;
	}

	// actor calibration
	//
	
	this->setSize(n);

	PmPosture hv, lhv;
	hv.setBody( this->getBody() );
	lhv.setBody( this->getBody() );

	file.open(name);
	for( int i=0; i<n; i++ )
	{
		for( int j=0; j<PM_MOTION_NUM_JOINT_IN_CCA_FILE; j++ )
		{
			int	joint = joint_sequence_cca[j];

			file >> tx >> ty >> tz >> rw >> rx >> ry >> rz;

			tx = model_factor * scale * tx;
			ty = model_factor * scale * ty;
			tz = model_factor * scale * tz;
			
			matrix m = Quater2Matrix(quater(rw, rx, ry, rz));

			if (joint != PmHuman::PELVIS)
			{
				hv.setTransf( joint, transf(m, vector(0, 0, 0)) );
			}
			else	{
				hv.setTransf( joint, transf(m, vector(tx, ty, tz)) );
			}
		}
	
		hv.setScale(1.0);

		GLOBAL2LOCAL(hv, lhv);
		lhv.setTransf( PmHuman::RIGHT_PALM, identity_transf );
		lhv.setTransf( PmHuman::LEFT_PALM, identity_transf );
		this->setPosture( i, lhv);

		this->setPosture(i, hv);
	}

	this->alignment();

	file.close();

	return n;
}




//----------------------------------------------------------------------//
//							File I/O for "*.con"						//
//----------------------------------------------------------------------//

int
PmLinearMotion::openCON( char* name )
{
	std::ifstream file( name, std::ios::in );
	if ( file.fail() ) return -1;

	char buf[PM_MAX_FRAMES+1];

	int i;
	for( i=0; i<getSize(); i++ )
		constraints[i].reset();

	while (!file.eof())
	{
		file.getline(buf, PM_MAX_FRAMES);
		if (file.eof()) break;

		int j = getBody()->getLinkNumber( buf );

		file.getline(buf, PM_MAX_FRAMES);

		for( i=0; i<getSize(); i++ )
			if ( buf[i] == '1' )
				constraints[i].push( j, postures[i].getGlobalTransf(j) );
	}

	file.close();
	return getSize();
}

int
PmLinearMotion::saveCON( char* name )
{
	std::ofstream file( name, std::ios::out );
	if ( file.fail() ) return -1;

	char buf[PM_MAX_FRAMES+1];

	int i, j;
	for( j=0; j<PM_HUMAN_NUM_LINKS; j++ )
	{
		int flag = FALSE;

		for( i=0; i<getSize(); i++ )
			if ( constraints[i].isConstrained(j) )
			{
				buf[i] = '1';
				flag = TRUE;
			}
			else
				buf[i] = '0';

		buf[i] = 0;

		if ( flag )
		{
			file << getBody()->getLinkName(j) << std::endl;
			file << buf << std::endl;
		}
	}

	file.close();
	return getSize();
}



/*
//----------------------------------------------------------------------//
//						File I/O for "*.asf/*.amc"						//
//----------------------------------------------------------------------//


#ifndef _NEW_ACCLAIM_FORMAT__
//
//  For acclaim files which are generated
//  from Acclaim.mp, Acclaim.mod, and Acclaim.ast
//
#define	PM_MOTION_NUM_JOINT_IN_ASF_FILE	25
#define ASF_SCALE 1.0

enum AsfPart	{
	root,
	lhipjoint, lfemur, ltibia, lfoot, ltoes,
	rhipjoint, rfemur, rtibia, rfoot, rtoes,
	lowerback, upperback, thorax, lowerneck, upperneck, head,
	lshoulderjoint, lhumerus, lradius, lhand,
	rshoulderjoint, rhumerus, rradius, rhand
};

static char *AsfPartName[PM_MOTION_NUM_JOINT_IN_ASF_FILE] =
{
	"root",
	"lhipjoint", "lfemur", "ltibia", "lfoot", "ltoes",
	"rhipjoint", "rfemur", "rtibia", "rfoot", "rtoes",
	"lowerback", "upperback", "thorax", "lowerneck", "upperneck", "head",
	"lshoulderjoint", "lhumerus", "lradius", "lhand",
	"rshoulderjoint", "rhumerus", "rradius", "rhand"
};

static int	AsfParent[] =
{
	root,
	root, lhipjoint, lfemur, ltibia, lfoot,
	root, rhipjoint, rfemur, rtibia, rfoot,
	root, lowerback, upperback, thorax, lowerneck, upperneck,
	thorax, lshoulderjoint, lhumerus, lradius,
	thorax, rshoulderjoint, rhumerus, rradius
};

static int	AsfJointSequence[] =
{
	PmHuman::PELVIS,
	PmHuman::UNDEFINED, PmHuman::UPPER_LEFT_LEG,  PmHuman::LOWER_LEFT_LEG,  PmHuman::LEFT_FOOT,  PmHuman::LEFT_TOE,  
	PmHuman::UNDEFINED, PmHuman::UPPER_RIGHT_LEG, PmHuman::LOWER_RIGHT_LEG, PmHuman::RIGHT_FOOT, PmHuman::RIGHT_TOE, 
	PmHuman::SPINE_1, PmHuman::SPINE_2, PmHuman::SPINE_3, PmHuman::CHEST, PmHuman::NECK, PmHuman::HEAD,
	PmHuman::LEFT_SHOULDER,  PmHuman::UPPER_LEFT_ARM,  PmHuman::LOWER_LEFT_ARM,  PmHuman::LEFT_PALM, 
	PmHuman::RIGHT_SHOULDER, PmHuman::UPPER_RIGHT_ARM, PmHuman::LOWER_RIGHT_ARM, PmHuman::RIGHT_PALM, 
};

vector	AsfBaseTranslation[PM_MOTION_NUM_JOINT_IN_ASF_FILE];
quater	AsfBaseRotation[PM_MOTION_NUM_JOINT_IN_ASF_FILE];

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
ConvertASF2ACTOR(char* asf_filename, char* skeleton_filename, m_real scale)
{
	std::ifstream is(asf_filename, std::ios::in);
	if (is.fail()) return;

	int		n;
	char	str[2048], keyword[256];
	while (1)
	{
		is.getline(str, 2048);	sscanf(str, "%s", keyword);

		if (strcmp(keyword, ":bonedata") == 0)	break;
	}
	
	is.getline(str, 2048);

	AsfBaseTranslation[0] = vector(0,0,0);
	AsfBaseRotation[0] = quater(1,0,0,0);

	char	part[256];
	float	tx, ty, tz, rx, ry, rz, s;
	for (int i = 1; i < PM_MOTION_NUM_JOINT_IN_ASF_FILE; i++)
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
PmLinearMotion::saveAMC( char* name )
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
			else
			{
				q = h.getRotation( AsfJointSequence[j] );
				u = Quater2EulerAngle(q);

				u = 180 * u / M_PI;

				file << u[0] << " " << u[1] << " " << u[2] << std::endl;
			}
		}
	}

	return this->getSize();
}

int
PmLinearMotion::openAMC( char* name, m_real scale, int subsample )
{
	assert( this->getBody() );

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


//	while (!file.eof())  // count the number of lines
//	{
//		file.getline(str, 2048);
//		if (file.eof()) break;
//		n++;
//	}
//	file.close();
//  file.clear();
//	n = (n-3)/(PM_MOTION_NUM_JOINT_IN_ASF_FILE-1);


	this->setSize( (n+subsample-1) / subsample );

	file.open( name );

	while (1) // skip the header
	{
		file >> str;
		if (strcmp(str, ":DEGREES") == 0)	break;
	}

	PmPosture hv;
	hv.setBody( this->getBody() );

	int		index;
	m_real	x, y, z, r, u, v;

	for( int i=0; i<n; i++ )
	{
		file >> index;

		for( int j=0; j<(PM_MOTION_NUM_JOINT_IN_ASF_FILE-2); j++ )
		{
			file >> str;

			if ( strcmp( str, "root" )==0 )
			{
				file >> x >> y >> z >> r >> u >> v;
				x = x * scale * ASF_SCALE;
				y = y * scale * ASF_SCALE;
				z = z * scale * ASF_SCALE;
				r = M_PI*r/180;
				u = M_PI*u/180;
				v = M_PI*v/180;

				quater q = exp(v*x_axis/2) * exp(u*y_axis/2) * exp(r*z_axis/2);
				hv.setTransf( PmHuman::PELVIS, transf(q,vector(x,y,z)) );
			}
			else
			{
				file >> r >> u >> v;
				r = M_PI*r/180;
				u = M_PI*u/180;
				v = M_PI*v/180;

				for( int k=0; k<PM_MOTION_NUM_JOINT_IN_ASF_FILE; k++ )
					if ( strcmp( str, AsfPartName[k] )==0 ) break;

				if ( k < PM_MOTION_NUM_JOINT_IN_ASF_FILE &&
					 AsfJointSequence[k] != PmHuman::UNDEFINED )
				{
					quater q = exp(v*z_axis/2) * exp(u*y_axis/2) * exp(r*x_axis/2);
					hv.setRotation( AsfJointSequence[k], q );
				}
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
#endif // old Acclaim format
*/

#ifdef _NEW_ACCLAIM_FORMAT__
//
//  For acclaim files which are generated
//  from Acclaim_F.mp, Acclaim_F.mod, and Acclaim_F.ast
//
#define	PM_MOTION_NUM_JOINT_IN_ASF_FILE	31
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

vector	AsfBaseTranslation[PM_MOTION_NUM_JOINT_IN_ASF_FILE];
quater	AsfBaseRotation[PM_MOTION_NUM_JOINT_IN_ASF_FILE];

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
ConvertASF2ACTOR(char* asf_filename, char* skeleton_filename, m_real scale)
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
PmLinearMotion::saveAMC( char* name )
{
//	assert( this->getBody() );

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
PmLinearMotion::openAMC( char* name, m_real scale, int subsample )
{
//	assert( this->getBody() );

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

	/*
	while (!file.eof())  // count the number of lines
	{
		file.getline(str, 2048);
		if (file.eof()) break;
		n++;
	}
	file.close();
	file.clear();
	n = (n-3)/(PM_MOTION_NUM_JOINT_IN_ASF_FILE-1);
	*/

	this->setSize( (n+subsample-1) / subsample );

	file.open( name );

	while (1) // skip the header
	{
		file >> str;
		if (strcmp(str, ":DEGREES") == 0)	break;
	}

	PmPosture hv;
//	hv.setBody( this->getBody() );

	int		index;
	quater	q;
	m_real	x, y, z, r, u, v;
	int	i, j, k;

	for( i=0; i<n; i++ )
	{
		file >> index;

		for( j=0; j<(PM_MOTION_NUM_JOINT_IN_ASF_FILE-2); j++ )
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
#endif // new Acclaim format

