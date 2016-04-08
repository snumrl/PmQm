
#include <fstream>
#include <string.h>
#include <stdlib.h>

#include "pm.h"

using namespace jhm;

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

