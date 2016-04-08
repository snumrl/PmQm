
#include <fstream>
#include <string.h>
#include <stdlib.h>

#include "pm.h"

using namespace jhm;

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

