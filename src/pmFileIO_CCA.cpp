
#include <fstream>
#include <string.h>
#include <stdlib.h>

#include "pm.h"

using namespace jhm;

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

