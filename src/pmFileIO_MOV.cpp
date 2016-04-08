
#include <fstream>
#include <string.h>
#include <stdlib.h>

#include "pm.h"

using namespace jhm;

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
	file.clear();

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
