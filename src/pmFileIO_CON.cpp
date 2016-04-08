
#include <fstream>
#include <string.h>
#include <stdlib.h>

#include "pm.h"

using namespace jhm;

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


