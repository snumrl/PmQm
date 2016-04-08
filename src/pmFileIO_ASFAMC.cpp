
#include <fstream>
#include <string.h>
#include <stdlib.h>

#include "pm.h"

using namespace jhm;


//----------------------------------------------------------------------//
//						File I/O for "*.asf/*.amc"						//
//----------------------------------------------------------------------//

extern void ConvertASF2ACTOR_BB( char*, char*, m_real );
extern int  saveAMC_BB( char* );
extern int  openAMC_BB( char*, m_real, int );

extern void ConvertASF2ACTOR_BB_F( char*, char*, m_real );
extern int  saveAMC_BB_F( char* );
extern int  openAMC_BB_F( char*, m_real, int );

extern void ConvertASF2ACTOR_Woody( char*, char*, m_real );
extern int  saveAMC_Woody( char* );
extern int  openAMC_Woody( char*, m_real, int );

extern void ConvertASF2ACTOR_Racket( char*, char*, m_real );
extern int  saveAMC_Racket( char* );
extern int  openAMC_Racket( char*, m_real, int );

extern void ConvertASF2ACTOR_BasketBall( char*, char*, m_real );
extern int  saveAMC_BasketBall( char* );
extern int  openAMC_BasketBall( char*, m_real, int );

void
ConvertASF2ACTOR(char* asf_filename, int type, char* skeleton_filename, m_real scale)
{
	switch( type )
	{
	case PmLinearMotion::PM_AMC_BODY_BUILDER:
		ConvertASF2ACTOR_BB( asf_filename, skeleton_filename, scale );
		break;
	case PmLinearMotion::PM_AMC_BODY_BUILDER_F:
		ConvertASF2ACTOR_BB_F( asf_filename, skeleton_filename, scale );
		break;
	case PmLinearMotion::PM_AMC_WOODY:
		ConvertASF2ACTOR_Woody( asf_filename, skeleton_filename, scale );
		break;
	case PmLinearMotion::PM_AMC_RACKET:
		ConvertASF2ACTOR_Racket( asf_filename, skeleton_filename, scale );
		break;
	case PmLinearMotion::PM_AMC_BASKETBALL:
		ConvertASF2ACTOR_BasketBall( asf_filename, skeleton_filename, scale );
		break;
	}
}

int
PmLinearMotion::saveAMC( char* name, int type )
{
	switch( type )
	{
	case PmLinearMotion::PM_AMC_BODY_BUILDER:
		return saveAMC_BB( name );
	case PmLinearMotion::PM_AMC_BODY_BUILDER_F:
		return saveAMC_BB_F( name );
	case PmLinearMotion::PM_AMC_WOODY:
		return saveAMC_Woody( name );
	case PmLinearMotion::PM_AMC_RACKET:
		return saveAMC_Racket( name );
	case PmLinearMotion::PM_AMC_BASKETBALL:
		return saveAMC_BasketBall( name );
	}

	return -1;
}

int
PmLinearMotion::openAMC( char* name, int type, m_real scale, int subsample )
{
	switch( type )
	{
	case PmLinearMotion::PM_AMC_BODY_BUILDER:
		return openAMC_BB( name, scale, subsample );
	case PmLinearMotion::PM_AMC_BODY_BUILDER_F:
		return openAMC_BB_F( name, scale, subsample );
	case PmLinearMotion::PM_AMC_WOODY:
		return openAMC_Woody( name, scale, subsample );
	case PmLinearMotion::PM_AMC_RACKET:
		return openAMC_Racket( name, scale, subsample );
	case PmLinearMotion::PM_AMC_BASKETBALL:
		return openAMC_BasketBall( name, scale, subsample );
	}

	return -1;
}
