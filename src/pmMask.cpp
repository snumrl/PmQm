
#include "pm.h"

using namespace jhm;

static int DOF_for_each_joint[PM_HUMAN_NUM_LINKS] =
{
	6,
	3, 3, 3, 3, 3,		// spine*4, chest
	3, 3,				// neck, head 
	3, 3,				// shoulders
	3, 3,				// clavicles
    3, 3, 1, 1,			// arms
	3, 3, 1, 1,			// legs
	3, 3, 1, 1,			// feet, toes
	3,					// right_palm
	3, 1, 1,			// fingers
	1, 1, 1,
	1, 1, 1,
	1, 1, 1,
	1, 1, 1,
    3,					// left_palm
	3, 1, 1,			// fingers
	1, 1, 1, 
	1, 1, 1,
	1, 1, 1,
	1, 1, 1,
	0, 0,				// left and right heel
};

int
GetDOF( int joint )
{
	return DOF_for_each_joint[joint];
}

int
GetDOF( PmMaskType mask )
{
	int dof = 0;

	for( int i=0; i<PM_HUMAN_NUM_LINKS; i++ )
		if ( mask & MaskBit(i) ) dof += GetDOF( i );

	return dof;
}

PmMaskType
MaskBit( int index )
{
/*
    PmMaskType mask=0x01;

    for( int i=0; i<index; i++ )
        mask *= 2;

    return mask;
*/
    return (PmMaskType)pow( 2., index );
}
