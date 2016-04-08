
#include "pm.h"

int				PenvManipulateLevel		= 3;
int				PenvControlRange		= 36;
bool			PenvFineAdjustment		= FALSE;
bool			PenvIKControl			= TRUE;
bool			PenvLocalControl		= TRUE;
bool			PenvInsertConstraints	= FALSE;
bool			PenvDampingEnabled		= TRUE;
bool			PenvJointLimitEnabled	= FALSE;
bool			PenvStaticBalanceEnabled= TRUE;
int				PenvPresmoothing		= 1;

int				PenvMRAsubdivisionType	= PENV_INTERPOLATORY_SUBDIVISION;
int				PenvMRAdownSamplingType	= PENV_UNIFORM_DOWN_SAMPLING;

int				PenvIKsolverType		= PENV_IK_NUMERICAL;
int				PenvIKsolverMethod		= PENV_IK_LS_DAMP_METHOD;

m_real			PenvDampingCoeff   [PM_HUMAN_NUM_LINKS + 1];
m_real			PenvJointLimitCoeff[PM_HUMAN_NUM_LINKS + 1];
m_real			PenvRigidityCoeff  [PM_HUMAN_NUM_LINKS + 1];

m_real			PenvIKerrorBound = 0.001;
m_real			PenvIKtolerance = 0.1;

bool			PenvMotionManipulationEnabled = TRUE;

void
PenvInitialize()
{
	for( int i=0; i<PM_HUMAN_NUM_LINKS+1; i++ )
	{
		PenvDampingCoeff[i]    = 0.1;
		PenvJointLimitCoeff[i] = 10;
		PenvRigidityCoeff[i]   = 1.0;
	}
}

