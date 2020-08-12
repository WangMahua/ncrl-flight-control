#include "sys_param.h"
#include "common_list.h"
#include "multirotor_geometry_param.h"

sys_param_data multirotor_geometry_param_list[MR_GEO_PARAM_LIST_SIZE];

void init_multirotor_geometry_param_list(void)
{
	init_sys_param_list(multirotor_geometry_param_list, MR_GEO_PARAM_LIST_SIZE);

	init_common_params();

	INIT_SYS_PARAM_FLOAT(MR_GEO_GAIN_ROLL_P, 300.0f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_GAIN_ROLL_D, 40.25f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_GAIN_PITCH_P, 300.0f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_GAIN_PITCH_D, 40.25f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_GAIN_YAW_P, 2900.0f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_GAIN_YAW_D, 200.0);
	INIT_SYS_PARAM_FLOAT(MR_GEO_GAIN_RATE_YAW, 2750.0f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_GAIN_POS_X, 3.6f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_GAIN_VEL_X, 2.2f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_GAIN_POS_Y, 3.6f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_GAIN_VEL_Y, 2.2f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_GAIN_POS_Z, 8.5f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_GAIN_VEL_Z, 4.0f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_GAIN_POS_X_I, 0.0f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_GAIN_POS_Y_I, 0.0f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_GAIN_POS_Z_I, 0.0f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_UAV_MASS, 1150.0f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_INERTIA_JXX, 0.01466f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_INERTIA_JYY, 0.01466f);
	INIT_SYS_PARAM_FLOAT(MR_GEO_INERTIA_JZZ, 0.02848f);

	load_param_list_from_flash();
}
