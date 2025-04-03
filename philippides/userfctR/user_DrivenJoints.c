/* ---------------------------
 * Robotran - MBsysC
 * 
 * Template file for the driven joint function
 * 
 * (c) Universite catholique de Louvain
 *     
 */

#include "math.h"

#include "mbs_data.h"


void user_DrivenJoints(MbsData *mbs_data,double tsim)
{
    int fixed_translations_ids[] = {
        mbs_data->joint_id[T_lh_fixed],
        mbs_data->joint_id[T_rh_fixed],
        mbs_data->joint_id[T_lk_fixed],
        mbs_data->joint_id[T_rk_fixed], 
        mbs_data->joint_id[R_hip]
    };

    int nb_ids = sizeof(fixed_translations_ids) / sizeof(fixed_translations_ids[0]);
    for (int i = 0; i < nb_ids; i++)
    {
        int elem = fixed_translations_ids[i];
        mbs_data->q[elem] = 0.0;
        mbs_data->qd[elem] = 0.0;
        mbs_data->qdd[elem] = 0.0;
    }
}

 
