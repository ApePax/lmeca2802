/* ---------------------------
 * Robotran - MBsysC
 * 
 * Template file for direct dynamics module
 * 
 * This files enable the user to call custom at
 * specific places in the time simulation. It is a template
 * file that can be edited by the user.
 * 
 * (c) Universite catholique de Louvain
 *     
 */

#include "math.h"

#include "mbs_data.h"
#include "mbs_dirdyn_struct.h"


/*! \brief user own initialization functions
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advanced users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advanced users.
 */
void user_dirdyn_init(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{

}

/*! \brief user own loop functions
 *
 * This function is called a every time step.
 * Warning: if the used integrator is multi-steps, user_dirdyn_loop is only called once:
 * i.e. at the real time step (and not internal time steps)
 *
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advanced users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advanced users.
 */
void user_dirdyn_loop(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{

}

/*! \brief user own finishing functions
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advanced users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advanced users.
 */
void user_dirdyn_finish(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{

}
