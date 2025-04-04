   /**
    *
    *   Universite catholique de Louvain
    *   Mechatronic, Electrical Energy, and Dynamic systems (iMMC/MEED) 
    *   https://www.robotran.be
    *   Contact : info@robotran.be
    *
    *
    *   MBsysC main script template for complete model:
    *   -----------------------------------------------
    *    This template loads the data file *.mbs and executes:
    *      - the coordinate partitioning module
    *      - the equilibrium module
    *      - the modal module
    *      - the direct dynamic module (time integration of
    *        equations of motion).
    *      - the inverse kinematics module
    *      - the inverse dynamics module
    *    It may be adapted and completed by the user.
    *
    *    (c) Universite catholique de Louvain
    */

#include <stdio.h>

#include "mbs_data.h"
#include "mbs_part.h"
#include "mbs_equil.h"
#include "mbs_modal.h"
#include "mbs_dirdyn.h"
#include "integrator.h"
#include "mbs_invdyn.h"
#include "mbs_solvekin.h"

#include "mbs_message.h"
#include "mbs_loader.h"
#include "mbs_set.h"

int main_original(int argc, char const *argv[])
{
    mbs_msg("Starting philippides_c MBS project!\n");

    
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                     LOADING                               *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    MbsData *mbs_data;

    mbs_msg("Loading the philippides_c data file !\n");
    mbs_data = mbs_load("philippides_c.mbs");
    mbs_msg("*.mbs file loaded!\n");
    
    
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *              COORDINATE PARTITIONING                      *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    MbsPart *mbs_part;
    mbs_data->process = 1;

    mbs_part = mbs_new_part(mbs_data);

    mbs_part->options->rowperm=1;
    mbs_part->options->verbose = 1;

    mbs_run_part(mbs_part, mbs_data);

    mbs_delete_part(mbs_part);
    
    
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                    EQUILIBRIUM                            *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    MbsEquil *mbs_equil;
    mbs_data->process = 2;

    mbs_equil = mbs_new_equil(mbs_data);

    // Equil options (see documentations for additional options)
    mbs_equil->options->method  = 1;
    mbs_equil->options->senstol = 1e-2;
    mbs_equil->options->verbose = 1;

    mbs_run_equil(mbs_equil, mbs_data);

    mbs_delete_equil(mbs_equil, mbs_data);
    
    
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                   MODAL ANALYSIS                          *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    MbsModal *mbs_modal;
    mbs_data->process = 4;

    mbs_modal = mbs_new_modal(mbs_data);

    // modal options (see documentations for additional options)
    mbs_modal->options->save_result = 1;
    mbs_modal->options->save_anim = 1;
    mbs_modal->options->mode_ampl=0.2;
    mbs_modal->options->verbose = 1;

    mbs_run_modal(mbs_modal, mbs_data);
    mbs_delete_modal(mbs_modal, mbs_data);
    
    
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                   DIRECT DYNAMICS                         *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    MbsDirdyn *mbs_dirdyn;
    mbs_data->process = 3;

    mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    // dirdyn options (see documentations for additional options)
    mbs_dirdyn->options->dt0 = 1e-3;
    mbs_dirdyn->options->tf  = 10.0;
    mbs_dirdyn->options->save2file = 1;
    //mbs_dirdyn->options->realtime = 1;

    mbs_run_dirdyn(mbs_dirdyn, mbs_data);

    mbs_delete_dirdyn(mbs_dirdyn, mbs_data);
    
    
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                   INVERSE KINEMATICS                      *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    MbsSolvekin *mbs_solvekin;
    mbs_data->process = 5;

    mbs_solvekin = mbs_new_solvekin(mbs_data);

    // solvekin options, to interpolate part of the previous motion
    mbs_solvekin->options->t0 = 1.33333;
    mbs_solvekin->options->tf = 1.4;
    mbs_solvekin->options->dt = 1e-4;
    mbs_solvekin->options->framerate = 10000;
    mbs_solvekin->options->motion = trajectory;
    mbs_solvekin->options->trajectoryqname = "../../resultsR/dirdyn_q.res";
    mbs_solvekin->options->trajectoryqdname = "../../resultsR/dirdyn_qd.res";
    mbs_solvekin->options->trajectoryqddname = "../../resultsR/dirdyn_qdd.res";

    mbs_run_solvekin(mbs_solvekin, mbs_data);

    mbs_delete_solvekin(mbs_solvekin, mbs_data);
    
    
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                     INVERSE DYNAMICS                      *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    MbsInvdyn *mbs_invdyn;
    mbs_data->process = 6;

    mbs_invdyn = mbs_new_invdyn(mbs_data);

    // mbs_invdyn options (see documentations for additional options)
    mbs_invdyn->options->motion = oneshot;

    mbs_run_invdyn(mbs_invdyn, mbs_data);

    mbs_delete_invdyn(mbs_invdyn, mbs_data);
    
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                   CLOSING OPERATIONS                      *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    mbs_delete_data(mbs_data);

    return 0;
}

