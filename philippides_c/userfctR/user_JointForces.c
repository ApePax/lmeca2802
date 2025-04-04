/* ---------------------------
 * Robotran - MBsysC
 * 
 * Template file for the user JointForces function
 * 
 * (c) Universite catholique de Louvain
 *     
 */

 #include "math.h" 

 #include "mbs_data.h"
 #include "user_model.h"
 #include "set_output.h"
 #include "useful_functions.h"
 
 double* user_JointForces(MbsData *mbs_data, double tsim)
 {
 /*-- Begin of user code --*/
 
 /* 
     // example for a spring-damper in joint number 2
     double K = 10;
     double C = 1;
     double L0 = 0.1;
     mbs_data->Qq[2] = -(K*(mbs_data->q[2]-L0) + C*mbs_data->qd[2]);
 */
     
 
 /*-- End of user code --*/
 
     return mbs_data->Qq;
 }