/* ---------------------------
 * Robotran - MBsysC
 * 
 * Template file for the external forces/torques function
 * 
 * (c) Universite catholique de Louvain
 *     
 */

 #include "math.h"

 #include "mbs_data.h"
 #include "mbs_project_interface.h"
 
 double* user_ExtForces(double PxF[4], double RxF[4][4], 
                        double VxF[4], double OMxF[4], 
                        double AxF[4], double OMPxF[4], 
                        MbsData *mbs_data, double tsim,int ixF)
 {
     double Fx=0.0, Fy=0.0, Fz=0.0;
     double Mx=0.0, My=0.0, Mz=0.0;
     double dxF[4] ={0.0, 0.0, 0.0, 0.0};
 
 
     double *SWr = mbs_data->SWr[ixF];
 
     // default application point of the force: anchor point to which it is attached
     int idpt = 0;
     idpt = mbs_data->xfidpt[ixF];
     dxF[1] = mbs_data->dpt[1][idpt];
     dxF[2] = mbs_data->dpt[2][idpt];
     dxF[3] = mbs_data->dpt[3][idpt];
 
 /* Begin of user declaration */
     
     
 
 /* End of user declaration */
 
 
     switch(ixF){
 
 /* Begin of user code */
         case 1: 
 
                     
         break;
 
     /*  case 2: 
 
             
         break;
     */
         
 /* End of user code */
 
     }
 
     SWr[1]=Fx;
     SWr[2]=Fy;
     SWr[3]=Fz;
     SWr[4]=Mx;
     SWr[5]=My;
     SWr[6]=Mz;
     SWr[7]=dxF[1];
     SWr[8]=dxF[2];
     SWr[9]=dxF[3];
 
     return SWr;
 }
 
  