//
//	MBsysTran - Release 8.1
//
//	Copyright 
//	Universite catholique de Louvain (UCLouvain) 
//	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
//	2, Place du Levant
//	1348 Louvain-la-Neuve 
//	Belgium 
//
//	http://www.robotran.be 
//
//	==> Generation Date: Tue Mar 25 18:27:47 2025
//	==> using automatic loading with extension .mbs 
//
//	==> Project name: philippides
//
//	==> Number of joints: 13
//
//	==> Function: F6 - Sensors Kinematics
//
//	==> Git hash: a9259031f25de7c9f86dbac08464ef31e84ae3b1
//
//	==> Input XML
//

#include <math.h> 

#include "mbs_data.h"
#include "mbs_sensor.h"

void mbs_sensor(MbsSensor *sens,
MbsData *s, int isens)
{
#include "mbs_sensor_philippides.h"

double *q, *qd, *qdd;
double **dpt;

q = s->q;
qd = s->qd;
qdd = s->qdd;

dpt = s->dpt;
 
// Trigonometric functions

S3 = sin(q[3]);
C3 = cos(q[3]);
S5 = sin(q[5]);
C5 = cos(q[5]);
S7 = sin(q[7]);
C7 = cos(q[7]);
S8 = sin(q[8]);
C8 = cos(q[8]);
S10 = sin(q[10]);
C10 = cos(q[10]);
S12 = sin(q[12]);
C12 = cos(q[12]);
S13 = sin(q[13]);
C13 = cos(q[13]);
 
// Augmented Joint Position Vectors

 
// Sensor Kinematics


switch(isens)
{
case 1:

ROcp1_15 = C3*C5-S3*S5;
ROcp1_35 = -C3*S5-S3*C5;
ROcp1_75 = C3*S5+S3*C5;
ROcp1_95 = C3*C5-S3*S5;
ROcp1_17 = ROcp1_15*C7-ROcp1_75*S7;
ROcp1_37 = ROcp1_35*C7-ROcp1_95*S7;
ROcp1_77 = ROcp1_15*S7+ROcp1_75*C7;
ROcp1_97 = ROcp1_35*S7+ROcp1_95*C7;
ROcp1_18 = ROcp1_17*C8-ROcp1_77*S8;
ROcp1_38 = ROcp1_37*C8-ROcp1_97*S8;
ROcp1_78 = ROcp1_17*S8+ROcp1_77*C8;
ROcp1_98 = ROcp1_37*S8+ROcp1_97*C8;
RLcp1_14 = q[4]*S3;
RLcp1_34 = q[4]*C3;
POcp1_14 = RLcp1_14+q[1];
POcp1_34 = RLcp1_34+q[2];
ORcp1_14 = RLcp1_34*qd[3];
ORcp1_34 = -RLcp1_14*qd[3];
VIcp1_14 = ORcp1_14+qd[1]+qd[4]*S3;
VIcp1_34 = ORcp1_34+qd[2]+qd[4]*C3;
ACcp1_14 = qdd[1]+ORcp1_34*qd[3]+RLcp1_34*qdd[3]+qdd[4]*S3+(2.0)*qd[3]*qd[4]*C3;
ACcp1_34 = qdd[2]-ORcp1_14*qd[3]-RLcp1_14*qdd[3]+qdd[4]*C3-(2.0)*qd[3]*qd[4]*S3;
OMcp1_25 = qd[3]+qd[5];
OPcp1_25 = qdd[3]+qdd[5];
RLcp1_16 = ROcp1_75*q[6];
RLcp1_36 = ROcp1_95*q[6];
POcp1_16 = POcp1_14+RLcp1_16;
POcp1_36 = POcp1_34+RLcp1_36;
ORcp1_16 = OMcp1_25*RLcp1_36;
ORcp1_36 = -OMcp1_25*RLcp1_16;
VIcp1_16 = ORcp1_16+VIcp1_14+ROcp1_75*qd[6];
VIcp1_36 = ORcp1_36+VIcp1_34+ROcp1_95*qd[6];
ACcp1_16 = ACcp1_14+OMcp1_25*ORcp1_36+(2.0)*OMcp1_25*ROcp1_95*qd[6]+OPcp1_25*RLcp1_36+ROcp1_75*qdd[6];
ACcp1_36 = ACcp1_34-OMcp1_25*ORcp1_16-(2.0)*OMcp1_25*ROcp1_75*qd[6]-OPcp1_25*RLcp1_16+ROcp1_95*qdd[6];
OMcp1_27 = OMcp1_25+qd[7];
OPcp1_27 = OPcp1_25+qdd[7];
OMcp1_28 = OMcp1_27+qd[8];
OPcp1_28 = OPcp1_27+qdd[8];
RLcp1_19 = ROcp1_18*dpt[1][7]+ROcp1_78*dpt[3][7];
RLcp1_39 = ROcp1_38*dpt[1][7]+ROcp1_98*dpt[3][7];
POcp1_19 = POcp1_16+RLcp1_19;
POcp1_39 = POcp1_36+RLcp1_39;
ORcp1_19 = OMcp1_28*RLcp1_39;
ORcp1_39 = -OMcp1_28*RLcp1_19;
VIcp1_19 = ORcp1_19+VIcp1_16;
VIcp1_39 = ORcp1_39+VIcp1_36;
ACcp1_19 = ACcp1_16+OMcp1_28*ORcp1_39+OPcp1_28*RLcp1_39;
ACcp1_39 = ACcp1_36-OMcp1_28*ORcp1_19-OPcp1_28*RLcp1_19;
sens->P[1] = POcp1_19;
sens->P[2] = dpt[2][7];
sens->P[3] = POcp1_39;
sens->R[1][1] = ROcp1_18;
sens->R[1][3] = ROcp1_38;
sens->R[2][2] = (1.0);
sens->R[3][1] = ROcp1_78;
sens->R[3][3] = ROcp1_98;
sens->V[1] = VIcp1_19;
sens->V[2] = 0;
sens->V[3] = VIcp1_39;
sens->OM[1] = 0;
sens->OM[2] = OMcp1_28;
sens->OM[3] = 0;
sens->A[1] = ACcp1_19;
sens->A[2] = 0;
sens->A[3] = ACcp1_39;
sens->OMP[1] = 0;
sens->OMP[2] = OPcp1_28;
sens->OMP[3] = 0;

break;

case 2:

ROcp2_15 = C3*C5-S3*S5;
ROcp2_35 = -C3*S5-S3*C5;
ROcp2_75 = C3*S5+S3*C5;
ROcp2_95 = C3*C5-S3*S5;
ROcp2_17 = ROcp2_15*C7-ROcp2_75*S7;
ROcp2_37 = ROcp2_35*C7-ROcp2_95*S7;
ROcp2_77 = ROcp2_15*S7+ROcp2_75*C7;
ROcp2_97 = ROcp2_35*S7+ROcp2_95*C7;
ROcp2_18 = ROcp2_17*C8-ROcp2_77*S8;
ROcp2_38 = ROcp2_37*C8-ROcp2_97*S8;
ROcp2_78 = ROcp2_17*S8+ROcp2_77*C8;
ROcp2_98 = ROcp2_37*S8+ROcp2_97*C8;
RLcp2_14 = q[4]*S3;
RLcp2_34 = q[4]*C3;
POcp2_14 = RLcp2_14+q[1];
POcp2_34 = RLcp2_34+q[2];
ORcp2_14 = RLcp2_34*qd[3];
ORcp2_34 = -RLcp2_14*qd[3];
VIcp2_14 = ORcp2_14+qd[1]+qd[4]*S3;
VIcp2_34 = ORcp2_34+qd[2]+qd[4]*C3;
ACcp2_14 = qdd[1]+ORcp2_34*qd[3]+RLcp2_34*qdd[3]+qdd[4]*S3+(2.0)*qd[3]*qd[4]*C3;
ACcp2_34 = qdd[2]-ORcp2_14*qd[3]-RLcp2_14*qdd[3]+qdd[4]*C3-(2.0)*qd[3]*qd[4]*S3;
OMcp2_25 = qd[3]+qd[5];
OPcp2_25 = qdd[3]+qdd[5];
RLcp2_16 = ROcp2_75*q[6];
RLcp2_36 = ROcp2_95*q[6];
POcp2_16 = POcp2_14+RLcp2_16;
POcp2_36 = POcp2_34+RLcp2_36;
ORcp2_16 = OMcp2_25*RLcp2_36;
ORcp2_36 = -OMcp2_25*RLcp2_16;
VIcp2_16 = ORcp2_16+VIcp2_14+ROcp2_75*qd[6];
VIcp2_36 = ORcp2_36+VIcp2_34+ROcp2_95*qd[6];
ACcp2_16 = ACcp2_14+OMcp2_25*ORcp2_36+(2.0)*OMcp2_25*ROcp2_95*qd[6]+OPcp2_25*RLcp2_36+ROcp2_75*qdd[6];
ACcp2_36 = ACcp2_34-OMcp2_25*ORcp2_16-(2.0)*OMcp2_25*ROcp2_75*qd[6]-OPcp2_25*RLcp2_16+ROcp2_95*qdd[6];
OMcp2_27 = OMcp2_25+qd[7];
OPcp2_27 = OPcp2_25+qdd[7];
OMcp2_28 = OMcp2_27+qd[8];
OPcp2_28 = OPcp2_27+qdd[8];
RLcp2_19 = ROcp2_18*dpt[1][8]+ROcp2_78*dpt[3][8];
RLcp2_39 = ROcp2_38*dpt[1][8]+ROcp2_98*dpt[3][8];
POcp2_19 = POcp2_16+RLcp2_19;
POcp2_39 = POcp2_36+RLcp2_39;
ORcp2_19 = OMcp2_28*RLcp2_39;
ORcp2_39 = -OMcp2_28*RLcp2_19;
VIcp2_19 = ORcp2_19+VIcp2_16;
VIcp2_39 = ORcp2_39+VIcp2_36;
ACcp2_19 = ACcp2_16+OMcp2_28*ORcp2_39+OPcp2_28*RLcp2_39;
ACcp2_39 = ACcp2_36-OMcp2_28*ORcp2_19-OPcp2_28*RLcp2_19;
sens->P[1] = POcp2_19;
sens->P[2] = dpt[2][8];
sens->P[3] = POcp2_39;
sens->R[1][1] = ROcp2_18;
sens->R[1][3] = ROcp2_38;
sens->R[2][2] = (1.0);
sens->R[3][1] = ROcp2_78;
sens->R[3][3] = ROcp2_98;
sens->V[1] = VIcp2_19;
sens->V[2] = 0;
sens->V[3] = VIcp2_39;
sens->OM[1] = 0;
sens->OM[2] = OMcp2_28;
sens->OM[3] = 0;
sens->A[1] = ACcp2_19;
sens->A[2] = 0;
sens->A[3] = ACcp2_39;
sens->OMP[1] = 0;
sens->OMP[2] = OPcp2_28;
sens->OMP[3] = 0;

break;

case 3:

ROcp3_15 = C3*C5-S3*S5;
ROcp3_35 = -C3*S5-S3*C5;
ROcp3_75 = C3*S5+S3*C5;
ROcp3_95 = C3*C5-S3*S5;
ROcp3_17 = ROcp3_15*C7-ROcp3_75*S7;
ROcp3_37 = ROcp3_35*C7-ROcp3_95*S7;
ROcp3_77 = ROcp3_15*S7+ROcp3_75*C7;
ROcp3_97 = ROcp3_35*S7+ROcp3_95*C7;
ROcp3_18 = ROcp3_17*C8-ROcp3_77*S8;
ROcp3_38 = ROcp3_37*C8-ROcp3_97*S8;
ROcp3_78 = ROcp3_17*S8+ROcp3_77*C8;
ROcp3_98 = ROcp3_37*S8+ROcp3_97*C8;
RLcp3_14 = q[4]*S3;
RLcp3_34 = q[4]*C3;
POcp3_14 = RLcp3_14+q[1];
POcp3_34 = RLcp3_34+q[2];
ORcp3_14 = RLcp3_34*qd[3];
ORcp3_34 = -RLcp3_14*qd[3];
VIcp3_14 = ORcp3_14+qd[1]+qd[4]*S3;
VIcp3_34 = ORcp3_34+qd[2]+qd[4]*C3;
ACcp3_14 = qdd[1]+ORcp3_34*qd[3]+RLcp3_34*qdd[3]+qdd[4]*S3+(2.0)*qd[3]*qd[4]*C3;
ACcp3_34 = qdd[2]-ORcp3_14*qd[3]-RLcp3_14*qdd[3]+qdd[4]*C3-(2.0)*qd[3]*qd[4]*S3;
OMcp3_25 = qd[3]+qd[5];
OPcp3_25 = qdd[3]+qdd[5];
RLcp3_16 = ROcp3_75*q[6];
RLcp3_36 = ROcp3_95*q[6];
POcp3_16 = POcp3_14+RLcp3_16;
POcp3_36 = POcp3_34+RLcp3_36;
ORcp3_16 = OMcp3_25*RLcp3_36;
ORcp3_36 = -OMcp3_25*RLcp3_16;
VIcp3_16 = ORcp3_16+VIcp3_14+ROcp3_75*qd[6];
VIcp3_36 = ORcp3_36+VIcp3_34+ROcp3_95*qd[6];
ACcp3_16 = ACcp3_14+OMcp3_25*ORcp3_36+(2.0)*OMcp3_25*ROcp3_95*qd[6]+OPcp3_25*RLcp3_36+ROcp3_75*qdd[6];
ACcp3_36 = ACcp3_34-OMcp3_25*ORcp3_16-(2.0)*OMcp3_25*ROcp3_75*qd[6]-OPcp3_25*RLcp3_16+ROcp3_95*qdd[6];
OMcp3_27 = OMcp3_25+qd[7];
OPcp3_27 = OPcp3_25+qdd[7];
OMcp3_28 = OMcp3_27+qd[8];
OPcp3_28 = OPcp3_27+qdd[8];
RLcp3_19 = ROcp3_78*dpt[3][9];
RLcp3_39 = ROcp3_98*dpt[3][9];
POcp3_19 = POcp3_16+RLcp3_19;
POcp3_39 = POcp3_36+RLcp3_39;
ORcp3_19 = OMcp3_28*RLcp3_39;
ORcp3_39 = -OMcp3_28*RLcp3_19;
VIcp3_19 = ORcp3_19+VIcp3_16;
VIcp3_39 = ORcp3_39+VIcp3_36;
ACcp3_19 = ACcp3_16+OMcp3_28*ORcp3_39+OPcp3_28*RLcp3_39;
ACcp3_39 = ACcp3_36-OMcp3_28*ORcp3_19-OPcp3_28*RLcp3_19;
sens->P[1] = POcp3_19;
sens->P[2] = 0;
sens->P[3] = POcp3_39;
sens->R[1][1] = ROcp3_18;
sens->R[1][3] = ROcp3_38;
sens->R[2][2] = (1.0);
sens->R[3][1] = ROcp3_78;
sens->R[3][3] = ROcp3_98;
sens->V[1] = VIcp3_19;
sens->V[2] = 0;
sens->V[3] = VIcp3_39;
sens->OM[1] = 0;
sens->OM[2] = OMcp3_28;
sens->OM[3] = 0;
sens->A[1] = ACcp3_19;
sens->A[2] = 0;
sens->A[3] = ACcp3_39;
sens->OMP[1] = 0;
sens->OMP[2] = OPcp3_28;
sens->OMP[3] = 0;

break;

case 4:

ROcp4_15 = C3*C5-S3*S5;
ROcp4_35 = -C3*S5-S3*C5;
ROcp4_75 = C3*S5+S3*C5;
ROcp4_95 = C3*C5-S3*S5;
ROcp4_17 = ROcp4_15*C7-ROcp4_75*S7;
ROcp4_37 = ROcp4_35*C7-ROcp4_95*S7;
ROcp4_77 = ROcp4_15*S7+ROcp4_75*C7;
ROcp4_97 = ROcp4_35*S7+ROcp4_95*C7;
ROcp4_18 = ROcp4_17*C8-ROcp4_77*S8;
ROcp4_38 = ROcp4_37*C8-ROcp4_97*S8;
ROcp4_78 = ROcp4_17*S8+ROcp4_77*C8;
ROcp4_98 = ROcp4_37*S8+ROcp4_97*C8;
RLcp4_14 = q[4]*S3;
RLcp4_34 = q[4]*C3;
POcp4_14 = RLcp4_14+q[1];
POcp4_34 = RLcp4_34+q[2];
ORcp4_14 = RLcp4_34*qd[3];
ORcp4_34 = -RLcp4_14*qd[3];
VIcp4_14 = ORcp4_14+qd[1]+qd[4]*S3;
VIcp4_34 = ORcp4_34+qd[2]+qd[4]*C3;
ACcp4_14 = qdd[1]+ORcp4_34*qd[3]+RLcp4_34*qdd[3]+qdd[4]*S3+(2.0)*qd[3]*qd[4]*C3;
ACcp4_34 = qdd[2]-ORcp4_14*qd[3]-RLcp4_14*qdd[3]+qdd[4]*C3-(2.0)*qd[3]*qd[4]*S3;
OMcp4_25 = qd[3]+qd[5];
OPcp4_25 = qdd[3]+qdd[5];
RLcp4_16 = ROcp4_75*q[6];
RLcp4_36 = ROcp4_95*q[6];
POcp4_16 = POcp4_14+RLcp4_16;
POcp4_36 = POcp4_34+RLcp4_36;
ORcp4_16 = OMcp4_25*RLcp4_36;
ORcp4_36 = -OMcp4_25*RLcp4_16;
VIcp4_16 = ORcp4_16+VIcp4_14+ROcp4_75*qd[6];
VIcp4_36 = ORcp4_36+VIcp4_34+ROcp4_95*qd[6];
ACcp4_16 = ACcp4_14+OMcp4_25*ORcp4_36+(2.0)*OMcp4_25*ROcp4_95*qd[6]+OPcp4_25*RLcp4_36+ROcp4_75*qdd[6];
ACcp4_36 = ACcp4_34-OMcp4_25*ORcp4_16-(2.0)*OMcp4_25*ROcp4_75*qd[6]-OPcp4_25*RLcp4_16+ROcp4_95*qdd[6];
OMcp4_27 = OMcp4_25+qd[7];
OPcp4_27 = OPcp4_25+qdd[7];
OMcp4_28 = OMcp4_27+qd[8];
OPcp4_28 = OPcp4_27+qdd[8];
RLcp4_19 = ROcp4_18*dpt[1][10]+ROcp4_78*dpt[3][10];
RLcp4_39 = ROcp4_38*dpt[1][10]+ROcp4_98*dpt[3][10];
POcp4_19 = POcp4_16+RLcp4_19;
POcp4_39 = POcp4_36+RLcp4_39;
ORcp4_19 = OMcp4_28*RLcp4_39;
ORcp4_39 = -OMcp4_28*RLcp4_19;
VIcp4_19 = ORcp4_19+VIcp4_16;
VIcp4_39 = ORcp4_39+VIcp4_36;
ACcp4_19 = ACcp4_16+OMcp4_28*ORcp4_39+OPcp4_28*RLcp4_39;
ACcp4_39 = ACcp4_36-OMcp4_28*ORcp4_19-OPcp4_28*RLcp4_19;
sens->P[1] = POcp4_19;
sens->P[2] = dpt[2][10];
sens->P[3] = POcp4_39;
sens->R[1][1] = ROcp4_18;
sens->R[1][3] = ROcp4_38;
sens->R[2][2] = (1.0);
sens->R[3][1] = ROcp4_78;
sens->R[3][3] = ROcp4_98;
sens->V[1] = VIcp4_19;
sens->V[2] = 0;
sens->V[3] = VIcp4_39;
sens->OM[1] = 0;
sens->OM[2] = OMcp4_28;
sens->OM[3] = 0;
sens->A[1] = ACcp4_19;
sens->A[2] = 0;
sens->A[3] = ACcp4_39;
sens->OMP[1] = 0;
sens->OMP[2] = OPcp4_28;
sens->OMP[3] = 0;

break;

case 5:

ROcp5_15 = C3*C5-S3*S5;
ROcp5_35 = -C3*S5-S3*C5;
ROcp5_75 = C3*S5+S3*C5;
ROcp5_95 = C3*C5-S3*S5;
ROcp5_17 = ROcp5_15*C7-ROcp5_75*S7;
ROcp5_37 = ROcp5_35*C7-ROcp5_95*S7;
ROcp5_77 = ROcp5_15*S7+ROcp5_75*C7;
ROcp5_97 = ROcp5_35*S7+ROcp5_95*C7;
ROcp5_18 = ROcp5_17*C8-ROcp5_77*S8;
ROcp5_38 = ROcp5_37*C8-ROcp5_97*S8;
ROcp5_78 = ROcp5_17*S8+ROcp5_77*C8;
ROcp5_98 = ROcp5_37*S8+ROcp5_97*C8;
RLcp5_14 = q[4]*S3;
RLcp5_34 = q[4]*C3;
POcp5_14 = RLcp5_14+q[1];
POcp5_34 = RLcp5_34+q[2];
ORcp5_14 = RLcp5_34*qd[3];
ORcp5_34 = -RLcp5_14*qd[3];
VIcp5_14 = ORcp5_14+qd[1]+qd[4]*S3;
VIcp5_34 = ORcp5_34+qd[2]+qd[4]*C3;
ACcp5_14 = qdd[1]+ORcp5_34*qd[3]+RLcp5_34*qdd[3]+qdd[4]*S3+(2.0)*qd[3]*qd[4]*C3;
ACcp5_34 = qdd[2]-ORcp5_14*qd[3]-RLcp5_14*qdd[3]+qdd[4]*C3-(2.0)*qd[3]*qd[4]*S3;
OMcp5_25 = qd[3]+qd[5];
OPcp5_25 = qdd[3]+qdd[5];
RLcp5_16 = ROcp5_75*q[6];
RLcp5_36 = ROcp5_95*q[6];
POcp5_16 = POcp5_14+RLcp5_16;
POcp5_36 = POcp5_34+RLcp5_36;
ORcp5_16 = OMcp5_25*RLcp5_36;
ORcp5_36 = -OMcp5_25*RLcp5_16;
VIcp5_16 = ORcp5_16+VIcp5_14+ROcp5_75*qd[6];
VIcp5_36 = ORcp5_36+VIcp5_34+ROcp5_95*qd[6];
ACcp5_16 = ACcp5_14+OMcp5_25*ORcp5_36+(2.0)*OMcp5_25*ROcp5_95*qd[6]+OPcp5_25*RLcp5_36+ROcp5_75*qdd[6];
ACcp5_36 = ACcp5_34-OMcp5_25*ORcp5_16-(2.0)*OMcp5_25*ROcp5_75*qd[6]-OPcp5_25*RLcp5_16+ROcp5_95*qdd[6];
OMcp5_27 = OMcp5_25+qd[7];
OPcp5_27 = OPcp5_25+qdd[7];
OMcp5_28 = OMcp5_27+qd[8];
OPcp5_28 = OPcp5_27+qdd[8];
RLcp5_19 = ROcp5_18*dpt[1][11]+ROcp5_78*dpt[3][11];
RLcp5_39 = ROcp5_38*dpt[1][11]+ROcp5_98*dpt[3][11];
POcp5_19 = POcp5_16+RLcp5_19;
POcp5_39 = POcp5_36+RLcp5_39;
ORcp5_19 = OMcp5_28*RLcp5_39;
ORcp5_39 = -OMcp5_28*RLcp5_19;
VIcp5_19 = ORcp5_19+VIcp5_16;
VIcp5_39 = ORcp5_39+VIcp5_36;
ACcp5_19 = ACcp5_16+OMcp5_28*ORcp5_39+OPcp5_28*RLcp5_39;
ACcp5_39 = ACcp5_36-OMcp5_28*ORcp5_19-OPcp5_28*RLcp5_19;
sens->P[1] = POcp5_19;
sens->P[2] = dpt[2][11];
sens->P[3] = POcp5_39;
sens->R[1][1] = ROcp5_18;
sens->R[1][3] = ROcp5_38;
sens->R[2][2] = (1.0);
sens->R[3][1] = ROcp5_78;
sens->R[3][3] = ROcp5_98;
sens->V[1] = VIcp5_19;
sens->V[2] = 0;
sens->V[3] = VIcp5_39;
sens->OM[1] = 0;
sens->OM[2] = OMcp5_28;
sens->OM[3] = 0;
sens->A[1] = ACcp5_19;
sens->A[2] = 0;
sens->A[3] = ACcp5_39;
sens->OMP[1] = 0;
sens->OMP[2] = OPcp5_28;
sens->OMP[3] = 0;

break;

case 6:

ROcp6_110 = C10*C3-S10*S3;
ROcp6_310 = -C10*S3-S10*C3;
ROcp6_710 = C10*S3+S10*C3;
ROcp6_910 = C10*C3-S10*S3;
ROcp6_112 = ROcp6_110*C12-ROcp6_710*S12;
ROcp6_312 = ROcp6_310*C12-ROcp6_910*S12;
ROcp6_712 = ROcp6_110*S12+ROcp6_710*C12;
ROcp6_912 = ROcp6_310*S12+ROcp6_910*C12;
ROcp6_113 = ROcp6_112*C13-ROcp6_712*S13;
ROcp6_313 = ROcp6_312*C13-ROcp6_912*S13;
ROcp6_713 = ROcp6_112*S13+ROcp6_712*C13;
ROcp6_913 = ROcp6_312*S13+ROcp6_912*C13;
RLcp6_14 = q[9]*S3;
RLcp6_34 = q[9]*C3;
POcp6_14 = RLcp6_14+q[1];
POcp6_34 = RLcp6_34+q[2];
ORcp6_14 = RLcp6_34*qd[3];
ORcp6_34 = -RLcp6_14*qd[3];
VIcp6_14 = ORcp6_14+qd[1]+qd[9]*S3;
VIcp6_34 = ORcp6_34+qd[2]+qd[9]*C3;
ACcp6_14 = qdd[1]+ORcp6_34*qd[3]+RLcp6_34*qdd[3]+qdd[9]*S3+(2.0)*qd[3]*qd[9]*C3;
ACcp6_34 = qdd[2]-ORcp6_14*qd[3]-RLcp6_14*qdd[3]+qdd[9]*C3-(2.0)*qd[3]*qd[9]*S3;
OMcp6_25 = qd[10]+qd[3];
OPcp6_25 = qdd[10]+qdd[3];
RLcp6_16 = ROcp6_710*q[11];
RLcp6_36 = ROcp6_910*q[11];
POcp6_16 = POcp6_14+RLcp6_16;
POcp6_36 = POcp6_34+RLcp6_36;
ORcp6_16 = OMcp6_25*RLcp6_36;
ORcp6_36 = -OMcp6_25*RLcp6_16;
VIcp6_16 = ORcp6_16+VIcp6_14+ROcp6_710*qd[11];
VIcp6_36 = ORcp6_36+VIcp6_34+ROcp6_910*qd[11];
ACcp6_16 = ACcp6_14+OMcp6_25*ORcp6_36+(2.0)*OMcp6_25*ROcp6_910*qd[11]+OPcp6_25*RLcp6_36+ROcp6_710*qdd[11];
ACcp6_36 = ACcp6_34-OMcp6_25*ORcp6_16-(2.0)*OMcp6_25*ROcp6_710*qd[11]-OPcp6_25*RLcp6_16+ROcp6_910*qdd[11];
OMcp6_27 = OMcp6_25+qd[12];
OPcp6_27 = OPcp6_25+qdd[12];
OMcp6_28 = OMcp6_27+qd[13];
OPcp6_28 = OPcp6_27+qdd[13];
RLcp6_19 = ROcp6_113*dpt[1][14]+ROcp6_713*dpt[3][14];
RLcp6_39 = ROcp6_313*dpt[1][14]+ROcp6_913*dpt[3][14];
POcp6_19 = POcp6_16+RLcp6_19;
POcp6_39 = POcp6_36+RLcp6_39;
ORcp6_19 = OMcp6_28*RLcp6_39;
ORcp6_39 = -OMcp6_28*RLcp6_19;
VIcp6_19 = ORcp6_19+VIcp6_16;
VIcp6_39 = ORcp6_39+VIcp6_36;
ACcp6_19 = ACcp6_16+OMcp6_28*ORcp6_39+OPcp6_28*RLcp6_39;
ACcp6_39 = ACcp6_36-OMcp6_28*ORcp6_19-OPcp6_28*RLcp6_19;
sens->P[1] = POcp6_19;
sens->P[2] = dpt[2][14];
sens->P[3] = POcp6_39;
sens->R[1][1] = ROcp6_113;
sens->R[1][3] = ROcp6_313;
sens->R[2][2] = (1.0);
sens->R[3][1] = ROcp6_713;
sens->R[3][3] = ROcp6_913;
sens->V[1] = VIcp6_19;
sens->V[2] = 0;
sens->V[3] = VIcp6_39;
sens->OM[1] = 0;
sens->OM[2] = OMcp6_28;
sens->OM[3] = 0;
sens->A[1] = ACcp6_19;
sens->A[2] = 0;
sens->A[3] = ACcp6_39;
sens->OMP[1] = 0;
sens->OMP[2] = OPcp6_28;
sens->OMP[3] = 0;

break;

case 7:

ROcp7_110 = C10*C3-S10*S3;
ROcp7_310 = -C10*S3-S10*C3;
ROcp7_710 = C10*S3+S10*C3;
ROcp7_910 = C10*C3-S10*S3;
ROcp7_112 = ROcp7_110*C12-ROcp7_710*S12;
ROcp7_312 = ROcp7_310*C12-ROcp7_910*S12;
ROcp7_712 = ROcp7_110*S12+ROcp7_710*C12;
ROcp7_912 = ROcp7_310*S12+ROcp7_910*C12;
ROcp7_113 = ROcp7_112*C13-ROcp7_712*S13;
ROcp7_313 = ROcp7_312*C13-ROcp7_912*S13;
ROcp7_713 = ROcp7_112*S13+ROcp7_712*C13;
ROcp7_913 = ROcp7_312*S13+ROcp7_912*C13;
RLcp7_14 = q[9]*S3;
RLcp7_34 = q[9]*C3;
POcp7_14 = RLcp7_14+q[1];
POcp7_34 = RLcp7_34+q[2];
ORcp7_14 = RLcp7_34*qd[3];
ORcp7_34 = -RLcp7_14*qd[3];
VIcp7_14 = ORcp7_14+qd[1]+qd[9]*S3;
VIcp7_34 = ORcp7_34+qd[2]+qd[9]*C3;
ACcp7_14 = qdd[1]+ORcp7_34*qd[3]+RLcp7_34*qdd[3]+qdd[9]*S3+(2.0)*qd[3]*qd[9]*C3;
ACcp7_34 = qdd[2]-ORcp7_14*qd[3]-RLcp7_14*qdd[3]+qdd[9]*C3-(2.0)*qd[3]*qd[9]*S3;
OMcp7_25 = qd[10]+qd[3];
OPcp7_25 = qdd[10]+qdd[3];
RLcp7_16 = ROcp7_710*q[11];
RLcp7_36 = ROcp7_910*q[11];
POcp7_16 = POcp7_14+RLcp7_16;
POcp7_36 = POcp7_34+RLcp7_36;
ORcp7_16 = OMcp7_25*RLcp7_36;
ORcp7_36 = -OMcp7_25*RLcp7_16;
VIcp7_16 = ORcp7_16+VIcp7_14+ROcp7_710*qd[11];
VIcp7_36 = ORcp7_36+VIcp7_34+ROcp7_910*qd[11];
ACcp7_16 = ACcp7_14+OMcp7_25*ORcp7_36+(2.0)*OMcp7_25*ROcp7_910*qd[11]+OPcp7_25*RLcp7_36+ROcp7_710*qdd[11];
ACcp7_36 = ACcp7_34-OMcp7_25*ORcp7_16-(2.0)*OMcp7_25*ROcp7_710*qd[11]-OPcp7_25*RLcp7_16+ROcp7_910*qdd[11];
OMcp7_27 = OMcp7_25+qd[12];
OPcp7_27 = OPcp7_25+qdd[12];
OMcp7_28 = OMcp7_27+qd[13];
OPcp7_28 = OPcp7_27+qdd[13];
RLcp7_19 = ROcp7_113*dpt[1][15]+ROcp7_713*dpt[3][15];
RLcp7_39 = ROcp7_313*dpt[1][15]+ROcp7_913*dpt[3][15];
POcp7_19 = POcp7_16+RLcp7_19;
POcp7_39 = POcp7_36+RLcp7_39;
ORcp7_19 = OMcp7_28*RLcp7_39;
ORcp7_39 = -OMcp7_28*RLcp7_19;
VIcp7_19 = ORcp7_19+VIcp7_16;
VIcp7_39 = ORcp7_39+VIcp7_36;
ACcp7_19 = ACcp7_16+OMcp7_28*ORcp7_39+OPcp7_28*RLcp7_39;
ACcp7_39 = ACcp7_36-OMcp7_28*ORcp7_19-OPcp7_28*RLcp7_19;
sens->P[1] = POcp7_19;
sens->P[2] = dpt[2][15];
sens->P[3] = POcp7_39;
sens->R[1][1] = ROcp7_113;
sens->R[1][3] = ROcp7_313;
sens->R[2][2] = (1.0);
sens->R[3][1] = ROcp7_713;
sens->R[3][3] = ROcp7_913;
sens->V[1] = VIcp7_19;
sens->V[2] = 0;
sens->V[3] = VIcp7_39;
sens->OM[1] = 0;
sens->OM[2] = OMcp7_28;
sens->OM[3] = 0;
sens->A[1] = ACcp7_19;
sens->A[2] = 0;
sens->A[3] = ACcp7_39;
sens->OMP[1] = 0;
sens->OMP[2] = OPcp7_28;
sens->OMP[3] = 0;

break;

case 8:

ROcp8_110 = C10*C3-S10*S3;
ROcp8_310 = -C10*S3-S10*C3;
ROcp8_710 = C10*S3+S10*C3;
ROcp8_910 = C10*C3-S10*S3;
ROcp8_112 = ROcp8_110*C12-ROcp8_710*S12;
ROcp8_312 = ROcp8_310*C12-ROcp8_910*S12;
ROcp8_712 = ROcp8_110*S12+ROcp8_710*C12;
ROcp8_912 = ROcp8_310*S12+ROcp8_910*C12;
ROcp8_113 = ROcp8_112*C13-ROcp8_712*S13;
ROcp8_313 = ROcp8_312*C13-ROcp8_912*S13;
ROcp8_713 = ROcp8_112*S13+ROcp8_712*C13;
ROcp8_913 = ROcp8_312*S13+ROcp8_912*C13;
RLcp8_14 = q[9]*S3;
RLcp8_34 = q[9]*C3;
POcp8_14 = RLcp8_14+q[1];
POcp8_34 = RLcp8_34+q[2];
ORcp8_14 = RLcp8_34*qd[3];
ORcp8_34 = -RLcp8_14*qd[3];
VIcp8_14 = ORcp8_14+qd[1]+qd[9]*S3;
VIcp8_34 = ORcp8_34+qd[2]+qd[9]*C3;
ACcp8_14 = qdd[1]+ORcp8_34*qd[3]+RLcp8_34*qdd[3]+qdd[9]*S3+(2.0)*qd[3]*qd[9]*C3;
ACcp8_34 = qdd[2]-ORcp8_14*qd[3]-RLcp8_14*qdd[3]+qdd[9]*C3-(2.0)*qd[3]*qd[9]*S3;
OMcp8_25 = qd[10]+qd[3];
OPcp8_25 = qdd[10]+qdd[3];
RLcp8_16 = ROcp8_710*q[11];
RLcp8_36 = ROcp8_910*q[11];
POcp8_16 = POcp8_14+RLcp8_16;
POcp8_36 = POcp8_34+RLcp8_36;
ORcp8_16 = OMcp8_25*RLcp8_36;
ORcp8_36 = -OMcp8_25*RLcp8_16;
VIcp8_16 = ORcp8_16+VIcp8_14+ROcp8_710*qd[11];
VIcp8_36 = ORcp8_36+VIcp8_34+ROcp8_910*qd[11];
ACcp8_16 = ACcp8_14+OMcp8_25*ORcp8_36+(2.0)*OMcp8_25*ROcp8_910*qd[11]+OPcp8_25*RLcp8_36+ROcp8_710*qdd[11];
ACcp8_36 = ACcp8_34-OMcp8_25*ORcp8_16-(2.0)*OMcp8_25*ROcp8_710*qd[11]-OPcp8_25*RLcp8_16+ROcp8_910*qdd[11];
OMcp8_27 = OMcp8_25+qd[12];
OPcp8_27 = OPcp8_25+qdd[12];
OMcp8_28 = OMcp8_27+qd[13];
OPcp8_28 = OPcp8_27+qdd[13];
RLcp8_19 = ROcp8_713*dpt[3][16];
RLcp8_39 = ROcp8_913*dpt[3][16];
POcp8_19 = POcp8_16+RLcp8_19;
POcp8_39 = POcp8_36+RLcp8_39;
ORcp8_19 = OMcp8_28*RLcp8_39;
ORcp8_39 = -OMcp8_28*RLcp8_19;
VIcp8_19 = ORcp8_19+VIcp8_16;
VIcp8_39 = ORcp8_39+VIcp8_36;
ACcp8_19 = ACcp8_16+OMcp8_28*ORcp8_39+OPcp8_28*RLcp8_39;
ACcp8_39 = ACcp8_36-OMcp8_28*ORcp8_19-OPcp8_28*RLcp8_19;
sens->P[1] = POcp8_19;
sens->P[2] = 0;
sens->P[3] = POcp8_39;
sens->R[1][1] = ROcp8_113;
sens->R[1][3] = ROcp8_313;
sens->R[2][2] = (1.0);
sens->R[3][1] = ROcp8_713;
sens->R[3][3] = ROcp8_913;
sens->V[1] = VIcp8_19;
sens->V[2] = 0;
sens->V[3] = VIcp8_39;
sens->OM[1] = 0;
sens->OM[2] = OMcp8_28;
sens->OM[3] = 0;
sens->A[1] = ACcp8_19;
sens->A[2] = 0;
sens->A[3] = ACcp8_39;
sens->OMP[1] = 0;
sens->OMP[2] = OPcp8_28;
sens->OMP[3] = 0;

break;

case 9:

ROcp9_110 = C10*C3-S10*S3;
ROcp9_310 = -C10*S3-S10*C3;
ROcp9_710 = C10*S3+S10*C3;
ROcp9_910 = C10*C3-S10*S3;
ROcp9_112 = ROcp9_110*C12-ROcp9_710*S12;
ROcp9_312 = ROcp9_310*C12-ROcp9_910*S12;
ROcp9_712 = ROcp9_110*S12+ROcp9_710*C12;
ROcp9_912 = ROcp9_310*S12+ROcp9_910*C12;
ROcp9_113 = ROcp9_112*C13-ROcp9_712*S13;
ROcp9_313 = ROcp9_312*C13-ROcp9_912*S13;
ROcp9_713 = ROcp9_112*S13+ROcp9_712*C13;
ROcp9_913 = ROcp9_312*S13+ROcp9_912*C13;
RLcp9_14 = q[9]*S3;
RLcp9_34 = q[9]*C3;
POcp9_14 = RLcp9_14+q[1];
POcp9_34 = RLcp9_34+q[2];
ORcp9_14 = RLcp9_34*qd[3];
ORcp9_34 = -RLcp9_14*qd[3];
VIcp9_14 = ORcp9_14+qd[1]+qd[9]*S3;
VIcp9_34 = ORcp9_34+qd[2]+qd[9]*C3;
ACcp9_14 = qdd[1]+ORcp9_34*qd[3]+RLcp9_34*qdd[3]+qdd[9]*S3+(2.0)*qd[3]*qd[9]*C3;
ACcp9_34 = qdd[2]-ORcp9_14*qd[3]-RLcp9_14*qdd[3]+qdd[9]*C3-(2.0)*qd[3]*qd[9]*S3;
OMcp9_25 = qd[10]+qd[3];
OPcp9_25 = qdd[10]+qdd[3];
RLcp9_16 = ROcp9_710*q[11];
RLcp9_36 = ROcp9_910*q[11];
POcp9_16 = POcp9_14+RLcp9_16;
POcp9_36 = POcp9_34+RLcp9_36;
ORcp9_16 = OMcp9_25*RLcp9_36;
ORcp9_36 = -OMcp9_25*RLcp9_16;
VIcp9_16 = ORcp9_16+VIcp9_14+ROcp9_710*qd[11];
VIcp9_36 = ORcp9_36+VIcp9_34+ROcp9_910*qd[11];
ACcp9_16 = ACcp9_14+OMcp9_25*ORcp9_36+(2.0)*OMcp9_25*ROcp9_910*qd[11]+OPcp9_25*RLcp9_36+ROcp9_710*qdd[11];
ACcp9_36 = ACcp9_34-OMcp9_25*ORcp9_16-(2.0)*OMcp9_25*ROcp9_710*qd[11]-OPcp9_25*RLcp9_16+ROcp9_910*qdd[11];
OMcp9_27 = OMcp9_25+qd[12];
OPcp9_27 = OPcp9_25+qdd[12];
OMcp9_28 = OMcp9_27+qd[13];
OPcp9_28 = OPcp9_27+qdd[13];
RLcp9_19 = ROcp9_113*dpt[1][17]+ROcp9_713*dpt[3][17];
RLcp9_39 = ROcp9_313*dpt[1][17]+ROcp9_913*dpt[3][17];
POcp9_19 = POcp9_16+RLcp9_19;
POcp9_39 = POcp9_36+RLcp9_39;
ORcp9_19 = OMcp9_28*RLcp9_39;
ORcp9_39 = -OMcp9_28*RLcp9_19;
VIcp9_19 = ORcp9_19+VIcp9_16;
VIcp9_39 = ORcp9_39+VIcp9_36;
ACcp9_19 = ACcp9_16+OMcp9_28*ORcp9_39+OPcp9_28*RLcp9_39;
ACcp9_39 = ACcp9_36-OMcp9_28*ORcp9_19-OPcp9_28*RLcp9_19;
sens->P[1] = POcp9_19;
sens->P[2] = dpt[2][17];
sens->P[3] = POcp9_39;
sens->R[1][1] = ROcp9_113;
sens->R[1][3] = ROcp9_313;
sens->R[2][2] = (1.0);
sens->R[3][1] = ROcp9_713;
sens->R[3][3] = ROcp9_913;
sens->V[1] = VIcp9_19;
sens->V[2] = 0;
sens->V[3] = VIcp9_39;
sens->OM[1] = 0;
sens->OM[2] = OMcp9_28;
sens->OM[3] = 0;
sens->A[1] = ACcp9_19;
sens->A[2] = 0;
sens->A[3] = ACcp9_39;
sens->OMP[1] = 0;
sens->OMP[2] = OPcp9_28;
sens->OMP[3] = 0;

break;

case 10:

ROcp10_110 = C10*C3-S10*S3;
ROcp10_310 = -C10*S3-S10*C3;
ROcp10_710 = C10*S3+S10*C3;
ROcp10_910 = C10*C3-S10*S3;
ROcp10_112 = ROcp10_110*C12-ROcp10_710*S12;
ROcp10_312 = ROcp10_310*C12-ROcp10_910*S12;
ROcp10_712 = ROcp10_110*S12+ROcp10_710*C12;
ROcp10_912 = ROcp10_310*S12+ROcp10_910*C12;
ROcp10_113 = ROcp10_112*C13-ROcp10_712*S13;
ROcp10_313 = ROcp10_312*C13-ROcp10_912*S13;
ROcp10_713 = ROcp10_112*S13+ROcp10_712*C13;
ROcp10_913 = ROcp10_312*S13+ROcp10_912*C13;
RLcp10_14 = q[9]*S3;
RLcp10_34 = q[9]*C3;
POcp10_14 = RLcp10_14+q[1];
POcp10_34 = RLcp10_34+q[2];
ORcp10_14 = RLcp10_34*qd[3];
ORcp10_34 = -RLcp10_14*qd[3];
VIcp10_14 = ORcp10_14+qd[1]+qd[9]*S3;
VIcp10_34 = ORcp10_34+qd[2]+qd[9]*C3;
ACcp10_14 = qdd[1]+ORcp10_34*qd[3]+RLcp10_34*qdd[3]+qdd[9]*S3+(2.0)*qd[3]*qd[9]*C3;
ACcp10_34 = qdd[2]-ORcp10_14*qd[3]-RLcp10_14*qdd[3]+qdd[9]*C3-(2.0)*qd[3]*qd[9]*S3;
OMcp10_25 = qd[10]+qd[3];
OPcp10_25 = qdd[10]+qdd[3];
RLcp10_16 = ROcp10_710*q[11];
RLcp10_36 = ROcp10_910*q[11];
POcp10_16 = POcp10_14+RLcp10_16;
POcp10_36 = POcp10_34+RLcp10_36;
ORcp10_16 = OMcp10_25*RLcp10_36;
ORcp10_36 = -OMcp10_25*RLcp10_16;
VIcp10_16 = ORcp10_16+VIcp10_14+ROcp10_710*qd[11];
VIcp10_36 = ORcp10_36+VIcp10_34+ROcp10_910*qd[11];
ACcp10_16 = ACcp10_14+OMcp10_25*ORcp10_36+(2.0)*OMcp10_25*ROcp10_910*qd[11]+OPcp10_25*RLcp10_36+ROcp10_710*qdd[11];
ACcp10_36 = ACcp10_34-OMcp10_25*ORcp10_16-(2.0)*OMcp10_25*ROcp10_710*qd[11]-OPcp10_25*RLcp10_16+ROcp10_910*qdd[11];
OMcp10_27 = OMcp10_25+qd[12];
OPcp10_27 = OPcp10_25+qdd[12];
OMcp10_28 = OMcp10_27+qd[13];
OPcp10_28 = OPcp10_27+qdd[13];
RLcp10_19 = ROcp10_113*dpt[1][18]+ROcp10_713*dpt[3][18];
RLcp10_39 = ROcp10_313*dpt[1][18]+ROcp10_913*dpt[3][18];
POcp10_19 = POcp10_16+RLcp10_19;
POcp10_39 = POcp10_36+RLcp10_39;
ORcp10_19 = OMcp10_28*RLcp10_39;
ORcp10_39 = -OMcp10_28*RLcp10_19;
VIcp10_19 = ORcp10_19+VIcp10_16;
VIcp10_39 = ORcp10_39+VIcp10_36;
ACcp10_19 = ACcp10_16+OMcp10_28*ORcp10_39+OPcp10_28*RLcp10_39;
ACcp10_39 = ACcp10_36-OMcp10_28*ORcp10_19-OPcp10_28*RLcp10_19;
sens->P[1] = POcp10_19;
sens->P[2] = dpt[2][18];
sens->P[3] = POcp10_39;
sens->R[1][1] = ROcp10_113;
sens->R[1][3] = ROcp10_313;
sens->R[2][2] = (1.0);
sens->R[3][1] = ROcp10_713;
sens->R[3][3] = ROcp10_913;
sens->V[1] = VIcp10_19;
sens->V[2] = 0;
sens->V[3] = VIcp10_39;
sens->OM[1] = 0;
sens->OM[2] = OMcp10_28;
sens->OM[3] = 0;
sens->A[1] = ACcp10_19;
sens->A[2] = 0;
sens->A[3] = ACcp10_39;
sens->OMP[1] = 0;
sens->OMP[2] = OPcp10_28;
sens->OMP[3] = 0;

break;

default:

break;

}


// Number of continuation lines = 0

}
