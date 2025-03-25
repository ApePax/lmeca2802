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
//	==> Function: F18 - Constraints Quadratic Velocity Terms (Jdqd)
//
//	==> Git hash: a9259031f25de7c9f86dbac08464ef31e84ae3b1
//
//	==> Input XML
//

#include <math.h> 

#include "mbs_data.h"

void mbs_cons_jdqd(double *Jdqd,
MbsData *s, double tsim)
{
#include "mbs_cons_jdqd_philippides.h"

double *q, *qd;
double **dpt, *lrod;

q = s->q;
qd = s->qd;

dpt = s->dpt;
lrod = s->lrod;

// Number of continuation lines = 0

#include "mbs_message.h"

mbs_msg("Your symbolic files seem obsolete, i.e. not up-to-date with your MBsysPad model. Please regenerate your symbolic files (MBsysPad->Tools->Generate Symbolic Files). Exiting.\n");
s->flag_stop = 1;
return;
}
