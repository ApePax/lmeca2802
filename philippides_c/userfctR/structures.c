#include "structures.h"
#include <stdio.h>
#include <stdlib.h>

// CSV parameters
const char* filename_CSV = "../../Walking_Patterns/WP.csv";
double FREQUENCY = 50.0;
int COLUMNS = 5; // time,q1_l,q1_r,q2_l,q2_r -> 5 columns

// Contact structures
Contact_Manager *cm = NULL;
HuntCrossleyHertz *hc_model = NULL;
ViscoelasticCoulombModel *vc_model = NULL;

// Reference trajectory
Trajectory *traj = NULL;

// Voltages applied on the motors
double *voltages = NULL;

