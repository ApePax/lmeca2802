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

// -------------------------------------------------------
//      CONTACT MANAGER
//      /!\ compute_penetrations: different from python,
//          *deltas and *deltas_dot are passed as arguments
// -------------------------------------------------------

typedef struct {
    int nSensors;
    double **Contact_PxF;   // Positions of the contact points for each sensor (nSensors x 4)
    double **Previous_PxF;  // Previous positions of the sensor (nSensors x 4)
    int *InContact;         // Status of the sensor (0 or 1) (nSensors)
    double *results;        // Force outputs on each sensor (4 * nSensors)
} Contact_Manager;

// Function to create and initialize the Contact_Manager structure
Contact_Manager* create_contact_manager(int nSensors) {
    Contact_Manager *cm = (Contact_Manager *)malloc(sizeof(Contact_Manager));
    cm->nSensors = nSensors;
    
    // Allocate memory for Contact_PxF and Previous_PxF
    cm->Contact_PxF = (double **)malloc(nSensors * sizeof(double *));
    cm->Previous_PxF = (double **)malloc(nSensors * sizeof(double *));
    
    for (int i = 0; i < nSensors; i++) {
        cm->Contact_PxF[i] = (double *)malloc(4 * sizeof(double)); // 4 values per sensor
        cm->Previous_PxF[i] = (double *)malloc(4 * sizeof(double)); // 4 values per sensor
    }
    
    // Allocate memory for InContact and results
    cm->InContact = (int *)malloc(nSensors * sizeof(int));
    cm->results = (double *)malloc(4 * nSensors * sizeof(double)); // 4 * nSensors results
    
    // Initialize values to NaN or zero
    for (int i = 0; i < nSensors; i++) {
        for (int j = 0; j < 4; j++) {
            cm->Contact_PxF[i][j] = NAN;
            cm->Previous_PxF[i][j] = NAN;
        }
        cm->InContact[i] = 0;
    }
    
    for (int i = 0; i < 4 * nSensors; i++) {
        cm->results[i] = 0.0;
    }

    return cm;
}

// Function to update the contact with a new sensor position
void update_contact(Contact_Manager *cm, int ixF, double PxF[4]) {
    if (isnan(cm->Previous_PxF[ixF][0])) {
        // Initial position
        for (int i = 0; i < 4; i++) {
            cm->Previous_PxF[ixF][i] = PxF[i];
        }
        
        if (PxF[3] <= 0) {
            for (int i = 0; i < 4; i++) {
                cm->Contact_PxF[ixF][i] = PxF[i];
            }
            cm->Contact_PxF[ixF][3] = 0.0; // Set z = 0
            cm->InContact[ixF] = 1;
        }
    } else if (PxF[3] <= 0) {
        // Penetration detection
        if (cm->InContact[ixF] == 0) {
            double ratio = cm->Previous_PxF[ixF][3] / (cm->Previous_PxF[ixF][3] - PxF[3]);
            for (int i = 0; i < 4; i++) {
                cm->Contact_PxF[ixF][i] = cm->Previous_PxF[ixF][i] + ratio * (PxF[i] - cm->Previous_PxF[ixF][i]);
            }
            cm->Contact_PxF[ixF][3] = 0.0; // Ensure z = 0
            cm->InContact[ixF] = 1;
        }
    } else {
        cm->InContact[ixF] = 0;
    }
    
    // Update the previous position
    for (int i = 0; i < 4; i++) {
        cm->Previous_PxF[ixF][i] = PxF[i];
    }
}

// Function to update slip contact
void update_slip_contact(Contact_Manager *cm, int ixF, double PxF[4], double delta_slip_x, double delta_slip_y, int slip) {
    if (slip == 1 && cm->InContact[ixF] == 1) {
        cm->Contact_PxF[ixF][1] = PxF[1] - delta_slip_x;
        cm->Contact_PxF[ixF][2] = PxF[2] - delta_slip_y;
    }
}

// Function to compute penetrations
///!!!\\\ different from python, *deltas and *deltas_dot are passed as arguments
void compute_penetrations(Contact_Manager *cm, int ixF, double PxF[4], double VxF[4], double *deltas, double *deltas_dot) {
    for (int i = 0; i < 3; i++) {
        deltas[i] = 0.0;
        deltas_dot[i] = 0.0;
    }
    
    if (cm->InContact[ixF] == 1) {
        for (int i = 0; i < 3; i++) {
            deltas[i] = PxF[i+1] - cm->Contact_PxF[ixF][i+1];
        }
        for (int i = 0; i < 3; i++) {
            deltas_dot[i] = VxF[i+1];
        }
    }
}

// Function to free the memory used by the Contact_Manager
void free_contact_manager(Contact_Manager *cm) {
    for (int i = 0; i < cm->nSensors; i++) {
        free(cm->Contact_PxF[i]);
        free(cm->Previous_PxF[i]);
    }
    free(cm->Contact_PxF);
    free(cm->Previous_PxF);
    free(cm->InContact);
    free(cm->results);
    free(cm);
}

Contact_Manager *cm = create_contact_manager(11);

// -------------------------------------------------------
//      FRICTION MODELS
//  /!\ arguments of tangential force: *Fx,*Fy,*delta_no_slip_x,*delta_no_slip_y,*slip
// -------------------------------------------------------
#include <stdio.h>
#include <math.h>

typedef struct {
    double k; // Stiffness coefficient
    double n; // Nonlinearity exponent
    double d; // Damping coefficient
} HuntCrossleyHertz;

typedef struct {
    double mu; // Coefficient of friction
    double k;  // Stiffness coefficient
    double d;  // Damping coefficient
} ViscoelasticCoulombModel;

// Function to initialize the Hunt-Crossley Hertz model
HuntCrossleyHertz* create_hunt_crossley_hertz(double k, double n, double d) {
    HuntCrossleyHertz *model = (HuntCrossleyHertz*)malloc(sizeof(HuntCrossleyHertz));
    model->k = k;
    model->n = n;
    model->d = d;
    return model;
}

// Function to compute the normal force using the Hunt-Crossley Hertz model
double compute_normal_force_hc(HuntCrossleyHertz *model, double delta, double delta_dot) {
    double Fz = 0.0;
    if (delta < 0) {
        Fz = model->k * pow(fabs(delta), model->n) * (1 - model->d * delta_dot);
    }
    return Fz;
}

// Function to initialize the Viscoelastic Coulomb model
ViscoelasticCoulombModel* create_viscoelastic_coulomb(double mu, double k, double d) {
    ViscoelasticCoulombModel *model = (ViscoelasticCoulombModel*)malloc(sizeof(ViscoelasticCoulombModel));
    model->mu = mu;
    model->k = k;
    model->d = d;
    return model;
}

// Function to compute tangential force using the Viscoelastic Coulomb model
void compute_tangential_force(ViscoelasticCoulombModel *model, double F_n, double delta_x, double delta_x_dot, double delta_y, double delta_y_dot, double *Fx, double *Fy, double *delta_no_slip_x, double *delta_no_slip_y, int *slip) {
    // Compute raw viscoelastic forces
    *Fx = -model->k * delta_x - model->d * delta_x_dot;
    *Fy = -model->k * delta_y - model->d * delta_y_dot;

    // Compute total force magnitude
    double F_total = sqrt(*Fx * *Fx + *Fy * *Fy);
    // Maximum Coulomb force
    double F_max = model->mu * F_n;

    // Initialize (assuming no slip at first)
    *delta_no_slip_x = 0;
    *delta_no_slip_y = 0;
    *slip = 0;

    // Apply saturation (if slip occurs)
    if (F_total > F_max) {
        double scaling_factor = F_max / F_total;
        *Fx *= scaling_factor;
        *Fy *= scaling_factor;
        *delta_no_slip_x = -( *Fx + model->d * delta_x_dot ) / model->k;
        *delta_no_slip_y = -( *Fy + model->d * delta_y_dot ) / model->k;
        *slip = 1;
    }
}

// Function to compute normal force using the Viscoelastic Coulomb model
double compute_normal_force_viscoelastic(ViscoelasticCoulombModel *model, double delta, double delta_dot) {
    double Fz = 0.0;
    if (delta <= 0) {
        Fz = -model->k * delta - model->d * delta_dot;
    }
    return Fz;
}
// Example usage of the Hunt-Crossley Hertz model
HuntCrossleyHertz *hc_model = create_hunt_crossley_hertz(1000.0, 1.5, 0.1);
// Example usage of the Viscoelastic Coulomb model
ViscoelasticCoulombModel *vc_model = create_viscoelastic_coulomb(0.5, 1000.0, 0.1);

double* user_ExtForces(double PxF[4], double RxF[4][4], 
    double VxF[4], double OMxF[4], 
    double AxF[4], double OMPxF[4], 
    MbsData *mbs_data, double tsim, int ixF)
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

// Begin user-defined force calculation
// Update contact information (function would need to be implemented as part of cm or similar)
update_contact(cm, ixF, PxF);

// Compute penetrations and velocities (function would need to be implemented)
double deltas[4], deltas_dot[4];
compute_penetrations(cm, ixF, PxF, VxF, deltas, deltas_dot);

// Compute normal force based on the chosen model (Hunt-Crossley-Hertz or Viscoelastic Coulomb)
int Model = 0;
if (Model == 0) {
Fz = compute_normal_force(hc_model,deltas[2], deltas_dot[2]);
}
else if (Model == 1) {
Fz = compute_normal_force(vc_model, deltas[2], deltas_dot[2]);
}

// Compute tangential forces
double delta_no_slip_x = 0.0, delta_no_slip_y = 0.0;
int slip = 0;
compute_tangential_force(vc_model,Fz, deltas[0], deltas_dot[0], deltas[1], deltas_dot[1],
    Fx, Fy, delta_no_slip_x, delta_no_slip_y, slip
);

// Update slip contact data
update_slip_contact(cm, ixF, PxF, delta_no_slip_x, delta_no_slip_y, slip);

// Concatenate force, torque, and force application point to the returned array.
double *SWr = mbs_data->SWr[ixF];
SWr[1] = Fx;
SWr[2] = Fy;
SWr[3] = Fz;
SWr[4] = Mx;
SWr[5] = My;
SWr[6] = Mz;
SWr[7] = dxF[1];
SWr[8] = dxF[2];
SWr[9] = dxF[3];

// Store results in mbs_data.
cm.results[4 * ixF] = Fx;
cm.results[4 * ixF + 1] = Fy;
cm.results[4 * ixF + 2] = Fz;
cm.results[4 * ixF + 3] = ixF;
if (ixF == 10) {
mbs_data->Force_Sensors = cm.results[4];
}

return SWr;
}


"""
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
"""