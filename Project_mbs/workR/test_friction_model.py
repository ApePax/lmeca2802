# -*- coding: utf-8 -*-
"""
Created on Thu Mar 20 16:22:55 2025

@author: mmora
"""

# -*- coding: utf-8 -*-
"""
Created on Thu Mar 20 15:53:26 2025

@author: mmora
"""

import numpy as np
import matplotlib.pyplot as plt

class Contact_Manager:
    def __init__(self, nSensors):
        """
        Initialize the memory of the contact points for the tangential contact forces.

        Parameters:
        nSensors (int) : Number of force sensors + 1
        """
        self.nSensors = nSensors
        self.Contact_PxF  = np.full((nSensors, 4), np.nan)
        self.Previous_PxF = np.full((nSensors, 4), np.nan)
        self.InContact    = np.zeros(nSensors, dtype=int)
        return 
    
    def update_contact(self, ixF, PxF):
        """
        Update the contact memory with the new sensor position.
        """
        PxF = PxF[:4]  # Ensure we're using the first 4 elements
        print("PxF:")
        print(PxF)
        # Initialization
        if np.isnan(self.Previous_PxF[ixF, 0]):
            self.Previous_PxF[ixF] = PxF
            if PxF[3] <= 0:
                self.Contact_PxF[ixF] = PxF
                self.Contact_PxF[ixF, 3] = 0.0
                self.InContact[ixF] = 1
        
        # Detection of penetration
        if PxF[3] <= 0:
            if self.InContact[ixF] == 0:
                ratio = self.Previous_PxF[ixF, 3] / (self.Previous_PxF[ixF, 3] - PxF[3])
                self.Contact_PxF[ixF, :4] = self.Previous_PxF[ixF, :4] + ratio * (PxF - self.Previous_PxF[ixF, :4])
                self.Contact_PxF[ixF, 3] = 0.0
                self.InContact[ixF] = 1
        else:
            self.InContact[ixF] = 0
        print("Previous:")
        print(self.Previous_PxF[ixF])
        self.Previous_PxF[ixF] = PxF
        print("Contact Point:")
        print(self.Contact_PxF[ixF])
        
        return
    
    def update_slip_contact(self, ixF, PxF, delta_slip_x, delta_slip_y, slip):
        if slip == 1 and self.InContact[ixF] == 1:
            self.Contact_PxF[ixF, 1:3] = PxF[1:3] -np.array([delta_slip_x, delta_slip_y])
        return
    
    def compute_penetrations(self, ixF, PxF, VxF):
        deltas = np.zeros(3)
        deltas_dot = np.zeros(3)
        if self.InContact[ixF] == 1:
            deltas = PxF[1:4] - self.Contact_PxF[ixF, 1:4]
            deltas_dot = VxF[1:4]
        return deltas, deltas_dot


class HuntCrossleyHertz:
    def __init__(self, k, n, d):
        """
        Initialize the Hunt-Crossley Hertz contact model.

        Parameters:
        k (float): Stiffness coefficient 
        n (float): Nonlinearity exponent (usually 3/2 for Hertzian contact)
        d (float): Damping coefficient
        """
        self.k = k
        self.n = n
        self.d = d

    def compute_normal_force(self, delta, delta_dot):
        """
        Compute the normal contact force.

        Parameters:
        delta (float): Penetration depth (m)
        delta_dot (float): Penetration rate (m/s)

        Returns:
        float: Normal force (N)
        """
        if delta < 0:
            Fz = self.k * (abs(delta) ** self.n) * (1 - self.d * delta_dot)
        else:
            Fz = 0  # No force if there's no contact
        return Fz


class ViscoelasticCoulombModel:
    def __init__(self, mu, k, d):
        """
        Initialize the Viscoelastic Coulomb friction model.

        Parameters:
        mu (float): Coefficient of friction
        k (float): Stiffness coefficient
        d (float): Damping coefficient
        """
        self.mu = mu
        self.k = k
        self.d = d

    def compute_tangential_force(self, F_n, delta_x, delta_x_dot, delta_y, delta_y_dot):
        """
        Compute the tangential friction force.

        Parameters:
        F_n (float): Normal force (N)
        delta_x, delta_y (float): Tangential displacements (m)
        delta_x_dot, delta_y_dot (float): Tangential velocities (m/s)

        Returns:
        tuple: (Fx, Fy, delta_no_slip_x, delta_no_slip_y, slip)
        """
        # Compute raw viscoelastic forces
        Fx = -self.k * delta_x - self.d * delta_x_dot
        Fy = -self.k * delta_y - self.d * delta_y_dot
        print(Fy)

        # Compute total force magnitude
        F_total = np.sqrt(Fx**2 + Fy**2)
        print(F_total)
        F_max = self.mu * F_n  # Maximum allowable force
        print(F_max)
        
        delta_no_slip_x, delta_no_slip_y = delta_x, delta_y
        slip = 0

        # Apply saturation
        if F_total > max(F_max,0):
            scaling_factor = F_max / F_total
            Fx *= scaling_factor
            Fy *= scaling_factor
            delta_no_slip_x = -(Fx + self.d * delta_x_dot) / self.k 
            delta_no_slip_y = -(Fy + self.d * delta_y_dot) / self.k
            slip = 1
          
        return Fx, Fy, delta_no_slip_x, delta_no_slip_y, slip
    
    def compute_normal_force(self, delta, delta_dot):
        """ Compute normal force using a viscoelastic model. """
        if(delta <= 0):
            Fz = -self.k * delta - self.d * delta_dot
        else:
            Fz = 0
        return Fz


# Instantiate models
hch_model = HuntCrossleyHertz(k=5e4, n=1.5, d=0.1)
vc_model = ViscoelasticCoulombModel(mu=0.8, k=2e4, d=100.0)

# Instantiate a ContactManager
cm = Contact_Manager(nSensors=2)


def user_ExtForces(PxF, VxF, ixF, Model):
    """Compute user-defined external forces."""
    Fx, Fy, Fz = 0.0, 0.0, 0.0
    # Update contact information
    cm.update_contact(ixF, PxF)

    # Compute penetrations
    deltas, deltas_dot = cm.compute_penetrations(ixF, PxF, VxF)
    print("deltas")
    print(deltas)
    print(deltas_dot)
    # Normal force computation
    # 0: Hunt-Crossley-Hertz, 1: Viscoelastic Coulomb
    if Model == 0:
        Fz = hch_model.compute_normal_force(deltas[2], deltas_dot[2])
    elif Model == 1:
        Fz = vc_model.compute_normal_force(deltas[2], deltas_dot[2])

    # Compute tangential forces
    print(Fz)
    Fx, Fy, delta_slip_x, delta_slip_y, slip = vc_model.compute_tangential_force(
        Fz, deltas[0], deltas_dot[0], deltas[1], deltas_dot[1]
    )

    # Update slip contact
    cm.update_slip_contact(ixF, PxF, delta_slip_x, delta_slip_y, slip)

    return Fx, Fy, Fz

def test_force_vs_penetration(Model):
    print("test z penetration")
    """ 
    Test and plot force along z for different penetration velocities using user_ExtForces.
    """
    z_values = np.linspace(0.01, -0.01, 100)  # Penetration depth from 0.001 to -0.01
    velocities = [0.05, -0.01 , 0, 0.01, 0.05]  # Different penetration speeds

    plt.figure(figsize=(8, 6))

    for v in velocities:
        Fz_values = []
        for z in z_values:
            # Create PxF and VxF vectors
            PxF = np.array([0, 0, 0, z])  # Position with penetration depth z
            VxF = np.array([0, 0, 0, v])  # Velocity with penetration rate v
            
            # Compute force using user_ExtForces
            Fx, Fy, Fz = user_ExtForces(PxF, VxF, 1,Model)
            
            Fz_values.append(Fz)

        plt.plot(z_values, Fz_values, label=f'V = {v} m/s')

    plt.xlabel("Penetration depth z (m)")
    plt.ylabel("Normal Force Fz (N)")
    plt.title("Vertical Penetration test")
    plt.legend()
    plt.grid()
    plt.show()
    return


def test_force_vs_xy(Model):
    """ 
    Test and plot force along y for a fixed penetration depth z = -0.01, 
    spanning x and y in [-0.01, 0.01] for different velocities.
    """
    velocities = [-0.05, 0, 0.05]  # Different penetration speeds
    fixed_z = -0.01  # Fixed penetration depth

    # Define the spans
    spans = {
        "y (0 to 0.01)": np.linspace(0, 0.01, 100)}
    """
        "y (0 to -0.01)": np.linspace(0, -0.01, 100),
        "x (0 to 0.01)": np.linspace(0, 0.01, 100),
        "x (0 to -0.01)": np.linspace(0, -0.01, 100)
    }"""

    for i, (label, values) in enumerate(spans.items()):
        plt.figure(figsize=(8, 6))
        
        for v in velocities:
            F_values = []
            CP_values = []
            cm.Contact_PxF[1] = np.zeros(4)
            for val in values:
                if "y" in label:
                    PxF = np.array([0, 0, val, fixed_z])  # Modify y
                    VxF = np.array([0, 0, v, 0])  # Velocity with penetration rate v
                    Fx, Fy, Fz = user_ExtForces(PxF, VxF, 1,Model)
                    F_values.append(Fy)
                    CP_values.append(cm.Contact_PxF[1,2])
                    
                else:
                    PxF = np.array([0, val, 0, fixed_z])  # Modify x
                    VxF = np.array([0, v, 0, 0])  # Velocity with penetration rate v
                    Fx, Fy, Fz = user_ExtForces(PxF, VxF, 1,Model)
                    F_values.append(Fx)
                
                
            print(CP_values)
            plt.plot(values,np.array(CP_values)*10000)
            plt.plot(values, F_values, label=f'V = {v} m/s')
            

        plt.xlabel(f"{label} displacement (m)")
        plt.ylabel("Tangential Force Fy (N)")
        plt.title(f"Tangential Force vs. {label} for Different Velocities")
        plt.legend()
        plt.grid()
        plt.show()

# Run tests
test_force_vs_xy(0)
test_force_vs_xy(1)
test_force_vs_penetration(0)
test_force_vs_penetration(1)