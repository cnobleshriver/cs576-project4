﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static TMPro.SpriteAssetUtilities.TexturePacker_JsonArray;

public class Pendulum : MonoBehaviour {
    // parameters of ths simulation (are specified through the Unity's interface)
    public float gravity_acceleration = 9.81f;
    public float mass = 1.0f;
    public float friction_coeficient = 0.0f;
    public float initial_angular_velocity = 0.0f;
    public float time_step_h = 0.05f;
    public string ode_method = "euler";

    // parameters that will be populated automatically from the geometry/components of the scene
    private float rod_length = 0.0f;
    private float c = 0.0f;
    private float omega_sqr = 0.0f;
    private GameObject pendulum = null;

    // the state vector stores two entries:
    // state[0] is the angle of pendulum (\theta) in radians
    // state[1] is the angular velocity of pendulum
    private Vector2 state;

    // Use this for initialization
    void Start ()
    {
        Time.fixedDeltaTime = time_step_h;      // set the simulation step - FixedUpdate is called every 'time_step_h' seconds 
        state = new Vector2(0.0f, 0.0f); // initialization of the state vector
        pendulum = GameObject.Find("Pendulum");
        if (pendulum == null)
        {
            Debug.LogError("Sphere not found! Did you delete it from the starter scene?");
        }
        GameObject rod = GameObject.Find("Quad");
        if (rod == null)
        {
            Debug.LogError("Rod not found! Did you delete it from the starter scene?");
        }
        rod_length = rod.transform.localScale.y; // finds rod length (based on quad's scale)
        
        state[0] = pendulum.transform.eulerAngles.z * Mathf.Deg2Rad; // initial angle is set from the starter scene
        state[1] = initial_angular_velocity; 

        c = friction_coeficient / mass;        // following the ODE specification
        omega_sqr = gravity_acceleration / rod_length;
    }

    // Update is called once per Time.fixedDeltaTime  sec
    void FixedUpdate ()
    {
        // complete this function (measure kinetic, potential, total energy)
        float kinetic_energy = 0.0f;    // change here
        float potential_energy = 0.0f;  // change here
        Debug.Log(kinetic_energy + potential_energy);

        OdeStep();
        pendulum.transform.eulerAngles = new Vector3(0.0f, 0.0f,  state[0] * Mathf.Rad2Deg  );
    }

    // complete this function!!!
   // PendulumDynamics that takes as input the angle and velocity (i.e., this is the state vector), and should return the derivatives for both (i.e., the derivative of the state vector). 
    // You should use this function in OdeStep!
    Vector2 PendulumDynamics(Vector2 input_state)
    {
        // change here (hint: use the variable input_state here, not the "state" variable defined above)
        return new Vector2(0.0f, 0.0f);
    }

    void OdeStep()
    {
        if (ode_method == "euler")
        {
			// think of the following line as: 
			// new_state = current_state +  step * derivative[current_state] 
			// Your derivative[ current_state ] is given by your PendulumDynamics function that takes as input the current state (angle, velocity) and returns the derivative of the input current state vector.
			state += time_step_h * PendulumDynamics(state);			
        }
        else if (ode_method == "trapezoidal")
        {
			// think of the following lines as:
			// new_state = current_state +  (step/2) * (derivative[current_state] + derivative[estimated_new_state])
			// where: estimated_new_state = current_state +  step * derivative[current_state]
            Vector2 derivative_current_state = PendulumDynamics(state);
            Vector2 derivative_estimated_new_state = PendulumDynamics(state + time_step_h * derivative_current_state);
            state += (time_step_h / 2.0f) * (derivative_current_state + derivative_estimated_new_state);			
        }
        else if (ode_method == "rk")
        {
			// implement the Runge-Kutta variant (RK4)
        }
        else if (ode_method == "semi-implicit")
        {
			// implement the symplectic method (also known as semi-implicit Euler method)
        }
        else
        {
            Debug.LogError("ODE method should be one of the: euler, trapezoidal, rk, semi-implicit");
        }
    }
}