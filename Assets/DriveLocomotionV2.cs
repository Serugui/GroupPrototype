using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem;


public class DriveLocomotionV2 : MonoBehaviour
{
    InputAction accelerate, brakePedal, turn;

    //Car SETUP

    public int maxSpeed = 120;//The maximum speed that the car can reach in km/h.
    public int accelerationMultiplier = 6;
    public int maxSteeringAngle = 30;
    //public float steeringSpeed = 0.5f; //Maybe not necessary with phisical steering wheel
    public int brakeForce = 300; //The strenkgth of the wheel brakes.
    public float decelerationMultiplier = 0.010f; //How fast it decelerates
    public float brakeDriftMultiplier = 0.5f; //How much grip the car loses when the user hit the handbrake.
    public Vector3 bodyMassCenter; //Contains the center of mass of the car.

    //Car Data

    [HideInInspector]
    public float carSpeed; //Used to store the speed of the car.
    [HideInInspector]
    public bool isDrifting; //Used to know whether the car is drifting or not.
    [HideInInspector]
    public bool isTractionLocked; //Used to know whether the traction of the car is locked or not.

    //Wheels

    public GameObject frontLeftMesh;
    public WheelCollider frontLeftCollider;
    [Space(10)]
    public GameObject frontRightMesh;
    public WheelCollider frontRightCollider;
    [Space(10)]
    public GameObject rearLeftMesh;
    public WheelCollider rearLeftCollider;
    [Space(10)]
    public GameObject rearRightMesh;
    public WheelCollider rearRightCollider;


    // The following particle systems are used as tire smoke when the car drifts.
    public ParticleSystem RLWParticleSystem;
    public ParticleSystem RRWParticleSystem;

    [Space(10)]
    // The following trail renderers are used as tire skids when the car loses traction.
    public TrailRenderer RLWTireSkid;
    public TrailRenderer RRWTireSkid;

    //Private variables

    Rigidbody carRigidbody; // Stores the car's rigidbody.
    float driftingAxis;
    float localVelocityZ;
    float localVelocityX;


    //Start drifting values
    WheelFrictionCurve FLwheelFriction;
    float FLWextremumSlip;
    WheelFrictionCurve FRwheelFriction;
    float FRWextremumSlip;
    WheelFrictionCurve RLwheelFriction;
    float RLWextremumSlip;
    WheelFrictionCurve RRwheelFriction;
    float RRWextremumSlip;

    void Start()
    {
        Cursor.visible = false;
        carRigidbody = gameObject.GetComponent<Rigidbody>();
        carRigidbody.centerOfMass = bodyMassCenter;

        GameManager.InputManager.inputActions.Drive.Accelerate.Enable();
        GameManager.InputManager.inputActions.Drive.BrakePedal.Enable();
        GameManager.InputManager.inputActions.Drive.Turn.Enable();

        //Initial setup to calculate the drift value of the car
        FLwheelFriction = new WheelFrictionCurve();
        FLwheelFriction.extremumSlip = frontLeftCollider.sidewaysFriction.extremumSlip;
        FLWextremumSlip = frontLeftCollider.sidewaysFriction.extremumSlip;
        FLwheelFriction.extremumValue = frontLeftCollider.sidewaysFriction.extremumValue;
        FLwheelFriction.asymptoteSlip = frontLeftCollider.sidewaysFriction.asymptoteSlip;
        FLwheelFriction.asymptoteValue = frontLeftCollider.sidewaysFriction.asymptoteValue;
        FLwheelFriction.stiffness = frontLeftCollider.sidewaysFriction.stiffness;
        FRwheelFriction = new WheelFrictionCurve();
        FRwheelFriction.extremumSlip = frontRightCollider.sidewaysFriction.extremumSlip;
        FRWextremumSlip = frontRightCollider.sidewaysFriction.extremumSlip;
        FRwheelFriction.extremumValue = frontRightCollider.sidewaysFriction.extremumValue;
        FRwheelFriction.asymptoteSlip = frontRightCollider.sidewaysFriction.asymptoteSlip;
        FRwheelFriction.asymptoteValue = frontRightCollider.sidewaysFriction.asymptoteValue;
        FRwheelFriction.stiffness = frontRightCollider.sidewaysFriction.stiffness;
        RLwheelFriction = new WheelFrictionCurve();
        RLwheelFriction.extremumSlip = rearLeftCollider.sidewaysFriction.extremumSlip;
        RLWextremumSlip = rearLeftCollider.sidewaysFriction.extremumSlip;
        RLwheelFriction.extremumValue = rearLeftCollider.sidewaysFriction.extremumValue;
        RLwheelFriction.asymptoteSlip = rearLeftCollider.sidewaysFriction.asymptoteSlip;
        RLwheelFriction.asymptoteValue = rearLeftCollider.sidewaysFriction.asymptoteValue;
        RLwheelFriction.stiffness = rearLeftCollider.sidewaysFriction.stiffness;
        RRwheelFriction = new WheelFrictionCurve();
        RRwheelFriction.extremumSlip = rearRightCollider.sidewaysFriction.extremumSlip;
        RRWextremumSlip = rearRightCollider.sidewaysFriction.extremumSlip;
        RRwheelFriction.extremumValue = rearRightCollider.sidewaysFriction.extremumValue;
        RRwheelFriction.asymptoteSlip = rearRightCollider.sidewaysFriction.asymptoteSlip;
        RRwheelFriction.asymptoteValue = rearRightCollider.sidewaysFriction.asymptoteValue;
        RRwheelFriction.stiffness = rearRightCollider.sidewaysFriction.stiffness;
    }

    private void FixedUpdate()
    {
        // We determine the speed of the car.
        carSpeed = (2 * Mathf.PI * frontLeftCollider.radius * frontLeftCollider.rpm * 60) / 1000;
        // Save the local velocity of the car in the x axis. Used to know if the car is drifting.
        localVelocityX = transform.InverseTransformDirection(carRigidbody.velocity).x;
        // Save the local velocity of the car in the z axis. Used to know if the car is going forward or backwards.
        localVelocityZ = transform.InverseTransformDirection(carRigidbody.velocity).z;

        Locomotion();
    }

    private void OnEnable()
    {
        turn = GameManager.InputManager.inputActions.Drive.Turn;
        turn.Enable();

        accelerate = GameManager.InputManager.inputActions.Drive.Accelerate;
        accelerate.Enable();

        brakePedal = GameManager.InputManager.inputActions.Drive.BrakePedal;
        brakePedal.Enable();
    }

    private void OnDisable()
    {
        turn.Disable();
        accelerate.Disable();
        brakePedal.Disable();
    }

    void Locomotion()
    {
        
        HandleSteering();
        ThrottleOn();
        Breaks();
        DecelerateCar();
        AnimateWheelMeshes();

    }

    void HandleSteering()
    {
        float turning = turn.ReadValue<Vector2>().x;

        // Apply steering angle to your wheel colliders
        var steeringAngle = turning * maxSteeringAngle;

        frontLeftCollider.steerAngle = steeringAngle;
        frontRightCollider.steerAngle = steeringAngle;
    }

    void ThrottleOn()
    {
       
         DriftCarPS();
        

        float acceleration = accelerate.ReadValue<float>();


        // If the car is going backwards, apply brakes to avoid strange behaviors or if is not accelerating and slow, stop.
        if (localVelocityZ < -1f)
        {
            // Apply brakes to stop the car from moving backward.
            Breaks();
            
        }
        else
        {
            if (Mathf.RoundToInt(carSpeed) < maxSpeed)
            {
                // Apply positive torque to go forward if maxSpeed has not been reached.
                float motorTorque = (accelerationMultiplier * 50f) * acceleration;

                // Apply torque to all wheels
                frontLeftCollider.brakeTorque = 0;
                frontLeftCollider.motorTorque = motorTorque;
                frontRightCollider.brakeTorque = 0;
                frontRightCollider.motorTorque = motorTorque;
                rearLeftCollider.brakeTorque = 0;
                rearLeftCollider.motorTorque = motorTorque;
                rearRightCollider.brakeTorque = 0;
                rearRightCollider.motorTorque = motorTorque;
            }
            else
            {
                // If the maxSpeed has been reached, stop applying torque to the wheels.
                frontLeftCollider.motorTorque = 0;
                frontRightCollider.motorTorque = 0;
                rearLeftCollider.motorTorque = 0;
                rearRightCollider.motorTorque = 0;
            }
        }
    }

    void Breaks()
    {
        float braking = brakePedal.ReadValue<float>();
        float actualBrakeForce = brakeForce * braking; // Multiplicar el valor del pedal por la fuerza de frenado máxima

        if (braking > 0f)
        {
            isTractionLocked = true;
            frontLeftCollider.brakeTorque = actualBrakeForce;
            frontRightCollider.brakeTorque = actualBrakeForce;
            rearLeftCollider.brakeTorque = actualBrakeForce;
            rearRightCollider.brakeTorque = actualBrakeForce;

            if (isDrifting == true)
            {
                Vector3 driftForce = carRigidbody.velocity.normalized * brakeDriftMultiplier;
                carRigidbody.AddForce(driftForce);
            }

        }
        else
        {
            isTractionLocked = false;

        }

        
    }

    // Nitro
    private void OnTriggerEnter(Collider other)
    {
        if (other.tag == "Nitro")
        {

            StartCoroutine(Nitro());


        }
    }

    IEnumerator Nitro()
    {

        print(Time.time);
        yield return new WaitForSeconds(0);
        print(Time.time);
        maxSpeed = maxSpeed * 2;
        accelerationMultiplier = accelerationMultiplier * 2;
        yield return new WaitForSeconds(5);
        maxSpeed = maxSpeed/2;
        accelerationMultiplier = accelerationMultiplier / 2;


    }


    public void DecelerateCar()
    {
        float acceleration = accelerate.ReadValue<float>();
        if (acceleration == 0)
        {
            carRigidbody.velocity *= 1f / (1f + decelerationMultiplier);
            // Since we want to decelerate the car, we are going to remove the torque from the wheels of the car.
            frontLeftCollider.motorTorque = 0;
            frontRightCollider.motorTorque = 0;
            rearLeftCollider.motorTorque = 0;
            rearRightCollider.motorTorque = 0;
            // If the magnitude of the car's velocity is less than 0.25f (very slow velocity), then stop the car completely and
            // also cancel the invoke of this method.
            if (carRigidbody.velocity.magnitude < 1.5f)
            {
                carRigidbody.velocity = Vector3.zero;
            }
        }
    }



    public void DriftCarPS()
    {
        if (Mathf.Abs(localVelocityX) > 2f)
        {
            isDrifting = true;
        }
        else
        {
            isDrifting = false;
        }

        if (isDrifting)
        {
            RLWParticleSystem.Play();
            RRWParticleSystem.Play();
        }
        else if (!isDrifting)
        {
            RLWParticleSystem.Stop();
            RRWParticleSystem.Stop();
        }



        if ((isTractionLocked || Mathf.Abs(localVelocityX) > 2f) && Mathf.Abs(carSpeed) > 30f)
        {
            RLWTireSkid.emitting = true;
            RRWTireSkid.emitting = true;
        }
        else
        {
            RLWTireSkid.emitting = false;
            RRWTireSkid.emitting = false;
        }


    }

    void AnimateWheelMeshes()
    {
       
            Quaternion FLWRotation;
            Vector3 FLWPosition;
            frontLeftCollider.GetWorldPose(out FLWPosition, out FLWRotation);
            frontLeftMesh.transform.position = FLWPosition;
            frontLeftMesh.transform.rotation = FLWRotation;

            Quaternion FRWRotation;
            Vector3 FRWPosition;
            frontRightCollider.GetWorldPose(out FRWPosition, out FRWRotation);
            frontRightMesh.transform.position = FRWPosition;
            frontRightMesh.transform.rotation = FRWRotation;

            Quaternion RLWRotation;
            Vector3 RLWPosition;
            rearLeftCollider.GetWorldPose(out RLWPosition, out RLWRotation);
            rearLeftMesh.transform.position = RLWPosition;
            rearLeftMesh.transform.rotation = RLWRotation;

            Quaternion RRWRotation;
            Vector3 RRWPosition;
            rearRightCollider.GetWorldPose(out RRWPosition, out RRWRotation);
            rearRightMesh.transform.position = RRWPosition;
            rearRightMesh.transform.rotation = RRWRotation;
        
        
    }
}



