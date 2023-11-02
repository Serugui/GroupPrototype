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
    public int maxReverseSpeed = 45; //The maximum speed that the car can reach while going on reverse in km/h.
    public int accelerationMultiplier = 2;
    public int maxSteeringAngle = 27;
    public float steeringSpeed = 0.5f; //Maybe not necessary with phisical steering wheel
    public int brakeForce = 350; //The strenkgth of the wheel brakes.
    public int decelerationMultiplier = 2; //How fast it decelerates
    public int handbrakeDriftMultiplier = 5; //How much grip the car loses when the user hit the handbrake.
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

    //Private variables

    Rigidbody carRigidbody; // Stores the car's rigidbody.
    float steeringAxis; // Used to know whether the steering wheel has reached the maximum value. It goes from -1 to 1.
    float throttleAxis; // Used to know whether the throttle has reached the maximum value. It goes from -1 to 1.
    float driftingAxis;
    float localVelocityZ;
    float localVelocityX;
    bool deceleratingCar;

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
        float braking = brakePedal.ReadValue<float>();

        HandleSteering();
        ThrottleOn();
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
        float acceleration = accelerate.ReadValue<float>();

        // Smoothly adjust the throttle power
        acceleration = Mathf.Clamp(acceleration + (Time.deltaTime * 3f), 0f, 1f);

        // If the car is going backwards, apply brakes to avoid strange behaviors.
        if (localVelocityZ < -1f)
        {
            // Apply brakes to stop the car from moving backward.
            frontLeftCollider.brakeTorque = brakeForce;
            frontRightCollider.brakeTorque = brakeForce;
            rearLeftCollider.brakeTorque = brakeForce;
            rearRightCollider.brakeTorque = brakeForce;
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
}



