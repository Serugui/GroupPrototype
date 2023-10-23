using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;


public class DriveLocomotion : MonoBehaviour
{
    InputAction accelerate, brakePedal, turn;

    CharacterController characterController;
    public Transform cameraContainer;

    public float maxSpeed = 10f;
    float speed = 0f;
    float accelerationMutliplier = 0.2f;
    float defaultDrag = 0.01f;
    float breakDrag = 0.1f;
    float drag = 0.2f;

    public float mouseSensitivity = 0.2f;
    public float gravity = 20.0f;
    public float lookUpClamp = -5f;
    public float lookDownClamp = 20f;

    Vector3 moveDirection = Vector3.zero;
    float rotateYaw, rotatePitch;

    void Start()
    {
        Cursor.visible = false;
        characterController = GetComponent<CharacterController>();

        GameManager.InputManager.inputActions.Drive.Accelerate.Enable();
        GameManager.InputManager.inputActions.Drive.BrakePedal.Enable();
        GameManager.InputManager.inputActions.Drive.Turn.Enable();
    }

    void Update()
    {
        RotateAndLook();
    }

    void FixedUpdate()
    {
        Debug.Log("Turn: " + turn.ReadValue<Vector2>().x);
        Debug.Log("Accelerate: " + accelerate.ReadValue<float>());
        Debug.Log("Break Pedal: " + brakePedal.ReadValue<float>());

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
        if (characterController.isGrounded) // When grounded, set y-axis to zero (to ignore it)
        {
            float acceleration = accelerate.ReadValue<float>();
            float breaking = brakePedal.ReadValue<float>();
            float turning = turn.ReadValue<Vector2>().x;

            drag = 1 - defaultDrag - (breakDrag * breaking);

            speed += acceleration * accelerationMutliplier;
            speed *= drag;

            if (speed <= 0.1)
            {
                speed = 0;
            }
            else if (speed >= maxSpeed)
            {
                speed = maxSpeed;
            }

            moveDirection = new Vector3(0f, 0f, speed);
            moveDirection = transform.TransformDirection(moveDirection);

            turning *= speed;
            turning = Mathf.Clamp(turning, -5f, +5f);
            transform.Rotate(0f, turning, 0f);
        }

        moveDirection.y -= gravity * Time.deltaTime;
        characterController.Move(moveDirection * Time.deltaTime);
    }

    void RotateAndLook()
    {
        //Vector2 lookInput = look.ReadValue<Vector2>();

        //rotateYaw = lookInput.x * mouseSensitivity;
        //rotateYaw += cameraContainer.transform.localRotation.eulerAngles.y;

        //rotatePitch -= lookInput.y * mouseSensitivity;
        //rotatePitch = Mathf.Clamp(rotatePitch, lookUpClamp, lookDownClamp);

        //cameraContainer.transform.localRotation = Quaternion.Euler(rotatePitch, rotateYaw, 0f);
    }
}
