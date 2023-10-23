using System;
using UnityEngine;
using UnityEngine.InputSystem;

public class InputManager : MonoBehaviour
{
    public SteeringControls inputActions;
    public event Action<InputActionMap> actionMapChange;

    private void Awake()
    {
        inputActions = new SteeringControls();
    }

    void Start()
    {
        ToggleActionMap(inputActions.Drive);
    }

    public void ToggleActionMap(InputActionMap actionMap)
    {
        if (actionMap.enabled)
        {
            return;
        }

        inputActions.Disable();
        actionMapChange?.Invoke(actionMap);
        actionMap.Enable();
    }
}
