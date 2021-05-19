using UnityEngine;

public class HS_FlythroughCamera : MonoBehaviour
{
    private float mainSpeed = 3.0f; //regular speed
    private float shiftAdd = 6.0f; //multiplied by how long shift is held.  Basically running
    private float camSens = 0.25f; //How sensitive it with mouse
    private Vector3 lastMouse = new Vector3(255, 255, 255); //kind of in the middle of the screen, rather than at the top (play)
    private Vector3 unClickedRotation;

    private void Start()
    {
        unClickedRotation = transform.eulerAngles;
    }
    void Update()
    {
        //if(Input.GetMouseButton(0) || Input.GetMouseButton(1))
        if (Input.GetMouseButton(1) || Input.GetMouseButton(2)) // right and middle click only 
        {
            lastMouse = Input.mousePosition - lastMouse;
            lastMouse = new Vector3(-lastMouse.y * camSens, lastMouse.x * camSens, 0);
            lastMouse = new Vector3(transform.eulerAngles.x + lastMouse.x, transform.eulerAngles.y + lastMouse.y, 0);
            transform.eulerAngles = lastMouse;
            unClickedRotation = lastMouse;
        }
        else
        {
            transform.eulerAngles = unClickedRotation;
        }
        lastMouse = Input.mousePosition;

        Vector3 p = GetBaseInput();

        if (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.Space))
        {
            p = p * shiftAdd;
        }
        else
        {
            p = p * mainSpeed;
        }

        p = p * Time.deltaTime;
        Vector3 newPosition = transform.position;

        transform.Translate(p);

        if (Input.GetKey(KeyCode.Q) || Input.GetKey(KeyCode.E))
        {
            newPosition.y = transform.position.y;
        }
        else
        {
            newPosition.x = transform.position.x;
            newPosition.z = transform.position.z;
        }
        
        transform.position = newPosition;

    }

    private Vector3 GetBaseInput()
    { //returns the basic values, if it's 0 than it's not active.
        Vector3 p_Velocity = new Vector3();
        if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow))
        {
            p_Velocity += new Vector3(0, 0, 1);
        }
        if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow))
        {
            p_Velocity += new Vector3(0, 0, -1);
        }
        if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow))
        {
            p_Velocity += new Vector3(-1, 0, 0);
        }
        if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow))
        {
            p_Velocity += new Vector3(1, 0, 0);
        }
        if (Input.GetKey(KeyCode.E))
        {
            p_Velocity += new Vector3(0, 1, 0);
        }
        if (Input.GetKey(KeyCode.Q))
        {
            p_Velocity += new Vector3(0, -1, 0);
        }
        return p_Velocity;
    }
}