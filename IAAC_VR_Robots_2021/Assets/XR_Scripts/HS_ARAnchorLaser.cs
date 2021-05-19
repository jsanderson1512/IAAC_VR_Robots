using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class HS_ARAnchorLaser : MonoBehaviour
{
    public GameObject spherePointPrefab;
    private GameObject spherePoint1Origin;
    private GameObject spherePoint2xaxis;
    private GameObject spherePoint3yaxis;
    private Matrix4x4 basePlane;

    public GameObject markerlessObject;

    private bool setAnchor;
    private bool touch1origin;
    private bool touch2xaxis;
    private bool touch3yaxis;
    private Vector3 touch1;
    private Vector3 touch2;
    private Vector3 touch3;

    public GameObject SetAnchorButtonOn;
    public GameObject myCam;
    public GameObject SuggestionText;

    void Start()
    {
        Screen.sleepTimeout = SleepTimeout.NeverSleep;

        markerlessObject.SetActive(false);

        spherePoint1Origin = Instantiate(spherePointPrefab);
        spherePoint1Origin.GetComponent<MeshRenderer>().material.color = Color.white;
        spherePoint2xaxis = Instantiate(spherePointPrefab);
        spherePoint2xaxis.GetComponent<MeshRenderer>().material.color = Color.red;
        spherePoint3yaxis = Instantiate(spherePointPrefab);
        spherePoint3yaxis.GetComponent<MeshRenderer>().material.color = Color.green;
        spherePoint1Origin.SetActive(false);
        spherePoint2xaxis.SetActive(false);
        spherePoint3yaxis.SetActive(false);
        SuggestionText.SetActive(false);
    }

    void Update()
    {
        if (setAnchor)
        {
            Vector3 origin = myCam.transform.position;
            Vector3 direction = myCam.transform.TransformDirection(Vector3.forward);

            if (touch1origin)
            {
                RaycastHit hit;
                if (Physics.Raycast(origin, direction, out hit, 3.0f)) //send a raycast out 3 meters
                {
                    touch1 = hit.point;
                    spherePoint1Origin.SetActive(true);
                    spherePoint1Origin.transform.position = touch1;
                    SuggestionText.SetActive(true);
                    SuggestionText.GetComponent<Text>().text = "TAP SCREEN TO SET \"ORIGIN POSITION\" ON THE GROUND";
                    if (Input.touchCount > 0)
                    {
                        var touch = Input.GetTouch(0);
                        if (touch.phase == TouchPhase.Began)
                        {
                            touch1origin = false;
                            touch2xaxis = true;
                            touch3yaxis = false;
                        }
                    }
                }
                else
                {
                    spherePoint1Origin.SetActive(false);
                    SuggestionText.SetActive(true);
                    SuggestionText.GetComponent<Text>().text = "SLOWLY LOOK AROUND A FLAT, OPEN AREA WITH YOUR CAMERA";
                }
            }
            else if (touch2xaxis)
            {
                RaycastHit hit;
                if (Physics.Raycast(origin, direction, out hit, 3.0f)) //send a raycast out 3 meters
                {
                    touch2 = hit.point;
                    spherePoint2xaxis.SetActive(true);
                    spherePoint2xaxis.transform.position = touch2;
                    SuggestionText.SetActive(true);
                    SuggestionText.GetComponent<Text>().text = "TAP SCREEN TO SET \"FORWARD DIRECTION\" ON THE GROUND";
                    if (Input.touchCount > 0)
                    {
                        var touch = Input.GetTouch(0);
                        if (touch.phase == TouchPhase.Began)
                        {
                            touch1origin = false;
                            touch2xaxis = false;
                            touch3yaxis = true;
                        }
                    }
                }
                else
                {
                    spherePoint2xaxis.SetActive(false);
                    SuggestionText.SetActive(true);
                    SuggestionText.GetComponent<Text>().text = "SLOWLY LOOK AROUND A FLAT, OPEN AREA WITH YOUR CAMERA";
                }
            }
            else if (touch3yaxis)
            {
                spherePoint3yaxis.SetActive(true);
                spherePoint3yaxis.transform.position = touch2;
                spherePoint3yaxis.transform.RotateAround(touch1, Vector3.up, -90.0f);
                touch3 = spherePoint3yaxis.transform.position;

                Vector3 p1 = touch1;
                Vector3 p2 = touch2;
                Vector3 p3 = touch3;
                Vector3 xAxis = (p1 - p2).normalized; //x axis is reversed from what we tapped.
                Vector3 zAxis = (p1 - p3).normalized; //z axis is reversed from what we tapped as the y.
                Vector3 yAxis = Vector3.Cross(zAxis, xAxis); //the y axis is the cross product of these other dudes
                zAxis = Vector3.Cross(xAxis, yAxis); //now get our corrected z axis

                basePlane = Matrix4x4.identity;
                //location
                basePlane.m03 = p1.x;
                basePlane.m13 = p1.y;
                basePlane.m23 = p1.z;
                //x direction
                basePlane.m00 = xAxis.x;
                basePlane.m10 = xAxis.y;
                basePlane.m20 = xAxis.z;
                //y direction
                basePlane.m01 = yAxis.x;
                basePlane.m11 = yAxis.y;
                basePlane.m21 = yAxis.z;
                //z direction
                basePlane.m02 = zAxis.x;
                basePlane.m12 = zAxis.y;
                basePlane.m22 = zAxis.z;

                Vector3 robPlaneCenter = new Vector3(basePlane.m03, basePlane.m13, basePlane.m23);



                markerlessObject.SetActive(true);
                markerlessObject.transform.position = robPlaneCenter;
                markerlessObject.transform.rotation = basePlane.rotation;

                setAnchor = false;
                touch1origin = false;
                touch2xaxis = false;
                touch3yaxis = false;
                SetAnchorButtonOn.SetActive(false);
                SuggestionText.SetActive(false);
            }
        }
    }

    public void StartAnchorSet()
    {
        setAnchor = !setAnchor;

        if (setAnchor)
        {





            SetAnchorButtonOn.SetActive(true);
            touch1origin = true;
            touch2xaxis = false;
            touch3yaxis = false;

            spherePoint1Origin.SetActive(false);
            spherePoint2xaxis.SetActive(false);
            spherePoint3yaxis.SetActive(false);
        }
        else
        {
            StopAnchorSet();
        }
        
    }

    public void StopAnchorSet()
    {
        SetAnchorButtonOn.SetActive(false);

        setAnchor = false;
        touch1origin = false;
        touch2xaxis = false;
        touch3yaxis = false;

        spherePoint1Origin.SetActive(false);
        spherePoint2xaxis.SetActive(false);
        spherePoint3yaxis.SetActive(false);
        SuggestionText.SetActive(false);
    }
}