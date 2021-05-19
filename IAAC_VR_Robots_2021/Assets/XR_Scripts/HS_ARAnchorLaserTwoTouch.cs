using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.ARFoundation;
using UnityEngine.XR.ARSubsystems;
using UnityEngine.UI;


[RequireComponent(typeof(ARRaycastManager))]
[RequireComponent(typeof(ARPlaneManager))]
[RequireComponent(typeof(ARPointCloudManager))]

public class HS_ARAnchorLaserTwoTouch : MonoBehaviour
{

    public GameObject prefab_2DHitMarker;
    public GameObject prefab_3DArrowMarker;
    public GameObject SuggestionText;

    public GameObject ARObject;


    private ARRaycastManager raycastManager;
    private bool setAnchorStart;
    private bool firstAnchorRegistered;





    // Start is called before the first frame update
    void Start()
    {
        ARObject.SetActive(false);
        SuggestionText.SetActive(false);
        setAnchorStart = false;
        firstAnchorRegistered = false;

        prefab_2DHitMarker = Instantiate(prefab_2DHitMarker,Vector3.zero,Quaternion.identity);
        prefab_2DHitMarker.SetActive(false);
        prefab_3DArrowMarker = Instantiate(prefab_3DArrowMarker, Vector3.zero, Quaternion.identity);
        prefab_3DArrowMarker.SetActive(false);

        raycastManager = gameObject.GetComponent<ARRaycastManager>();

        gameObject.GetComponent<ARRaycastManager>().enabled = false;
        gameObject.GetComponent<ARPlaneManager>().enabled = false;
        gameObject.GetComponent<ARPointCloudManager>().enabled = false;
    }

    // Update is called once per frame
    void Update()
    {
        if (!setAnchorStart)
        {
            return;
        }
        else
        {
            // shoot a raycast from the center of the screen
            List<ARRaycastHit> hits = new List<ARRaycastHit>();
            //locate the center of the screen
            Vector2 screenCenter = new Vector2(Screen.width / 2, Screen.height / 2);
            //Cast a ray against trackables, in this case planes
            raycastManager.Raycast(screenCenter, hits, TrackableType.Planes);

            //if we hit an AR Plane then update the position and rotation of the visual plane to be the same as the hit transforms
            if (hits.Count > 0)
            {

                prefab_2DHitMarker.transform.position = hits[0].pose.position;
                prefab_2DHitMarker.transform.rotation = hits[0].pose.rotation;

                //make the visual plane visible if not already
                if (!prefab_2DHitMarker.activeInHierarchy)
                {
                    prefab_2DHitMarker.SetActive(true);
                }



                if(!firstAnchorRegistered)
                {
                    //THIS WILL HAVE AN 'UPDATE' BEFORE YOU TAP (NOTHING HAPPENS)
                    SuggestionText.GetComponent<Text>().text = "TAP SCREEN TO SET \"AR ORIGIN\" ON THE GROUND";

                    //TAP WILL PLACE THE ORIGIN
                    if (Input.touchCount > 0 && Input.touches[0].phase == TouchPhase.Began)
                    {

                        prefab_3DArrowMarker.transform.position = hits[0].pose.position;
                        prefab_3DArrowMarker.transform.rotation = hits[0].pose.rotation;
                        prefab_3DArrowMarker.SetActive(true);

                        firstAnchorRegistered = true;

                    }

                }
                else
                {
                    //THIS WILL HAVE AN 'UPDATE' BEFORE YOU TAP (THE UPDATE WILL ROTATE THE 3D MARKER)
                    SuggestionText.GetComponent<Text>().text = "TAP SCREEN TO SET \"AR DIRECTION\" ON THE GROUND";

                    //this will make the 3d marker "look at" the current position of the raycast
                    Vector3 targetPostition = new Vector3(hits[0].pose.position.x,prefab_3DArrowMarker.transform.position.y,hits[0].pose.position.z);
                    prefab_3DArrowMarker.transform.LookAt(targetPostition);


                    //TAP WILL PLACE THE ACTUAL ANCHOR
                    if (Input.touchCount > 0 && Input.touches[0].phase == TouchPhase.Began)
                    {

                        ARObject.transform.position = prefab_3DArrowMarker.transform.position;
                        ARObject.transform.rotation = prefab_3DArrowMarker.transform.rotation;
                        ARObject.SetActive(true);
                        prefab_2DHitMarker.SetActive(false);
                        prefab_3DArrowMarker.SetActive(false);
                        setAnchorStart = false;
                        firstAnchorRegistered = false;
                        SuggestionText.SetActive(false);

                        gameObject.GetComponent<ARPlaneManager>().enabled = false;
                        gameObject.GetComponent<ARPointCloudManager>().enabled = false;
                        gameObject.GetComponent<ARRaycastManager>().enabled = false;

                        GameObject[] arPlanes = GameObject.FindGameObjectsWithTag("AR_Plane");
                        foreach (GameObject g in arPlanes)
                        {
                            g.SetActive(false);
                        }
                    }
                }

            }
            else
            {
                SuggestionText.GetComponent<Text>().text = "SLOWLY LOOK AROUND A FLAT, OPEN AREA WITH YOUR CAMERA";

                if (!prefab_2DHitMarker.activeInHierarchy)
                {
                    prefab_2DHitMarker.SetActive(false);
                }
            }
        }
    }




    public void StartARAnchor()
    {
        ARObject.SetActive(false);
        SuggestionText.SetActive(true);
        setAnchorStart = true;
        firstAnchorRegistered = false;


        prefab_2DHitMarker.SetActive(false);
        prefab_3DArrowMarker.SetActive(false);


        gameObject.GetComponent<ARRaycastManager>().enabled = true;
        gameObject.GetComponent<ARPlaneManager>().enabled = true;
        gameObject.GetComponent<ARPointCloudManager>().enabled = true;
    }

}
