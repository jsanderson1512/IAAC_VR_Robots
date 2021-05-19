using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.ARFoundation;
using UnityEngine.XR.ARSubsystems;
using UnityEngine.UI;

public class HS_ARAnchorLaserSimple : MonoBehaviour
{
    public GameObject placementLocationMarker;
    public GameObject ARObject;
    private ARRaycastManager raycastManager;
    private bool setAnchor;
    public GameObject SuggestionText;


    // Start is called before the first frame update

    private void Start()
    {
        //get the components
        raycastManager = FindObjectOfType<ARRaycastManager>();
        placementLocationMarker = Instantiate(placementLocationMarker);
        placementLocationMarker.SetActive(false);
        ARObject.SetActive(false);
        SuggestionText.SetActive(false);


        gameObject.GetComponent<ARPlaneManager>().enabled = false;
        gameObject.GetComponent<ARPointCloudManager>().enabled = false;

    }


    private void Update()
    {
        if (setAnchor)
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
                SuggestionText.GetComponent<Text>().text = "TAP SCREEN TO SET \"AR ORIGIN\" ON THE GROUND";

                placementLocationMarker.transform.position = hits[0].pose.position;
                placementLocationMarker.transform.rotation = hits[0].pose.rotation;

                //make the visual plane visible if not already
                if (!placementLocationMarker.activeInHierarchy)
                {
                    placementLocationMarker.SetActive(true);
                }

                if (Input.touchCount > 0 && Input.touches[0].phase == TouchPhase.Began)
                {
                    ARObject.transform.position = hits[0].pose.position;
                    ARObject.transform.rotation = hits[0].pose.rotation;
                    ARObject.SetActive(true);
                    placementLocationMarker.SetActive(false);
                    setAnchor = false;
                    SuggestionText.SetActive(false);

                    gameObject.GetComponent<ARPlaneManager>().enabled = false;
                    gameObject.GetComponent<ARPointCloudManager>().enabled = false;


                    GameObject[] arPlanes = GameObject.FindGameObjectsWithTag("AR_Plane");
                    foreach (GameObject g in arPlanes)
                    {
                        g.SetActive(false);
                    }

                }
            }
            else
            {
                SuggestionText.GetComponent<Text>().text = "SLOWLY LOOK AROUND A FLAT, OPEN AREA WITH YOUR CAMERA";

                if (!placementLocationMarker.activeInHierarchy)
                {
                    placementLocationMarker.SetActive(false);
                }
            }
        }

    }

    public void StartARAnchor()
    {
        ARObject.SetActive(false);
        SuggestionText.SetActive(true);
        setAnchor = true;

        gameObject.GetComponent<ARPlaneManager>().enabled = true;
        gameObject.GetComponent<ARPointCloudManager>().enabled = true;
    }
}
