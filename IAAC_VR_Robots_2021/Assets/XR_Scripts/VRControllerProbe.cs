using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class VRControllerProbe : MonoBehaviour
{
    public Text suggestionText;
    public GameObject ProbePoint;
    public GameObject[] anchorObject;
    public string[] anchorObjectDescription;
    private int anchorObjectNumber = -1;
    public GameObject prefab_3DArrowMarker;

    private bool setAnchorStart;
    private bool firstAnchorRegistered;
    //private bool 

    void Start()
    {
        suggestionText.gameObject.SetActive(false);


        SteamVR_TrackedController trackedController = GetComponent<SteamVR_TrackedController>();
        trackedController.TriggerClicked += new ClickedEventHandler(DoClick);
        trackedController.MenuButtonClicked += new ClickedEventHandler(MenuClick);


        //JEFF CHECK PLAYER PREFS FOR ORIGINAL POS AND ROT


        for(int i = 0; i < anchorObjectDescription.Length; i++)
        {
            if (PlayerPrefs.HasKey(anchorObjectDescription[i] + "anchorPosX"))
            {
                Vector3 anchorPos = new Vector3(PlayerPrefs.GetFloat(anchorObjectDescription[i] + "anchorPosX"), PlayerPrefs.GetFloat(anchorObjectDescription[i] + "anchorPosY"), PlayerPrefs.GetFloat(anchorObjectDescription[i] + "anchorPosZ"));
                Vector3 anchorRot = new Vector3(PlayerPrefs.GetFloat(anchorObjectDescription[i] + "anchorRotX"), PlayerPrefs.GetFloat(anchorObjectDescription[i] + "anchorRotY"), PlayerPrefs.GetFloat(anchorObjectDescription[i] + "anchorRotZ"));

                anchorObject[i].transform.position = anchorPos;
                anchorObject[i].transform.eulerAngles = anchorRot;

            }
            else
            {
                //anchorObject.SetActive(false);
            }
        }

        setAnchorStart = false;
        firstAnchorRegistered = false;
        
        prefab_3DArrowMarker = Instantiate(prefab_3DArrowMarker, Vector3.zero, Quaternion.identity);
        prefab_3DArrowMarker.SetActive(false);
        
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
            if (!firstAnchorRegistered)
            {
                return;

            }
            else
            {
                //this will make the 3d marker "look at" the current position of the raycast
                Vector3 targetPostition = new Vector3(ProbePoint.transform.position.x, prefab_3DArrowMarker.transform.position.y, ProbePoint.transform.position.z);
                prefab_3DArrowMarker.transform.LookAt(targetPostition);

            }
        }

    }



    void DoClick(object sender, ClickedEventArgs e)
    {
        Debug.Log("hey i clicked the trigger button");

        if (!setAnchorStart)
        {
            return;
        }
        else
        {
            if (!firstAnchorRegistered)
            {
                anchorObject[anchorObjectNumber].SetActive(false);

                prefab_3DArrowMarker.transform.position = ProbePoint.transform.position;
                //prefab_3DArrowMarker.transform.rotation = ProbePoint.transform.rotation;
                prefab_3DArrowMarker.SetActive(true);

                firstAnchorRegistered = true;
            }
            else
            {
                anchorObject[anchorObjectNumber].transform.position = prefab_3DArrowMarker.transform.position;
                anchorObject[anchorObjectNumber].transform.rotation = prefab_3DArrowMarker.transform.rotation;

                //register this in playerprefs

                PlayerPrefs.SetFloat(anchorObjectDescription[anchorObjectNumber] + "anchorPosX", anchorObject[anchorObjectNumber].transform.position.x);
                PlayerPrefs.SetFloat(anchorObjectDescription[anchorObjectNumber] + "anchorPosY", anchorObject[anchorObjectNumber].transform.position.y);
                PlayerPrefs.SetFloat(anchorObjectDescription[anchorObjectNumber] + "anchorPosZ", anchorObject[anchorObjectNumber].transform.position.z);
                PlayerPrefs.SetFloat(anchorObjectDescription[anchorObjectNumber] + "anchorRotX", anchorObject[anchorObjectNumber].transform.eulerAngles.x);
                PlayerPrefs.SetFloat(anchorObjectDescription[anchorObjectNumber] + "anchorRotY", anchorObject[anchorObjectNumber].transform.eulerAngles.y);
                PlayerPrefs.SetFloat(anchorObjectDescription[anchorObjectNumber] + "anchorRotZ", anchorObject[anchorObjectNumber].transform.eulerAngles.z);


                anchorObject[anchorObjectNumber].SetActive(true);
                prefab_3DArrowMarker.SetActive(false);
                setAnchorStart = false;
                firstAnchorRegistered = false;
                suggestionText.gameObject.SetActive(false);

            }
        }
    }


    void MenuClick(object sender, ClickedEventArgs e)
    {
        Debug.Log("hey i clicked the menu button");

        anchorObjectNumber++;
        if(anchorObjectNumber >= anchorObject.Length)
        {
            //reset
            anchorObjectNumber = -1;
            setAnchorStart = false;
            firstAnchorRegistered = false;
            prefab_3DArrowMarker.SetActive(false);
            suggestionText.gameObject.SetActive(false);
        }
        else
        {
            suggestionText.gameObject.SetActive(true);
            suggestionText.text = "Now Probing Object: " + anchorObjectDescription[anchorObjectNumber];

            setAnchorStart = true;
            firstAnchorRegistered = false;
            prefab_3DArrowMarker.SetActive(false);
        }

    }
}
