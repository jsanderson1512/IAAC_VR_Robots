using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IAAC_VRControllerButtons : MonoBehaviour
{
    #region PUBLIC VARIABLES
    public float laserLength = 3.0f;
    public GameObject sceneManager;


    #endregion

    #region PRIVATE VARIABLES

    private bool dontHighlight;
    private Material[] tempMaterialsHigh;
    private Material[] matsHigh;
    private GameObject tempSelectedObject;


    #endregion

    #region AWAKE AND START FUNCTIONS
    private void Awake()
    {


    }

    void Start()
    {
        SteamVR_TrackedController trackedController = GetComponent<SteamVR_TrackedController>();
        trackedController.TriggerClicked += new ClickedEventHandler(DoClick);
        trackedController.TriggerUnclicked += new ClickedEventHandler(UnClick);
        trackedController.PadClicked += new ClickedEventHandler(PadClicked);
        trackedController.MenuButtonClicked += new ClickedEventHandler(MenuClick);
        trackedController.Gripped += new ClickedEventHandler(Gripped);

    }
    #endregion

    private void Update()
    {
        Vector3 origin = transform.position;
        Vector3 direction = transform.forward;

        RaycastHit hit;
        Ray ray = new Ray(origin, direction);
        if (Physics.Raycast(ray, out hit))
        {
            string myTag = hit.transform.tag;
            GameObject touchObject = hit.transform.gameObject;

            if (Vector3.Distance(origin, hit.point) < laserLength)
            {



                //i shot out a ray and it hit something
                //Debug.Log("I hit something");
                GameObject hitObject = hit.transform.gameObject;
                if (hitObject.GetComponent<InteractionController>())
                {
                    //i shot out a ray and hit something with an interaction controller
                    //Debug.Log("I hit something with an interaction controller on it");
                    RayHit(hitObject);



                }



                else
                {
                    //i hit something but it didn't have an interaction controller on it
                    //Debug.Log("I missed the interaction controller");
                    RayMissed();
                }




            }
            else
            {
                RayMissed();
            }
        }
        else
        {//this means the raycast missed
            RayMissed();
        }
    }




    void RayHit(GameObject touchObject)
    {
        if (tempSelectedObject != touchObject)
        {
            if (tempSelectedObject != null)
            {
                UnHighlightObj(tempSelectedObject);
            }
        }
        tempSelectedObject = touchObject;
        HighlightObj(tempSelectedObject);
    }
    void RayMissed()
    {
        if (tempSelectedObject != null)
        {
            UnHighlightObj(tempSelectedObject);
            tempSelectedObject = null;
        }
    }
    void HighlightObj(GameObject highlightThis)
    {
        MeshRenderer rend = highlightThis.transform.gameObject.GetComponent<MeshRenderer>();
        if (rend != null)
        {
            if (!dontHighlight)
            {
                tempMaterialsHigh = highlightThis.transform.gameObject.GetComponent<Renderer>().sharedMaterials;
                matsHigh = new Material[tempMaterialsHigh.Length];
                Material highlightMaterial = highlightThis.GetComponent<InteractionController>().HighlightMaterial;
                for (int i = 0; i < tempMaterialsHigh.Length; i++)
                {
                    matsHigh[i] = highlightMaterial;
                }
                highlightThis.transform.gameObject.GetComponent<Renderer>().sharedMaterials = matsHigh;
                dontHighlight = true;
            }
        }
    }
    void UnHighlightObj(GameObject unHighlightThis)
    {
        MeshRenderer rend = unHighlightThis.GetComponent<MeshRenderer>();
        if (rend != null)
        {
            unHighlightThis.transform.gameObject.GetComponent<Renderer>().sharedMaterials = tempMaterialsHigh;
            dontHighlight = false;
        }
    }









    void DoClick(object sender, ClickedEventArgs e)
    {
        Debug.Log("hey i clicked the trigger button");


        if (tempSelectedObject)
        {
            InteractionController[] myInteractions = tempSelectedObject.GetComponents<InteractionController>();
            foreach (InteractionController t in myInteractions)
            {
                t.DoTheThing(gameObject);
            }
            //i clicked on something with an interaction controller on it
            //Debug.Log("I clicked something with an interaction controller on it");
            //hitObject.GetComponent<TriggerInteraction>().DoTheThing(gameObject); //send this method information about myself
        }


    }
    void UnClick(object sender, ClickedEventArgs e)
    {
        Debug.Log("hey i UN clicked the trigger button");
        

    }
    void PadClicked(object sender, ClickedEventArgs e)
    {


        Debug.Log("hey i clicked the pad button");


        float PadLimitHigh = 0.7f;
        float PadLimitLow = 0.3f;

        if (e.padX < -(PadLimitHigh) && e.padY < PadLimitLow)
        { //Left

            Debug.Log("hey i clicked the pad button left");
            //do the thing on the left lcick whatever


        }
        else if (e.padX > PadLimitHigh && e.padY < PadLimitLow)
        { //Right
            Debug.Log("hey i clicked the pad button right");

        }
        

    }

    void Gripped(object sender, ClickedEventArgs e)
    {
        Debug.Log("hey i clicked the grip button");

    }

    void MenuClick(object sender, ClickedEventArgs e)
    {
        Debug.Log("hey i clicked the menu button");

    }
}