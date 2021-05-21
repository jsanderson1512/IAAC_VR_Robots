using UnityEngine;
using System.Collections;
using UnityEngine.UI;

public class CameraRaycastController : MonoBehaviour
{
    private bool dontHighlight;
    private Material[] tempMaterialsHigh;
    private Material[] matsHigh;
    private GameObject tempSelectedObject;




    void Update()
    {

        Ray myRay = this.GetComponent<Camera>().ScreenPointToRay(Input.mousePosition);




        RaycastHit myRayHit;
        if (Physics.Raycast(myRay, out myRayHit, 100.0f))
        {
            //i shot out a ray and it hit something
            //Debug.Log("I hit something");
            GameObject hitObject = myRayHit.transform.gameObject;
            if (hitObject.GetComponent<InteractionController>())
            {
                //i shot out a ray and hit something with an interaction controller
                //Debug.Log("I hit something with an interaction controller on it");
                RayHit(hitObject);


                    if (Input.GetMouseButtonDown(0))
                    {
                        InteractionController[] myInteractions = hitObject.GetComponents<InteractionController>();
                        foreach (InteractionController t in myInteractions)
                        {
                            t.DoTheThing(gameObject);
                        }
                        //i clicked on something with an interaction controller on it
                        //Debug.Log("I clicked something with an interaction controller on it");
                        //hitObject.GetComponent<TriggerInteraction>().DoTheThing(gameObject); //send this method information about myself
                    }
                
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
            //i missed all colliders within 100.0f 
            //Debug.Log("I didn't hit anything");
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
}
