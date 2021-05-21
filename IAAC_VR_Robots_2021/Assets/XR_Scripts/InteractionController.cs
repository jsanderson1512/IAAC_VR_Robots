using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;

public class InteractionController : MonoBehaviour
{
    //!!rename this "InteractionController" after class

    public Material HighlightMaterial;

    [System.Serializable]
    public enum InteractionType // your custom enumeration
    {
        AnimationController,
        SceneChangeController,
        OnOffController,
        MakeCameraChild
    };
    public InteractionType myType = InteractionType.AnimationController;  // this public var should appear as a drop down

    //animation stuff
    public GameObject ObjectWithAnimation;
    private Animator theAnimator;

    //scene load stuff
    public string SceneToLoad;

    //on off stuff
    public int NumberOfThingsToTurnOn = 1;
    public int NumberOfThingsToTurnOff = 1;
    public GameObject[] ClickTurnsTheseON;
    public GameObject[] ClickTurnsTheseOFF;
    private bool OnOffSwitch;

    //Make Camera Child stuff
    public GameObject objectToBecomeParent; 


    private void Start()
    {
        if (myType == InteractionType.OnOffController)
        {
            OnOff(false, true);
        }
        else if (myType == InteractionType.AnimationController)
        {
            theAnimator = ObjectWithAnimation.GetComponent<Animator>();
            if (theAnimator != null)
            {
                string animName = theAnimator.runtimeAnimatorController.animationClips[0].name;
                //Debug.Log("my animation is called: " + animName);
                //play on start but set to false so it stops
                theAnimator.Play(animName, 0, 0);
                theAnimator.enabled = false;
            }
        }

        else if (myType == InteractionType.MakeCameraChild)
        {
            theAnimator = objectToBecomeParent.GetComponent<Animator>();
            if (theAnimator != null)
            {
                string animName = theAnimator.runtimeAnimatorController.animationClips[0].name;
                //Debug.Log("my animation is called: " + animName);
                //play on start but set to false so it stops
                theAnimator.Play(animName, 0, 0);
                theAnimator.enabled = false;
            }
        }
    }
    private void OnTriggerEnter(Collider other)
    {
        Debug.Log("i bumped into something called: " + other.name);


        DoTheThing(other.gameObject);

        /*
        GameObject theCam = other.GetComponentInChildren<Camera>().gameObject;
        if(theCam!=null)
        {
            DoTheThing(theCam);
        }
        */
    }
    public void DoTheThing(GameObject myCamera)
    {
        //i clicked on this thing with an interaction controller on it
        //Debug.Log("I did the thing");
        if (myType == InteractionType.AnimationController)
        {
            //play or pause animation
            //note, the animator must make a transition to exit if it is not on loop.

            string animName = theAnimator.runtimeAnimatorController.animationClips[0].name;
            Debug.Log("my animation is called: " + animName);

            if (theAnimator.GetCurrentAnimatorStateInfo(0).IsName(animName))
            {
                Debug.Log("hey my animation is in the middle of playing or on loop");

                if (theAnimator.isActiveAndEnabled)
                {
                    Debug.Log("my animation was playing and enabled, it will stop now");
                    theAnimator.enabled = false;
                }
                else
                {
                    Debug.Log("my animation was in the middle of playing playing but not enabled");
                    theAnimator.enabled = true;
                }
            }
            else
            {
                Debug.Log("hey my animation ended, I'm going to start it over");
                //this will be called if the animation has completed only.
                theAnimator.enabled = true;
                theAnimator.Play(animName, 0, 0);
            }

        }
        else if (myType == InteractionType.SceneChangeController)
        {
            //load a scene
            Debug.Log("scene");
            if (SceneToLoad != null && SceneToLoad != "")
            {
                if (Application.CanStreamedLevelBeLoaded(SceneToLoad))
                {
                    SceneManager.LoadScene(SceneToLoad);

                }
                else
                {
                    Debug.LogError("Hey, I couldn't find that scene name. Check if it is spelled correctly in the TriggerInteraction and if it is added to build settings (file > build settings)");
                }
            }
            else
            {
                Debug.LogError("Hey, your scene name is blank, please enter the scene name and ensure that the scene is added to your build settings (file > build settings)");
            }
        }
        else if (myType == InteractionType.OnOffController)
        {
            Debug.Log("on off");

            OnOffSwitch = !OnOffSwitch;
            if (OnOffSwitch)
            {
                OnOff(true, false);
            }
            else
            {
                OnOff(false, true);
            }
        }

        else if (myType == InteractionType.MakeCameraChild)
        {

            //cam becomes a child of something
            //animation plays to move something

            //i guess after its done, the camera is no longer a child

            Debug.Log("I called MakeCameraChild");

            //play or pause animation
            //note, the animator must make a transition to exit if it is not on loop.
            /*
            if (!myCamera.GetComponent<Camera>())
            {
                //if i am a collider, it will be the parent already
                myCamera = myCamera.GetComponentInChildren<Camera>().gameObject;

            }
            */
            //if it is the camera, it should have a parent. 
            //GameObject myCameraParent = myCamera.transform.parent.gameObject;
            GameObject myCameraParent = myCamera;
            myCameraParent.transform.position = objectToBecomeParent.transform.position;
            myCameraParent.transform.rotation = objectToBecomeParent.transform.rotation;

            myCameraParent.transform.parent = objectToBecomeParent.transform;
            


            if (theAnimator != null)
            {
                string animName = theAnimator.runtimeAnimatorController.animationClips[0].name;
                Debug.Log("my animation is called: " + animName);

                //play the animation from the beginning
                theAnimator.enabled = true;
                theAnimator.Play(animName, 0, 0);
            }
        }
    }
    void OnOff(bool bool1, bool bool2)
    {
        foreach (GameObject g in ClickTurnsTheseON)
        {
            if (g != null)
            {
                g.SetActive(bool1);
            }
        }
        foreach (GameObject g in ClickTurnsTheseOFF)
        {
            if (g != null)
            {
                g.SetActive(bool2);
            }
        }
    }
}
