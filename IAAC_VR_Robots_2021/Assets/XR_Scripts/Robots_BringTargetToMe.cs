using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Robots_BringTargetToMe : MonoBehaviour
{
    public GameObject RobotTarget;
    // a public buffer range to change targets with
    public int buffer;
    public float time;
    public Slider myTargetSlider;
    public Slider myTimeSlider;

    private Quaternion startRotationController;
    private Quaternion startRotationTarget;
    private bool remoteRotateOn = false;

    private Coroutine play;
    private Coroutine rewind;


    public GameObject listOfPointsParent;
    private List<Transform> myPoints = new List<Transform>();
    private Transform currentTarget;
    private int targetNumber = 0;
    private List<Transform> posPoints = new List<Transform>();
    private List<Transform> negPoints = new List<Transform>();

    public GameObject controllerL;
    public GameObject controllerR;
    private GameObject myController;

    private void Start()
    {
        currentTarget = new GameObject().transform;

        GetPoints();

    }

    public void GetPoints()
    {

        myPoints = new List<Transform>();

        int childrenNumber = listOfPointsParent.transform.childCount;
        for (int i = 0; i < childrenNumber; i++)
        {
            myPoints.Add(listOfPointsParent.transform.GetChild(i).transform);

        }

        if(myPoints.Count > targetNumber)
        {
            currentTarget = myPoints[targetNumber];

        }
    }

    public void ComeToMe()
    {

        if (controllerL.activeSelf)
        {
            myController = controllerL;
        }
        else if (controllerR.activeSelf)
        {
            myController = controllerR;
        }


        RobotTarget.transform.position = myController.transform.position;
        RobotTarget.transform.rotation = myController.transform.rotation;

        RobotTarget.transform.RotateAround(RobotTarget.transform.position, RobotTarget.transform.right, -90);
    }

    public void RemoteRotateStart()
    {

        if (controllerL.activeSelf)
        {
            myController = controllerL;
        }
        else if (controllerR.activeSelf)
        {
            myController = controllerR;
        }


        startRotationController = myController.transform.rotation;
        startRotationTarget = currentTarget.transform.rotation;
        remoteRotateOn = true;

    }
    public void RemoteRotateStop()
    {
        remoteRotateOn = false;

    }
    public void NextPoint()
    {
        bool resetThing = false;
        if (remoteRotateOn)
        {
            resetThing = true;
        }
        RemoteRotateStop();
        //GREY WE DONT WANT TO SEND THIS WHEN WE CLICK NEXT, WE WANT TO SEND ALL OF THEM ONCE YOU'RE DONE EDITING
        //this.gameObject.GetComponent<Robots_SendInstructionToMaster>().SendToMaster();
        targetNumber++;

        if (targetNumber < myPoints.Count)
        {
            currentTarget = myPoints[targetNumber];

        }
        else
        {
            targetNumber = myPoints.Count-1;
            currentTarget = myPoints[targetNumber];

            Debug.Log("hey i reached the end of the list");

        }


        if (resetThing)
        {
            RemoteRotateStart();

        }

    }

    public void PrevPoint()
    {
        bool resetThing = false;
        if (remoteRotateOn)
        {
            resetThing = true;
        }
        RemoteRotateStop();


        //GREY WE DONT WANT TO SEND THIS WHEN WE CLICK NEXT, WE WANT TO SEND ALL OF THEM ONCE YOU'RE DONE EDITING
        //this.gameObject.GetComponent<Robots_SendInstructionToMaster>().SendToMaster();


        targetNumber--;

        if (targetNumber >= 0 )
        {
            currentTarget = myPoints[targetNumber];

        }
        else
        {
            targetNumber = 0;
            currentTarget = myPoints[targetNumber];

            Debug.Log("hey i reached the end of the list");
        }

        if (resetThing)
        {
            RemoteRotateStart();

        }

    }
    /*
   public class PlayCoroutine : MonoBehaviour
    {
        public float smoothing = 1f;

        private void Start()
        {
            StartCoroutine(currentTarget, myPoints);         }
        }
        IEnumerator MyCoroutine (PlaySim)

    }
        {
            //Turn off resetThing just in case
            bool resetThing = false;

        
            targetNumber++;

            if (targetNumber < myPoints.Count)
            {
                currentTarget = myPoints[targetNumber];

            }
            else
            {
                targetNumber = myPoints.Count - 1;
                currentTarget = myPoints[targetNumber];

                Debug.Log("hey i reached the end of the list");

            }


            if (resetThing)
            {
                RemoteRotateStart();

            }

        }
    */

    //GREY
    IEnumerator PlaySim()
    {
        //int playNum = myPoints.Count - targetNumber;
        while (targetNumber < myPoints.Count - 1) 
        {
            targetNumber++;
            currentTarget = myPoints[targetNumber];
            float speed = time;
            yield return new WaitForSeconds(speed);

        }

        Debug.Log("we have played the simulation");
        yield return new WaitForSeconds(.1f);
    }

    IEnumerator RewindSim()
    {
        //int playNum = myPoints.Count - targetNumber;
        while (targetNumber > 0)
        {
            targetNumber--;
            currentTarget = myPoints[targetNumber];
            float speed = time;
            yield return new WaitForSeconds(speed);

        }

        Debug.Log("we have played the simulation");
        yield return new WaitForSeconds(.1f);
    }

    public void PlayButton()
    {
        play = StartCoroutine(PlaySim());
    }

    public void RewindButton()
    {
        rewind = StartCoroutine(RewindSim());
    }

    public void PauseButton()
    {
        if (play != null)
        {
            StopCoroutine(play);
        }
        if (rewind != null)
        {
            StopCoroutine(rewind);
        }

    } 

    public void TargetSliderChanged()
    {
        buffer = (int)myTargetSlider.value;
    }

    public void TimeSliderChanged()
    {
        time = (float)myTimeSlider.value;
    }


    public void GetBuffer()
    {
        //establish the sublist to modify within the larger list of "myPoints"
        //maybe do this by grabbing the index of the currentTarget within myPoints, and then creating a range of indices up and down from that target?

        Debug.Log("hey i found my currentTarget, and its Index is" + targetNumber);
        currentTarget = myPoints[targetNumber];
        // I need to find a series of numbers between 0 and 1 for the positive, and 0 and -1 for the negative, and then use the LERP function for quaternions
        for (int i = (-buffer); i < buffer; i++)
        {

            if (i == 0)
            {
                Quaternion differenceRotation = startRotationController * Quaternion.Inverse(myController.transform.rotation);
                Quaternion finalRotation = differenceRotation * startRotationTarget;
                currentTarget.transform.rotation = finalRotation;
            }
            else
            {
                int newIndex = targetNumber + i;
                if (newIndex < myPoints.Count && newIndex >= 0)
                {
                    float lerpIndex = Mathf.Abs(i) / (float)buffer;
                    Debug.Log("Lerp, my index is" + " " + lerpIndex);
                    Quaternion slerpRotation = Quaternion.Lerp(myController.transform.rotation, startRotationController, (float)lerpIndex);
                    Quaternion differenceRotation = startRotationController * Quaternion.Inverse(slerpRotation);
                    Quaternion finalRotation = differenceRotation * startRotationTarget;



                    myPoints[newIndex].transform.rotation = finalRotation;
                }
            }
        }

    }


    public void SendAllPointsToMachina()
    {
        for (int i = 0; i < myPoints.Count; i++)
        {

            RobotTarget.transform.position = myPoints[i].transform.position;
            RobotTarget.transform.rotation = myPoints[i].transform.rotation;

            this.gameObject.GetComponent<Robots_SendInstructionToMaster>().SendToMaster();
        }
    }
    private void Update()
    {
        RobotTarget.transform.position = currentTarget.transform.position;
        RobotTarget.transform.rotation = currentTarget.transform.rotation;


        if (!remoteRotateOn)
        {
            return;
        }
        else
        {
            GetBuffer();
            //Quaternion differenceRotation = startRotationController * Quaternion.Inverse(myController.transform.rotation);
            //Quaternion finalRotation = differenceRotation * startRotationTarget;

            //currentTarget.transform.rotation = finalRotation;
            /*
            Debug.Log("hey i found my currentTarget, and its Index is" + targetNumber);
            currentTarget = myPoints[targetNumber];
            int inv = buffer;
            // I need to find a series of numbers between 0 and 1 for the positive, and 0 and -1 for the negative, and then use the LERP function for quaternions
            for (int i = buffer; i > 0; i--)
            {

                if (i == buffer)
                {
                    Quaternion differenceRotation = startRotationController * Quaternion.Inverse(myController.transform.rotation);
                    Quaternion finalRotation = differenceRotation * startRotationTarget;
                    currentTarget.transform.rotation = finalRotation;
                }
                else
                {
                    Quaternion differenceRotation = startRotationController * Quaternion.Inverse(myController.transform.rotation);
                    Quaternion finalRotation = differenceRotation * startRotationTarget;
                    float lerpIndex = i / (float)buffer;
                    Debug.Log("Lerp, my index is" + " " + lerpIndex);
                   
                    int newIndex = targetNumber + i;
                    myPoints[newIndex].transform.rotation = Quaternion.Slerp(currentTarget.transform.rotation, finalRotation, (float)lerpIndex);
                    
                }
            }
            */
        }
        


    }
}
