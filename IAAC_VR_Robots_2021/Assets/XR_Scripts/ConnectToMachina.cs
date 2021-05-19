using System.Collections;
using System.Collections.Generic;
using UnityEngine;


#if UNITY_STANDALONE_WIN
using WebSocketSharp;
#endif

using System.Linq;
using System;
using System.Text;
using UnityEngine.UI;


public class ConnectToMachina : MonoBehaviour
{
    private List<Vector3> grasshopperPos = new List<Vector3>();
    private List<Quaternion> grasshopperRot = new List<Quaternion>();
    private List<string> grasshopperID = new List<string>();

    List<GameObject> myChildren = new List<GameObject>();

    private List<GameObject> grasshopperTargets = new List<GameObject>();
    private bool messageUpdated;
    //this is all from Machina Documentation here: https://github.com/RobotExMachina/Machina.NET/blob/master/Docs/Reference.md
    //Jeff: switched syntax on all below from "socket.sendMessage" to "socket.Send"

    

    public Text ConsoleText;
    private string textForConsole;
    public GameObject followerRobot;
    public GameObject controllerRobot;
    public GameObject importedPointsParent;
    public GameObject modelTarget;

#if UNITY_STANDALONE_WIN
    [HideInInspector]
    public WebSocket machinaBridgeSocket;

#endif


    private Vector3 slavePos;
    private Quaternion slaveRot;
    private bool slaveMove;



    public void MoveSlaveToPosition(Vector3 thePosition, Quaternion theRotation)
    {
        Debug.Log("moving slave robot");
        followerRobot.GetComponent<RobotController_Kinematics>().movementMode = RobotController_Kinematics.MovementMode.InverseKinematics;
        followerRobot.GetComponent<RobotController_Kinematics>().RobotTarget_RobotSpace.transform.localPosition = thePosition;
        followerRobot.GetComponent<RobotController_Kinematics>().RobotTarget_RobotSpace.transform.localRotation = theRotation;
    }


#if UNITY_STANDALONE_WIN
    private void Start()
    {
        StartCoroutine(CheckConnectionAlive());

    }

    void OnApplicationQuit()
    {
        if (machinaBridgeSocket != null)
        {
            machinaBridgeSocket.Close();
            machinaBridgeSocket = null;
        }
        Debug.Log("quitting");
        Application.Quit();
    }

    IEnumerator CheckConnectionAlive()
    {
        while (true)
        {
            Debug.Log("checking connection");
            if (machinaBridgeSocket != null)
            {
                if (machinaBridgeSocket.IsAlive == true)
                {
                    Debug.Log("connected");
                }
                else
                {
                    textForConsole = "Not Connected to Machina Bridge";
                }
            }
            else
            {
                textForConsole = "Not Connected to Machina Bridge";
            }

            ConsoleText.text = textForConsole;

            yield return new WaitForSeconds(5);
        }
    }

    public void ConnectToBridge()
    {
        machinaBridgeSocket = new WebSocket("ws://127.0.0.1:6999/Bridge?name=Unity");//this is the machina bridge socket
        machinaBridgeSocket.Connect();
        textForConsole = "Connected to Machina Bridge";//this needs to say not connected if there is a connection issue...
        SpeedTo(100);
        machinaBridgeSocket.OnMessage += (sender, e) =>
        {



            Debug.Log("Message Received from " + ((WebSocket)sender).Url + ", Data : " + e.Data);



            if (e.Data.Contains("action-issued"))
            {
                textForConsole = "Action Issued";

                string[] messageComponents = e.Data.Split(':');
                string myPlane = "";
                string myRot = "";

                //Message Received from ws://127.0.0.1:6999/Bridge?name=Unity, Data : {"event":"action-executed",
                //"last":"Message(\"0,675.4,460.66,1211.6,0,0,1,0,-1,0,1,0,0\");","id":16,"pendDev":2,"pendTot":2,"pos":[1251.913,0,1502.5],"ori":[-0.5,0,-0.866,0,1,0],"quat":[0.5,0,0.866,0],"axes":[0,0,0,0,30,0],"extax":null,"conf":null}

                //List<Vector3> grasshopperSinglePos = new List<Vector3>();
                //List<Quaternion> grasshopperSingleRot = new List<Quaternion>();

                for (int i = 0; i < messageComponents.Length; i++)
                {
                    if (messageComponents[i].Contains("Message"))
                    //"Message(\"0,675.4,460.66,1211.6,0,0,1,0,-1,0,1,0,0\");","id"
                    {
                        myPlane = messageComponents[i];
                        Debug.Log("Here is your message" + myPlane);

                    }
                }

                if (myPlane != "")
                {
                    //  MESSAGE SHOULD BE  //  "Message(\"2,1212.5,460.66,1211.6,0,0,1,0,-1,0,1,0,0\");","id"
                    myPlane = myPlane.Split('"')[2]; //split off the front bit
                    Debug.Log("MY POS IS " + myPlane);
                    //  MESSAGE SHOULD BE  //  2,1212.5,460.66,1211.6,0,0,1,0,-1,0,1,0,0\
                    myPlane = myPlane.Replace("\\", ""); //split off the end bit
                    //  MESSAGE SHOULD BE  //  2,1212.5,460.66,1211.6,0,0,1,0,-1,0,1,0,0
                    //DebuLog("MY FINAL POS IS " + myPlane);
                    string[] eachPosString = myPlane.Split(',');

                    grasshopperID.Add(eachPosString[0]);

                    Vector3 tempPos = new Vector3(float.Parse(eachPosString[1]), float.Parse(eachPosString[2]), float.Parse(eachPosString[3]));
                    tempPos = new Vector3(tempPos.x / 1000.000f, tempPos.y / 1000.000f, tempPos.z / 1000.000f);//robot units to unity units
                    tempPos = new Vector3(-1.0f * tempPos.x, tempPos.z, -1.0f * tempPos.y);//flip flop some units from robots to unity

                    grasshopperPos.Add(tempPos);

                    Vector3 tempRotX = new Vector3(float.Parse(eachPosString[4]), float.Parse(eachPosString[5]), float.Parse(eachPosString[6]));
                    Vector3 tempRotY = new Vector3(float.Parse(eachPosString[7]), float.Parse(eachPosString[8]), float.Parse(eachPosString[9]));
                    Vector3 tempRotZ = new Vector3(float.Parse(eachPosString[10]), float.Parse(eachPosString[11]), float.Parse(eachPosString[12]));

                    //Plane tempRotPlane = new Plane(tempRotX, tempRotY, tempRotZ);
                    Quaternion tempRot = Quaternion.LookRotation(tempRotZ, Vector3.up);
                    //Vector3 tempRot = new Vector3(tempRotX, tempRotY, tempRotZ);
                    //Quaternion tempRot = new Quaternion(float.Parse(eachRotString[0]), float.Parse(eachRotString[1]), float.Parse(eachRotString[2]), float.Parse(eachRotString[3]));
                    tempRot = new Quaternion(-1.0f * tempRot.y, tempRot.w, -1.0f * tempRot.z, -1.0f * tempRot.x);//flip flop some units from robots to unity

                    grasshopperRot.Add(tempRot);

                    //Debug.Log("im adding the tempPos: " + tempPos + "to grasshopperPos");

                }
                //"Message(\"Transform: move to [802.232, 599.404, 860.99] mm and rotate to [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
                //if (myRot != "")
                //{

                    //myRot = myRot.Split(']')[3]; //grab just the end
                    //, [0, 0, 1]
                    //Debug.Log("temprot = " + myRot);
                    //myRot = myRot.Split('[')[1]; //split off the front
                    //0, 0, 1
                    //Debug.Log("temprot = " + myRot);
                    //myRot = myRot.Replace(" ", "");
                    //0,0,1
                    //Debug.Log("temprot = " + myRot);
                    //string[] eachRotString = myRot.Split(',');

                    //Vector3 tempRotZ = new Vector3(float.Parse(eachRotString[0]), float.Parse(eachRotString[1]), float.Parse(eachRotString[2]));
                    //Vector3 tempRotY = new Vector3(float.Parse(eachRotString[3]), float.Parse(eachRotString[4]), float.Parse(eachRotString[5]));
                    //Vector3 tempRotZ = new Vector3(float.Parse(eachRotString[6]), float.Parse(eachRotString[7]), float.Parse(eachRotString[8]));

                    //Plane tempRotPlane = new Plane(tempRotX, tempRotY, tempRotZ);
                    //Quaternion tempRot = Quaternion.LookRotation(tempRotZ, Vector3.up);
                    //Vector3 tempRot = new Vector3(tempRotX, tempRotY, tempRotZ);
                    //Quaternion tempRot = new Quaternion(float.Parse(eachRotString[0]), float.Parse(eachRotString[1]), float.Parse(eachRotString[2]), float.Parse(eachRotString[3]));
                    //tempRot = new Quaternion(-1.0f * tempRot.y, tempRot.w, -1.0f * tempRot.z, -1.0f * tempRot.x);//flip flop some units from robots to unity

                    //grasshopperRot.Add(tempRot);

                    //Debug.Log("im adding the tempRot: " + tempRot + "to grasshopperRot");


                //}

                messageUpdated = true;


                //Message Received from ws://127.0.0.1:6999/Bridge?name=Unity, Data : {"event":"action-issued","last"
                //:"Message(\"from bridge\");","id":3,"pos"
                //:[1251.913,0,1502.5],"ori":[-0.5,0,-0.866,0,1,0],"quat":[0.5,0,0.866,0],"axes":[0,0,0,0,30,0],"extax":null,"conf":null}

            }
            else if (e.Data.Contains("action-released"))
            {
                textForConsole = "Action Released";



            }
            else if (e.Data.Contains("action-executed"))
            {
                textForConsole = "Action Executed";
                //here, i could set the position of the "actual robot" to match the "preview robot"
                //THIS GETS RECEIVED WHETHER YOU SEND THE INSTRUCTION OR NOW... SO SOMEONE COULD BE SENDING GRASSHOPPER INSTRUCTIONS...



                string[] messageComponents = e.Data.Split(':');
                string myPos = "";
                string myRot = "";

                for (int i = 0; i < messageComponents.Length; i++)
                {
                    if (messageComponents[i].Contains("pos"))
                    {
                        myPos = messageComponents[i + 1];
                    }
                    else if (messageComponents[i].Contains("quat"))
                    {
                        myRot = messageComponents[i + 1];
                    }
                }

                if (myPos != "")
                {

                    myPos = myPos.Split(']')[0]; //split off the end bit
                    myPos = myPos.Replace("[", "");//get rid of the front bit
                    string[] eachPosString = myPos.Split(',');

                    slavePos = new Vector3(float.Parse(eachPosString[0]), float.Parse(eachPosString[1]), float.Parse(eachPosString[2]));
                    slavePos = new Vector3(slavePos.x / 1000.0f, slavePos.y / 1000.0f, slavePos.z / 1000.0f);//robot units to unity units
                    slavePos = new Vector3(-1.0f * slavePos.x, slavePos.z, -1.0f * slavePos.y);//flip flop some units from robots to unity

                    Debug.Log("im going to move the slave robot to: " + slavePos);

                }

                if (myRot != "")
                {

                    myRot = myRot.Split(']')[0]; //split off the end bit
                    myRot = myRot.Replace("[", "");//get rid of the front bit
                    string[] eachRotString = myRot.Split(',');

                    slaveRot = new Quaternion(float.Parse(eachRotString[0]), float.Parse(eachRotString[1]), float.Parse(eachRotString[2]), float.Parse(eachRotString[3]));
                    slaveRot = new Quaternion(-1.0f * slaveRot.y, slaveRot.w, -1.0f * slaveRot.z, -1.0f * slaveRot.x);//flip flop some units from robots to unity

                    Debug.Log("im going to rotate the slave robot to: " + slaveRot);

                }

                slaveMove = true;
                //this sends this one command to be completed in the update loop. for some reason
                //i'm having trouble sending commands within this event


            }
            else if (e.Data.Contains("controller-disconnected"))
            {
                textForConsole = "Controller not Connected - Please connect Machina Bridge to a physical or virtual robot";
                //here, i could set the position of the "actual robot" to match the "preview robot"
                //THIS GETS RECEIVED WHETHER YOU SEND THE INSTRUCTION OR NOW... SO SOMEONE COULD BE SENDING GRASSHOPPER INSTRUCTIONS...


            }

            /*
             MESSAGES FORMATTED LIKE THIS: ACTION ISSUED, ACTION RELEASED, ACTION EXECUTED

            Message Received from ws://127.0.0.1:6999/Bridge, Data : {"event":"action-issued","last":"Move(100,0,0);",
            "id":1,"pos":[698,-1155,1092],"ori":[-0.968,0.201,0.148,0.237,0.927,0.29],"quat":[0.058,-0.1119,-0.9799,-0.1544],
            "axes":[-59.385,16.181,2.38,5.315,89.695,-47.239],"extax":null,"conf":null}

            Message Received from ws://127.0.0.1:6999/Bridge, Data : {"event":"action-released","last":"Move(100,0,0);"
            ,"id":1,"pend":0,"pos":[698,-1155,1092],"ori":[-0.968,0.201,0.148,0.237,0.927,0.29],"quat":[0.058,-0.1119,-0.9799,-0.1544],
            "axes":[-59.385,16.181,2.38,5.315,89.695,-47.239],"extax":null,"conf":null}
             
            Message Received from ws://127.0.0.1:6999/Bridge, Data : {"event":"action-executed","last":"Move(100,0,0);"
            ,"id":1,"pendDev":0,"pendTot":0,"pos":[798,-1155,1092],"ori":[-0.968,0.201,0.148,0.237,0.927,0.29],"quat":[0.058,-0.1119,-0.9799,-0.1544],
            "axes":null,"extax":null,"conf":null}


            Message Received from ws://127.0.0.1:6999/Bridge, Data : {"event":"controller-disconnected"}
            */
        };
    }


    private void Update()
    {

        if (machinaBridgeSocket == null)
        {
            return;
        }
        else
        {
            ConsoleText.text = textForConsole;

            if (followerRobot && slaveMove)
            {
                MoveSlaveToPosition(slavePos, slaveRot);
                //also, send this to everyone else...
                gameObject.GetComponent<Robots_SendInstructionToMaster>().SlavePosFromMaster(slavePos, slaveRot);

                //IMPORTANT FOR GREY -- THIS IS WHERE WE AUTOMATICALLY GO TO THE NEXT POINT
                //this.gameObject.GetComponent<Robots_BringTargetToMe>().NextPoint();

                slaveMove = false;
            }


            if (messageUpdated)
            {
                Debug.Log("sending a message");




                for (int i = 0; i < grasshopperPos.Count; i++)
                {
                    bool doesMatch = false;
                    int matchingChild = 0;

                    for(int k = 0; k<myChildren.Count;k++)
                    {
                        if(grasshopperID[i] == myChildren[k].name)
                        {
                            doesMatch = true;
                            matchingChild = k;

                        }
                    }


                    if(doesMatch)
                    {
                        //replace values

                        myChildren[matchingChild].transform.localPosition = grasshopperPos[i];
                        myChildren[matchingChild].transform.localRotation = grasshopperRot[i];

                    }

                    else
                    {
                        //make new thing

                        //GameObject hello = new GameObject(grasshopperID[i]);
                        GameObject hello = Instantiate(modelTarget, grasshopperPos[i], grasshopperRot[i]);
                        hello.transform.SetParent(importedPointsParent.transform, false);
                        hello.name = grasshopperID[i];
                        //Instantiate(modelTarget, grasshopperPos[i], grasshopperRot[i]);

                        //hello.transform.parent = importedPointsParent.transform;

                        //hello.transform.localPosition = grasshopperPos[i];
                        //hello.transform.localRotation = grasshopperRot[i];

                        myChildren.Add(hello);

                        grasshopperTargets.Add(hello);
                    }


                }

                //HERE WE WANT TO USE THE GET POINTS COMMAND TO SET A LIST OF POINTS IN "ROBOTS_BRINGTARGETTOME"
                gameObject.GetComponent<Robots_BringTargetToMe>().GetPoints();
                messageUpdated = false;

            }

        }

    }


    /// <summary>
    /// Moves the device along a specified vector relative to its current position.
    /// </summary>
    /// <param name="xInc">A explanation of this really important number.</param>
    /// <param name="yInc">A explanation of this really important number.</param>
    /// <param name="zInc">A explanation of this really important number.</param>
    public void Move(float xInc, float yInc, float zInc)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("Move(" + xInc + "," + yInc + "," + zInc + ");");
    }

    /// <summary>
    /// Moves the device to an absolute position in global coordinates.
    /// </summary>
    /// <param name="x"></param>
    /// <param name="y"></param>
    /// <param name="z"></param>
    public void MoveTo(float x, float y, float z)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("MoveTo(" + x + "," + y + "," + z + ");");
    }

    /// <summary>
    /// Performs a compound absolute rotation + translation transformation, or in other words, sets both a new absolute position and orientation for the device in the same action.
    /// </summary>
    /// <param name="x"></param>
    /// <param name="y"></param>
    /// <param name="z"></param>
    /// <param name="x0"></param>
    /// <param name="x1"></param>
    /// <param name="x2"></param>
    /// <param name="y0"></param>
    /// <param name="y1"></param>
    /// <param name="y2"></param>
    public void TransformTo(float x, float y, float z, double x0, double x1, double x2, double y0, double y1, double y2)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("TransformTo(" + x + "," + y + "," + z + "," +
            x0 + "," + x1 + "," + x2 + "," +
            y0 + "," + y1 + "," + y2 + ");");
    }

    /// <summary>
    /// Rotates the device a specified angle in degrees along the specified vector.
    /// </summary>
    /// <param name="x"></param>
    /// <param name="y"></param>
    /// <param name="z"></param>
    /// <param name="angleInc"></param>
    public void Rotate(float x, float y, float z, float angleInc)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("Rotate(" + x + "," + y + "," + z + "," + angleInc + ");");
    }

    /// <summary>
    /// Rotates the devices to an absolute orientation, usually defined by the two main X and Y axes.
    /// </summary>
    /// <param name="x0"></param>
    /// <param name="x1"></param>
    /// <param name="x2"></param>
    /// <param name="y0"></param>
    /// <param name="y1"></param>
    /// <param name="y2"></param>
    public void RotateTo(double x0, double x1, double x2, double y0, double y1, double y2)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("RotateTo(" + x0 + "," + x1 + "," + x2 + "," +
            y0 + "," + y1 + "," + y2 + ");");
    }

    public void Axes(double j1, double j2, double j3, double j4, double j5, double j6)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("Axes(" + j1 + "," + j2 + "," + j3 + "," + j4 + "," + j5 + "," + j6 + ");");
    }

    public void AxesTo(double j1, double j2, double j3, double j4, double j5, double j6)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("AxesTo(" + j1 + "," + j2 + "," + j3 + "," + j4 + "," + j5 + "," + j6 + ");");
    }

    public void Speed(double speedInc)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("Speed(" + speedInc + ");");
    }

    public void SpeedTo(double speed)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("SpeedTo(" + speed + ");");
    }

    public void Acceleration(double accelerationInc)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("Acceleration(" + accelerationInc + ");");
    }

    public void AccelerationTo(double acceleration)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("AccelerationTo(" + acceleration + ");");
    }

    public void RotationSpeed(double rotationSpeedInc)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("RotationSpeed(" + rotationSpeedInc + ");");
    }

    public void RotationSpeedTo(double rotationSpeed)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("RotationSpeedTo(" + rotationSpeed + ");");
    }

    public void JointSpeed(double jointSpeedInc)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("JointSpeed(" + jointSpeedInc + ");");
    }

    public void JointSpeedTo(double jointSpeed)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("JointSpeedTo(" + jointSpeed + ");");
    }

    public void JointAcceleration(double jointAccelerationInc)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("JointAcceleration(" + jointAccelerationInc + ");");
    }

    public void JointAccelerationTo(double jointAcceleration)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("JointAccelerationTo(" + jointAcceleration + ");");
    }

    public void Precision(double precisionInc)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("Precision(" + precisionInc + ");");
    }

    public void PrecisionTo(double precision)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("PrecisionTo(" + precision + ");");
    }

    public void MotionMode(string mode)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("MotionMode(\"" + mode + "\");");
    }

    public void Message(string msg)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("Message(\"" + msg + "\");");
    }

    public void Wait(int millis)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("Wait(" + millis + ");");
    }

    public void PushSettings()
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("PushSettings();");
    }

    public void PopSettings()
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("PopSettings();");
    }

    public void ToolCreate(string name, double x, double y, double z, double x0, double x1, double x2, double y0, double y1, double y2, double weight, double cogX, double cogY, double cogZ)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("Tool.Create(\"" + name + "\"," + x + "," + y + "," + z + "," + x0 + "," + x1 + "," + x2 + "," + y0 + "," + y1 + "," + y2 + "," + weight + "," + cogX + "," + cogY + "," + cogZ + ");");
    }

    public void Attach(string name)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("Attach(\"" + name + "\");");
    }

    public void Detach()
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("Detach()");
    }

    public void WriteDigital(int pin, bool on)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("WriteDigital(" + pin + "," + on + ");");
    }

    public void WriteAnalog(int pin, double value)
    {
        if (machinaBridgeSocket != null)
            machinaBridgeSocket.Send("WriteAnalog(" + pin + "," + value + ");");
    }

#endif
}