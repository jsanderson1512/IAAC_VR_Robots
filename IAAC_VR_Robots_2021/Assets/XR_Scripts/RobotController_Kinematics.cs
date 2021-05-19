using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

/// <summary>
/// Manages robot motion by setting robot joint angles, either directly or using IK and a target plane.
/// </summary>
public class RobotController_Kinematics : MonoBehaviour
{



    [System.Serializable]
    public enum RobotModel // this public var should appear as a drop down
    {
        ABB_IRB_120_3,// this is UPenn's small robots
        ABB_IRB_4600_60, // this is UPenn's robot
        ABB_IRB_4600_150, //this is greyshed's abe
        UR_3,
        UR_5,
        UR_10
    };
    [System.Serializable]
    public enum MovementMode // this public var should appear as a drop down
    {
        ForwardKinematics, //should we use sliders or incoming fk values to control axis position directly?
        InverseKinematics, //should we instead determine joint angles using inverse kinematics and a plane target?
        RandomKinematics //should we do the silly robot dance and randomly flail about?
    };

    public RobotModel robotModel = RobotModel.ABB_IRB_4600_60;
    public MovementMode movementMode = MovementMode.ForwardKinematics;

    public bool trace = false; //tell us if the robot is going to trace the path

    //jeff these two things i instantiate on start...made private
    public Transform RobotTarget_WorldSpace; //this will be the camera if camera chase is on;
    public Transform ToolCenterPoint_WorldSpace;
    private Transform tcp; //store the transformations of our current rob target and tool center point

    public Transform RobotTarget_RobotSpace;
    private Transform robCoordTarget;

    public Vector3 TCPDifferencePosition;
    public Quaternion TCPDifferenceRotation;


    public GameObject[] joints = new GameObject[6]; //these game objects represent (and hold the contents of) each joint, and will be transformed by joint angles...
    public GameObject[] links = new GameObject[4]; //these gameobjects are only for Abe (IRB 6400) to hold piston/counterweight info

    private float[] jointAngles = { 0f, 0f, 0f, 0f, 0f, 0f }; //an array to hold our joint angles for setting robot position
    private float[] axisDir = { 1f, 1f, 1f, 1f, 1f, 1f }; //Direction of axis motion (positive or negative).  Used in random flailing...
    private double[] lastAngles = { 0f, 0f, 0f, 0f, 0f, 0f }; //store the last known angles so that we can choose the correct kinematic configuration, or at the very least stay put if we can't reach a point

    public List<List<Pose>> targetPaths = new List<List<Pose>>(); //hold the paths to drive through


    private bool started = false; //use a boolean variable to reset our time counter
    private float startTime = 0f; //store our starting time so we can know how much has elapsed
    public float speed = .1f; //speed of robot in m/s
    public float axisSpeed = 0.5f; //axis speed

    public float[] toolOrient = { 0f, 0f, 0f, 0f }; //store our tool direction as quaternion (in robot coord system)

    [Header("Forward Kinematics Sliders")]
    //create some sliders to alter our joint angles (numbers should be adjusted to axis limit values for specific robot), and float values for each that will later fill our joint angles array
    [Range(-360.0f, 360.0f)]
    public float axis1 = 20f;
    [Range(-360.0f, 360.0f)]
    public float axis2 = 25f;
    [Range(-360.0f, 360.0f)]
    public float axis3 = 90f;
    [Range(-360.0f, 360.0f)]
    public float axis4 = 0f;
    [Range(-360.0f, 360.0f)]
    public float axis5 = 0f;
    [Range(-360.0f, 360.0f)]
    public float axis6 = 0f;

    //create dummy values for robot kinematics
    Pose pose1 = new Pose(new Vector3(1, 1, 1), new Quaternion(0, 1, 0, 0));
    Pose pose2 = new Pose(new Vector3(0, 1, 1), new Quaternion(0, 1, 0, 0));
    Pose pose3 = new Pose(new Vector3(1, 1, 2), new Quaternion(0, 1, 0, 0));
    Pose pose4 = new Pose(new Vector3(1, 0, 1), new Quaternion(0, 1, 0, 0));




    //HOW TO CALCULATE DH PARAMETERS:
    // https://blog.robotiq.com/how-to-calculate-a-robots-forward-kinematics-in-5-easy-steps
    // https://www.youtube.com/watch?v=rA9tm0gTln8&ab_channel=TekkotsuRobotics
    // d - the distance between the previous x-axis and the current x-axis, along the previous z-axis.
    // θ - the angle around the z-axis between the previous x-axis and the current x-axis.
    // a(or r) - the length of the common normal, which is the distance between the previous z-axis and the current z-axis
    // α - the angle around the common normal to between the previous z-axis and current z-axis.


    private double[] a = { 0, 0, 0, 0, 0, 0 }; //dh parameters for our robot (actual values to be set in "start"...these are for a 6400) 
    private double[] d = { 0, 0, 0, 0, 0, 0 };

    private double[] aUR3 = { 0, -243.65, -213.25, 0, 0, 0 }; //dh parameters for UR3
    private double[] dUR3 = { 151.9, 0, 0, 112.35, 85.35, 81.9 };

    private double[] aUR5 = { 0, -425.00, -392.25, 0, 0, 0 }; //dh parameters for UR5
    private double[] dUR5 = { 89.459, 0, 0, 109.15, 94.65, 82.3 };

    private double[] aUR10 = { 0, -612, -572.3, 0, 0, 0 }; //dh parameters for UR10
    private double[] dUR10 = { 127.3, 0, 0, 163.941, 115.7, 92.2 };

    private double[] a6400_150 = { 188, 950, 225, 0, 0, 0 }; //dh parameters for our robot (actual values to be set in "start"...these are for a 6400) 
    private double[] d6400_150 = { 900, 0, 0, 1300, 0, 200 };



    /*
     // from: https://github.com/visose/Robots/blob/master/Libraries/nagami.xml
    <RobotCell name="Nagami IRB4600" manufacturer="ABB">
        <Mechanisms>
          <RobotArm model="IRB4600-205N" manufacturer="ABB" payload="45">
            <Base x="0.000" y="0.000" z="0.000" q1="1.000" q2="0.000" q3="0.000" q4="0.000"/>
            <Joints>
              <Revolute number="1" a ="175" d ="495" minrange = "-180" maxrange ="180" maxspeed ="175"/>
              <Revolute number="2" a ="900" d ="0" minrange = "-90" maxrange ="150" maxspeed ="175"/>
              <Revolute number="3" a ="175" d ="0" minrange = "-180" maxrange ="75" maxspeed ="175"/>
              <Revolute number="4" a ="0" d ="960" minrange = "-400" maxrange ="400" maxspeed ="250"/>
              <Revolute number="5" a ="0" d ="0" minrange = "-125" maxrange ="120" maxspeed ="250"/>
              <Revolute number="6" a ="0" d ="135" minrange = "-400" maxrange ="400" maxspeed ="360"/>
            </Joints>
          </RobotArm>
          <Custom model="MU100" manufacturer="ABB" payload="500">
            <Base x="0.000" y="0.000" z="0.000" q1="1.000" q2="0.000" q3="0.000" q4="0.000"/>
            <Joints>
              <Revolute number="7" a ="0.000" d ="0.000" minrange = "-INF" maxrange ="INF" maxspeed ="19800"/>
            </Joints>
          </Custom>
        </Mechanisms>
        <IO>
          <DO names="DO10_1,DO10_2,DO10_3,DO10_4,DO10_5,DO10_6,DO10_7,DO10_8,DO10_9,DO10_10,DO10_11,DO10_12,DO10_13,DO10_14,DO10_15,DO10_16"/>
          <DI names="DI10_1,DI10_2,DI10_3,DI10_4,DI10_5,DI10_6,DI10_7,DI10_8,DI10_9,DI10_10,DI10_11,DI10_12,DI10_13,DI10_14,DI10_15,DO10_16"/>
          <AO names="AO10_1,AO10_2"/>
          <AI names="AI10_1,AI10_2"/>
        </IO>
    </RobotCell>
    */


    private double[] a6400_60 = { 175, 900, 175, 0, 0, 0 }; //dh parameters for our robot (actual values to be set in "start"...these are for a 6400) 
    private double[] d6400_60 = { 495, 0, 0, 960, 0, 135 };

    /*
    // from: https://github.com/visose/Robots/blob/master/Libraries/bartlett.xml
     <RobotCell name="Bartlett-IRB120" manufacturer="ABB">
    <Mechanisms>
      <RobotArm model="IRB120" manufacturer="ABB" payload="3">
        <Base x="0.000" y="0.000" z="0.000" q1="1.000" q2="0.000" q3="0.000" q4="0.000"/>
        <Joints>
          <Revolute number="1" a ="0" d ="290" minrange = "-165" maxrange ="165" maxspeed ="250"/>
          <Revolute number="2" a ="270" d ="0" minrange = "-110" maxrange ="110" maxspeed ="250"/>
          <Revolute number="3" a ="70" d ="0" minrange = "-110" maxrange ="70" maxspeed ="250"/>
          <Revolute number="4" a ="0" d ="302" minrange = "-160" maxrange ="160" maxspeed ="320"/>
          <Revolute number="5" a ="0" d ="0" minrange = "-120" maxrange ="120" maxspeed ="320"/>
          <Revolute number="6" a ="0" d ="72" minrange = "-400" maxrange ="400" maxspeed ="420"/>
        </Joints>
      </RobotArm>
    </Mechanisms>
    <IO>
      <DO names="DO10_1,DO10_2,DO10_3,DO10_4,DO10_5,DO10_6,DO10_7,DO10_8,DO10_9,DO10_10,DO10_11,DO10_12,DO10_13,DO10_14,DO10_15,DO10_16"/>
      <DI names="DI10_1,DI10_2,DI10_3,DI10_4,DI10_5,DI10_6,DI10_7,DI10_8,DI10_9,DI10_10,DI10_11,DI10_12,DI10_13,DI10_14,DI10_15,DO10_16"/>
    </IO>
  </RobotCell>
    */
    private double[] a120_3 = { 0, 270, 70, 0, 0, 0 }; //dh parameters for our robot (actual values to be set in "start"...these are for a 120) 
    private double[] d120_3 = { 290, 0, 0, 302, 0, 72 };


    double[][] axisRange = new double[6][]; //hold our axis limits


    void Start()
    {
        //jeff these two things i instantiate on start... made private
        tcp = new GameObject().transform;
        tcp.name = "tcp";
        tcp.transform.parent = gameObject.transform;
        tcp.transform.localPosition = new Vector3(0, 0, 0);
        tcp.localEulerAngles = new Vector3(0, 0, 0);

        robCoordTarget = new GameObject().transform;
        robCoordTarget.name = "RobotTarget_RobotSpaceRobotUnits";
        robCoordTarget.transform.parent = gameObject.transform;
        robCoordTarget.transform.localPosition = new Vector3(0, 0, 0);
        robCoordTarget.localEulerAngles = new Vector3(0, 0, 0);

        if (RobotTarget_RobotSpace == null)
        {
            RobotTarget_RobotSpace = new GameObject().transform;
            RobotTarget_RobotSpace.name = "RobotTarget_RobotSpaceUnityUnits";
            RobotTarget_RobotSpace.transform.parent = gameObject.transform;
            RobotTarget_RobotSpace.transform.localPosition = new Vector3(0, 0, 0);
            RobotTarget_RobotSpace.localEulerAngles = new Vector3(0, 0, 0);
        }

        TCPDifferencePosition = ToolCenterPoint_WorldSpace.localPosition;
        TCPDifferenceRotation = ToolCenterPoint_WorldSpace.localRotation;
        //TCPDifferenceRotation = new Quaternion( TCPDifferenceRotation.x, TCPDifferenceRotation.z, TCPDifferenceRotation.y, TCPDifferenceRotation.w); //new Quaternion(-q.x, -q.z, q.y, -q.w);

        //do this at application start
        //currentRobTarget.SetPositionAndRotation(new Vector3(0, 0, 0), new Quaternion(1, 0, 0, 0));
        List<Pose> fakePath1 = new List<Pose> { pose1, pose2, pose3, pose4 };
        List<Pose> fakePath2 = new List<Pose> { pose3, pose2, pose1, pose4 };
        targetPaths.Add(fakePath1);
        targetPaths.Add(fakePath2);


        //determine which kind of robot this is based on the tag of our gameobject, so we can set our DH params and other necessary info accordingly.
        #region Set Specific DH and AxisRange Values for the Selected Robot
        if (robotModel == RobotModel.UR_3)//UR3
        {
            a = aUR3;
            d = dUR3;
        }
        else if (robotModel == RobotModel.UR_5)//UR5
        {
            a = aUR5;
            d = dUR5;
        }
        else if (robotModel == RobotModel.UR_10)//UR10
        {
            a = aUR10;
            d = dUR10;
        }
        else if (robotModel == RobotModel.ABB_IRB_4600_150)//6400-150 This is Abe
        {
            a = a6400_150;
            d = d6400_150;

            double[] axis1Range = { -180, 190 };
            double[] axis2Range = { -70, 70 };
            double[] axis3Range = { -28, 105 };
            double[] axis4Range = { -300, 300 };
            double[] axis5Range = { -120, 120 };
            double[] axis6Range = { -300, 300 };

            axisRange[0] = axis1Range;
            axisRange[1] = axis2Range;
            axisRange[2] = axis3Range;
            axisRange[3] = axis4Range;
            axisRange[4] = axis5Range;
            axisRange[5] = axis6Range;
        }
        else if (robotModel == RobotModel.ABB_IRB_4600_60)//this is UPenn Robot
        {
            //jeff working here

            a = a6400_60;
            d = d6400_60;

            double[] axis1Range = { -180, 180 };
            double[] axis2Range = { -90, 150 };
            double[] axis3Range = { -180, 75 };
            double[] axis4Range = { -400, 400 };
            double[] axis5Range = { -125, 120 };
            double[] axis6Range = { -400, 400 };




            axisRange[0] = axis1Range;
            axisRange[1] = axis2Range;
            axisRange[2] = axis3Range;
            axisRange[3] = axis4Range;
            axisRange[4] = axis5Range;
            axisRange[5] = axis6Range;

        }
        else if (robotModel == RobotModel.ABB_IRB_120_3)//this is UPenn Robot
        {
            //jeff working here

            a = a120_3;
            d = d120_3;

            double[] axis1Range = { -165, 165 };
            double[] axis2Range = { -110, 110 };
            double[] axis3Range = { -110, 70 };
            double[] axis4Range = { -182, 182 };//i changed this because the IK response is near 180 or -180
            //double[] axis4Range = { -160, 160 };

            double[] axis5Range = { -120, 120 };
            double[] axis6Range = { -400, 400 };


             

            axisRange[0] = axis1Range;
            axisRange[1] = axis2Range;
            axisRange[2] = axis3Range;
            axisRange[3] = axis4Range;
            axisRange[4] = axis5Range;
            axisRange[5] = axis6Range;

        }
        #endregion

    }

    public void SetJointAngles(float[] incomingAngles)
    {
        jointAngles[0] = incomingAngles[0]; //set our joint angles from the sliders
        jointAngles[1] = incomingAngles[1];
        jointAngles[2] = incomingAngles[2];
        jointAngles[3] = incomingAngles[3];
        jointAngles[4] = incomingAngles[4];
        jointAngles[5] = incomingAngles[5];
    }

    void Update()
    {


        #region ForwardKinematics (Working Fine with Sliders)
        //if we are setting joint angles directly, set them.
        if (movementMode == MovementMode.ForwardKinematics)
        {
            // Debug.Log("im using fk, joint angle [0] =" + jointAngles[0]);
            //  Debug.Log("im using fk, joint angle [1] =" + jointAngles[1]);
            //  Debug.Log("im using fk, joint angle [2] =" + jointAngles[2]);
            //  Debug.Log("im using fk, joint angle [3] =" + jointAngles[3]);
            //  Debug.Log("im using fk, joint angle [4] =" + jointAngles[4]);
            //  Debug.Log("im using fk, joint angle [5] =" + jointAngles[5]);


            jointAngles[0] = axis1; //set our joint angles from the sliders
            jointAngles[1] = axis2;
            jointAngles[2] = axis3;
            jointAngles[3] = axis4;
            jointAngles[4] = axis5;
            jointAngles[5] = axis6;


            if (robotModel == RobotModel.UR_3 || robotModel == RobotModel.UR_5 || robotModel == RobotModel.UR_10)
            {
                //adjust for having rotated joints on two axis in the home position
                jointAngles[0] += 90;
                jointAngles[1] += 90;
                jointAngles[3] += 90;
            }
        } //end if useFK
        #endregion

        #region RandomKinematics (Working Fine)
        //else, if we're using random vars, flail about like a mad robot
        else if (movementMode == MovementMode.RandomKinematics)
        {
            for (int i = 0; i < 6; i++)
            {
                jointAngles[i] = jointAngles[i] + axisSpeed * axisDir[i]; //add some angle to each joint
                if (jointAngles[i] > (i + 1) * 30 || jointAngles[i] < -30 * (i + 1))
                { //if you move beyond a certain number of degrees (different per joint), change direction
                    axisDir[i] *= -1; //reverse our direction of movement
                }
            }

            //set our angle sliders so we can switch back and forth between ik and fk
            axis1 = jointAngles[0];
            axis2 = jointAngles[1];
            axis3 = jointAngles[2];
            axis4 = jointAngles[3];
            axis5 = jointAngles[4];
            axis6 = jointAngles[5];

        } //end if useRandom
        #endregion

        else if (movementMode == MovementMode.InverseKinematics)
        {

                //>>>>>>>MANIPULATING TCP BASED ON CHANGES IN ROBOT MODEL IN UNITY
                //define our tool data from the toolpos and toolOrient values hardcoded in

                Vector3 TCPDifferencePosition2 = new Vector3(-1 * 1000f * TCPDifferencePosition.x, -1 * 1000f * TCPDifferencePosition.z, 1000f * TCPDifferencePosition.y);
                tcp.position = TCPDifferencePosition2;

                tcp.rotation = new Quaternion(toolOrient[1], toolOrient[2], toolOrient[3], toolOrient[0]); //unity uses x,y,z,w convention, rapid is w,x,y,z                                                                                                      

                if (RobotTarget_WorldSpace)
                {
                    //convert world space target into local space of each robot
                    RobotTarget_RobotSpace.position = RobotTarget_WorldSpace.position;//set position in world to match the robot target
                                                                                      //RobotTarget_RobotSpace.localPosition = RobotTarget_RobotSpace.localPosition + new Vector3(TCPDifferencePosition.x, TCPDifferencePosition.y, TCPDifferencePosition.z);

                    RobotTarget_RobotSpace.rotation = RobotTarget_WorldSpace.rotation;//set position in world to match the robot target
                }

                //works!
                RobotTarget_RobotSpace.localRotation = RobotTarget_RobotSpace.localRotation * Quaternion.Inverse(TCPDifferenceRotation);

                //Debug.Log("temporary debug: robot target quat is: " + RobotTarget_RobotSpace.localRotation);
                //<<<<<<<MANIPULATING TCP BASED ON CHANGES IN ROBOT MODEL IN UNITY
            



            Vector3 myPos = RobotTarget_RobotSpace.localPosition;  //jeff changed this to local position so its in each robot coordinate system
            Quaternion myRotation = RobotTarget_RobotSpace.localRotation; //jeff changed this to local rotation





            if (trace)
            {
                if (!started)
                {
                    startTime = Time.time; //store the current time
                    started = true;
                }
                //if we are tracing a preview path, get our CurrentRobTarget transform along the designated curves, replacing our fixed position and rotation of currentRobTaret
                Pose currentPlane = getCurrentPlane(targetPaths, startTime, speed); //given the list of paths, our start time, and linear speed, return a plane for where we should be at roughly at the current time
                myPos = currentPlane.position;  //get the sub components of our target plane
                myRotation = currentPlane.rotation;
                RobotTarget_RobotSpace.SetPositionAndRotation(myPos, myRotation);
            }//end if trace





            //convert plane from unity convention to robot convention
            Vector3 targetPos = new Vector3(-1 * myPos.x * 1000f, -1 * myPos.z * 1000f, myPos.y * 1000f); //convert positon to mm and "correct" (robot) coordinate system
            Quaternion targetOrient = new Quaternion(-1 * myRotation.x, -1 * myRotation.z, myRotation.y, -1 * myRotation.w); //new Quaternion(-q.x, -q.z, q.y, -q.w);



            //set a transform variable to store our current target
            robCoordTarget.SetPositionAndRotation(targetPos, targetOrient);


            //may need to transform the quaternion of the tool

            //Vector3 targetPos = new Vector3(-1 * myPos.x * 1000f, -1 * myPos.z * 1000f, myPos.y * 1000f); //convert positon to mm and "correct" (robot) coordinate system
            //Quaternion targetOrient = new Quaternion(-1 * myRotation.x, -1 * myRotation.z, myRotation.y, -1 * myRotation.w); //new Quaternion(-q.x, -q.z, q.y, -q.w);


            //get the inverse kinematic solution for this target plane, given the target, the tooldata, and the last target
            double[] ikResponse = new double[0];
            //use the appropriate kinematic solver

            if (robotModel == RobotModel.ABB_IRB_4600_150)
            {
                ikResponse = ABB6400InverseKinematics(robCoordTarget, tcp, lastAngles);
            }
            else if (robotModel == RobotModel.ABB_IRB_4600_60)
            {
                ikResponse = ABB6400InverseKinematics(robCoordTarget, tcp, lastAngles);
            }
            else if (robotModel == RobotModel.ABB_IRB_120_3)
            {
                ikResponse = ABB6400InverseKinematics(robCoordTarget, tcp, lastAngles);
            }
            else if (robotModel == RobotModel.UR_3 || robotModel == RobotModel.UR_5 || robotModel == RobotModel.UR_10)
            {
                ikResponse = URInverseKinematics(robCoordTarget, tcp, lastAngles);
            }

            //set our joint angles based on the response from this function...
            jointAngles[0] = (float)ikResponse[0];
            jointAngles[1] = (float)ikResponse[1];
            jointAngles[2] = (float)ikResponse[2];
            jointAngles[3] = (float)ikResponse[3];
            jointAngles[4] = (float)ikResponse[4];
            jointAngles[5] = (float)ikResponse[5];

            axis1 = jointAngles[0]; //set our angle sliders so we can switch back and forth between ik and fk
            axis2 = jointAngles[1];
            axis3 = jointAngles[2];
            axis4 = jointAngles[3];
            axis5 = jointAngles[4];
            axis6 = jointAngles[5];


            //unsure if this is valid...? Just changing the sliders...
            if (robotModel == RobotModel.UR_3 || robotModel == RobotModel.UR_5 || robotModel == RobotModel.UR_10)
            {
                //adjust for having rotated joints on two axis in the home position
                axis2 -= 90;
                axis4 -= 90;
            }
        }//end if IK

        lastAngles = Array.ConvertAll(jointAngles, x => (double)x); //cast all of our float angles to a double.  Messy, sorry.

        if (robotModel == RobotModel.ABB_IRB_4600_150)
        {
            //if Abe, set our transforms accordingly
            //get the angle of the upper piston
            float angleAxis2 = Mathf.Deg2Rad * jointAngles[1];
            float L = Mathf.Sin(Mathf.Abs(angleAxis2)) * 280;
            float B = Mathf.Sqrt(280f * 280f - L * L);
            float upperPistonAngle = Mathf.Atan(L / (950 - B));
            if (angleAxis2 < 0) upperPistonAngle *= -1;
            //if 3-2 > 65 || abs(3-2) >65 -parallelogram state
            //set the position of our pistons and counterweights
            links[0].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, upperPistonAngle * Mathf.Rad2Deg));
            links[1].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, 180 + (-1 * (180 - (upperPistonAngle * Mathf.Rad2Deg) - jointAngles[1]))));
            links[2].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, -1 * (jointAngles[2] - jointAngles[1])));
            links[3].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, jointAngles[2]));
            joints[0].transform.localRotation = Quaternion.Euler(new Vector3(0f, -1 * jointAngles[0], 0f));
            joints[1].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, jointAngles[1]));
            joints[2].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, jointAngles[2] - jointAngles[1]));
            joints[3].transform.localRotation = Quaternion.Euler(new Vector3(jointAngles[3], 0f, 0f));
            joints[4].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, jointAngles[4]));
            joints[5].transform.localRotation = Quaternion.Euler(new Vector3(jointAngles[5], 0f, 0f));
        }
        else if (robotModel == RobotModel.ABB_IRB_4600_60)
        {
            //U Penn Robot, working here.

            joints[0].transform.localRotation = Quaternion.Euler(new Vector3(0f, -1 * jointAngles[0], 0f)); //reversing y direction
            joints[1].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, jointAngles[1])); //reversing z direction
            joints[2].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, -1 * jointAngles[2])); //reversing z direction
            joints[3].transform.localRotation = Quaternion.Euler(new Vector3(jointAngles[3], 0f, 0f)); //reversing z direction
            joints[4].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, -1 * jointAngles[4]));
            joints[5].transform.localRotation = Quaternion.Euler(new Vector3(-1 * jointAngles[5], 0f, 0f));
        }
        else if (robotModel == RobotModel.ABB_IRB_120_3)
        {
            //U Penn Robot, working here.

            joints[0].transform.localRotation = Quaternion.Euler(new Vector3(0f, -1 * jointAngles[0], 0f)); //y
            joints[1].transform.localRotation = Quaternion.Euler(new Vector3(0f,  0f, jointAngles[1])); //z
            joints[2].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, -1 * jointAngles[2] )); //z
            joints[3].transform.localRotation = Quaternion.Euler(new Vector3(jointAngles[3], 0f, 0f)); //x
            joints[4].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, -1 * jointAngles[4]));//z
            joints[5].transform.localRotation = Quaternion.Euler(new Vector3(-1 * jointAngles[5], 0f, 0f));//x
        }
        else if (robotModel == RobotModel.UR_3 || robotModel == RobotModel.UR_5 || robotModel == RobotModel.UR_10)
        {
            //otherwise we are a UR arm
            //set the transformation data for each joint based on the joint values, no matter which type of kinematic option we are using (FK/IK/Random)
            joints[0].transform.localRotation = Quaternion.Euler(new Vector3(0f, -1 * jointAngles[0], 0f));
            joints[1].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, -1 * jointAngles[1]));
            joints[2].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, -1 * jointAngles[2]));
            joints[3].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, -1 * jointAngles[3]));
            joints[4].transform.localRotation = Quaternion.Euler(new Vector3(0f, -1 * jointAngles[4], 0f));
            joints[5].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, -1 * jointAngles[5]));
        }
    } //end Update

    #region ABB IRB 6400 InverseKinematics
    /// <summary>
    /// Gets joint angles of a robot target, in Robot coordinates, right hand rule, etc. (mm and quaternions)
    /// </summary>
    /// <remarks>
    /// Modified spherical wrist IK with added parameters for counterweight and pistons, need to check parallelogram state, etc. in future.-RLJ
    /// </remarks>
    /// <returns>
    /// 6 joint angles, in degrees.
    /// </returns>
    /// <param name="theTarget">The target we should get the IK solution for.</param>
    /// <param name="theTool">The tooldata of our current tool, in right hand rule/robot coords/mm.</param>
    /// <param name="previousAngles">The angles of where the robot was currently at.</param>
    double[] ABB6400InverseKinematics(Transform theTarget, Transform theTool, double[] previousAngles)
    {
        // from: https://github.com/visose/Robots/blob/master/Robots/Kinematics.cs

        List<string> errors = errors = new List<string>();

        bool isUnreachable = false;

        //a and d values come from start function

        double[] joints = new double[6];
        double l2 = Math.Sqrt(a[2] * a[2] + d[3] * d[3]);
        double ad2 = Math.Atan2(a[2], d[3]);


        /*
        //Matrix4x4 flange = theTarget.localToWorldMatrix; //get the matrix for our target transformation
        Matrix4x4 flange = Matrix4x4.TRS(theTarget.localPosition, theTarget.localRotation, theTarget.localScale);


        //theTool.rotation = Quaternion.Inverse(theTool.rotation);

        //Matrix4x4 toolM = theTool.worldToLocalMatrix; //get the inverted matrix for our tool transformation
        //Matrix4x4 toolM = theTool.localToWorldMatrix; //get the inverted matrix for our tool transformation
        Matrix4x4 toolM = Matrix4x4.TRS(theTool.localPosition, theTool.localRotation, theTool.localScale);
        toolM = toolM.inverse;
        //JEFF NEED TO INVERT QUATERNION??


        flange = flange * toolM; //get the transformed target plane that tells us where the flange (no tool) needs to be

        Vector3 flangeCenter = new Vector3(flange.m03, flange.m13, flange.m23); //center point of our tool
        Quaternion flangeOrient = flange.rotation; //direction of our tool

        Vector3 flangeNormal = flangeOrient * new Vector3(0, 0, 1); //get the z axis vector of our flange


        Vector3 center = flangeCenter - flangeNormal * (float)d[5]; //move the flange back along axis 6 to find the centerpoint of axis5

        */

        Matrix4x4 flange = theTarget.localToWorldMatrix; //get the matrix for our target transformation
        Matrix4x4 toolM = theTool.worldToLocalMatrix; //get the inverted matrix for our tool transformation
        flange = flange * toolM; //get the transformed target plane that tells us where the flange (no tool) needs to be

        Vector3 flangeCenter = new Vector3(flange.m03, flange.m13, flange.m23); //center point of our tool
        Quaternion flangeOrient = flange.rotation; //direction of our tool
        Vector3 flangeNormal = flangeOrient * new Vector3(0, 0, 1); //get the z axis vector of our flange

        Vector3 center = flangeCenter - flangeNormal * (float)d[5]; //move the flange back along axis 6 to find the centerpoint of axis5



        joints[0] = Math.Atan2(center.y, center.x); //axis 1 is going to point in this direction
        double ll = Math.Sqrt(center.x * center.x + center.y * center.y);
        Vector3 p1 = new Vector3((float)a[0] * center.x / (float)ll, (float)a[0] * center.y / (float)ll, (float)d[0]);





        double l3 = (center - p1).magnitude;
        double l1 = a[1];
        double beta = Math.Acos((l1 * l1 + l3 * l3 - l2 * l2) / (2 * l1 * l3));
        if (double.IsNaN(beta))
        {
            beta = 0;
            isUnreachable = true;
        }


        double ttl = new Vector3(center.x - p1.x, center.y - p1.y, 0).magnitude;
        // if (p1.X * (center.X - p1.X) < 0)

        double al = Math.Atan2(center.z - p1.z, ttl);

        joints[1] = beta + al;

        double gama = Math.Acos((l1 * l1 + l2 * l2 - l3 * l3) / (2 * l1 * l2));
        if (double.IsNaN(gama))
        {
            gama = Math.PI;
            isUnreachable = true;
        }


        joints[2] = gama - ad2 - Math.PI / 2;

        double[] c = new double[3];
        double[] s = new double[3];
        for (int i = 0; i < 3; i++)
        {
            c[i] = Math.Cos(joints[i]);
            s[i] = Math.Sin(joints[i]);
        }

        Matrix4x4 arr = new Matrix4x4();
        arr[0, 0] = (float)(c[0] * (c[1] * c[2] - s[1] * s[2])); arr[0, 1] = (float)s[0]; arr[0, 2] = (float)(c[0] * (c[1] * s[2] + s[1] * c[2])); arr[0, 3] = (float)(c[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * c[0]);
        arr[1, 0] = (float)(s[0] * (c[1] * c[2] - s[1] * s[2])); arr[1, 1] = (float)-c[0]; arr[1, 2] = (float)(s[0] * (c[1] * s[2] + s[1] * c[2])); arr[1, 3] = (float)(s[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * s[0]);
        arr[2, 0] = (float)(s[1] * c[2] + c[1] * s[2]); arr[2, 1] = 0; arr[2, 2] = (float)(s[1] * s[2] - c[1] * c[2]); arr[2, 3] = (float)(a[2] * (s[1] * c[2] + c[1] * s[2]) + a[1] * s[1] + d[0]);
        arr[3, 0] = 0; arr[3, 1] = 0; arr[3, 2] = 0; arr[3, 3] = 1;



        Matrix4x4 in123 = arr.inverse;

        Matrix4x4 transform = theTarget.localToWorldMatrix; //get the matrix for our target transformation

        Matrix4x4 mr = in123 * transform;
        joints[3] = Math.Atan2(mr[1, 2], mr[0, 2]);
        joints[4] = Math.Acos(mr[2, 2]);
        joints[5] = Math.Atan2(mr[2, 1], -mr[2, 0]);


        for (int i = 0; i < 6; i++)
        {
            if (joints[i] > Math.PI) joints[i] -= 2 * Math.PI;
            if (joints[i] < -Math.PI) joints[i] += 2 * Math.PI;
        }

        if (isUnreachable)
            errors.Add($"Target out of reach");

        if (Math.Abs(1 - mr[2, 2]) < 0.0001)
            errors.Add($"Near wrist singularity");

        float SingularityTol = 0.0001f;
        if (new Vector3(center.x, center.y, 0).magnitude < a[0] + SingularityTol)
            errors.Add($"Near overhead singularity");

        for (int i = 0; i < 6; i++)
        {
            if (double.IsNaN(joints[i])) joints[i] = 0;
        }
        //return joints;




        //convert rads to degrees
        double[] degs = { (float)joints[0] * Mathf.Rad2Deg, (float)joints[1] * Mathf.Rad2Deg, (float)joints[2] * Mathf.Rad2Deg, (float)joints[3] * Mathf.Rad2Deg, (float)joints[4] * Mathf.Rad2Deg, (float)joints[5] * Mathf.Rad2Deg };


        if (robotModel == RobotModel.ABB_IRB_4600_60)//this is UPenn Robot
        {
            //make some corrections
            degs[1] = 90f - degs[1];
            degs[5] = 180f - degs[5];

        }
        else if (robotModel == RobotModel.ABB_IRB_120_3)//this is UPenn Robot
        {
            //make some corrections
            degs[1] = 90f - degs[1];


            //degs[3] = 180f - Mathf.Abs((float)degs[3]);
            
            //degs[4] = 90.0f - degs[4] ;

            //degs 2 is too high... maybe 180
            //degs 3 is too high... maybe 180

            degs[5] = 180f - degs[5];

        }
        else if (robotModel == RobotModel.ABB_IRB_4600_150)//6400-150 This is Abe
        {

            degs[1] = 90f - degs[1]; //correct for parallelogram kinematics
            degs[4] *= -1;
            degs[2] = degs[1] - degs[2];//correct for parallelogram kinematics
            degs[5] = 180f - degs[5];
            degs[5] *= -1;

            //Debug.Log(degs[1] + "," + degs[2] + "," + Math.Abs(degs[1] - degs[2]));
            if (Math.Abs(degs[1] - degs[2]) > 60)
            {
                errors.Add("Parallelogram State"); //approximate.  Maybe should use 60...
            }
        }





        for (int i = 0; i < 6; i++)
        {
            if (degs[i] > axisRange[i][1] || degs[i] < axisRange[i][0])
            {
                //Debug.Log("Axis " + (i+1) + " out of range." + degs[i]);
                errors.Add("Axis " + (i + 1) + " out of range.");
            }
        }

        if (errors.Count > 0)
        {
            //we have an error
            return previousAngles;
        }
        else
        {
            return degs;
        }











    }
    #endregion

    #region URInverseKinematics
    /// <summary>
    /// Gets joint angles of a robot target, in Robot coordinates, right hand rule, etc. (mm and quaternions)
    /// </summary>
    /// <remarks>
    ///Produces 4-8 solutions depending on elbow bend, etc.  Currently uses a probably very dumb strategy to choose between them:  the one with the least angle change from the previous position.-RLJ
    /// </remarks>
    /// <returns>
    /// 6 joint angles, in degrees.
    /// </returns>
    /// <param name="theTarget">The target we should get the IK solution for.</param>
    /// <param name="theTool">The tooldata of our current tool, in right hand rule/robot coords/mm.</param>
    /// <param name="previousAngles">The angles of where the robot was currently at.</param>
    double[] URInverseKinematics(Transform theTarget, Transform theTool, double[] previousAngles)
    {
        //from: 
        //https://github.com/ros-industrial/universal_robot/blob/indigo-devel/ur_kinematics/src/ur_kin.cpp and 
        double[] joints = new double[6];
        //convert incoming values to radians once.
        double[] previousAnglesRad = { previousAngles[0] * Mathf.Deg2Rad, previousAngles[1] * Mathf.Deg2Rad, previousAngles[2] * Mathf.Deg2Rad, previousAngles[3] * Mathf.Deg2Rad, previousAngles[4] * Mathf.Deg2Rad, (float)previousAngles[5] * Mathf.Deg2Rad };

        //return joints;
        double[] q_sols = new double[8 * 6]; //store each possible solution
        double d1 = d[0]; //set our vars from the array so I don't have to retype everything
        double a2 = a[1];
        double a3 = a[2];
        double d4 = d[3];
        double d5 = d[4];
        double d6 = d[5];

        int num_sols = 0; //store our number of solutions
        double ZERO_THRESH = 0.00000001;
        const double PI = Math.PI;

        Matrix4x4 flange = theTarget.localToWorldMatrix; //get the matrix for our target transformation
        Matrix4x4 toolM = theTool.worldToLocalMatrix; //get the inverted matrix for our tool transformation
        flange = flange * toolM; //get the transformed target plane that tells us where the flange (no tool) needs to be
        Vector3 flangeCenter = new Vector3(flange.m03, flange.m13, flange.m23); //center point of our tool
        Quaternion flangeOrient = flange.rotation; //direction of our tool
        Vector3 flangeNormal = flangeOrient * new Vector3(0, 0, 1); //get the z axis vector of our flange

        double T00 = flange.m00; //set our vars from the flange matrix so I don't have to retype everything
        double T01 = flange.m01;
        double T02 = flange.m02;
        double T03 = flange.m03;
        double T10 = flange.m10;
        double T11 = flange.m11;
        double T12 = flange.m12;
        double T13 = flange.m13;
        double T20 = flange.m20;
        double T21 = flange.m21;
        double T22 = flange.m22;
        double T23 = flange.m23;
        double q6_des = 0; //look into what this should be, or will likely get strange results sometimes.

        //we might need to rotate our flange 90 degrees

        ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
        double[] q1 = new double[2];

        double A = d6 * T12 - T13;
        double B = d6 * T02 - T03;
        double R = A * A + B * B;
        if (Math.Abs(A) < ZERO_THRESH)
        {
            double div;
            if (Math.Abs(Math.Abs(d4) - Math.Abs(B)) < ZERO_THRESH)
                div = -Math.Sign(d4) * Math.Sign(B);
            else
                div = -d4 / B;
            double arcsin = Math.Asin(div);
            if (Math.Abs(arcsin) < ZERO_THRESH)
                arcsin = 0.0;
            if (arcsin < 0.0)
                q1[0] = arcsin + 2.0 * PI;
            else
                q1[0] = arcsin;
            q1[1] = PI - arcsin;
        }
        else if (Math.Abs(B) < ZERO_THRESH)
        {
            double div;
            if (Math.Abs(Math.Abs(d4) - Math.Abs(A)) < ZERO_THRESH)
                div = Math.Sign(d4) * Math.Sign(A);
            else
                div = d4 / A;
            double arccos = Math.Acos(div);
            q1[0] = arccos;
            q1[1] = 2.0 * PI - arccos;
        }
        else if (d4 * d4 > R)
        {
            Debug.Log("Exception Num Solutions: " + num_sols);
            return previousAngles; //maybe this isn't what we should do
        }
        else
        {
            double arccos = Math.Acos(d4 / Math.Sqrt(R));
            double arctan = Math.Atan2(-B, A);
            double pos = arccos + arctan;
            double neg = -arccos + arctan;
            if (Math.Abs(pos) < ZERO_THRESH)
                pos = 0.0;
            if (Math.Abs(neg) < ZERO_THRESH)
                neg = 0.0;
            if (pos >= 0.0)
                q1[0] = pos;
            else
                q1[0] = 2.0 * PI + pos;
            if (neg >= 0.0)
                q1[1] = neg;
            else
                q1[1] = 2.0 * PI + neg;
        }

        ////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
        double[][] q5 = new double[2][];
        q5[0] = new double[2];
        q5[1] = new double[2];

        for (int i = 0; i < 2; i++)
        {
            double numer = (T03 * Math.Sin(q1[i]) - T13 * Math.Cos(q1[i]) - d4);
            double div;
            if (Math.Abs(Math.Abs(numer) - Math.Abs(d6)) < ZERO_THRESH)
            {
                div = Math.Sign(numer) * Math.Sign(d6);
            }

            else
            {
                div = numer / d6;
            }
            double arccos = Math.Acos(div);
            q5[i][0] = arccos;
            q5[i][1] = 2.0 * PI - arccos;
        }

        ////////////////////////////////////////////////////////////////////////////////


        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                double c1 = Math.Cos(q1[i]), s1 = Math.Sin(q1[i]);
                double c5 = Math.Cos(q5[i][j]), s5 = Math.Sin(q5[i][j]);
                double q6;
                ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
                if (Math.Abs(s5) < ZERO_THRESH)
                    q6 = q6_des;
                else
                {
                    q6 = Math.Atan2(Math.Sign(s5) * -(T01 * s1 - T11 * c1), Math.Sign(s5) * (T00 * s1 - T10 * c1));
                    if (Math.Abs(q6) < ZERO_THRESH)
                        q6 = 0.0;
                    if (q6 < 0.0)
                        q6 += 2.0 * PI;
                }
                ////////////////////////////////////////////////////////////////////////////////

                double[] q2 = new double[2];
                double[] q3 = new double[2];
                double[] q4 = new double[2];

                ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
                double c6 = Math.Cos(q6), s6 = Math.Sin(q6);
                double x04x = -s5 * (T02 * c1 + T12 * s1) - c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1));
                double x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5;
                double p13x = d5 * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1)) - d6 * (T02 * c1 + T12 * s1) +
                              T03 * c1 + T13 * s1;
                double p13y = T23 - d1 - d6 * T22 + d5 * (T21 * c6 + T20 * s6);

                double c3 = (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3) / (2.0 * a2 * a3);
                if (Math.Abs(Math.Abs(c3) - 1.0) < ZERO_THRESH)
                    c3 = Math.Sign(c3);
                else if (Math.Abs(c3) > 1.0)
                {
                    // TODO NO SOLUTION
                    continue;
                }
                double arccos = Math.Acos(c3);
                q3[0] = arccos;
                q3[1] = 2.0 * PI - arccos;
                double denom = a2 * a2 + a3 * a3 + 2 * a2 * a3 * c3;
                double s3 = Math.Sin(arccos);
                A = (a2 + a3 * c3);
                B = a3 * s3;
                q2[0] = Math.Atan2((A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom);
                q2[1] = Math.Atan2((A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom);
                double c23_0 = Math.Cos(q2[0] + q3[0]);
                double s23_0 = Math.Sin(q2[0] + q3[0]);
                double c23_1 = Math.Cos(q2[1] + q3[1]);
                double s23_1 = Math.Sin(q2[1] + q3[1]);
                q4[0] = Math.Atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0);
                q4[1] = Math.Atan2(c23_1 * x04y - s23_1 * x04x, x04x * c23_1 + x04y * s23_1);
                ////////////////////////////////////////////////////////////////////////////////
                for (int k = 0; k < 2; k++)
                {
                    if (Math.Abs(q2[k]) < ZERO_THRESH)
                        q2[k] = 0.0;
                    else if (q2[k] < 0.0) q2[k] += 2.0 * PI;
                    if (Math.Abs(q4[k]) < ZERO_THRESH)
                        q4[k] = 0.0;
                    else if (q4[k] < 0.0) q4[k] += 2.0 * PI;
                    q_sols[num_sols * 6 + 0] = q1[i]; q_sols[num_sols * 6 + 1] = q2[k];
                    q_sols[num_sols * 6 + 2] = q3[k]; q_sols[num_sols * 6 + 3] = q4[k];
                    q_sols[num_sols * 6 + 4] = q5[i][j]; q_sols[num_sols * 6 + 5] = q6;
                    num_sols++;
                }
            }
        }

        if (num_sols > 0)
        {
            //if we have one or more solution...
            //a pretty quick and dirty way to find a solution that is near to the current position:  check total angle change.  Most likely this is not the correct way to do this...
            double recordLow = double.MaxValue; //set a really big number as our record for smallest number
            int recordSolution = 0; //store our best answer
            for (int i = 0; i < num_sols; i++)
            { //for each found solution
                double distSum = 0; //store our total joint difference
                for (int j = 0; j < 6; j++)
                {
                    distSum += Math.Abs(previousAnglesRad[j] - q_sols[i * 6 + j]); //get the angle difference between where the robot is at, and where it would be with this solution for this axis
                }
                if (distSum < recordLow)
                {   //we've got a new record for least motion...
                    recordLow = distSum;
                    recordSolution = i;
                }
            }


            //Debug.Log("NUMBER OF SOLUTIONS: " + num_sols);
            for (int i = 0; i < 6; i++)
            {   //get the record solution from each value
                joints[i] = q_sols[recordSolution * 6 + i];
            }
            //Debug.Log(joints[0] + ", " + joints[1] + ", " + joints[2] + ", " + joints[3] + ", " + joints[4] + ", " + joints[5]);

            double[] degs = { (float)joints[0] * Mathf.Rad2Deg, (float)joints[1] * Mathf.Rad2Deg, (float)joints[2] * Mathf.Rad2Deg, (float)joints[3] * Mathf.Rad2Deg, (float)joints[4] * Mathf.Rad2Deg, (float)joints[5] * Mathf.Rad2Deg };
            //degs = degs2;

            //argh
            degs[1] += 90f; //rlj
            degs[3] += 90f; //rlj
            //degs[5] = 90f - degs[5]; //jsa

            //Debug.Log(degs[0] + ", " + degs[1] + ", " + degs[2] + ", " + degs[3] + ", " + degs[4] + ", " + degs[5]);
            return degs;
        }

        else
        {
            return previousAngles;
        }
    }
    #endregion

    #region Get Current Plane Method
    /// <summary>
    /// Gets the plane that is linearly interpolated along a list of planes based on start time and linear speed (for smooth animation of paths)
    /// </summary>
    /// <remarks>
    /// Does not account for acceleration and speed changes, but does return planes linearly interpolated between given planes.-RLJ
    /// </remarks>
    /// <returns>
    /// The current plane.
    /// </returns>
    /// <param name="crvs">The entire list of target curves, as a nested list (path curves are separated into groups of targets, with linear interpolation between each path).</param>
    /// <param name="beginTime">The start time of the motion.</param>
    /// <param name="velocity">Linear speed in m/s.</param>
    Pose getCurrentPlane(List<List<Pose>> crvs, float beginTime, float velocity)
    {
        Pose theAnswer = new Pose(); //the pose to return
        float goalDist = (Time.time - beginTime) * velocity; //distance along path at current speed
        float totalDistance = 0;
        bool firstPt = true; //check if we are on the very first plane
        Pose lastPose = new Pose();
        for (int i = 0; i < crvs.Count; i++)
        { //for each curve
            for (int j = 0; j < crvs[i].Count; j++)
            { //for each plane/vertex on this curve
                if (firstPt)
                {   //if we are on the very first point, we can't interpolate from the last one
                    if (goalDist == 0)
                    {   //if we haven't moved at all, return the very first point
                        return crvs[0][0];
                    }
                    lastPose = crvs[0][0]; //now that we've returned the first point once, save that value and use it to interplate to on the next round
                    firstPt = false; //don't come back here.
                }
                else
                {   //if not the first point...
                    float segLength = Vector3.Distance(lastPose.position, crvs[i][j].position); //get the length of this curve segment (between two vertices)
                    if (segLength + totalDistance > goalDist)
                    {   //if the length of this segment added to the distance travelled so far is greater than our goal distance
                        //the target plane must fall within this line segment...
                        float goalSegLength = goalDist - totalDistance; //get the length along this segment
                        float fraction = goalSegLength / segLength; //get the percentage along this segment 
                        Vector3 thePos = Vector3.Lerp(lastPose.position, crvs[i][j].position, fraction); //interpolate along this segment
                        Quaternion theOrient = Quaternion.Lerp(lastPose.rotation, crvs[i][j].rotation, fraction);
                        return new Pose(thePos, theOrient); //return that plane
                    }
                    else
                    {
                        totalDistance += segLength; //add this segment length to the total travelled length, and check the next segment...
                        lastPose = crvs[i][j];
                        theAnswer = crvs[i][j];
                    }
                }
            }
        }
        return theAnswer;
    }
    #endregion
}
