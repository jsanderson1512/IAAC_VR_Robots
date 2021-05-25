using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using System;

public class IAAC_TwistPublisher : UnityPublisher<RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist>
{
    public Transform targetPose;
    public Transform robotBaseFrame;
    public bool autoname = true;
    public bool autopublish = false;
    public bool testing = false;

    public void OnEnable()
    {
        if (autoname)
            this.Topic += "_" + targetPose.gameObject.name.Replace(" ", string.Empty);
        if (this.robotBaseFrame == null)
            this.robotBaseFrame = targetPose.parent;
    }

    void Update()
    {
        if (autopublish)
            PublishTwist();
    }

    public void PublishTwist()
    {
        Vector3 unityCenter = this.robotBaseFrame.InverseTransformPoint(targetPose.position);
        Vector3 unityEuler = targetPose.eulerAngles - this.robotBaseFrame.eulerAngles;
        Debug.Log("Target euler: " + targetPose.eulerAngles + ". Base euler: " + this.robotBaseFrame.eulerAngles + ". Result: " + unityEuler);
        Debug.Log("Twist Publisher: linear: " + unityCenter.UnityVector3ToRosVector3().SpecialToString() + ", angular: " + unityEuler.UnityEulerToRosEuler().SpecialToString());
        if (!testing)
            Publish(new RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist(
                unityCenter.UnityVector3ToRosVector3(),
                unityEuler.UnityEulerToRosEuler()));
    }
}
