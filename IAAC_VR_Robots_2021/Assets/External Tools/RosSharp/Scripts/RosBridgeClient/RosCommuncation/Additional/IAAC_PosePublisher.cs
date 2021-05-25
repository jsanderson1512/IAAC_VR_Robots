using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using System;

public class IAAC_PosePublisher : UnityPublisher<RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose>
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
            PublishPose();
    }

    public void PublishPose()
    {
        Vector3 unityCenter = this.robotBaseFrame.InverseTransformPoint(targetPose.position);
        Vector3 unityEuler = targetPose.eulerAngles - this.robotBaseFrame.eulerAngles;
        Debug.Log("Pose Publisher: point: " + unityCenter.UnityVector3ToRosVector3().SpecialToString() + ", quaternion: " + unityEuler.UnityEulerToRosEuler().SpecialToString());
        if (!testing)
            Publish(new RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose(
                unityCenter.UnityVector3ToRosPoint(),
                unityEuler.UnityEulerToRosQuaternion()));
    }
}
