using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using System;

public class PosePublisher : UnityPublisher<RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose>
{
    public Transform targetPose;
    public Transform robotBaseFrame;

    public void OnEnable()
    {
        //this.Topic += "_" + targetPose.gameObject.name.Replace(" ", string.Empty);
        if (this.robotBaseFrame == null)
            this.robotBaseFrame = targetPose.parent;
    }

    void Update()
    {
        Vector3 unityCenter = this.robotBaseFrame.InverseTransformPoint(targetPose.position);
        Vector3 unityEuler = targetPose.eulerAngles - this.robotBaseFrame.eulerAngles;
        Publish(new RosSharp.RosBridgeClient.MessageTypes.Geometry.Pose(
            unityCenter.UnityPointToRosPoint(),
            unityEuler.UnityEulerToRosQuaternion()));
    }
}
