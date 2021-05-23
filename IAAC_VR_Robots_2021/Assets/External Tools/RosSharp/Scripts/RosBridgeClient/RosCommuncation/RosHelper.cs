using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;
using System;

public static class RosHelper
{
    public static RosSharp.RosBridgeClient.MessageTypes.Geometry.Point UnityPointToRosPoint(this Vector3 unityPoint)
    {
        return new RosSharp.RosBridgeClient.MessageTypes.Geometry.Point(
            Convert.ToDouble(unityPoint.x),
            Convert.ToDouble(unityPoint.z),
            Convert.ToDouble(unityPoint.y));
    }
    public static Vector3 RosPointToUnity(this RosSharp.RosBridgeClient.MessageTypes.Geometry.Point rosPoint)
    {
        return new Vector3(
            Convert.ToSingle(rosPoint.x),
            Convert.ToSingle(rosPoint.z),
            Convert.ToSingle(rosPoint.y));
    }

    public static RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion UnityEulerToRosQuaternion(this Vector3 unityEulerOrientation) {
        Quaternion unityOrientation = Quaternion.Euler(unityEulerOrientation.x, unityEulerOrientation.z, unityEulerOrientation.y);

        return new RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion(
            Convert.ToDouble(unityOrientation.x),
            Convert.ToDouble(unityOrientation.y),
            Convert.ToDouble(unityOrientation.z),
            Convert.ToDouble(unityOrientation.w));
    }

    public static Vector3 RosQuaternionToUnityEuler(this RosSharp.RosBridgeClient.MessageTypes.Geometry.Quaternion rosOrientation)
    {
        Quaternion unityOrientation = new Quaternion(
            Convert.ToSingle(rosOrientation.x),
            Convert.ToSingle(rosOrientation.y),
            Convert.ToSingle(rosOrientation.z),
            Convert.ToSingle(rosOrientation.w));
        Vector3 rosEulerOrientation = unityOrientation.eulerAngles;

        return new Vector3(
            rosEulerOrientation.x, 
            rosEulerOrientation.z, 
            rosEulerOrientation.y);
    }
}
