using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;


public class IAAC_IOPublisher : UnityPublisher<RosSharp.RosBridgeClient.MessageTypes.Std.Int32MultiArray>
{
    private RosSharp.RosBridgeClient.MessageTypes.Std.Int32MultiArray message;

    // publish IO message once when called
    public void SetIO(int pinID, int state)
    {
        int[] ioArray = new int[] {pinID, state};
        message.data = ioArray;
        Publish(message);
    }
}
