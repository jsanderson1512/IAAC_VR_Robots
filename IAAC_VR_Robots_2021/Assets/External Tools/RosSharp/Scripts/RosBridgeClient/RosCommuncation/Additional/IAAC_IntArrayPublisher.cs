using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.RosBridgeClient;


public class IAAC_IntArrayPublisher : UnityPublisher<RosSharp.RosBridgeClient.MessageTypes.Std.Int32MultiArray>
{
    private RosSharp.RosBridgeClient.MessageTypes.Std.Int32MultiArray message;

    // Update is called once per frame
    public void SendNewIO(int pinID, int state)
    {
        int[] ioArray = new int[] {pinID, state};
        message.data = ioArray;
        Publish(message);
    }
}
