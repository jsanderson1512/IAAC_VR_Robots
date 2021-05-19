#if UNITY_STANDALONE_WIN


using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Solid.Arduino;
using Solid.Arduino.Firmata;
using System;


// adapted from https://github.com/SolidSoils/Arduino
// by Jeffrey Anderson, 3/2/2021

public class TestingArduino : MonoBehaviour
{
    private ArduinoSession session;
    private SerialBaudRate myBaudRate;
    private bool lightOn = false;

    [System.Serializable]
    public enum BaudRates // this public var should appear as a drop down
    {
        _300,
        _600,
        _1200,
        _2400,
        _4800,
        _9600,
        _14400,
        _19200,
        _28800,
        _31250,
        _38400,
        _57600,
        _115200,
    };

    public string ComPort = "COM3";
    public BaudRates rate = BaudRates._57600;

    private void Start()
    {
        if(rate == BaudRates._300)
            myBaudRate = SerialBaudRate.Bps_300;
        else if (rate == BaudRates._600)
            myBaudRate = SerialBaudRate.Bps_600;
        else if (rate == BaudRates._1200)
            myBaudRate = SerialBaudRate.Bps_1200;
        else if (rate == BaudRates._2400)
            myBaudRate = SerialBaudRate.Bps_2400;
        else if (rate == BaudRates._4800)
            myBaudRate = SerialBaudRate.Bps_4800;
        else if (rate == BaudRates._9600)
            myBaudRate = SerialBaudRate.Bps_9600;
        else if (rate == BaudRates._14400)
            myBaudRate = SerialBaudRate.Bps_14400;
        else if (rate == BaudRates._19200)
            myBaudRate = SerialBaudRate.Bps_19200;
        else if (rate == BaudRates._28800)
            myBaudRate = SerialBaudRate.Bps_28800;
        else if (rate == BaudRates._31250)
            myBaudRate = SerialBaudRate.Bps_31250;
        else if (rate == BaudRates._38400)
            myBaudRate = SerialBaudRate.Bps_38400;
        else if (rate == BaudRates._57600)
            myBaudRate = SerialBaudRate.Bps_57600;
        else if (rate == BaudRates._115200)
            myBaudRate = SerialBaudRate.Bps_115200;


        
        ISerialConnection connection = GetConnection();

        if (connection != null)
        {
            session = new ArduinoSession(connection);
            Debug.Log("Press space key");
        }
        else
        {
            Debug.Log("Connection Failed");
        }
    }

    private ISerialConnection GetConnection()
    {
        Debug.Log("Searching Arduino connection...");
        ISerialConnection connection = new EnhancedSerialConnection(ComPort, myBaudRate);

        if (connection == null)
        {
            Debug.Log("No connection found. Make shure your Arduino board is attached to a USB port.");
        }
        else
        {
            Debug.Log("Connected to port: " + connection.PortName + " at: " + connection.BaudRate + " baud.");
        }

        return connection;
    }
    

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            PerformBasicTest(session);
        }
    }

    private void PerformBasicTest(IFirmataProtocol session)
    {

        lightOn = !lightOn;

        session.SetDigitalPinMode(13, PinMode.DigitalOutput);

        if (lightOn)
        {
            session.SetDigitalPin(13, true);
        }
        else
        {
            session.SetDigitalPin(13, false);
        }
    }
}
#endif