using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Net.Sockets;
using System.Net;
using System.Text;
using System.Linq;

[System.Serializable]
public class AxleInfo
{
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor;
    public bool steering;
}

public class MATLABController : MonoBehaviour
{
    public List<AxleInfo> axleInfos;
    public float maxMotorTorque;
    public float maxSteeringAngle;
    TcpListener listener;
    Byte[] bytes;
    String reply;
    String request;
    internal Boolean socketReady = false;
    TcpClient mySocket;
    NetworkStream theStream;
    String Host = "localhost";
    Int32 ClientPort = 55000;
    Int32 ServerPort = 55001;
    Double leftSpeedUpFactor;
    Double rightSpeedUpFactor;

    void Start()
    {
        setupSocket();
        Debug.Log("Socket is set up...");
    }

    void Update()
    {
        int layerMask = 1 << 8;
        layerMask = ~layerMask;
        Vector3 myPosition = transform.position + new Vector3(0, 0.75f, 0);

        float[] minDist = { 1000f, 1000f, 1000f };

        Vector3[] leftSpots = { new Vector3(0f, 0f, -1.25f), new Vector3(0f, 0f, 0f), new Vector3(0f, 0f, 1.25f) };
        Vector3[] frontSpots = { new Vector3(-1.25f, 0f, 0f), new Vector3(0f, 0f, 0f), new Vector3(1.25f, 0f, 0f) };
        Vector3[] rightSpots = { new Vector3(0f, 0f, -1.25f), new Vector3(0f, 0f, 0f), new Vector3(0f, 0f, 1.25f) };
        var spots = new[]{ leftSpots, frontSpots, rightSpots };
        var directions = new[]{ Vector3.left, Vector3.forward, Vector3.right };
        int k = 0;
        foreach (var sd in spots.Zip(directions, (s,d) => new { Spots=s, Direction=d }))
        {
            for (int i1 = 0; i1 < sd.Spots.Length; i1++)
            {
                Vector3 i = sd.Spots[i1];
                RaycastHit hit;
                if (Physics.Raycast(myPosition + i, transform.TransformDirection(sd.Direction), out hit, Mathf.Infinity, layerMask))
                {
                    Debug.DrawRay(myPosition + i, transform.TransformDirection(sd.Direction) * hit.distance, Color.yellow);
                    if (hit.distance < minDist[k])
                    {
                        minDist[k] = hit.distance;
                    }
                }
                else
                {
                    Debug.DrawRay(myPosition + i, transform.TransformDirection(sd.Direction) * 1000, Color.white);
                }
            }
            k++;
        }

        try
        {
            String reading1 = (Math.Floor(minDist[0]) > 100 ? 100 : Math.Floor(minDist[0])).ToString();
            int len1 = reading1.Length;
            String reading2 = (Math.Floor(minDist[1]) > 100 ? 100 : Math.Floor(minDist[1])).ToString();
            int len2 = reading2.Length;
            String reading3 = (Math.Floor(minDist[2]) > 100 ? 100 : Math.Floor(minDist[2])).ToString();
            int len3 = reading3.Length;
            request = (len1 <= 0 ? "0" : "") + (len1 <= 1 ? "0" : "") + (len1 <= 2 ? "0" : "") + reading1;
            String request2 = (len2 <= 0 ? "0" : "") + (len2 <= 1 ? "0" : "") + (len2 <= 2 ? "0" : "") + reading2;
            String request3 = (len3 <= 0 ? "0" : "") + (len3 <= 1 ? "0" : "") + (len3 <= 2 ? "0" : "") + reading3;
            Byte[] sendBytes = Encoding.UTF8.GetBytes(request);
            theStream.Write(sendBytes, 0, sendBytes.Length);
            sendBytes = Encoding.UTF8.GetBytes(request2);
            theStream.Write(sendBytes, 0, sendBytes.Length);
            sendBytes = Encoding.UTF8.GetBytes(request3);
            theStream.Write(sendBytes, 0, sendBytes.Length);
            /*Debug.Log(request + ", " + request2 + ", " + request3 + " is sent...");*/

            TcpClient client = listener.AcceptTcpClient();
            NetworkStream stream = client.GetStream();
            while ((_ = stream.Read(bytes, 0, bytes.Length)) != 0)
            {
                reply = Encoding.UTF8.GetString(bytes);
            }
            String[] results = reply.Split(',');
            leftSpeedUpFactor = Double.Parse(results[0]);
            rightSpeedUpFactor = Double.Parse(results[1]);
            /*Debug.Log(leftSpeedUpFactor.ToString() + " " + rightSpeedUpFactor.ToString());*/
            client.Close();

            /*Byte[] receiveBytes = new byte[sizeof(Double)];
            mySocket.GetStream().Read(receiveBytes, 0, sizeof(Double));
            reply = Encoding.UTF8.GetString(receiveBytes);
            double _reply = Double.Parse(reply);
            Debug.Log(_reply); */
        }
        catch (Exception e)
        {
            Debug.Log("Socket error: " + e);
            Debug.Log(e.StackTrace);
        }

        /*float motor = maxMotorTorque * Input.GetAxis("Vertical");*/
        /*float steering = maxSteeringAngle * Input.GetAxis("Horizontal");*/

        foreach (AxleInfo axleInfo in axleInfos)
        {
            if (axleInfo.steering)
            {
                float steering;
                if (leftSpeedUpFactor == rightSpeedUpFactor)
                {
                    steering = 0;
                }
                else
                {
                    steering = maxSteeringAngle * Math.Max(
                        (float)leftSpeedUpFactor, (float)rightSpeedUpFactor
                        ) * (leftSpeedUpFactor > rightSpeedUpFactor ? 1.0f : -1.0f);
                }
                axleInfo.leftWheel.steerAngle = steering;
                axleInfo.rightWheel.steerAngle = steering;
            }
            if (axleInfo.motor)
            {
                /*if (leftSpeedUpFactor < 0.3)
                {
                    leftSpeedUpFactor *= 0.005;
                }
                else if (leftSpeedUpFactor > 0.8)
                {
                    leftSpeedUpFactor *= 10;
                }
                else if (leftSpeedUpFactor >= 0.3 && leftSpeedUpFactor <= 0.8)
                {
                    leftSpeedUpFactor *= 5;
                }

                if (rightSpeedUpFactor < 0.3)
                {
                    rightSpeedUpFactor *= 0.005;
                }
                else if (rightSpeedUpFactor > 0.8)
                {
                    rightSpeedUpFactor *= 10;
                }
                else if (rightSpeedUpFactor >= 0.3 && leftSpeedUpFactor <= 0.8)
                {
                    rightSpeedUpFactor *= 5;
                }*/
                axleInfo.leftWheel.motorTorque = (float)(maxMotorTorque/* * leftSpeedUpFactor*/);
                axleInfo.rightWheel.motorTorque = (float)(maxMotorTorque/* * rightSpeedUpFactor*/);
            }
            ApplyLocalPositionToVisuals(axleInfo.leftWheel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel);
        }
        Quaternion q = transform.rotation;
        q.eulerAngles = new Vector3(0, q.eulerAngles.y, 0);
        transform.rotation = q;
    }

    public void FixedUpdate()
    {
        /*float motor = maxMotorTorque * Input.GetAxis("Vertical");
        float steering = maxSteeringAngle * Input.GetAxis("Horizontal");

        foreach (AxleInfo axleInfo in axleInfos)
        {
            if (axleInfo.steering)
            {
                axleInfo.leftWheel.steerAngle = steering;
                axleInfo.rightWheel.steerAngle = steering;
            }
            if (axleInfo.motor)
            {
                axleInfo.leftWheel.motorTorque = (float)(motor * leftSpeedUpFactor);
                axleInfo.rightWheel.motorTorque = (float)(motor * rightSpeedUpFactor);
            }
            ApplyLocalPositionToVisuals(axleInfo.leftWheel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel);
        }*/
    }

    // finds the corresponding visual wheel
    // correctly applies the transform
    public void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        if (collider.transform.childCount == 0)
        {
            return;
        }

        Transform visualWheel = collider.transform.GetChild(0);

        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);

        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation;
    }

    public void setupSocket()
    {
        try
        {
            listener = new TcpListener(IPAddress.Parse("127.0.0.1"), ServerPort);
            listener.Start();
            bytes = new Byte[256];

            mySocket = new TcpClient(Host, ClientPort);
            theStream = mySocket.GetStream();
        }
        catch (Exception e)
        {
            Debug.Log("Socket error: " + e);
            Debug.Log(e.StackTrace);
        }
    }
}

/*using System.Collections;
using System.Net;
using System.Net.Sockets;
using System;
using System.IO;
using UnityEngine;
using System.Text;

public class MATLABController : MonoBehaviour
{
    // Use this for initialization
    
    
    // Update is called once per frame
    void Update()
    {
        
    }
    
}*/
