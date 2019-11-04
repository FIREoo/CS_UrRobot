﻿using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UrRobot.Coordinates;

namespace UrRobot.Socket
{
    public enum mode
    {
        End = -1,
        stop = 0,
        recordj = 1,
        recordp = 2,
        jmovej = 3,
        pmovep = 4,
        gripper = 5,
        grip = 6,
        Rmovep = 7,
        Rmovej = 8,
        jog = 10,
        jservoj = 11,
        pservoj = 12,
        moveByFile = 99
    }
    public enum tcpState
    {
        none = -1,
        startListener = 1,
        waitAccept = 2,
        Connect = 3,
        Disconnect = 4
    }

    public class UrSocketControl
    {
        ~UrSocketControl()
        {
            stopServer();
            //stopClient();
        }
        public mode cmd = mode.stop;

        static bool serverOn = false;
        public bool isServerRunning
        {
            get { return serverOn; }
        }

        public delegate void ServerState(tcpState state);
        public event ServerState stateChange;

        public delegate void ControlFunction();
        public ControlFunction dynamicGrip;

        #region //---Client---//
        bool isConect = false;
        TcpClient urTcpClient;
        System.Net.Sockets.Socket urSocket;
        public void creatClient(string IP)
        {
            if (isConect) { Console.WriteLine("已經連線"); return; }

            if (IP == "auto" || IP == "Auto" || IP == "")
            {
                List<string> lstIPAddress = new List<string>();
                IPHostEntry IpEntry = Dns.GetHostEntry(Dns.GetHostName());
                foreach (IPAddress ipa in IpEntry.AddressList)
                {
                    if (ipa.AddressFamily == AddressFamily.InterNetwork)
                        lstIPAddress.Add(ipa.ToString());
                }

                foreach (string find_ip in lstIPAddress)
                    if (find_ip.IndexOf("192.168.1.") >= 0)
                        IP = find_ip;
                    else
                    {
                        Console.WriteLine("沒連到UR網路?");
                        return;
                    }
            }
            int connectPort = 30002;

            urTcpClient = new TcpClient();
            try
            {
                urTcpClient.Connect(IP, connectPort);
                urSocket = urTcpClient.Client;
                Console.WriteLine("連線成功 !!");
                isConect = true;
            }
            catch
            {
                Console.WriteLine
                           ("主機 {0} 通訊埠 {1} 無法連接  !!", IP, connectPort);
                return;
            }
        }

        public void client_SendData(string msg)
        {
            if (!isConect) { Console.WriteLine("尚未連線:isConect"); return; }
            if (urTcpClient == null) { Console.WriteLine("尚未連線:TcpClient"); return; }
            if (urSocket == null) { Console.WriteLine("尚未連線:Socket"); return; }

            String str = msg;
            Byte[] myBytes = Encoding.ASCII.GetBytes(str);
            urSocket.Send(myBytes, myBytes.Length, 0);
        }
        #endregion ---Client---

        #region  //---Server---//
        Thread thread_server;
        string sMsg = "";

        TcpListener serverListener;
        System.Net.Sockets.Socket acceptSocket;
        public void startServer(string ip, int port)
        {
            if (serverOn) // is the server is on, return
            { Console.WriteLine("已經在執行了"); return; }

            if (ip == "auto" || ip == "Auto" || ip == "")
            {
                List<string> lstIPAddress = new List<string>();
                IPHostEntry IpEntry = Dns.GetHostEntry(Dns.GetHostName());
                foreach (IPAddress ipa in IpEntry.AddressList)
                {
                    if (ipa.AddressFamily == AddressFamily.InterNetwork)
                        lstIPAddress.Add(ipa.ToString());
                }

                foreach (string find_ip in lstIPAddress)
                    if (find_ip.IndexOf("192.168.1.") >= 0)
                        ip = find_ip;
                    else
                    {
                        Console.WriteLine("沒連到UR網路?");
                        return;
                    }
            }

            IPAddress IPAddress = IPAddress.Parse(ip);
            serverListener = new TcpListener(IPAddress, port);
            thread_server = new Thread(() => Server());
            thread_server.Start();
        }

        private void Server()
        {
            //creat tcp
            try
            {
                serverListener.Start();
                Console.WriteLine("Start server at:" + ((IPEndPoint)serverListener.LocalEndpoint).Address.ToString());
                stateChange?.Invoke(tcpState.startListener);
            }
            catch (Exception ex) { Console.WriteLine("tcp start error\n" + ex); return; }

            try
            {
                stateChange?.Invoke(tcpState.waitAccept);
                acceptSocket = serverListener.AcceptSocket();//等待接受，接受後繼續執行
            }
            catch
            {
                stateChange?.Invoke(tcpState.none);
                Console.WriteLine("Waiting accept socket abort!");
                return;
            }
            //Thread.Sleep(1000);
            Console.WriteLine("Client is connect");
            stateChange?.Invoke(tcpState.Connect);

            serverOn = true;
            cmd = mode.stop;
            while (serverOn && acceptSocket.Connected)
            {
                if (cmd == mode.pmovep)//done
                {
                    if (!_sendMsg("pmovep")) break;
                    sMsg = _waitRead(); if (sMsg == "End") break;

                    if (!_sendMsg($"p[{val_pos[0]},{val_pos[1]},{val_pos[2]},{val_pos[3]},{val_pos[4]},{val_pos[5]}]")) break;

                    sMsg = _waitRead(); if (sMsg == "End") break;
                    Console.WriteLine(sMsg);
                    if (sMsg != "UR:done")
                        Console.WriteLine("error!! UR robot didn't finish work?");
                    cmd = mode.stop;
                }
                else if (cmd == mode.jmovej)//done
                {
                    if (!_sendMsg("jmovej")) break;
                    sMsg = _waitRead(); if (sMsg == "End") break;
                    Console.WriteLine(sMsg);

                    if (!_sendMsg($"[{val_joint[0]},{val_joint[1]},{val_joint[2]},{val_joint[3]},{val_joint[4]},{val_joint[5]}]")) break;

                    sMsg = _waitRead(); if (sMsg == "End") break;
                    Console.WriteLine(sMsg);
                    if (sMsg != "UR:done")
                        Console.WriteLine("error!! UR robot didn't finish work?");
                    cmd = mode.stop;
                }
                else if (cmd == mode.jservoj)//done
                {
                    if (!_sendMsg("jservoj")) break;
                    sMsg = _waitRead(); if (sMsg == "End") break;
                    Console.WriteLine(sMsg);

                    if (!_sendMsg($"[{val_joint[0]},{val_joint[1]},{val_joint[2]},{val_joint[3]},{val_joint[4]},{val_joint[5]}]")) break;

                    sMsg = _waitRead(); if (sMsg == "End") break;
                    Console.WriteLine(sMsg);//joint
                }
                else if (cmd == mode.pservoj)//done
                {
                    if (!_sendMsg("pservoj")) break;
                    sMsg = _waitRead(); if (sMsg == "End") break;
                    Console.WriteLine(sMsg);

                    if (!_sendMsg($"p[{val_pos[0]},{val_pos[1]},{val_pos[2]},{val_pos[3]},{val_pos[4]},{val_pos[5]}]")) break;

                    sMsg = _waitRead(); if (sMsg == "End") break;
                    Console.WriteLine(sMsg);//position
                }
                else if (cmd == mode.Rmovep)//done
                {
                    if (!_sendMsg("Rmovep")) break;
                    sMsg = _waitRead(); if (sMsg == "End") break;

                    if (!_sendMsg($"p[{val_pos[0]},{val_pos[1]},{val_pos[2]},{val_pos[3]},{val_pos[4]},{val_pos[5]}]")) break;

                    sMsg = _waitRead(); if (sMsg == "End") break;
                    Console.WriteLine(sMsg);
                    if (sMsg != "UR:done")
                        Console.WriteLine("error!! UR robot didn't finish work?");
                    cmd = mode.stop;
                }
                else if (cmd == mode.Rmovej)//done
                {
                    if (!_sendMsg("Rmovej")) break;
                    sMsg = _waitRead(); if (sMsg == "End") break;
                    Console.WriteLine(sMsg);

                    if (!_sendMsg($"[{val_joint[0]},{val_joint[1]},{val_joint[2]},{val_joint[3]},{val_joint[4]},{val_joint[5]}]")) break;

                    sMsg = _waitRead(); if (sMsg == "End") break;
                    Console.WriteLine(sMsg);
                    if (sMsg != "UR:done")
                        Console.WriteLine("error!! UR robot didn't finish work?");
                    cmd = mode.stop;
                }
                else if (cmd == mode.recordj)//done
                {
                    if (!_sendMsg("recordj")) break;
                    sMsg = _waitRead(); if (sMsg == "End") break;
                    Console.WriteLine(sMsg);//拿到joint
                }
                else if (cmd == mode.recordp)//遺棄 (因為 position 轉 joint 會出問題，錄製position很常無法執行
                {
                }
                else if (cmd == mode.jog)//未完成
                {
                }
                else if (cmd == mode.gripper)//done
                {
                    if (!_sendMsg("gripper")) break;
                    sMsg = _waitRead(); if (sMsg == "End") break;
                    Console.WriteLine(sMsg);

                    if (!_sendMsg($"[{val_grip[0]},{val_grip[1]},{val_grip[2]},0,0,0]")) break;

                    sMsg = _waitRead(); if (sMsg == "End") break;
                    Console.WriteLine(sMsg);
                    if (sMsg != "UR:done")
                        Console.WriteLine("error!! UR robot didn't finish work?");

                    cmd = mode.stop;
                }
                else if (cmd == mode.grip)//遺棄
                {
                }
                else if (cmd == mode.stop)//done
                {
                    if (!_sendMsg("stop")) break;
                    sMsg = _waitRead(); if (sMsg == "End") break;
                    Console.WriteLine(sMsg); //拿到 postion
                }
                else if (cmd == mode.End)//done
                {
                    break;
                }
            }//while serverOn

            acceptSocket.Close();
            serverListener.Stop();
            stateChange?.Invoke(tcpState.Disconnect);
            Console.WriteLine("server disconnect");
            serverOn = false;

            #region subfunction
            string _waitRead()
            {
                try
                {
                    int bufferSize = acceptSocket.ReceiveBufferSize;
                    byte[] myBufferBytes = new byte[bufferSize];
                    int L = acceptSocket.Receive(myBufferBytes);
                    string msg_read = Encoding.ASCII.GetString(myBufferBytes, 0, L);
                    return msg_read;
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Socket read fail :" + ex);
                    cmd = mode.End;
                    return "End";
                }
            }
            bool _sendMsg(string str)
            {
                try
                {
                    Byte[] myBytes = Encoding.ASCII.GetBytes(str);
                    acceptSocket.Send(myBytes, myBytes.Length, 0);
                }
                catch
                {
                    Console.WriteLine("Socket send fail"); //可能是client先結束了
                    cmd = mode.End;
                    return false;
                }
                return true;
            }
            #endregion subfunction
        }


        public void stopServer()
        {
            serverOn = false;
            if (acceptSocket != null)
                acceptSocket.Close();
            if (serverListener != null)
                serverListener.Stop();

            if (thread_server != null)
                thread_server.Abort();

            stateChange?.Invoke(tcpState.Disconnect);

        }
        #endregion //---Server---//

        public void Stop()
        {
            cmd = mode.stop;
        }

        public bool getPosition(out URCoordinates pos)
        {
            if (cmd != mode.stop)
            {
                pos = new URCoordinates();
                return false;
            }

            pos = URCoordinates.str2urc(sMsg);

            return true;

        }

        float[] val_grip = new float[3];
        public void goGripper(int pos, int force = 0, int speed = 0, bool wait = true)
        {
            if (pos > 255) pos = 255;
            else if (pos < 0) pos = 0;

            if (force > 255) force = 255;
            else if (force < 0) force = 0;

            if (speed > 255) speed = 255;
            else if (speed < 0) speed = 0;

            val_grip[0] = pos / 1000.0f;
            val_grip[1] = force / 1000.0f;
            val_grip[2] = speed / 1000.0f;
            cmd = mode.gripper;

            if (wait)
                while (cmd != mode.stop) ;
        }

        float[] val_pos = new float[6];
        public void goPosition(URCoordinates pos)
        {
            val_pos[0] = pos.X.M;
            val_pos[1] = pos.Y.M;
            val_pos[2] = pos.Z.M;
            val_pos[3] = pos.Rx.rad;
            val_pos[4] = pos.Ry.rad;
            val_pos[5] = pos.Rz.rad;
            cmd = mode.pmovep;
            while (cmd != mode.stop) ;
        }

        float[] val_joint = new float[6];
        public void goJoint(float j1, float j2, float j3, float j4, float j5, float j6)
        {
            val_joint[0] = j1;
            val_joint[1] = j2;
            val_joint[2] = j3;
            val_joint[3] = j4;
            val_joint[4] = j5;
            val_joint[5] = j6;
            cmd = mode.jmovej;
            while (cmd != mode.stop) ;
        }

        public void goTrack(float j1, float j2, float j3, float j4, float j5, float j6)
        {
            val_joint[0] = j1;
            val_joint[1] = j2;
            val_joint[2] = j3;
            val_joint[3] = j4;
            val_joint[4] = j5;
            val_joint[5] = j6;
            cmd = mode.jservoj;
        }
        public void goTrack(URCoordinates pos)
        {
            val_pos[0] = pos.X.M;
            val_pos[1] = pos.Y.M;
            val_pos[2] = pos.Z.M;
            val_pos[3] = pos.Rx.rad;
            val_pos[4] = pos.Ry.rad;
            val_pos[5] = pos.Rz.rad;
            cmd = mode.pservoj;
        }

        public void goRelativePosition(URCoordinates pos)
        {
            val_pos[0] = pos.X.M;
            val_pos[1] = pos.Y.M;
            val_pos[2] = pos.Z.M;
            val_pos[3] = pos.Rx.rad;
            val_pos[4] = pos.Ry.rad;
            val_pos[5] = pos.Rz.rad;
            cmd = mode.Rmovep;
            while (cmd != mode.stop) ;
        }
        public void goRelativePosition(Unit x = null, Unit y = null, Unit z = null, Angle Rx = null, Angle Ry = null, Angle Rz = null)
        {
            if (x == null) x = new Unit();
            if (y == null) y = new Unit();
            if (z == null) z = new Unit();
            if (Rx == null) Rx = new Angle();
            if (Ry == null) Ry = new Angle();
            if (Rz == null) Rz = new Angle();

            val_pos[0] = x.M;
            val_pos[1] = y.M;
            val_pos[2] = z.M;
            val_pos[3] = Rx.rad;
            val_pos[4] = Ry.rad;
            val_pos[5] = Rz.rad;
            cmd = mode.Rmovep;
            while (cmd != mode.stop) ;
        }
        public void goRelativeJoint(float j1 = 0, float j2 = 0, float j3 = 0, float j4 = 0, float j5 = 0, float j6 = 0)
        {
            val_joint[0] = j1;
            val_joint[1] = j2;
            val_joint[2] = j3;
            val_joint[3] = j4;
            val_joint[4] = j5;
            val_joint[5] = j6;
            cmd = mode.Rmovej;
            while (cmd != mode.stop) ;
        }
        public void goRelativeJoint(Angle j1 = null, Angle j2 = null, Angle j3 = null, Angle j4 = null, Angle j5 = null, Angle j6 = null)
        {
            if (j1 == null) j1 = new Angle();
            if (j2 == null) j2 = new Angle();
            if (j3 == null) j3 = new Angle();
            if (j4 == null) j4 = new Angle();
            if (j5 == null) j5 = new Angle();
            if (j6 == null) j6 = new Angle();

            val_joint[0] = j1.rad;
            val_joint[1] = j2.rad;
            val_joint[2] = j3.rad;
            val_joint[3] = j4.rad;
            val_joint[4] = j5.rad;
            val_joint[5] = j6.rad;
            cmd = mode.Rmovej;
            while (cmd != mode.stop) ;
        }
        public void goFunction()
        {
            dynamicGrip.Invoke();
        }
        public bool goFile(string file = "")
        {
            if (file == "")//使用上一次的紀錄
                file = fileFullPath;
            if (file.IndexOf(rootPath) < 0)
                file = rootPath + file;
            if (file.IndexOf(".path") < 0)
                file += ".path";
            if (!File.Exists(file))
            {
                Console.WriteLine("file doesn't exist!");
                return false;
            }


            string[] fileLine = System.IO.File.ReadAllLines(file);
            foreach (string line in fileLine)
            {
                string theCmd = line.Substring(0, line.IndexOf("("));
                if (theCmd == "movej")
                {
                    string info = line.Substring(line.IndexOf("[") + 1, line.IndexOf("]") - line.IndexOf("[") - 1);
                    string[] joint = info.Split(',');
                    goJoint(joint[0].toFloat(), joint[1].toFloat(), joint[2].toFloat(), joint[3].toFloat(), joint[4].toFloat(), joint[5].toFloat());
                }
                else if (theCmd == "movep")
                {
                    string info = line.Substring(line.IndexOf("[") + 1, line.IndexOf("]") - line.IndexOf("[") - 1);
                    string[] pos = info.Split(',');
                    goPosition(new URCoordinates(pos[0].toFloat(), pos[1].toFloat(), pos[2].toFloat(), pos[3].toFloat(), pos[4].toFloat(), pos[5].toFloat()));
                }
                else if (theCmd == "Rmovej")
                {
                    string info = line.Substring(line.IndexOf("[") + 1, line.IndexOf("]") - line.IndexOf("[") - 1);
                    string[] pos = info.Split(',');
                    goPosition(new URCoordinates(pos[0].toFloat(), pos[1].toFloat(), pos[2].toFloat(), pos[3].toFloat(), pos[4].toFloat(), pos[5].toFloat()));
                }
                else if (theCmd == "Rmovep")
                {
                    string info = line.Substring(line.IndexOf("[") + 1, line.IndexOf("]") - line.IndexOf("[") - 1);
                    string[] pos = info.Split(',');
                    goPosition(new URCoordinates(pos[0].toFloat(), pos[1].toFloat(), pos[2].toFloat(), pos[3].toFloat(), pos[4].toFloat(), pos[5].toFloat()));
                }
                else if (theCmd == "rq_move")
                {
                    string info = line.Substring(line.IndexOf("(") + 1, line.IndexOf(")") - line.IndexOf("(") - 1);
                    goGripper(info.toInt());
                }
                else if (theCmd == "sleep")
                {
                    string info = line.Substring(line.IndexOf("(") + 1, line.IndexOf(")") - line.IndexOf("(") - 1);
                    Thread.Sleep(info.toInt());
                }
                else if (theCmd == "function")
                {
                    goFunction();//會等 Function結束才會下一行，所以不用擔心手臂繼續會跑下一行指令
                }
                else if (theCmd == "")
                    continue;
                else
                    Console.WriteLine($"Error at : {theCmd}");
            }

            return true;
        }

        #region //---Record---//
        public string rootPath = "Path\\";
        string fileFullPath = "";
        bool isRecord = false;
        StreamWriter txt_record;
        public void startRecord(string fileName = "record.path")
        {
            if (isRecord == true)
            {
                Console.WriteLine("Already in record mode");
                return;
            }
            isRecord = true;
            if (!Directory.Exists(rootPath))
                Directory.CreateDirectory(rootPath);

            if (fileName.IndexOf(".path") < 0)
                fileName += ".path";

            fileFullPath = rootPath + fileName;

            txt_record = new StreamWriter(fileFullPath, false);//false =>覆寫
            cmd = mode.recordj;
        }

        public void Record_joint()
        {
            if (isRecord == false)
            {
                Console.WriteLine("Record mode unable");
                return;
            }
            txt_record.WriteLine($"movej({sMsg})");
        }
        public void Record_point(URCoordinates pos)
        {
            if (isRecord == false)
            {
                Console.WriteLine("Record mode unable");
                return;
            }
            txt_record.WriteLine($"movep({pos.ToString("p[]")})");
        }
        public void Record_grip(int pos, bool withMove = true)
        {
            if (isRecord == false)
            {
                Console.WriteLine("Record mode unable");
                return;
            }
            if (withMove)
                goGripper(pos);
            cmd = mode.recordj;
            txt_record.WriteLine($"rq_move({pos})");
        }
        public void Record_sleep(int ms)
        {
            if (isRecord == false)
            {
                Console.WriteLine("Record mode unable");
                return;
            }
            txt_record.WriteLine($"sleep({ms})");
        }

        public void endRecord()
        {
            if (isRecord == true)
            {
                isRecord = false;
                cmd = mode.stop;
                txt_record.Flush();
                txt_record.Close();
            }
            else
            {
                Console.WriteLine("Record mode already off");
                return;
            }
        }
        #endregion //---Record---//


    }
}
