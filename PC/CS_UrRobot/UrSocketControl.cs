/*
 *v3.1 jogj jogp模式完成 
 * 
 */

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Text;
using System.Threading;
using UrRobot.Coordinates;


[assembly: AssemblyVersion("3.2")]
/*
 *  v3.1
 *  jogj jogp模式完成 
 *  
 *  v3.2
 *  逆向運動學完成
 *  建立 goTrack2 (但效率低落)
 *  配上速度修改
 */

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
        jogj = 10,
        jogp = 16,
        jservoj = 11,
        pservoj = 12,
        pmovej = 13,
        force = 14,
        joint = 50,
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
        public Version Ver
        {
            get
            {
                System.Reflection.Assembly thisAssem = typeof(UrSocketControl).Assembly;
                System.Reflection.AssemblyName thisAssemName = thisAssem.GetName();
                Version ver = thisAssemName.Version;
                return ver;
            }
        }
        public mode cmd = mode.stop;
        public string rootPath = "Path\\";
        bool serverOn = false;
        public bool isServerRunning
        {
            get { return serverOn; }
        }

        public delegate void ServerState(tcpState state);
        public event ServerState stateChange;

        public delegate bool ControlFunction(string msg);
        public ControlFunction dynamicFunction;

        public delegate void URmsg(string msg);
        public URmsg UrPosGet;

        public delegate void URcmd(mode cmd);
        public URcmd UrCmd;

        public class Client
        {
            public event ServerState stateChange;
            public string rootPath;
            public Client()
            {
                UrSocketControl ur = new UrSocketControl();
                rootPath = ur.rootPath;
                for (int i = 0; i < ClientForceFilter.Count(); i++)
                    ClientForceFilter[i] = new URCoordinates();
            }
            bool isConect = false;
            TcpClient client_realTime;
            TcpClient client_dashboard;
            TcpClient client_RTDE;
            System.Net.Sockets.Socket urSocket_realTime;
            System.Net.Sockets.Socket urSocket_dashboard;
            System.Net.Sockets.Socket urSocket_RTDE;
            /// <summary>手臂座標(透過client得到) </summary>
            private URCoordinates ClientPos = new URCoordinates();
            /// <summary>手臂座標(透過client得到) </summary>
            private URJoint ClientJoint = new URJoint();
            /// <summary>手臂力回受(透過client得到) </summary>
            private URCoordinates ClientForce = new URCoordinates();
            /// <summary>手臂力回受(透過client得到) </summary>
            private URCoordinates[] ClientForceFilter = new URCoordinates[30];
            private string _ip = "";

            public bool ClientConnect(string IP)
            {
                if (isConect) { Console.WriteLine("已經連線"); return true; }
                stateChange?.Invoke(tcpState.waitAccept);
                //if (urTcpClient != null)
                //    if (urTcpClient.Client.Connected)
                //    { Console.WriteLine("已經連線"); return true; }

                int Port_realTime = 30003;
                int Port_RTDE = 30004;
                int Port_dashboard = 29999;

                client_realTime = new TcpClient();
                client_dashboard = new TcpClient();
                client_RTDE = new TcpClient();
                try
                {
                    client_realTime.Connect(IP, Port_realTime);
                    urSocket_realTime = client_realTime.Client;
                    client_dashboard.Connect(IP, Port_dashboard);
                    urSocket_dashboard = client_dashboard.Client;

                    //client_RTDE.Connect(IP, Port_RTDE);
                    //urSocket_RTDE = client_RTDE.Client;

                    Console.WriteLine("UR Client is connected");
                    stateChange?.Invoke(tcpState.Connect);
                    isConect = true;
                    _ip = IP;
                }
                catch
                {
                    Console.WriteLine("Error : Connect fail");
                    stateChange?.Invoke(tcpState.Disconnect);
                    isConect = false;
                    return false;
                }

                return true;
            }
            private bool ClientSend(string msg, System.Net.Sockets.Socket socket)
            {
                if (!isConect) { Console.WriteLine("尚未連線:isConect"); return false; }
                //if (client_realTime == null) { Console.WriteLine("尚未連線:TcpClient"); return false; }
                //if (urSocket_realTime == null) { Console.WriteLine("尚未連線:Socket"); return false; }
                try
                {
                    String str = msg;
                    Byte[] myBytes = Encoding.ASCII.GetBytes(str);
                    socket.Send(myBytes, myBytes.Length, 0);
                    return true;
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Socket read fail :" + ex);
                    return false;
                }
            }

            public bool ClientGoFile(string file)
            {
                //這不是goFile只是跑file裡的
                if (file.IndexOf(rootPath) < 0)
                    file = rootPath + file;
                if (file.IndexOf(".path") < 0)
                    file += ".path";
                if (!File.Exists(file))
                {
                    Console.WriteLine("file doesn't exist!");
                    return false;
                }
                foreach (string str in File.ReadAllLines(file))
                    if (!ClientSend(str + "\n", urSocket_realTime))
                        return false;

                return true;
            }

            private string ClientRead(System.Net.Sockets.Socket socket)
            {

                if (!isConect) { Console.WriteLine("尚未連線:isConect"); return ""; }
                //if (client_realTime == null) { Console.WriteLine("尚未連線:TcpClient"); return ""; }
                //if (urSocket_realTime == null) { Console.WriteLine("尚未連線:Socket"); return ""; }
                try
                {
                    int bufferSize = socket.ReceiveBufferSize;
                    byte[] myBufferBytes = new byte[bufferSize];
                    int L = socket.Receive(myBufferBytes);
                    string msg_read = Encoding.ASCII.GetString(myBufferBytes, 0, L);
                    return msg_read;
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Socket read fail :" + ex);
                    return "";
                }
            }
            public void Client_RTDE()
            {
                if (!isConect) { Console.WriteLine("尚未連線:isConect"); return; }
                //if (client_realTime == null) { Console.WriteLine("尚未連線:TcpClient"); return; }
                //if (urSocket_realTime == null) { Console.WriteLine("尚未連線:Socket"); return; }
                urSocket_realTime.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReceiveTimeout, 1000);

                System.Threading.Tasks.Task.Run(() =>
                {
                    try
                    {
                        int pivot_force = 0;
                        while (isConect)
                        {
                            int bufferSize = client_realTime.ReceiveBufferSize;
                            byte[] myBufferBytes = new byte[bufferSize];
                            int dataLength = urSocket_realTime.Receive(myBufferBytes);
                            ClientPos = UrDecode(444, myBufferBytes);//30002 是 308
                            ClientJoint = UrDecode_joint(252, myBufferBytes);//RTDE : 300瞬時 joints
                            ClientForce = UrDecode(540, myBufferBytes);
                            //force filter FIFO
                            ClientForceFilter[pivot_force] = ClientForce;
                            pivot_force++;
                            if (pivot_force == ClientForceFilter.Count()) pivot_force = 0;
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("RTDE fail!", ex.Message);
                        isConect = false;
                        ClientConnect(_ip);
                        Client_RTDE();
                    }
                });

                URCoordinates UrDecode(int index, byte[] buffer)
                {
                    URCoordinates rtn = new URCoordinates();

                    double value = _getValue(index, 1);
                    rtn.X.M = (float)value;
                    value = _getValue(index, 2);
                    rtn.Y.M = (float)value;
                    value = _getValue(index, 3);
                    rtn.Z.M = (float)value;
                    value = _getValue(index, 4);
                    rtn.Rx.rad = (float)value;
                    value = _getValue(index, 5);
                    rtn.Ry.rad = (float)value;
                    value = _getValue(index, 6);
                    rtn.Rz.rad = (float)value;
                    return rtn;

                    double _getValue(int startIndex, int joint)
                    {
                        byte[] b = new byte[8];
                        for (int i = 0; i < 8; i++)
                            b[7 - i] = buffer[(joint - 1) * 8 + i + startIndex];
                        return BitConverter.ToDouble(b, 0);
                    }
                }
                URJoint UrDecode_joint(int index, byte[] buffer)
                {
                    URJoint rtn = new URJoint();

                    double value = _getValue(index, 1);
                    rtn.J1.rad = (float)value;
                    value = _getValue(index, 2);
                    rtn.J2.rad = (float)value;
                    value = _getValue(index, 3);
                    rtn.J3.rad = (float)value;
                    value = _getValue(index, 4);
                    rtn.J4.rad = (float)value;
                    value = _getValue(index, 5);
                    rtn.J5.rad = (float)value;
                    value = _getValue(index, 6);
                    rtn.J6.rad = (float)value;
                    return rtn;

                    double _getValue(int startIndex, int joint)
                    {
                        byte[] b = new byte[8];
                        for (int i = 0; i < 8; i++)
                            b[7 - i] = buffer[(joint - 1) * 8 + i + startIndex];
                        return BitConverter.ToDouble(b, 0);
                    }
                }
            }
            public void Client_RTDE2()
            {
                if (!isConect) { Console.WriteLine("尚未連線:isConect"); return; }
                //if (client_realTime == null) { Console.WriteLine("尚未連線:TcpClient"); return; }
                //if (urSocket_realTime == null) { Console.WriteLine("尚未連線:Socket"); return; }
                urSocket_RTDE.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReceiveTimeout, 1000);

                System.Threading.Tasks.Task.Run(() =>
                {
                    try
                    {
                        int pivot_force = 0;
                        while (isConect)
                        {
                            int bufferSize = client_RTDE.ReceiveBufferSize;
                            byte[] myBufferBytes = new byte[bufferSize];


                            String str = "v";
                            Byte[] tmp = Encoding.ASCII.GetBytes(str);
                            Byte[] myBytes = new Byte[2]; //uint16_t??
                            myBytes[0] = 0;
                            myBytes[1] = tmp[0];
                            urSocket_RTDE.Send(myBytes, myBytes.Length, SocketFlags.None);

                            int dataLength = urSocket_RTDE.Receive(myBufferBytes);

                            Console.WriteLine($"Time:{convert_Int(myBufferBytes, 0)}");
                            //ClientPos = UrDecode(444, myBufferBytes);//30002 是 308
                            //ClientJoint = UrDecode_joint(252, myBufferBytes);//RTDE : 300瞬時 joints
                            //ClientForce = UrDecode(540, myBufferBytes);
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("RTDE fail!", ex.Message);
                        isConect = false;
                        ClientConnect(_ip);
                        Client_RTDE2();
                    }
                });

                URCoordinates UrDecode(int index, byte[] buffer)
                {
                    URCoordinates rtn = new URCoordinates();

                    double value = _getValue(index, 1);
                    rtn.X.M = (float)value;
                    value = _getValue(index, 2);
                    rtn.Y.M = (float)value;
                    value = _getValue(index, 3);
                    rtn.Z.M = (float)value;
                    value = _getValue(index, 4);
                    rtn.Rx.rad = (float)value;
                    value = _getValue(index, 5);
                    rtn.Ry.rad = (float)value;
                    value = _getValue(index, 6);
                    rtn.Rz.rad = (float)value;
                    return rtn;

                    double _getValue(int startIndex, int joint)
                    {
                        byte[] b = new byte[8];
                        for (int i = 0; i < 8; i++)
                            b[7 - i] = buffer[(joint - 1) * 8 + i + startIndex];
                        return BitConverter.ToDouble(b, 0);
                    }
                    double _getDoubel(int startIndex, int joint)
                    {
                        byte[] b = new byte[8];
                        for (int i = 0; i < 8; i++)
                            b[7 - i] = buffer[(joint - 1) * 8 + i + startIndex];
                        return BitConverter.ToDouble(b, 0);
                    }
                }
                int convert_Int(byte[] buffer, int startIndex)
                {
                    byte[] b = new byte[4];
                    for (int i = 0; i < 4; i++)
                        b[3 - i] = buffer[i + startIndex];
                    return BitConverter.ToInt16(b, 0);
                }
                URJoint UrDecode_joint(int index, byte[] buffer)
                {
                    URJoint rtn = new URJoint();

                    double value = _getValue(index, 1);
                    rtn.J1.rad = (float)value;
                    value = _getValue(index, 2);
                    rtn.J2.rad = (float)value;
                    value = _getValue(index, 3);
                    rtn.J3.rad = (float)value;
                    value = _getValue(index, 4);
                    rtn.J4.rad = (float)value;
                    value = _getValue(index, 5);
                    rtn.J5.rad = (float)value;
                    value = _getValue(index, 6);
                    rtn.J6.rad = (float)value;
                    return rtn;

                    double _getValue(int startIndex, int joint)
                    {
                        byte[] b = new byte[8];
                        for (int i = 0; i < 8; i++)
                            b[7 - i] = buffer[(joint - 1) * 8 + i + startIndex];
                        return BitConverter.ToDouble(b, 0);
                    }
                }
            }
            public URCoordinates getPosition()
            {
                return ClientPos;
            }
            public URJoint getJoint()
            {
                return ClientJoint;
            }
            public URCoordinates getFilterForce()
            {
                URCoordinates rtn = new URCoordinates();
                for (int i = 0; i < ClientForceFilter.Count(); i++)
                {
                    rtn.X += ClientForceFilter[i].X;
                    rtn.Y += ClientForceFilter[i].Y;
                    rtn.Z += ClientForceFilter[i].Z;
                    rtn.Rx += ClientForceFilter[i].Rx;
                    rtn.Ry += ClientForceFilter[i].Ry;
                    rtn.Rz += ClientForceFilter[i].Rz;
                }
                rtn.X = (rtn.X.M / ClientForceFilter.Count()).M();
                rtn.Y = (rtn.Y.M / ClientForceFilter.Count()).M();
                rtn.Z = (rtn.Z.M / ClientForceFilter.Count()).M();
                rtn.Rx = (rtn.Rx.rad / ClientForceFilter.Count()).rad();
                rtn.Ry = (rtn.Ry.rad / ClientForceFilter.Count()).rad();
                rtn.Rz = (rtn.Ry.rad / ClientForceFilter.Count()).rad();

                return rtn;
            }
            public URCoordinates getForce()
            {
                return ClientForce;
            }

            public void ClientDisconnect()
            {
                urSocket_realTime.Close();
                client_realTime.Close();
                stateChange?.Invoke(tcpState.Disconnect);
                isConect = false;
            }

            public enum DashBoardCommand
            {
                load = 1,
                play = 2,
                stop = 3,
                pause = 4,
                power_on = 5,
                power_off = 6,
                brake_release = 7,
                popup = 8,
                close_popup = 9
            }
            public string ClientCmd(DashBoardCommand cmd, string info = "")
            {
                if (!isConect) { Console.WriteLine("尚未連線:isConect"); return ""; }
                if (cmd == DashBoardCommand.load)
                {
                    string load = "Socket_control.urp";
                    if (info != "")
                        load = info;
                    if (!ClientSend($"load {load}" + "\n", urSocket_dashboard))
                        return "Error";
                    string msg = ClientRead(urSocket_dashboard);
                    return msg;
                }
                else if (cmd == DashBoardCommand.play)
                {
                    if (!ClientSend("play" + "\n", urSocket_dashboard))
                        return "Error";
                    string msg = ClientRead(urSocket_dashboard);
                    return msg;
                }
                else if (cmd == DashBoardCommand.pause)
                {
                    if (!ClientSend("pause" + "\n", urSocket_dashboard))
                        return "Error";
                    string msg = ClientRead(urSocket_dashboard);
                    return msg;
                }
                else if (cmd == DashBoardCommand.stop)
                {
                    if (!ClientSend("stop" + "\n", urSocket_dashboard))
                        return "Error";
                    string msg = ClientRead(urSocket_dashboard);
                    return msg;
                }
                else if (cmd == DashBoardCommand.power_on)
                {
                    if (!ClientSend("power on" + "\n", urSocket_dashboard))
                        return "Error";
                    string msg = ClientRead(urSocket_dashboard);
                    return msg;
                }
                else if (cmd == DashBoardCommand.power_off)
                {
                    if (!ClientSend("power off" + "\n", urSocket_dashboard))
                        return "Error";
                    string msg = ClientRead(urSocket_dashboard);
                    return msg;
                }
                else if (cmd == DashBoardCommand.brake_release)
                {
                    if (!ClientSend("brake release" + "\n", urSocket_dashboard))
                        return "Error";
                    string msg = ClientRead(urSocket_dashboard);
                    return msg;
                }
                else if (cmd == DashBoardCommand.popup)
                {
                    if (!ClientSend($"popup {info}" + "\n", urSocket_dashboard))
                        return "Error";
                    string msg = ClientRead(urSocket_dashboard);
                    return msg;
                }
                else if (cmd == DashBoardCommand.close_popup)
                {
                    if (!ClientSend("close popup" + "\n", urSocket_dashboard))
                        return "Error";
                    string msg = ClientRead(urSocket_dashboard);
                    return msg;
                }
                return "不支援";
            }
        }

        #region  //---Server---\\
        Thread thread_server;
        string sMsg = "";
        public string urMsg
        {
            get
            { return sMsg; }
        }
        bool gripperObjectDetect = false;
        public bool urGrip
        {
            get
            {
                return gripperObjectDetect;
            }
        }


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

            //path
            if (Directory.Exists(rootPath) == false)
            {
                Directory.CreateDirectory(rootPath);
            }

        }
        //value
        float[] val_grip = new float[3];
        float[] val_pos = new float[6];
        float[] val_joint = new float[6];
        float velocity = 1.05f;
        float acceleration = 1.4f;
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
            Console.WriteLine("Client is accept");
            stateChange?.Invoke(tcpState.Connect);

            serverOn = true;
            cmd = mode.stop;
            sMsg = "";
            while (serverOn && acceptSocket.Connected)
            {
                if (sMsg == "End")
                    cmd = mode.End;

                if (cmd == mode.pmovep)//v3 done
                {
                    if (!_sendMsg("pmovep")) goto End;

                    sMsg = _waitRead();
                    if (CheckEnd()) goto End;

                    if (!_sendMsg(msgPosFormat(val_pos, acceleration, velocity))) goto End;

                    sMsg = _waitRead();
                    if (CheckEnd()) goto End;

                    if (!msgCheck(sMsg, "UR:done", "error!! UR robot didn't finish work?")) goto End;
                    cmd = mode.stop;
                }//v3 done
                else if (cmd == mode.jmovej) //v3 done
                {
                    if (!_sendMsg("jmovej")) goto End;
                    sMsg = _waitRead();
                    if (CheckEnd()) goto End;

                    if (!_sendMsg(msgJointFormat(val_joint, acceleration, velocity))) goto End;

                    sMsg = _waitRead();
                    if (CheckEnd()) goto End;

                    if (!msgCheck(sMsg, "UR:done", "error!! UR robot didn't finish work?")) goto End;
                    cmd = mode.stop;
                }//v3 done

                else if (cmd == mode.jservoj)//v3 done
                {
                    if (!_sendMsg("jservoj")) goto End;
                    sMsg = _waitRead();
                    if (CheckEnd()) goto End;

                    if (!_sendMsg(msgjServojFormat(val_joint))) goto End;

                    sMsg = _waitRead();
                    if (CheckEnd()) goto End;
                    Console.WriteLine(sMsg);//joint
                }//v3 done
                else if (cmd == mode.pservoj)//done
                {
                    if (!_sendMsg("pservoj")) goto End;
                    sMsg = _waitRead();
                    if (CheckEnd()) goto End;
                    //Console.WriteLine(sMsg);

                    if (!_sendMsg(msgpServojFormat(val_pos))) break;

                    sMsg = _waitRead();
                    if (CheckEnd()) goto End;
                    //Console.WriteLine(sMsg);//position
                }//v3 done(with IK bug)

                else if (cmd == mode.Rmovep)//done
                {
                    if (!_sendMsg("Rmovep")) break;
                    sMsg = _waitRead();
                    if (CheckEnd()) goto End;

                    if (!_sendMsg(msgPosFormat(val_pos, acceleration, velocity))) goto End;

                    sMsg = _waitRead();
                    if (CheckEnd()) goto End;
                    if (!msgCheck(sMsg, "UR:done", "error!! UR robot didn't finish work?")) goto End;

                    cmd = mode.stop;
                }//v3 done
                else if (cmd == mode.Rmovej)//done
                {
                    if (!_sendMsg("Rmovej")) goto End;
                    sMsg = _waitRead();
                    if (CheckEnd()) goto End;

                    if (!_sendMsg(msgJointFormat(val_joint, acceleration, velocity))) break;

                    sMsg = _waitRead();
                    if (CheckEnd()) goto End;
                    if (!msgCheck(sMsg, "UR:done", "error!! UR robot didn't finish work?")) goto End;

                    cmd = mode.stop;
                }//v3 done

                else if (cmd == mode.recordj)//done
                {
                    if (!_sendMsg("recordj")) break;
                    sMsg = _waitRead();
                    if (CheckEnd()) goto End;

                    Console.WriteLine(sMsg);//拿到joint
                }//v3 done
                else if (cmd == mode.recordp)//done
                {
                    if (!_sendMsg("recordp")) break;
                    sMsg = _waitRead();
                    if (CheckEnd()) goto End;

                    Console.WriteLine(sMsg);//拿到pos
                }//v3.2 done(IK bug)

                else if (cmd == mode.jogj)//v3 done
                {
                    if (!_sendMsg("jogj")) break;
                    sMsg = _waitRead();// jog模式不准stop，除非中斷
                    if (CheckEnd()) goto End;
                    Console.WriteLine(sMsg);//??

                    if (!_sendMsg(msgjServojFormat(val_joint))) break;

                    sMsg = _waitRead();
                    if (CheckEnd()) goto End;

                    Console.WriteLine(sMsg);//joint
                }//v3.1 done
                else if (cmd == mode.jogp)
                {
                    if (!_sendMsg("jogp")) break;
                    sMsg = _waitRead();// jog模式不准stop，除非中斷
                    if (CheckEnd()) goto End;
                    Console.WriteLine(sMsg);

                    if (!_sendMsg(msgpServojFormat(val_pos))) break;

                    sMsg = _waitRead();
                    if (CheckEnd()) goto End;
                    Console.WriteLine(sMsg);//joint
                }//v3.1 done

                else if (cmd == mode.force)//
                {
                    //只需執行一次
                    if (!_sendMsg("force")) break;
                    sMsg = _waitRead();//拿到joint
                    if (CheckEnd()) goto End;

                    if (!_sendMsg($"[{val_pos[0]},{val_pos[1]},{val_pos[2]},{val_pos[3]},{val_pos[4]},{val_pos[5]},{val_forceVector[0]},{val_forceVector[1]},{val_forceVector[2]},{val_forceVector[3]},{val_forceVector[4]},{val_forceVector[5]}]")) break;

                    sMsg = _waitRead();
                    if (CheckEnd()) goto End;
                    if (!msgCheck(sMsg, "UR:get value", "error!! UR set force fail?")) goto End;

                    //轉換force mode後換成取joint數值
                    cmd = mode.joint;
                }//v3 done
                else if (cmd == mode.gripper)//done
                {
                    if (!_sendMsg("gripper")) goto End;
                    sMsg = _waitRead();//get "UR:gripper"
                    if (CheckEnd()) goto End;

                    if (!_sendMsg($"[{val_grip[0]},{val_grip[1]},{val_grip[2]},0,0,0,0,0,0,0,0,0]")) goto End;

                    sMsg = _waitRead(); //UR:doen 0 or 1
                    if (CheckEnd()) goto End;

                    if (sMsg.IndexOf("UR:done") > 0)
                        Console.WriteLine("error!! UR robot didn't finish work?");
                    if (sMsg.IndexOf("1") > 0)
                        gripperObjectDetect = true;
                    else if (sMsg.IndexOf("0") > 0)
                        gripperObjectDetect = false;

                    cmd = mode.stop;
                }//v3 done
                else if (cmd == mode.stop)//done
                {
                    if (!_sendMsg("stop")) break;

                    sMsg = _waitRead();//拿到 postion
                    if (CheckEnd()) goto End;
                    //UrPosGet?.Invoke(sMsg);
                }//v3 done
                else if (cmd == mode.joint)//done
                {//get joint BUT DO NOT freedrive
                    if (!_sendMsg("joint")) break;

                    sMsg = _waitRead(); //拿到 joint
                    if (CheckEnd()) goto End;
                }//v3 done
                else if (cmd == mode.End)//done
                {
                    goto End;
                }

            }//while serverOn
        End:
            cmd = mode.End;
            UrCmd?.Invoke(cmd);
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
                    acceptSocket.Send(myBytes, myBytes.Length, SocketFlags.None);
                }
                catch
                {
                    Console.WriteLine("Socket send fail"); //可能是client先結束了
                    cmd = mode.End;
                    return false;
                }
                return true;
            }
            string msgPosFormat(float[] pos, float acc, float speed)
            {
                return $"p[{pos[0]},{pos[1]},{pos[2]},{pos[3]},{pos[4]},{pos[5]},{acc.ToString("0.000")},{speed.ToString("0.000")},0,0,0,0]";
            }
            string msgJointFormat(float[] joint, float acc, float speed)
            {
                return $"[{joint[0]},{joint[1]},{joint[2]},{joint[3]},{joint[4]},{joint[5]},{acc.ToString("0.000")},{speed.ToString("0.000")},0,0,0,0]";
            }
            string msgjServojFormat(float[] joint, float t = 0.008f, float lookahead_time = 0.1f, float gain = 300)
            {
                return $"[{joint[0]},{joint[1]},{joint[2]},{joint[3]},{joint[4]},{joint[5]},0,0,{t},{lookahead_time},{gain},0]";
            }
            string msgpServojFormat(float[] pos, float t = 0.008f, float lookahead_time = 0.1f, float gain = 300)
            {
                return $"p[{pos[0]},{pos[1]},{pos[2]},{pos[3]},{pos[4]},{pos[5]},0,0,{t},{lookahead_time},{gain},0]";
            }
            bool msgCheck(string msg, string check, string error_msg)
            {
                if (msg != check)
                {
                    Console.WriteLine(error_msg);
                    return false;
                }

                return true;
            }
            bool CheckEnd()
            {
                if (sMsg == "End" || cmd == mode.End)
                {
                    cmd = mode.End;
                    return true;
                }
                return false;
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
            End();
        }
        #endregion \\---Server---//


        public void Stop()
        {
            cmd = mode.stop;
            UrCmd?.Invoke(cmd);
        }
        public void End()
        {
            cmd = mode.End;
            UrCmd?.Invoke(cmd);
        }
        public void recvJoint()
        {
            cmd = mode.joint;
            UrCmd?.Invoke(cmd);
        }

        public bool getPosition(ref URCoordinates pos)
        {
            if (cmd == mode.stop)
            {
                pos = URCoordinates.str2urc(sMsg);
                return true;
            }

            pos = new URCoordinates();
            return false;
        }


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
            UrCmd?.Invoke(cmd);

            if (wait)
            {
                while (cmd != mode.stop && cmd != mode.End) ;
                UrCmd?.Invoke(cmd);
            }
        }
        /// <summary>go position and return when finished</summary>
        public void goPosition(URCoordinates pos, float acc = 1.2f, float speed = 0.25f)
        {
            acceleration = acc;
            velocity = speed;
            val_pos[0] = pos.X.M;
            val_pos[1] = pos.Y.M;
            val_pos[2] = pos.Z.M;
            val_pos[3] = pos.Rx.rad;
            val_pos[4] = pos.Ry.rad;
            val_pos[5] = pos.Rz.rad;
            cmd = mode.pmovep;
            UrCmd?.Invoke(cmd);
            while (cmd != mode.stop && cmd != mode.End) ;
            UrCmd?.Invoke(cmd);
        }
        /// <summary>注意!! 不能給距離現在太近的點</summary>
        public void goPosition2(URCoordinates pos, float acc = 1.2f, float speed = 0.25f)
        {
            acceleration = acc;
            velocity = speed;
            val_pos[0] = pos.X.M;
            val_pos[1] = pos.Y.M;
            val_pos[2] = pos.Z.M;
            val_pos[3] = pos.Rx.rad;
            val_pos[4] = pos.Ry.rad;
            val_pos[5] = pos.Rz.rad;
            cmd = mode.pmovej;
            while (cmd != mode.stop && cmd != mode.End) ;
        }

        /// <summary>go joint and return when finished</summary>
        public void goJoint(URJoint joint, float acc = 1.4f, float speed = 1.05f)
        {
            acceleration = acc;
            velocity = speed;
            val_joint[0] = joint.J1.rad;
            val_joint[1] = joint.J2.rad;
            val_joint[2] = joint.J3.rad;
            val_joint[3] = joint.J4.rad;
            val_joint[4] = joint.J5.rad;
            val_joint[5] = joint.J6.rad;
            cmd = mode.jmovej;
            UrCmd?.Invoke(cmd);
            while (cmd != mode.stop && cmd != mode.End) ;
            UrCmd?.Invoke(cmd);
        }

        //jog
        /// <summary>go jog joint till stop command</summary>
        public void goJog(URJoint joint, float acc = 1.4f, float speed = 1.05f)
        {
            acceleration = acc;
            velocity = speed;
            val_joint[0] = joint.J1.rad;
            val_joint[1] = joint.J2.rad;
            val_joint[2] = joint.J3.rad;
            val_joint[3] = joint.J4.rad;
            val_joint[4] = joint.J5.rad;
            val_joint[5] = joint.J6.rad;
            cmd = mode.jogj;
            UrCmd?.Invoke(cmd);
        }
        /// <summary>go jog position till stop command</summary>
        public void goJog(URCoordinates pos, float acc = 1.2f, float speed = 0.25f)
        {
            acceleration = acc;
            velocity = speed;
            val_pos[0] = pos.X.M;
            val_pos[1] = pos.Y.M;
            val_pos[2] = pos.Z.M;
            val_pos[3] = pos.Rx.rad;
            val_pos[4] = pos.Ry.rad;
            val_pos[5] = pos.Rz.rad;
            cmd = mode.jogp;
            UrCmd?.Invoke(cmd);
        }

        //track
        /// <summary>go tracking joint till stop command</summary>
        public void goTrack(URJoint joint, float acc = 1.4f, float speed = 1.05f)
        {
            acceleration = acc;
            velocity = speed;
            val_joint[0] = joint.J1.rad;
            val_joint[1] = joint.J2.rad;
            val_joint[2] = joint.J3.rad;
            val_joint[3] = joint.J4.rad;
            val_joint[4] = joint.J5.rad;
            val_joint[5] = joint.J6.rad;
            cmd = mode.jservoj;
            UrCmd?.Invoke(cmd);
        }
        /// <summary>go tracking position till stop command</summary>
        public void goTrack(URCoordinates pos, float acc = 1.2f, float speed = 0.25f)
        {
            acceleration = acc;
            velocity = speed;
            val_pos[0] = pos.X.M;
            val_pos[1] = pos.Y.M;
            val_pos[2] = pos.Z.M;
            val_pos[3] = pos.Rx.rad;
            val_pos[4] = pos.Ry.rad;
            val_pos[5] = pos.Rz.rad;
            cmd = mode.pservoj;
            UrCmd?.Invoke(cmd);
        }
        public void goTrack2(URCoordinates pos, URJoint nowJoint, float acc = 1.4f, float speed = 1.05f)
        {
            acceleration = acc;
            velocity = speed;
            URJoint[] joints = Kinematic.Kinematic.InverseKinematic(pos);
            //nowJoint = URJoint.str2joint(sMsg);
            if (nowJoint == new URJoint())
                return;
            URJoint trackJ = Kinematic.Kinematic.Select(joints, nowJoint);
            if (trackJ.J1.rad > Math.PI)
                trackJ.J1.rad -= (float)(2.0 * Math.PI);
            var a = nowJoint.ToString();
            var b = trackJ.ToString();

            val_joint[0] = trackJ.J1.rad;
            val_joint[1] = trackJ.J2.rad;
            val_joint[2] = trackJ.J3.rad;
            val_joint[3] = trackJ.J4.rad;
            val_joint[4] = trackJ.J5.rad;
            val_joint[5] = trackJ.J6.rad;
            cmd = mode.jservoj;
        }
        public void stopTrack()
        {
            Stop();
        }

        //force
        float[] val_forceVector = new float[6];
        public void goForceMode(URCoordinates pos, URCoordinates vector)
        {
            val_pos[0] = pos.X.M / 1000;
            val_pos[1] = pos.Y.M / 1000;
            val_pos[2] = pos.Z.M / 1000;
            val_pos[3] = pos.Rx.rad / 1000;
            val_pos[4] = pos.Ry.rad / 1000;
            val_pos[5] = pos.Rz.rad / 1000;


            val_forceVector[0] = vector.X.M > 0 ? 1 : 0;
            val_forceVector[1] = vector.Y.M > 0 ? 1 : 0;
            val_forceVector[2] = vector.Z.M > 0 ? 1 : 0;
            val_forceVector[3] = vector.Rx.rad > 0 ? 1 : 0;
            val_forceVector[4] = vector.Ry.rad > 0 ? 1 : 0;
            val_forceVector[5] = vector.Rz.rad > 0 ? 1 : 0;
            cmd = mode.force;
            UrCmd?.Invoke(cmd);
            while (cmd != mode.joint && cmd != mode.stop && cmd != mode.End) ;
            UrCmd?.Invoke(cmd);
        }
        public void stopForce()
        {
            Stop();
        }
        //relative
        /// <summary>go relative position and return when finished</summary>
        public void goRelativePosition(URCoordinates pos, float acc = 1.2f, float speed = 0.25f)
        {
            acceleration = acc;
            velocity = speed;
            val_pos[0] = pos.X.M;
            val_pos[1] = pos.Y.M;
            val_pos[2] = pos.Z.M;
            val_pos[3] = pos.Rx.rad;
            val_pos[4] = pos.Ry.rad;
            val_pos[5] = pos.Rz.rad;
            cmd = mode.Rmovep;
            UrCmd?.Invoke(cmd);
            while (cmd != mode.stop && cmd != mode.End) ;
            UrCmd?.Invoke(cmd);
        }
        /// <summary>go relative position and return when finished</summary>
        public void goRelativePosition(Unit x = null, Unit y = null, Unit z = null, Angle Rx = null, Angle Ry = null, Angle Rz = null, float acc = 1.2f, float speed = 0.25f)
        {
            acceleration = acc;
            velocity = speed;
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
            UrCmd?.Invoke(cmd);
            while (cmd != mode.stop && cmd != mode.End) ;
            UrCmd?.Invoke(cmd);
        }
        /// <summary>go relative joint and return when finished</summary>
        public void goRelativeJoint(Angle j1 = null, Angle j2 = null, Angle j3 = null, Angle j4 = null, Angle j5 = null, Angle j6 = null, float acc = 1.4f, float speed = 1.05f)
        {
            acceleration = acc;
            velocity = speed;
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
            UrCmd?.Invoke(cmd);
            while (cmd != mode.stop && cmd != mode.End) ;
            UrCmd?.Invoke(cmd);
        }
        //other
        public void goFunction(string info)
        {
            dynamicFunction.Invoke(info);
            //while (cmd != mode.stop && cmd != mode.End) ;
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

            string[] fileLine = File.ReadAllLines(file);
            try
            {
                for (int i = 0; i < fileLine.Length; i++)
                {
                    string line = fileLine[i];
                    //除去空白
                    line.Replace(" ", "");
                    //不理會註解
                    if (line.IndexOf("//") >= 0)
                        line = line.Substring(0, line.IndexOf("//"));

                    if (line.IndexOf("(") < 0)
                        continue;

                    string theCmd = line.Substring(0, line.IndexOf("("));
                    if (theCmd == "movej")
                    {
                        string info = line.Substring(line.IndexOf("[") + 1, line.IndexOf("]") - line.IndexOf("[") - 1);
                        string[] joint = info.Split(',');
                        goJoint(new URJoint(joint[0].toAngleRad(), joint[1].toAngleRad(), joint[2].toAngleRad(), joint[3].toAngleRad(), joint[4].toAngleRad(), joint[5].toAngleRad()));
                    }
                    else if (theCmd == "movep")
                    {
                        string info = line.Substring(line.IndexOf("[") + 1, line.IndexOf("]") - line.IndexOf("[") - 1);
                        string[] pos = info.Split(',');
                        goPosition(new URCoordinates(pos[0].toUnitM(), pos[1].toUnitM(), pos[2].toUnitM(), pos[3].toAngleRad(), pos[4].toAngleRad(), pos[5].toAngleRad()));
                    }
                    else if (theCmd == "movep2")
                    {
                        string info = line.Substring(line.IndexOf("[") + 1, line.IndexOf("]") - line.IndexOf("[") - 1);
                        string[] pos = info.Split(',');
                        goPosition2(new URCoordinates(pos[0].toUnitM(), pos[1].toUnitM(), pos[2].toUnitM(), pos[3].toAngleRad(), pos[4].toAngleRad(), pos[5].toAngleRad()));
                    }
                    else if (theCmd == "Rmovej")
                    {
                        string info = line.Substring(line.IndexOf("[") + 1, line.IndexOf("]") - line.IndexOf("[") - 1);
                        string[] joint = info.Split(',');
                        goRelativeJoint(joint[0].toAngleRad(), joint[1].toAngleRad(), joint[2].toAngleRad(), joint[3].toAngleRad(), joint[4].toAngleRad(), joint[5].toAngleRad());
                    }
                    else if (theCmd == "Rmovep")
                    {
                        string info = line.Substring(line.IndexOf("[") + 1, line.IndexOf("]") - line.IndexOf("[") - 1);
                        string[] pos = info.Split(',');
                        goRelativePosition(new URCoordinates(pos[0].toUnitM(), pos[1].toUnitM(), pos[2].toUnitM(), pos[3].toAngleRad(), pos[4].toAngleRad(), pos[5].toAngleRad()));
                    }
                    else if (theCmd == "jservoj")
                    {
                        string info = line.Substring(line.IndexOf("[") + 1, line.IndexOf("]") - line.IndexOf("[") - 1);
                        string[] joint = info.Split(',');
                        goTrack(new URJoint(joint[0].toAngleRad(), joint[1].toAngleRad(), joint[2].toAngleRad(), joint[3].toAngleRad(), joint[4].toAngleRad(), joint[5].toAngleRad()));
                    }
                    else if (theCmd == "pservoj")
                    {
                        string info = line.Substring(line.IndexOf("[") + 1, line.IndexOf("]") - line.IndexOf("[") - 1);
                        string[] pos = info.Split(',');
                        goTrack(new URCoordinates(pos[0].toUnitM(), pos[1].toUnitM(), pos[2].toUnitM(), pos[3].toAngleRad(), pos[4].toAngleRad(), pos[5].toAngleRad()));
                    }
                    else if (theCmd == "rq_move")
                    {
                        string info = line.Substring(line.IndexOf("(") + 1, line.IndexOf(")") - line.IndexOf("(") - 1);
                        goGripper(info.toInt(), force: 50, speed: 128);
                    }
                    else if (theCmd == "sleep" || theCmd == "Sleep")
                    {
                        string info = line.Substring(line.IndexOf("(") + 1, line.IndexOf(")") - line.IndexOf("(") - 1);
                        Thread.Sleep(info.toInt());
                    }
                    else if (theCmd == "function")
                    {
                        string info = line.Substring(line.IndexOf("(") + 1, line.IndexOf(")") - line.IndexOf("(") - 1);
                        goFunction(info);//會等 Function結束才會下一行，所以不用擔心手臂繼續會跑下一行指令
                    }
                    else if (theCmd == "")
                        continue;
                    else
                        Console.WriteLine($"Error at : {theCmd}");
                }
            }
            catch
            {
                Console.WriteLine($"Error : path txt error");
                return false;
            }

            Stop();
            return true;
        }



        #region //---Record---//

        string fileFullPath = "";
        public bool isRecord = false;
        StreamWriter txt_record;
        /// <summary>Record joint (Compatible with old version)</summary>
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
            UrCmd?.Invoke(cmd);
        }
        public void startRecordJoint(string fileName = "record.path")
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
            UrCmd?.Invoke(cmd);
        }
        public void startRecordPos(string fileName = "record.path")
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
            cmd = mode.recordp;
            UrCmd?.Invoke(cmd);
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
        public void Record_point()
        {
            if (isRecord == false)
            {
                Console.WriteLine("Record mode unable");
                return;
            }
            txt_record.WriteLine($"movep({sMsg})");
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
            mode tmp = cmd;
            if (withMove)
                goGripper(pos);

            if (isRecord == false)
            {
                Console.WriteLine("Record mode unable");
                return;
            }
            cmd = tmp;
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
                UrCmd?.Invoke(cmd);
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

        public class PathCmd
        {
            public enum txtCmd
            {
                none = 0,
                movej = 1,
                movep = 2,
                movep2 = 3,
                Rmovej = 4,
                Rmovep = 5,
                jservoj = 6,
                pservoj = 7,
                sleep = 8,
                rq_move = 9,
                function = 10
            }
            public txtCmd Cmd { get; } = txtCmd.none;
            object txtInfo;
            public PathCmd(string msg)
            {
                //去除空格
                msg.Replace(" ", "");

                if (msg.IndexOf("//") == 0)//代表是開頭
                {
                    Cmd = txtCmd.none;
                    txtInfo = msg;
                }
                else if (msg.IndexOf("//") > 0)//代表註解在後面
                    msg = msg.Substring(0, msg.IndexOf("//"));//所以去除註解
                else//代表沒有註解
                {
                    string theCmd = msg.Substring(0, msg.IndexOf("("));
                    if (theCmd == txtCmd.movej.ToString())
                    {
                        Cmd = txtCmd.movej;

                        string info = msg.Substring(msg.IndexOf("[") + 1, msg.IndexOf("]") - msg.IndexOf("[") - 1);
                        string[] joint = info.Split(',');
                        txtInfo = new URJoint(joint[0].toAngleRad(), joint[1].toAngleRad(), joint[2].toAngleRad(), joint[3].toAngleRad(), joint[4].toAngleRad(), joint[5].toAngleRad());
                    }
                    else if (theCmd == txtCmd.movep.ToString())
                    {
                        Cmd = txtCmd.movep;
                        string info = msg.Substring(msg.IndexOf("[") + 1, msg.IndexOf("]") - msg.IndexOf("[") - 1);
                        string[] pos = info.Split(',');
                        txtInfo = new URCoordinates(pos[0].toUnitM(), pos[1].toUnitM(), pos[2].toUnitM(), pos[3].toAngleRad(), pos[4].toAngleRad(), pos[5].toAngleRad());
                    }
                    else if (theCmd == txtCmd.movep2.ToString())
                    {
                        Cmd = txtCmd.movep2;
                        string info = msg.Substring(msg.IndexOf("[") + 1, msg.IndexOf("]") - msg.IndexOf("[") - 1);
                        string[] pos = info.Split(',');
                        txtInfo = new URCoordinates(pos[0].toUnitM(), pos[1].toUnitM(), pos[2].toUnitM(), pos[3].toAngleRad(), pos[4].toAngleRad(), pos[5].toAngleRad());
                    }
                    else if (theCmd == txtCmd.Rmovej.ToString())
                    {
                        Cmd = txtCmd.Rmovej;
                        string info = msg.Substring(msg.IndexOf("[") + 1, msg.IndexOf("]") - msg.IndexOf("[") - 1);
                        string[] joint = info.Split(',');
                        txtInfo = new URJoint(joint[0].toAngleRad(), joint[1].toAngleRad(), joint[2].toAngleRad(), joint[3].toAngleRad(), joint[4].toAngleRad(), joint[5].toAngleRad());
                    }
                    else if (theCmd == txtCmd.Rmovep.ToString())
                    {
                        Cmd = txtCmd.Rmovep;
                        string info = msg.Substring(msg.IndexOf("[") + 1, msg.IndexOf("]") - msg.IndexOf("[") - 1);
                        string[] pos = info.Split(',');
                        txtInfo = new URCoordinates(pos[0].toUnitM(), pos[1].toUnitM(), pos[2].toUnitM(), pos[3].toAngleRad(), pos[4].toAngleRad(), pos[5].toAngleRad());
                    }
                    else if (theCmd == txtCmd.jservoj.ToString())
                    {
                        Cmd = txtCmd.jservoj;
                        string info = msg.Substring(msg.IndexOf("[") + 1, msg.IndexOf("]") - msg.IndexOf("[") - 1);
                        string[] joint = info.Split(',');
                        txtInfo = (new URJoint(joint[0].toAngleRad(), joint[1].toAngleRad(), joint[2].toAngleRad(), joint[3].toAngleRad(), joint[4].toAngleRad(), joint[5].toAngleRad()));
                    }
                    else if (theCmd == txtCmd.pservoj.ToString())
                    {
                        Cmd = txtCmd.pservoj;
                        string info = msg.Substring(msg.IndexOf("[") + 1, msg.IndexOf("]") - msg.IndexOf("[") - 1);
                        string[] pos = info.Split(',');
                        txtInfo = (new URCoordinates(pos[0].toUnitM(), pos[1].toUnitM(), pos[2].toUnitM(), pos[3].toAngleRad(), pos[4].toAngleRad(), pos[5].toAngleRad()));
                    }
                    else if (theCmd == txtCmd.rq_move.ToString())
                    {
                        Cmd = txtCmd.rq_move;
                        string info = msg.Substring(msg.IndexOf("(") + 1, msg.IndexOf(")") - msg.IndexOf("(") - 1);
                        txtInfo = info.toInt();
                    }
                    else if (theCmd == txtCmd.sleep.ToString() || theCmd == "Sleep")
                    {
                        Cmd = txtCmd.sleep;
                        string info = msg.Substring(msg.IndexOf("(") + 1, msg.IndexOf(")") - msg.IndexOf("(") - 1);
                        txtInfo = info.toInt();
                    }
                    else if (theCmd == txtCmd.function.ToString())
                    {
                        Cmd = txtCmd.function;
                        string info = msg.Substring(msg.IndexOf("(") + 1, msg.IndexOf(")") - msg.IndexOf("(") - 1);
                        txtInfo = info;
                    }
                    else
                        txtInfo = "N/A";
                }



            }

            public object info
            {
                get { return txtInfo; }
            }



        }

    }
}
