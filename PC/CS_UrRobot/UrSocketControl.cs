using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.IO;
using UrRobot.Coordinates;
using System.Numerics;

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
        jog = 10,
        jservoj = 11,
        pservoj = 121,
        moveByFile = 99
    }
    public class UrSocketControl
    {
        public mode cmd = mode.stop;

        static bool serverOn = false;
        public bool isServerRunning
        {
            get { return serverOn; }
        }

        NetworkStream myNetworkStream;
        TcpClient myTcpClient;
        private bool clientConect = false;

        #region //---Client---//
        public void creatClient(string IP)
        {
            string hostName = IP;
            int connectPort = 30002;
            myTcpClient = new TcpClient();
            try
            {
                myTcpClient.Connect(hostName, connectPort);
                Console.WriteLine("UR連線成功 !!\n");
                clientConect = true;
            }
            catch
            {
                Console.WriteLine
                           ("主機 {0} 通訊埠 {1} 無法連接  !!", hostName, connectPort);
                return;
            }
        }

        public void client_SendData(string msg)
        {
            if (!clientConect) { Console.WriteLine("尚未連線"); return; }

            Byte[] myBytes = Encoding.ASCII.GetBytes(msg);
            myNetworkStream = myTcpClient.GetStream();
            myNetworkStream.Write(myBytes, 0, myBytes.Length);
        }
        #endregion ---Client---

        #region  //---Server---//
        Thread thread_server;
        string sMsg = "";
        public void startServer(string ip, int port)
        {
            if (serverOn) // is the server is on, return
            { Console.WriteLine("已經在執行了"); return; }

            IPAddress IPAddress = IPAddress.Parse(ip);
            TcpListener tcpListener = new TcpListener(IPAddress, port);
            thread_server = new Thread(() => Server(tcpListener, ref cmd));
            thread_server.Start();
        }
        private void Server(TcpListener tcp, ref mode cmd)
        {
            //creat tcp
            try
            {
                tcp.Start();
                Console.WriteLine("Start server at:" + ((IPEndPoint)tcp.LocalEndpoint).Address.ToString());
                //OnLinkState(new LinkArgs("Wait Connect"));
            }
            catch (Exception ex) { Console.WriteLine("tcp start error\n" + ex); return; }

            TcpClient UR_Client = tcp.AcceptTcpClient();
            NetworkStream stream;
            if (UR_Client.Client.Connected)
            {
                Console.WriteLine("Client is connect");
                stream = UR_Client.GetStream();
                stream.WriteTimeout = 100;
                //OnLinkState(new LinkArgs("Connect"));
                serverOn = true;
                cmd = mode.stop;
                while (serverOn)
                {
                    if (cmd == mode.pmovep)//done
                    {
                        if (!_sendMsg("pmovep", ref cmd)) break;
                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;

                        if (!_sendMsg($"p[{val_pos[0]},{val_pos[1]},{val_pos[2]},{val_pos[3]},{val_pos[4]},{val_pos[5]}]", ref cmd)) break;

                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                        Console.WriteLine(sMsg);
                        if (sMsg != "UR:done")
                            Console.WriteLine("error!! UR robot didn't finish work?");
                        cmd = mode.stop;
                    }
                    else if (cmd == mode.jmovej)//done
                    {
                        if (!_sendMsg("jmovej", ref cmd)) break;
                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                        Console.WriteLine(sMsg);

                        if (!_sendMsg($"[{val_joint[0]},{val_joint[1]},{val_joint[2]},{val_joint[3]},{val_joint[4]},{val_joint[5]}]", ref cmd)) break;

                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                        Console.WriteLine(sMsg);
                        if (sMsg != "UR:done")
                            Console.WriteLine("error!! UR robot didn't finish work?");
                        cmd = mode.stop;
                    }
                    else if (cmd == mode.jservoj)//done
                    {
                        if (!_sendMsg("jservoj", ref cmd)) break;
                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                        Console.WriteLine(sMsg);

                        if (!_sendMsg($"[{val_joint[0]},{val_joint[1]},{val_joint[2]},{val_joint[3]},{val_joint[4]},{val_joint[5]}]", ref cmd)) break;

                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                        Console.WriteLine(sMsg);//joint
                    }
                    else if (cmd == mode.pservoj)
                    {
                        if (!_sendMsg("pservoj", ref cmd)) break;
                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                        Console.WriteLine(sMsg);

                        if (!_sendMsg($"p[{val_pos[0]},{val_pos[1]},{val_pos[2]},{val_pos[3]},{val_pos[4]},{val_pos[5]}]", ref cmd)) break;

                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                        Console.WriteLine(sMsg);//position
                    }
                    else if (cmd == mode.recordj)//done
                    {
                        if (!_sendMsg("recordj", ref cmd)) break;
                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
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
                        if (!_sendMsg("gripper", ref cmd)) break;
                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                        Console.WriteLine(sMsg);

                        if (!_sendMsg($"[{val_grip[0]},{val_grip[1]},{val_grip[2]},0,0,0]", ref cmd)) break;

                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
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
                        if (!_sendMsg("stop", ref cmd)) break;
                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                        Console.WriteLine(sMsg); //拿到 postion
                    }
                    else if (cmd == mode.End)//done
                    {
                        break;
                    }
                }//while serverOn
                stream.Close();

            }//if client connect
            UR_Client.Dispose();
            tcp.Stop();
            //OnLinkState(new LinkArgs("Disconnect"));
            Console.WriteLine("server disconnect");
            serverOn = false;

            #region subfunction
            string _waitRead(ref mode c)
            {
                string msg_read = "";
                while (serverOn)
                {
                    byte[] arrayBytesRequest = new byte[UR_Client.Available];
                    try
                    {
                        int nRead = stream.Read(arrayBytesRequest, 0, arrayBytesRequest.Length);//讀取UR手臂的數值
                        if (nRead > 0)//確認有收到東西後
                        {
                            msg_read = ASCIIEncoding.ASCII.GetString(arrayBytesRequest);
                            return msg_read;
                        }//read>0 //所以如果沒有收到的話就繼續再收一次
                    }
                    catch
                    {
                        c = mode.End;
                        return "End";
                    }
                }
                return "End";
            }
            bool _sendMsg(string str, ref mode c)
            {
                try
                {
                    byte[] arrayBytesAnswer = ASCIIEncoding.ASCII.GetBytes(str);
                    stream.Write(arrayBytesAnswer, 0, arrayBytesAnswer.Length);
                }
                catch
                {
                    //可能是client先結束了
                    Console.WriteLine("可能是client先結束了");
                    c = mode.End;
                    return false;
                }
                return true;
            }
            #endregion subfunction
        }

        public void stopServer()
        {
            serverOn = false;
            if (thread_server != null)
                thread_server.Abort();
            //OnLinkState(new LinkArgs("disconnect"));
        }
        #endregion //---Server---//

        public void Stop()
        {
            cmd = mode.stop;
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


        public void goFile(string file = "")
        {
            if (file == "")
                file = fileFullPath;

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
            }
        }

        #region //---Record---//
        string rootPath = "Path\\";
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
            fileFullPath = rootPath + fileName;
            txt_record = new StreamWriter(fileFullPath, false);//false覆寫
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

   public class IK
    {
        // d (unit: mm)
        double d1 = 0.1519;
        double d2 = 0;
        double d3 = 0;
        double d4 = 0.11235;
        double d5 = 0.08535;
        double d6 = 0.0819;

        // a (unit: mm)
        double a1 = 0;
        double a2 = -0.24365;
        double a3 = -0.21325;
        double a4 = 0;
        double a5 = 0;
        double a6 = 0;

        // List type of D-H parameter
        // Do not remove these
        public IK()
        {
            double[] d = new[] { d1, d2, d3, d4, d5, d6 };// unit: mm
            double[] a = new[] { a1, a2, a3, a4, a5, a6 };// unit: mm
            double[] alpha = new[] { Math.PI / 2d, 0, 0, Math.PI / 2d, -Math.PI / 2d, 0 };// unit: radian
        }

        public void ur2ros(double px, double py, double pz, double Rx, double Ry, double Rz)
        {
            double[] ros_pose = new double[7];

            ros_pose[0] = px;
            ros_pose[1] = py;
            ros_pose[2] = pz;

            double angle = Math.Sqrt(Rx * Rx + Ry * Ry + Rz * Rz);
            double[] direction = new[] { Rx / angle, Ry / angle, Rz / angle };

    // np_T = tf.rotation_matrix(angle, direction)
    //np_q = tf.quaternion_from_matrix(np_T)
                
    //ros_pose.orientation.x = np_q[0]
    //ros_pose.orientation.y = np_q[1]
    //ros_pose.orientation.z = np_q[2]
    //ros_pose.orientation.w = np_q[3]



        }
        public void rotation_matrix(double angle,double[] direction)
        {
            double sina = Math.Sin(angle);
            double cosa = Math.Cos(angle);
            double[,] R = new[,] { { cosa, 0, 0 }, { 0, cosa, 0 }, { 0, 0, cosa } };
        }


    }

}
