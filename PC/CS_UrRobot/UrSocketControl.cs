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

        #region  //---Server---//
        Thread thread_server;
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
                //OnLinkState(new LinkArgs("Wait Connect"));
            }
            catch (Exception ex) { Console.WriteLine("tcp start error\n" + ex); return; }

            TcpClient UR_Client = tcp.AcceptTcpClient();
            NetworkStream stream;
            if (UR_Client.Client.Connected)
            {
                string sMsg = "";
                stream = UR_Client.GetStream();
                stream.WriteTimeout = 100;
                //OnLinkState(new LinkArgs("Connect"));
                serverOn = true;
                cmd = mode.stop;
                while (serverOn)
                {
                    if (cmd == mode.moveByFile) { }

                    else if(cmd == mode.pmovep)//done
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
                    else if (cmd == mode.stop)//done
                    {
                        if (!_sendMsg("stop", ref cmd)) break;
                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                        Console.WriteLine(sMsg); //拿到 postion
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
            val_joint[0] = j1;
            cmd = mode.jmovej;
        }

        #region //---Record---//
        public void startRecord()
        {
            cmd = mode.recordj;
        }
        #endregion //---Record---//

        void tmp()
        {
          //  URCoordinates urc = new URCoordinates(3.2.M(), 3.2.M(), 3.2.M(), 3.2.M(), 3.2.M(), 3.2.M());

        }
    }
}
