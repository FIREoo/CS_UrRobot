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
    class UrSocketControl
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
                    if (cmd == mode.moveByFile)
                    {
                        string[] fileList = System.IO.File.ReadAllLines(ReadingFile);
                        for (int p = 0; p < fileList.Count();)
                        {
                            if (p >= fileList.Count())//代表最後一行了
                                break;
                            if (fileList[p] == "")
                            { p++; continue; }

                            switch (fileList[p])
                            {
                                case "position":
                                    while (true)
                                    {
                                        p++;
                                        if (p >= fileList.Count())//代表最後一行了
                                            break;
                                        if (fileList[p].IndexOf('[') == -1)//代表是指令
                                            break;
                                        _sendMsg("pmovep", ref cmd);//record pos模式

                                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                        Console.WriteLine("Robot : " + sMsg);//應該會是獲得 movep

                                        byte[] _pcount = new byte[1] { 1 };
                                        stream.Write(_pcount, 0, 1);//@test   1個點

                                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                        Console.WriteLine("Robot : " + sMsg);//應該會是獲得"set" 也就是要開始給座標點

                                        _sendMsg("(" + fileList[p].Substring(2, fileList[p].Length - 3) + ")", ref cmd);//point

                                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                        Console.WriteLine("Robot : " + sMsg);//應該會是獲得"work done" 
                                    }
                                    break;
                                case "joint":
                                    while (true)
                                    {
                                        p++;
                                        if (p >= fileList.Count())//代表最後一行了
                                            break;
                                        if (fileList[p].IndexOf('[') == -1)//代表是指令
                                            break;
                                        _sendMsg("jmovej", ref cmd);//jmovej模式

                                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                        Console.WriteLine("Robot : " + sMsg);//應該會是獲得 movej

                                        byte[] _pcount = new byte[1] { 1 };
                                        stream.Write(_pcount, 0, 1);//@test   1個點

                                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                        Console.WriteLine("Robot : " + sMsg);//應該會是獲得"set" 也就是要開始給座標點

                                        _sendMsg("(" + fileList[p].Substring(1, fileList[p].Length - 2) + ")", ref cmd);//joint

                                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                        Console.WriteLine("Robot : " + sMsg);//應該會是獲得"work done" 
                                    }
                                    break;
                                case "Rmovej":
                                    while (true)
                                    {
                                        p++;
                                        if (p >= fileList.Count())//代表最後一行了
                                            break;
                                        if (fileList[p].IndexOf('[') == -1)//代表是指令
                                            break;
                                        _sendMsg("Rmovej", ref cmd);//Rmovej模式 //相對移動 joint

                                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                        Console.WriteLine("Robot : " + sMsg);//應該會是獲得 Rmovej

                                        byte[] _pcount = new byte[1] { 1 };
                                        stream.Write(_pcount, 0, 1);//@test   1個點

                                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                        Console.WriteLine("Robot : " + sMsg);//應該會是獲得"set" 也就是要開始給座標點

                                        _sendMsg("(" + fileList[p].Substring(1, fileList[p].Length - 2) + ")", ref cmd);

                                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                        Console.WriteLine("Robot : " + sMsg);//應該會是獲得"work done" 
                                    }
                                    break;
                                case "Rmovep":
                                    while (true)
                                    {
                                        p++;
                                        if (p >= fileList.Count())//代表最後一行了
                                            break;
                                        if (fileList[p].IndexOf('[') == -1)//代表是指令
                                            break;
                                        _sendMsg("Rmovep", ref cmd);//Rmovep模式

                                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                        Console.WriteLine("Robot : " + sMsg);//應該會是獲得 Rmovej

                                        byte[] _pcount = new byte[1] { 1 };
                                        stream.Write(_pcount, 0, 1);//@test   1個點

                                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                        Console.WriteLine("Robot : " + sMsg);//應該會是獲得"set" 也就是要開始給座標點

                                        _sendMsg("(" + fileList[p].Substring(2, fileList[p].Length - 3) + ")", ref cmd);//point

                                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                        Console.WriteLine("Robot : " + sMsg);//應該會是獲得"work done" 
                                    }
                                    break;
                                case "pmovej":
                                    while (true)
                                    {
                                        p++;
                                        if (p >= fileList.Count())//代表最後一行了
                                            break;
                                        if (fileList[p].IndexOf('[') == -1)//代表是指令
                                            break;
                                        _sendMsg("pmovej", ref cmd);//record pos模式

                                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                        Console.WriteLine("Robot : " + sMsg);//應該會是獲得 movep

                                        byte[] _pcount = new byte[1] { 1 };
                                        stream.Write(_pcount, 0, 1);//@test   1個點

                                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                        Console.WriteLine("Robot : " + sMsg);//應該會是獲得"set" 也就是要開始給座標點

                                        _sendMsg("(" + fileList[p].Substring(2, fileList[p].Length - 3) + ")", ref cmd);//point

                                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                        Console.WriteLine("Robot : " + sMsg);//應該會是獲得"work done" 
                                    }
                                    break;
                                case "gripper"://gripper 只能一行 gripper 一行 數字
                                    p++;
                                    _sendMsg("gripper", ref cmd);//gripper模式

                                    sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                    Console.WriteLine("Robot : " + sMsg);//應該會是獲得 gripper

                                    byte[] pcount = new byte[3] { (byte)fileList[p].toInt(), 0, 150 };
                                    stream.Write(pcount, 0, 3);//pos force speed
                                                               //_sendMsg("(" + rq_pos + "," + 0 + "," + 150 + ")");

                                    sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                    Console.WriteLine("Robot : " + sMsg);//應該會是獲得"work done" 

                                    p++;
                                    break;
                                case "sleep":
                                    p++;
                                    Thread.Sleep(fileList[p].toInt());
                                    p++;
                                    break;

                                case "test":
                                    p++;
                                    if (p >= fileList.Count())//代表最後一行了
                                        break;
                                    if (fileList[p].IndexOf('[') == -1)//代表是指令
                                        break;
                                    _sendMsg("test", ref cmd);

                                    List<string> servoCmd = new List<string>();
                                    servoCmd.Add(fileList[p]);
                                    while (true)
                                    {
                                        p++;
                                        if (p >= fileList.Count())//代表最後一行了
                                            break;
                                        if (fileList[p].IndexOf('[') == -1)//代表是指令
                                            break;
                                        servoCmd.Add(fileList[p]);
                                    }

                                    sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                    Console.WriteLine("Robot : " + sMsg);//應該會是獲得 test

                                    for (int i = 0; i < servoCmd.Count; i++)
                                    {
                                        string str = $"( {servoCmd[i].Substring(1, servoCmd[i].Length - 2)})";
                                        _sendMsg(str, ref cmd);
                                    }
                                    _sendMsg("(0,0,-100,0,0,0)", ref cmd);

                                    sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                                    Console.WriteLine("Robot : " + sMsg);//應該會是獲得"work done" 
                                    break;
                            }
                            if (sMsg == "End") break;
                        }
                        cmd = mode.stop;
                    }

                    else if (cmd == mode.stop)
                    {
                        while (cmd == mode.stop)
                        {
                            if (!_sendMsg("stop", ref cmd)) break;
                            sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                            Console.WriteLine(sMsg); //拿到
                            if (sMsg != "UR:stop")
                                Console.WriteLine("error!! UR robot didn't stop?");
                        }
                    }
                    else if (cmd == mode.recordj)
                    {
                        if (!_sendMsg("recordj", ref cmd)) break;
                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                        Console.WriteLine("UR : " + sMsg);//應該會是獲得 robot joint
                    }
                    else if (cmd == mode.recordp)//遺棄 (因為 position 轉 joint 會出問題，錄製position很常無法執行
                    {
                    }
                    else if (cmd == mode.jog)//未完成
                    {
                    }
                    else if (cmd == mode.gripper)
                    {
                        if (!_sendMsg("gripper", ref cmd)) break;
                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                        Console.WriteLine(sMsg);

                        int rq_pos = 0;
                        int rq_force = 0;
                        int rq_speed = 10;
                        if (!_sendMsg($"[{rq_pos},{rq_force},{rq_speed},0,0,0]", ref cmd)) break;

                        sMsg = _waitRead(ref cmd); if (sMsg == "End") break;
                        Console.WriteLine(sMsg);
                        if (sMsg != "UR:done")
                            Console.WriteLine("error!! UR robot didn't finish work?");

                        cmd = mode.stop;
                    }
                    else if (cmd == mode.grip)//遺棄
                    {
                    }
                    else if (cmd == mode.End)
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
            thread_server.Abort();
            //OnLinkState(new LinkArgs("disconnect"));
        }


        #endregion //---Server---//
        void tmp()
        {
            URCoordinates urc = new URCoordinates(3.2.M(), 3.2.M(), 3.2.M(), 3.2.M(), 3.2.M(), 3.2.M());

        }
    }
}
