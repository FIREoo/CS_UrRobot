using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using UrRobot.Socket;
using UrRobot.Coordinates;
using System.Net;
using System.Net.Sockets;
using System.Windows.Forms;
using System.IO;
using System.ComponentModel;
using System.Collections.ObjectModel;
using WpfCustomControl_UrRobot;

namespace Wpf_UrControlExample
{
    /// <summary>
    /// MainWindow.xaml 的互動邏輯
    /// </summary>
    public partial class MainWindow : Window
    {
        UrSocketControl UR = new UrSocketControl();
        ObservableCollection<ListViewData> ListViewDataCollection = new ObservableCollection<ListViewData>();
        public MainWindow()
        {
            InitializeComponent();
            LV_pathData.ItemsSource = ListViewDataCollection;
        }

        private void Btn_startServer_Click(object sender, RoutedEventArgs e)
        {
            UR.stopServer();
            UR.startServer("192.168.0.111", 888);

            Dispatcher.Invoke(() => { rect_serverState.Fill = (DefaultColor.Yellow); });

            UR.stateChange += (state) =>
            {
                if (state == tcpState.Connect)
                    Dispatcher.Invoke(() => { rect_serverState.Fill = (DefaultColor.Green); });
                else if (state == tcpState.Disconnect)
                    Dispatcher.Invoke(() => { rect_serverState.Fill = (DefaultColor.Red); });
            };
        }

        private void Btn_gripper_Click(object sender, RoutedEventArgs e)
        {
            UR.goGripper(int.Parse(tb_grip.Text), int.Parse(tb_grip_f.Text), int.Parse(tb_grip_s.Text));
            UR.goGripper(255, 0, 200);
        }

        private void Btn_goPos_Click(object sender, RoutedEventArgs e)
        {
            Task.Run(() => { UR.goPosition(new URCoordinates(0.45.M(), 0.0.M(), 0.14.M(), 3.14.rad(), 0.rad(), 0.rad())); });
        }

        private void Btn_goJoint_Click(object sender, RoutedEventArgs e)
        {
            Task.Run(() => { UR.goJoint(new URJoint(3.14.rad(), -1.57.rad(), 0.rad(), -1.57.rad(), 0.rad(), 0.rad()));}); 
        }

        private void Btn_recordj_Click(object sender, RoutedEventArgs e)
        {
            UR.startRecord();
        }

        private void Btn_robotStop_Click(object sender, RoutedEventArgs e)
        {
            UR.Stop();

        }

        private void Btn_servoj1_Click(object sender, RoutedEventArgs e)
        {
            //UR.goTrack(new URJoint(0.rad(), 0.rad(),0.rad(), 0.rad(), 0.rad(),0.rad()));
            UR.goTrack2(new URCoordinates(-455.87.mm(), -326.28.mm(), 72.63.mm(), (Math.PI / 2.0).rad(), 0.rad(), 0.rad()), new URJoint());
        }

        private void Btn_servoj2_Click(object sender, RoutedEventArgs e)
        {
            UR.goTrack(new URJoint(-0.5.rad(), 0.rad(), 0.rad(), 0.rad(), 0.rad(), 0.rad()));
            //UR.goTrack2(new URCoordinates(-455.87.mm(), -326.28.mm(), 72.63.mm(), (Math.PI / 2.0).rad(), 0.rad(), 0.rad()));
        }

        private void Btn_record_Click(object sender, RoutedEventArgs e)
        {
            UR.startRecord();
        }

        private void Btn_write_Click(object sender, RoutedEventArgs e)
        {
            UR.Record_joint();
        }

        private void Btn_endRecord_Click(object sender, RoutedEventArgs e)
        {
            UR.endRecord();
        }

        private void Btn_goFile_Click(object sender, RoutedEventArgs e)
        {
            UR.goFile("Path\\record.path");
        }

        private void Btn_Rmovep_Click(object sender, RoutedEventArgs e)
        {
            UR.goRelativePosition();
        }

        private void Btn_Rmovej_Click(object sender, RoutedEventArgs e)
        {
            UR.goRelativeJoint(j6: 1.rad());
        }
        //client
        UrSocketControl.Client URc = new UrSocketControl.Client();
        private void Btn_connectClient_Click(object sender, RoutedEventArgs e)
        {

            if (!URc.ClientConnect("192.168.0.200"))
                Console.WriteLine("Client連線失敗");

            URc.Client_RTDE();
        }

        private void Btn_clientPos_Click(object sender, RoutedEventArgs e)
        {
            Task.Run(() =>
            {
                while (true)
                    this.Dispatcher.Invoke((Action)(() => { lb_clientPos.Content = URc.getPosition().ToString("(3)", "0.00"); }));
            });
        }

        private void Btn_clientForce_Click(object sender, RoutedEventArgs e)
        {
            Task.Run(() =>
            {
                while (true)
                {
                    this.Dispatcher.Invoke((Action)(() => { lb_clientForce.Content = URc.getFilterForce().ToString("3", "0"); }));
                }
            });
        }

        //file system
        List<UrSocketControl.PathCmd> pathPack = new List<UrSocketControl.PathCmd>();
        private void Btn_openPathFile_Click(object sender, RoutedEventArgs e)
        {
            var fileContent = string.Empty;
            string filePath = string.Empty;

            using (OpenFileDialog openFileDialog = new OpenFileDialog())
            {
                openFileDialog.InitialDirectory = System.Environment.CurrentDirectory;
                openFileDialog.Filter = "path files(*.path)| *.path";
                openFileDialog.RestoreDirectory = true;

                if (openFileDialog.ShowDialog() == System.Windows.Forms.DialogResult.OK)
                {
                    string[] fileLine = File.ReadAllLines(openFileDialog.FileName);
                    foreach (string line in fileLine)
                    {
                        pathPack.Add(new UrSocketControl.PathCmd(line));
                    }

                }
            }
        }

        private void Btn_force_Click(object sender, RoutedEventArgs e)
        {
            UR.goForceMode(new URCoordinates(0.M(), 0.M(), 10.M(), 0.deg(), 0.deg(), 0.deg()), new URCoordinates(0.M(), 0.M(), 1.M(), 0.deg(), 0.deg(), 0.deg()));
        }
        #region //---dashboard---\\
        private void Btn_DB_load_Click(object sender, RoutedEventArgs e)
        {
            Console.WriteLine(URc.ClientCmd(UrSocketControl.Client.DashBoardCommand.load, ""));
        }
        private void Btn_DB_play_Click(object sender, RoutedEventArgs e)
        {
            URc.ClientCmd(UrSocketControl.Client.DashBoardCommand.play, "");
        }
        private void Btn_DB_pause_Click(object sender, RoutedEventArgs e)
        {
            URc.ClientCmd(UrSocketControl.Client.DashBoardCommand.pause, "");
        }
        private void Btn_DB_stop_Click(object sender, RoutedEventArgs e)
        {
            URc.ClientCmd(UrSocketControl.Client.DashBoardCommand.stop, "");
        }
        private void Btn_DB_powerOn_Click(object sender, RoutedEventArgs e)
        {
            URc.ClientCmd(UrSocketControl.Client.DashBoardCommand.power_on, "");
        }

        private void Btn_DB_powerOff_Click(object sender, RoutedEventArgs e)
        {
            URc.ClientCmd(UrSocketControl.Client.DashBoardCommand.power_off, "");
        }

        private void Btn_DB_breakRelease_Click(object sender, RoutedEventArgs e)
        {
            URc.ClientCmd(UrSocketControl.Client.DashBoardCommand.brake_release, "");
        }
        private void Btn_DB_popup_Click(object sender, RoutedEventArgs e)
        {
            URc.ClientCmd(UrSocketControl.Client.DashBoardCommand.popup, "pop!");
        }

        private void Btn_DB_closePopup_Click(object sender, RoutedEventArgs e)
        {
            URc.ClientCmd(UrSocketControl.Client.DashBoardCommand.close_popup, "");
        }


        #endregion \\---dashboard---//



        private void Btn_jog_PreviewMouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.MouseDevice.LeftButton == MouseButtonState.Pressed)
            {
                UR.goJog(new URJoint(tb_jog1.Text.toFloat().rad(), tb_jog2.Text.toFloat().rad(), tb_jog3.Text.toFloat().rad(), tb_jog4.Text.toFloat().rad(), tb_jog5.Text.toFloat().rad(), tb_jog6.Text.toFloat().rad()));
            }
            else if (e.MouseDevice.RightButton == MouseButtonState.Pressed)
            {
                UR.goJog(new URJoint(-(tb_jog1.Text.toFloat()).rad(), -(tb_jog2.Text.toFloat()).rad(), -(tb_jog3.Text.toFloat()).rad(), -(tb_jog4.Text.toFloat()).rad(), -(tb_jog5.Text.toFloat()).rad(), -(tb_jog6.Text.toFloat()).rad()));
            }
            e.Handled = true;
        }
        private void Btn_jogp_PreviewMouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.MouseDevice.LeftButton == MouseButtonState.Pressed)
            {
                UR.goJog(new URCoordinates(tb_jog1.Text.toFloat().mm(), tb_jog2.Text.toFloat().mm(), tb_jog3.Text.toFloat().mm(), tb_jog4.Text.toFloat().rad(), tb_jog5.Text.toFloat().rad(), tb_jog6.Text.toFloat().rad()));
            }
            else if (e.MouseDevice.RightButton == MouseButtonState.Pressed)
            {
                UR.goJog(new URCoordinates(-(tb_jog1.Text.toFloat()).mm(), -(tb_jog2.Text.toFloat()).mm(), -(tb_jog3.Text.toFloat()).mm(), -(tb_jog4.Text.toFloat()).rad(), -(tb_jog5.Text.toFloat()).rad(), -(tb_jog6.Text.toFloat()).rad()));
            }
            e.Handled = true;
        }
        private void Btn_jog_PreviewMouseUp(object sender, MouseButtonEventArgs e)
        {
            UR.Stop();
            e.Handled = true;
        }

        private void Btn_connectClient2_Click(object sender, RoutedEventArgs e)
        {
            if (!URc.ClientConnect("192.168.0.200"))
                Console.WriteLine("Client連線失敗");

            URc.Client_RTDE2();
        }
    }

    public class ListViewData : INotifyPropertyChanged
    {
        bool _check;
        string col1;

        public ListViewData(string col1, SolidColorBrush C1)
        {
            _check = false;
            Col1 = col1;
            Color1 = C1;

        }
        public bool isChecked
        {
            set
            {
                _check = value;
                NotifyPropertyChanged("isChecked");
            }
            get { return _check; }
        }
        public string Col1
        {
            set
            {
                col1 = value;
                NotifyPropertyChanged("Col1");
            }
            get { return col1; }
        }

        public SolidColorBrush Color1 { get; set; } = new SolidColorBrush(Colors.Black);

        public event PropertyChangedEventHandler PropertyChanged;
        protected void NotifyPropertyChanged(string propertyName)
        {
            if (PropertyChanged != null)
            { PropertyChanged(this, new PropertyChangedEventArgs(propertyName)); }
        }
    }
    public static class DefaultColor
    {
        public static SolidColorBrush DarkRed = new SolidColorBrush(Color.FromRgb(140, 68, 64));
        public static SolidColorBrush Red = new SolidColorBrush(Color.FromRgb(174, 83, 80));
        public static SolidColorBrush LightRed = new SolidColorBrush(Color.FromRgb(197, 129, 126));
        public static SolidColorBrush DarkGreen = new SolidColorBrush(Color.FromRgb(79, 120, 67));
        public static SolidColorBrush Green = new SolidColorBrush(Color.FromRgb(105, 159, 89));
        public static SolidColorBrush LightGreen = new SolidColorBrush(Color.FromRgb(177, 206, 168));
        public static SolidColorBrush DarkBlue = new SolidColorBrush(Color.FromRgb(59, 78, 169));
        public static SolidColorBrush Blue = new SolidColorBrush(Color.FromRgb(91, 102, 189));
        public static SolidColorBrush LightBlue = new SolidColorBrush(Color.FromRgb(153, 161, 211));
        public static SolidColorBrush DarkYellow = new SolidColorBrush(Color.FromRgb(230, 159, 13));
        public static SolidColorBrush Yellow = new SolidColorBrush(Color.FromRgb(243, 180, 48));
        public static SolidColorBrush LightYellow = new SolidColorBrush(Color.FromRgb(248, 205, 116));

    }

}
