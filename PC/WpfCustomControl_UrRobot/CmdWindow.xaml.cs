using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using UrRobot.Coordinates;

namespace WpfCustomControl_UrRobot
{
    /// <summary>
    /// Window1.xaml 的互動邏輯
    /// </summary>
    public partial class CmdWindow : Window
    {
        UserControl_UrRobot Main;
        public CmdWindow(UserControl_UrRobot main)
        {
            InitializeComponent();
            Main = main;
        }

        //move control
        private void Btn_pmovep_Click(object sender, RoutedEventArgs e)
        {
            URCoordinates P = Main.getUIPosition();
            Task.Run(() => { Main.UR.goPosition(P); });
        }
        private void Btn_pservoj_Click(object sender, RoutedEventArgs e)
        {
            Main.UR.goTrack(Main.getUIPosition());
        }
        private void Btn_pmovej_Click(object sender, RoutedEventArgs e)
        {
            URJoint J = Main.getUIJoint();
            Task.Run(() => { Main.UR.goJoint(J); });
        }
        private void Btn_jservoj_Click(object sender, RoutedEventArgs e)
        {
            Main.UR.goTrack(Main.getUIJoint());
        }


        //-----Record-----
        private void Btn_record_startP_Click(object sender, RoutedEventArgs e)
        {
            Main.UR.startRecordPos();
        }
        private void Btn_record_writeP_Click(object sender, RoutedEventArgs e)
        {
            Main.UR.Record_point();
        }

        private void Btn_record_startJ_Click(object sender, RoutedEventArgs e)
        {
            Main.UR.startRecordJoint();
        }
        private void Btn_record_writeJ_Click(object sender, RoutedEventArgs e)
        {
            Main.UR.Record_joint();
        }

        private void Btn_record_grip_Click(object sender, RoutedEventArgs e)
        {
            Main.UR.goGripper(int.Parse(tb_grip.Text), int.Parse(tb_grip_force.Text), int.Parse(tb_grip_speed.Text));
        }
        private void Btn_record_end_Click(object sender, RoutedEventArgs e)
        {
            Main.UR.endRecord();
        }


        //Top most settings
        private void Grid_PreviewMouseDown(object sender, MouseButtonEventArgs e)
        {
            if (this.Topmost == true)
            {
                this.Topmost = false;
                poly_tm1.Fill = new SolidColorBrush(Color.FromRgb(100, 100, 100));
                poly_tm2.Fill = new SolidColorBrush(Color.FromRgb(100, 100, 100));
            }
            else
            {
                this.Topmost = true;
                poly_tm1.Fill = new SolidColorBrush(Color.FromRgb(255, 100, 100));
                poly_tm2.Fill = new SolidColorBrush(Color.FromRgb(255, 100, 100));
            }
        }


        //調整 座標數值
        private void Tb_in_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            if (e.Delta > 0)
            {
                ((TextBox)sender).Text = (float.Parse(((TextBox)sender).Text) + 5).ToString("0.0");
            }
            else if (e.Delta < 0)
            {
                ((TextBox)sender).Text = (float.Parse(((TextBox)sender).Text) - 5).ToString("0.0");
            }
        }
        private void Tb_in_PreviewMouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.ChangedButton == MouseButton.Middle)
                ((TextBox)sender).Text = "0.0";
        }


        //Drag Drop
        private void Grid_pos_Drop(object sender, DragEventArgs e)
        {
            if (e.Data.GetDataPresent(DataFormats.StringFormat))
            {
                string dataString = (string)e.Data.GetData(DataFormats.StringFormat);
                try
                {
                    URCoordinates pos = URCoordinates.str2urc(dataString);
                    tb_inP1.Text = (pos.X.mm).ToString("0.0");
                    tb_inP2.Text = (pos.Y.mm).ToString("0.0");
                    tb_inP3.Text = (pos.Z.mm).ToString("0.0");
                    tb_inP4.Text = (pos.Rx.rad).ToString("0.000");
                    tb_inP5.Text = (pos.Ry.rad).ToString("0.000");
                    tb_inP6.Text = (pos.Rz.rad).ToString("0.000");
                }
                catch
                {
                    MessageBox.Show("URCoordinates transfer fail!");
                }
            }
        }
        private void Grid_joint_Drop(object sender, DragEventArgs e)
        {
            if (e.Data.GetDataPresent(DataFormats.StringFormat))
            {
                string dataString = (string)e.Data.GetData(DataFormats.StringFormat);
                try
                {
                    URJoint joint = URJoint.str2joint(dataString);

                    tb_inJ1.Text = (joint.J1.rad).ToString("0.000");
                    tb_inJ2.Text = (joint.J2.rad).ToString("0.000");
                    tb_inJ3.Text = (joint.J3.rad).ToString("0.000");
                    tb_inJ4.Text = (joint.J4.rad).ToString("0.000");
                    tb_inJ5.Text = (joint.J5.rad).ToString("0.000");
                    tb_inJ6.Text = (joint.J6.rad).ToString("0.000");
                }
                catch
                {
                    MessageBox.Show("URJoint transfer fail!");
                }
            }
        }
    }
}
