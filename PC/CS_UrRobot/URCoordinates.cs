using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace UrRobot.Coordinates
{
    static public class ex
    {
        static public Unit M(this double value)
        {
            return new Unit((float)value);
        }
        static public Unit M(this float value)
        {
            return new Unit((float)value);
        }
        static public Unit M(this int value)
        {
            return new Unit((float)value);
        }
        static public Unit mm(this double value)
        {
            return new Unit((float)value / 1000f);
        }
        static public Unit mm(this float value)
        {
            return new Unit((float)value / 1000f);
        }
        static public Unit mm(this int value)
        {
            return new Unit((float)value / 1000f);
        }
        static public Angle rad(this double value)
        {
            return new Angle((float)value);
        }
        static public Angle rad(this float value)
        {
            return new Angle((float)value);
        }
        static public Angle rad(this int value)
        {
            return new Angle((float)value);
        }
        static public Angle deg(this double value)
        {
            return new Angle(value * (float)Math.PI / 180.0f);
        }
        static public Angle deg(this float value)
        {
            return new Angle(value * (float)Math.PI / 180.0f);
        }
        static public Angle deg(this int value)
        {
            return new Angle(value * (float)Math.PI / 180.0f);
        }

        static public int toInt(this string str)
        {
            try
            {
                return int.Parse(str);
            }
            catch
            {
                Console.WriteLine("to int fail!");
                return 0;
            }
        }
        static public float toFloat(this string str)
        {
            try
            {
                return float.Parse(str);
            }
            catch
            {
                Console.WriteLine("to float fail!");
                return 0;
            }
        }
        static public bool toBool(this string str)
        {
            if (str == "T" || str == "t" || str == "True" || str == "true" || str == "1" || str == "H" || str == "h" || str == "High" || str == "high")
                return true;
            else
                return false;
        }
        static public double toDouble(this string str)
        {
            try
            {
                return double.Parse(str);
            }
            catch
            {
                Console.WriteLine("to double fail!");
                return 0;
            }
        }
        static public Angle toAngleRad(this string str)
        {
            try
            {
                float f = float.Parse(str);
                return f.rad();
            }
            catch
            {
                Console.WriteLine("to float fail!");
                return new Angle();
            }
        }
        static public Unit toUnitM(this string str)
        {
            try
            {
                float f = float.Parse(str);
                return f.M();
            }
            catch
            {
                Console.WriteLine("to float fail!");
                return new Unit();
            }
        }

        public static bool IsBetween<T>(this T item, T start, T end)
        {
            return Comparer<T>.Default.Compare(item, start) >= 0
                && Comparer<T>.Default.Compare(item, end) <= 0;
        }
    }
    public class Unit
    {
        float _M = 0;
        public Unit(float v)
        {
            _M = v;
        }
        public Unit(double v)
        {
            _M = (float)v;
        }
        public Unit()
        {
            _M = 0;
        }

        public float M
        {
            get { return _M; }
            set { _M = value; }
        }
        public float mm
        {
            get { return _M * 1000.0f; }
            set { _M = value / 1000.0f; }
        }

        public static Unit operator +(Unit p1, Unit p2)
        {
            Unit rtn = new Unit();
            rtn.M = p1.M + p2.M;
            return rtn;
        }
        public static Unit operator -(Unit p1, Unit p2)
        {
            Unit rtn = new Unit();
            rtn.M = p1.M - p2.M;
            return rtn;
        }
        public static Unit operator -(Unit p1)
        {
            Unit rtn = new Unit();
            rtn.M = -p1.M;
            return rtn;
        }
    }
    public class Angle
    {
        float _rad = 0;
        public Angle(float v)
        {
            _rad = v;
        }
        public Angle(double v)
        {
            _rad = (float)v;
        }
        public Angle()
        {
            _rad = 0;
        }
        public float rad
        {
            get { return _rad; }
            set { _rad = value; }
        }
        public float deg
        {
            get { return _rad * 180.0f / (float)Math.PI; }
            set { _rad = value * (float)Math.PI / 180.0f; }
        }

        public static Angle operator +(Angle p1, Angle p2)
        {
            Angle rtn = new Angle();
            rtn.rad = p1.rad + p2.rad;
            return rtn;
        }
        public static Angle operator -(Angle p1, Angle p2)
        {
            Angle rtn = new Angle();
            rtn.rad = p1.rad - p2.rad;
            return rtn;
        }
        public static Angle operator -(Angle p1)
        {
            Angle rtn = new Angle();
            rtn.rad = -p1.rad;
            return rtn;
        }
    }


    public class URCoordinates
    {
        public Unit X { get; set; } = new Unit();
        public Unit Y { get; set; } = new Unit();
        public Unit Z { get; set; } = new Unit();
        public Angle Rx { get; set; } = new Angle();
        public Angle Ry { get; set; } = new Angle();
        public Angle Rz { get; set; } = new Angle();
        public byte Grip { get; set; } = 0;

        public static bool operator ==(URCoordinates p1, URCoordinates p2)
        {
            if (
                    p1.X.M == p2.X.M &&
                    p1.Y.M == p2.Y.M &&
                    p1.Z.M == p2.Z.M &&
                    p1.Rx.rad == p2.Rx.rad &&
                    p1.Ry.rad == p2.Ry.rad &&
                     p1.Rz.rad == p2.Rz.rad
                 )
                return true;
            else
                return false;
        }
        public static bool operator !=(URCoordinates p1, URCoordinates p2)
        {
            if (
                    p1.X.M == p2.X.M &&
                    p1.Y.M == p2.Y.M &&
                    p1.Z.M == p2.Z.M &&
                    p1.Rx.rad == p2.Rx.rad &&
                    p1.Ry.rad == p2.Ry.rad &&
                     p1.Rz.rad == p2.Rz.rad
                 )
                return false;
            else
                return true;
        }


        public URCoordinates()
        {
            X = 0.M();
            Y = 0.M();
            Z = 0.M();
            Rx = 0.rad();
            Ry = 0.rad();
            Rz = 0.rad();
            Grip = 0;
        }

        public URCoordinates(Unit _x, Unit _y, Unit _z = null, Angle _Rx = null, Angle _Ry = null, Angle _Rz = null, byte _G = 0)
        {
            if (Z == null) Z = new Unit();
            else Z = _z;
            if (Rx == null) Rx = new Angle();
            else Rx = _Rx;
            if (Ry == null) Ry = new Angle();
            else Ry = _Ry;
            if (Rz == null) Rz = new Angle();
            else Rz = _Rz;

            X = _x;
            Y = _y;
            Z = _z;

            Grip = _G;
        }
        //public URCoordinates(float _x, float _y, float _z, float _Rx, float _Ry, float _Rz, byte _G = 0, string unit = "M")
        //{
        //    if (unit == "M")
        //    {
        //        X = new Unit(_x);
        //        Y = new Unit(_y);
        //        Z = new Unit(_z);
        //        Rx = new Angle(_Rx);
        //        Ry = new Angle(_Ry);
        //        Rz = new Angle(_Rz);
        //        Grip = _G;
        //    }
        //}
        //public URCoordinates(double _x, double _y, double _z, double _Rx, double _Ry, double _Rz, byte _G = 0, string unit = "M")
        //{
        //    if (unit == "M")
        //    {
        //        X = new Unit(_x);
        //        Y = new Unit(_y);
        //        Z = new Unit(_z);
        //        Rx = new Angle(_Rx);
        //        Ry = new Angle(_Ry);
        //        Rz = new Angle(_Rz);
        //        Grip = _G;
        //    }
        //}
        public URCoordinates(URCoordinates input)
        {
            X = input.X;
            Y = input.Y;
            Z = input.Z;
            Rx = input.Rx;
            Ry = input.Ry;
            Rz = input.Rz;
        }

        /// <summary>
        /// e.g.
        /// "p[]" → p[X,Y,Z,Rx,Ry,Rz]
        /// "[]" → [X,Y,Z,Rx,Ry,Rz]
        /// "()" → (X,Y,Z,Rx,Ry,Rz)
        /// "(3)" → (X,Y,Z)
        /// "[7]" → [X,Y,Z,Rx,Ry,Rz,Grip]
        /// </summary>
        /// <param name="format"></param>
        /// <returns></returns>
        public string ToString(string format = "[]", string stringFormat = "")
        {
            string rtn = "";
            if (format.IndexOf('p') >= 0)
                rtn += "p";
            if (format.IndexOf('[') >= 0)
                rtn += "[";
            if (format.IndexOf('(') >= 0)
                rtn += "(";

            if (format.IndexOf('2') >= 0)
                rtn += $"{X.M.ToString(stringFormat)},{Y.M.ToString(stringFormat)}";
            else if (format.IndexOf('3') >= 0)
                rtn += $"{X.M.ToString(stringFormat)},{Y.M.ToString(stringFormat)},{Z.M.ToString(stringFormat)}";
            else if (format.IndexOf('6') >= 0)
                rtn += $"{X.M.ToString(stringFormat)},{Y.M.ToString(stringFormat)},{Z.M.ToString(stringFormat)},{Rx.rad.ToString(stringFormat)},{Ry.rad.ToString(stringFormat)},{Rz.rad.ToString(stringFormat)}";
            else if (format.IndexOf('7') >= 0)
                rtn += $"{X.M},{Y.M},{Z.M},{Rx.rad},{Ry.rad},{Rz.rad},{Grip}";
            else
                rtn += $"{X.M.ToString(stringFormat)},{Y.M.ToString(stringFormat)},{Z.M.ToString(stringFormat)},{Rx.rad.ToString(stringFormat)},{Ry.rad.ToString(stringFormat)},{Rz.rad.ToString(stringFormat)}";

            if (format.IndexOf(']') >= 0)
                rtn += "]";
            if (format.IndexOf(')') >= 0)
                rtn += ")";

            return rtn;

        }

        public static URCoordinates str2urc(string str)
        {
            try
            {
                str = str.Substring(str.IndexOf("p[") + 2, str.IndexOf("]") - (str.IndexOf("p[") + 2));

                string[] pos = str.Split(',');
                URCoordinates rtn = new URCoordinates(0, 0, 0, 0, 0, 0, 0);
                rtn.X.M = pos[0].toFloat();
                rtn.Y.M = pos[1].toFloat();
                rtn.Z.M = pos[2].toFloat();
                rtn.Rx.rad = pos[3].toFloat();
                rtn.Ry.rad = pos[4].toFloat();
                rtn.Rz.rad = pos[5].toFloat();
                return rtn;
            }
            catch
            {
                return new URCoordinates();
            }




        }


        public static Vector3 ToRotVector(Vector3 rpy)
        {
            float roll = rpy.X;
            float pitch = rpy.Y;
            float yaw = rpy.Z;
            if (roll == 0 && pitch == 0 && yaw == 0)
                return new Vector3 { X = 0, Y = 0, Z = 0 };
            Matrix3 RollM = new Matrix3();
            RollM.M00 = 1; RollM.M01 = 0; RollM.M02 = 0;
            RollM.M10 = 0; RollM.M11 = Math.Cos(roll); RollM.M12 = -Math.Sin(roll);
            RollM.M20 = 0; RollM.M21 = Math.Sin(roll); RollM.M22 = Math.Cos(roll);

            Matrix3 PitchM = new Matrix3();
            PitchM.M00 = Math.Cos(pitch); PitchM.M01 = 0; PitchM.M02 = Math.Sin(pitch);
            PitchM.M10 = 0; PitchM.M11 = 1; PitchM.M12 = 0;
            PitchM.M20 = -Math.Sin(pitch); PitchM.M21 = 0; PitchM.M22 = Math.Cos(pitch);

            Matrix3 YawM = new Matrix3();
            YawM.M00 = Math.Cos(yaw); YawM.M01 = -Math.Sin(yaw); YawM.M02 = 0;
            YawM.M10 = Math.Sin(yaw); YawM.M11 = Math.Cos(yaw); YawM.M12 = 0;
            YawM.M20 = 0; YawM.M21 = 0; YawM.M22 = 1;

            Matrix3 Rot = new Matrix3();

            //rot = yaw * pitch * roll
            Rot = Matrix3.Dot(YawM, Matrix3.Dot(PitchM, RollM));

            double rotSum = Rot.M00 + Rot.M11 + Rot.M22 - 1;
            double alpha = Math.Acos(rotSum / 2);
            double theta = 0;
            if (roll >= 0)
                theta = alpha;
            else
                theta = 2 * Math.PI - alpha;
            double my = 1d / (2 * Math.Sin(theta));

            double rx = my * (Rot.M21 - Rot.M12) * theta;
            double ry = my * (Rot.M02 - Rot.M20) * theta;
            double rz = my * (Rot.M10 - Rot.M01) * theta;

            Vector3 rotationVector = new Vector3();
            rotationVector.X = (float)rx;
            rotationVector.Y = (float)ry;
            rotationVector.Z = (float)rz;
            return rotationVector;
        }
        public class Vector3
        {
            public Vector3()
            {
                X = 0;
                Y = 0;
                Z = 0;
            }
            public Vector3(float x, float y, float z)
            {
                X = x;
                Y = y;
                Z = z;
            }
            public Vector3(Angle x, Angle y, Angle z)
            {
                X = x.rad;
                Y = y.rad;
                Z = z.rad;
            }
            public float X;
            public float Y;
            public float Z;
        }
        public class Matrix3
        {
            public double M00;
            public double M10;
            public double M20;

            public double M01;
            public double M11;
            public double M21;

            public double M02;
            public double M12;
            public double M22;

            public static Matrix3 Dot(Matrix3 m1, Matrix3 m2)
            {
                Matrix3 rtn = new Matrix3();

                rtn.M00 = m1.M00 * m2.M00 + m1.M01 * m2.M10 + m1.M02 * m2.M20;
                rtn.M01 = m1.M00 * m2.M01 + m1.M01 * m2.M11 + m1.M02 * m2.M21;
                rtn.M02 = m1.M00 * m2.M02 + m1.M01 * m2.M12 + m1.M02 * m2.M22;

                rtn.M10 = m1.M10 * m2.M00 + m1.M11 * m2.M10 + m1.M12 * m2.M20;
                rtn.M11 = m1.M10 * m2.M01 + m1.M11 * m2.M11 + m1.M12 * m2.M21;
                rtn.M12 = m1.M10 * m2.M02 + m1.M11 * m2.M12 + m1.M12 * m2.M22;

                rtn.M20 = m1.M20 * m2.M00 + m1.M21 * m2.M10 + m1.M22 * m2.M20;
                rtn.M21 = m1.M20 * m2.M01 + m1.M21 * m2.M11 + m1.M22 * m2.M21;
                rtn.M22 = m1.M20 * m2.M02 + m1.M21 * m2.M12 + m1.M22 * m2.M22;

                return rtn;
            }
        }
    }

    public class URJoint
    {
        public Angle J1 { get; set; } = new Angle();
        public Angle J2 { get; set; } = new Angle();
        public Angle J3 { get; set; } = new Angle();
        public Angle J4 { get; set; } = new Angle();
        public Angle J5 { get; set; } = new Angle();
        public Angle J6 { get; set; } = new Angle();

        public URJoint(Angle j1 = null, Angle j2 = null, Angle j3 = null, Angle j4 = null, Angle j5 = null, Angle j6 = null)
        {
            if (j1 == null) j1 = new Angle();
            if (j2 == null) j2 = new Angle();
            if (j3 == null) j3 = new Angle();
            if (j4 == null) j4 = new Angle();
            if (j5 == null) j5 = new Angle();
            if (j6 == null) j6 = new Angle();

            J1 = j1;
            J2 = j2;
            J3 = j3;
            J4 = j4;
            J5 = j5;
            J6 = j6;
        }

        public static bool operator ==(URJoint joint1, URJoint joint2)
        {
            if (
                    joint1.J1.rad == joint2.J1.rad &&
                    joint1.J2.rad == joint2.J2.rad &&
                    joint1.J3.rad == joint2.J3.rad &&
                    joint1.J4.rad == joint2.J4.rad &&
                    joint1.J5.rad == joint2.J5.rad &&
                     joint1.J6.rad == joint2.J6.rad
                 )
                return true;
            else
                return false;
        }
        public static bool operator !=(URJoint joint1, URJoint joint2)
        {
            if (
                    joint1.J1.rad == joint2.J1.rad &&
                    joint1.J2.rad == joint2.J2.rad &&
                    joint1.J3.rad == joint2.J3.rad &&
                    joint1.J4.rad == joint2.J4.rad &&
                    joint1.J5.rad == joint2.J5.rad &&
                     joint1.J6.rad == joint2.J6.rad
                 )
                return false;
            else
                return true;
        }

        public string toString()
        {
            return "N/A";
        }
    }

}
