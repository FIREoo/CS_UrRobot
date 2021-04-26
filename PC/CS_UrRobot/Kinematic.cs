using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UrRobot.Coordinates;

namespace UrRobot.Kinematic
{
    public static class DHtable
    {
        //UR3
        static double gripper = 0.130;//0.163(鈞凱版本)
        public static double d1 = 0.1519;
        static double d2 = 0;
        static double d3 = 0;
        public static double d4 = 0.11235;
        public static double d5 = 0.08535;
        public static double d6 = 0.0819 + gripper;
        public static double[] D = { d1, d2, d3, d4, d5, d6 };

        static double a1 = 0;
        public static double a2 = -0.24365;
        public static double a3 = -0.21325;
        static double a4 = 0;
        static double a5 = 0;
        static double a6 = 0;
        public static double[] A = { a1, a2, a3, a4, a5, a6 };

        public static double[] alpha = { Math.PI / 2, 0, 0, Math.PI / 2, -Math.PI / 2, 0 };
    }
    public class RotateMatrix
    {
        double[,] Value = new double[3, 3];
        public RotateMatrix()
        {
            Value = new double[3, 3];
        }
        public RotateMatrix(double[,] array)
        {
            if (array.GetLength(0) != 3 || array.GetLength(1) != 3)
                throw new InvalidOperationException("array need to be [3,3]");
            Value = array.Clone() as double[,];
        }
        public double[,] toArray()
        {
            return Value;
        }

    }
    public class RotationVector
    {
        //rad
        public double Rx;
        public double Ry;
        public double Rz;
        public RotationVector()
        {
            Rx = 0;
            Ry = 0;
            Rz = 0;
        }
        public RotationVector(double rx, double ry, double rz)
        {
            Rx = rx;
            Ry = ry;
            Rz = rz;
        }
    }
    public class TransformationMatrix
    {
        double[,] Value = new double[4, 4];
        public TransformationMatrix()
        {
            Value = new double[4, 4];
        }
        public TransformationMatrix(double[,] array)
        {
            if (array.GetLength(0) != 4 || array.GetLength(1) != 4)
                throw new InvalidOperationException("array need to be [4,4]");
            Value = array.Clone() as double[,];
        }
        public TransformationMatrix(URCoordinates TCP)
        {
            RotationVector RV = new RotationVector(TCP.Rx.rad, TCP.Ry.rad, TCP.Rz.rad);
            RotateMatrix RM = RotationVetor2RotateMatrix(RV);

            var R = RM.toArray();
            Value = new double[,] {
            {R[0,0],R[0,1],R[0,2],TCP.X.M },
            {R[1,0],R[1,1],R[1,2],TCP.Y.M },
            {R[2,0],R[2,1],R[2,2],TCP.Z.M },
            {0,0,0,1 } };
        }
        public double[,] toArray()
        {
            return Value;
        }
        RotateMatrix RotationVetor2RotateMatrix(RotationVector RV)
        {
            double x = RV.Rx;
            double y = RV.Ry;
            double z = RV.Rz;
            double theta = Math.Sqrt(x * x + y * y + z * z);
            x /= theta;
            y /= theta;
            z /= theta;

            double C = Math.Cos(theta);
            double[,] M1 = new double[,] { { C, 0, 0 }, { 0, C, 0 }, { 0, 0, C } };

            double[,] rrt = new double[,] { { x * x, x * y, x * z }, { x * y, y * y, y * z }, { x * z, y * z, z * z } };
            double C1 = 1.0 - Math.Cos(theta);
            double[,] M2 = new double[,] { { x * x * C1, x * y * C1, x * z * C1 }, { x * y * C1, y * y * C1, y * z * C1 }, { x * z * C1, y * z * C1, z * z * C1 } };

            double S = Math.Sin(theta);
            double[,] r_x = new double[,] { { 0, -z, y }, { z, 0, -x }, { -y, x, 0 } };
            double[,] M3 = new double[,] { { 0, -z * S, y * S }, { z * S, 0, -x * S }, { -y * S, x * S, 0 } };
            // R = cos(theta)*I + (1 - cos(theta))*r*rT + sin(theta)*[r_x]
            double[,] R = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {
                    R[i, j] = M1[i, j] + M2[i, j] + M3[i, j];
                }
            return new RotateMatrix(R);

        }
    }


    public static class Kinematic
    {
        public static TransformationMatrix ForwardKinematic(URJoint joints)
        {
            double[,] T_06 = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };//T_06 = np.matrix(np.identity(4))
            double[][,] DH = new double[6][,];
            DH[0] = new double[4, 4];
            DH[1] = new double[4, 4];
            DH[2] = new double[4, 4];
            DH[3] = new double[4, 4];
            DH[4] = new double[4, 4];
            DH[5] = new double[4, 4];

            for (int i = 0; i < 6; i++)
            {
                T_06 = Matrix.MatrixProduct(T_06,
                    HTM(i, new double[] { joints.J1.rad, joints.J2.rad, joints.J3.rad, joints.J4.rad, joints.J5.rad, joints.J6.rad }));
                DH[i] = T_06;
            }


            TransformationMatrix rtn = new TransformationMatrix(DH[5]);//回傳第6軸的座標
            return rtn;
            double[,] HTM(int i, double[] theta)
            {//theta unit : rad
                double[,] Rot_z = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };//Rot_z = np.matrix(np.identity(4))
                Rot_z[0, 0] = Math.Cos(theta[i]);
                Rot_z[1, 1] = Math.Cos(theta[i]);
                Rot_z[0, 1] = -Math.Sin(theta[i]);
                Rot_z[1, 0] = Math.Sin(theta[i]);

                double[,] Trans_z = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };//Trans_z = np.matrix(np.identity(4))
                Trans_z[2, 3] = DHtable.D[i];

                double[,] Trans_x = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };//Trans_z = np.matrix(np.identity(4))
                Trans_x[0, 3] = DHtable.A[i];

                double[,] Rot_x = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };//Rot_x = np.matrix(np.identity(4))

                Rot_x[1, 1] = Math.Cos(DHtable.alpha[i]);
                Rot_x[2, 2] = Math.Cos(DHtable.alpha[i]);
                Rot_x[1, 2] = -Math.Sin(DHtable.alpha[i]);
                Rot_x[2, 1] = Math.Sin(DHtable.alpha[i]);

                //A_i = Rot_z * Trans_z * Trans_x * Rot_x
                double[,] A_i = Matrix.MatrixProduct(Matrix.MatrixProduct(Matrix.MatrixProduct(Rot_z, Trans_z), Trans_x), Rot_x);
                return A_i;
            }

        }
        public static URJoint[] InverseKinematic(URCoordinates TCP)
        {
            TransformationMatrix TM_TCP = new TransformationMatrix(TCP);

            double[,] desired_pos = TM_TCP.toArray();
            double[,] th = new double[6, 8];
            double[,] A = new double[,] { { 0, 0, -DHtable.d6, 1 } };
            double[,] B = new double[,] { { 0, 0, 0, 1 } };
            double[,] P_05 = Matrix.Minus(Matrix.MatrixProduct(desired_pos, A.Transpose()), B.Transpose());

            //---theta 1---
            double psi = Math.Atan2(P_05[1, 0], P_05[0, 0]);
            double p05 = Math.Sqrt(P_05[2 - 1, 0] * P_05[2 - 1, 0] + P_05[1 - 1, 0] * P_05[1 - 1, 0]);
            double check = DHtable.d4 / p05;
            if (check > 1) check = 1;
            else if (check < -1) check = -1;
            double phi = Math.Acos(check);

            double tmp = Math.PI / 2.0 + psi + phi;
            th[0, 0] = tmp;
            th[0, 1] = tmp;
            th[0, 2] = tmp;
            th[0, 3] = tmp;
            tmp = Math.PI / 2.0 + psi - phi;
            th[0, 4] = tmp;
            th[0, 5] = tmp;
            th[0, 6] = tmp;
            th[0, 7] = tmp;


            //---theta5---
            double[] cl = new double[] { 0, 4 };
            for (int i = 0; i < cl.Length; i++)
            {
                int c = (int)cl[i];
                double[,] T_10 = AH(1, th, c).Inverse();
                double[,] T_16 = Matrix.MatrixProduct(T_10, desired_pos);
                th[4, c] = +Math2.Acos((T_16[2, 3] - DHtable.d4) / DHtable.d6);
                th[4, c + 1] = +Math2.Acos((T_16[2, 3] - DHtable.d4) / DHtable.d6);
                th[4, c + 2] = -Math2.Acos((T_16[2, 3] - DHtable.d4) / DHtable.d6);
                th[4, c + 3] = -Math2.Acos((T_16[2, 3] - DHtable.d4) / DHtable.d6);
            }


            //---theta6---
            cl = new double[] { 0, 2, 4, 6 };
            for (int i = 0; i < cl.Length; i++)
            {
                int c = (int)cl[i];
                double[,] T_10 = AH(1, th, c).Inverse();
                double[,] T_16 = Matrix.MatrixProduct(T_10, desired_pos).Inverse2();
                th[5, c] = Math2.Atan2((-T_16[1, 2] / Math.Sin(th[4, c])), (T_16[0, 2] / Math.Sin(th[4, c])));
                th[5, c + 1] = Math2.Atan2((-T_16[1, 2] / Math.Sin(th[4, c])), (T_16[0, 2] / Math.Sin(th[4, c])));
            }

            //---theta3---
            cl = new double[] { 0, 2, 4, 6 };
            for (int i = 0; i < cl.Length; i++)
            {
                int c = (int)cl[i];
                double[,] T_10 = AH(1, th, c).Inverse();
                double[,] T_65 = AH(6, th, c);
                double[,] T_54 = AH(5, th, c);
                //var tm00 = Matrix.MatrixProduct(T_54, T_65);
                //var tm1 = tm00.Inverse();
                double[,] T_14 = Matrix.MatrixProduct(Matrix.MatrixProduct(T_10, desired_pos), Matrix.MatrixProduct(T_54, T_65).Inverse2());
                //double[,] T_14 = Matrix.MatrixProduct(Matrix.MatrixProduct(Matrix.MatrixProduct(T_10, desired_pos), T_54.Inverse()), T_65.Inverse());
                double[,] AA = new double[,] { { 0, -DHtable.d4, 0, 1 } };
                double[,] BB = new double[,] { { 0, 0, 0, 1 } };
                double[,] P_13 = Matrix.Minus(Matrix.MatrixProduct(T_14, AA.Transpose()), BB.Transpose());
                // check = (linalg.norm(P_13)**2 - a2**2 - a3**2) / (2 * a2 * a3)
                double norm_squre = P_13[0, 0] * P_13[0, 0] + P_13[1, 0] * P_13[1, 0] + P_13[2, 0] * P_13[2, 0] + P_13[3, 0] * P_13[3, 0];
                double t3 = Math2.Acos((norm_squre - DHtable.a2 * DHtable.a2 - DHtable.a3 * DHtable.a3) / (2 * DHtable.a2 * DHtable.a3));
                th[2, c] = t3;
                th[2, c + 1] = -t3;
            }
            //---theta2-- //---theta4---
            cl = new double[] { 0, 1, 2, 3, 4, 5, 6, 7 };
            for (int i = 0; i < cl.Length; i++)
            {
                int c = (int)cl[i];

                double[,] T_10 = AH(1, th, c).Inverse();
                double[,] T_65 = AH(6, th, c).Inverse2();
                double[,] T_54 = AH(5, th, c).Inverse();
                double[,] T_14 = Matrix.MatrixProduct(Matrix.MatrixProduct(Matrix.MatrixProduct(T_10, desired_pos), T_65), T_54);
                double[,] AA = new double[,] { { 0, -DHtable.d4, 0, 1 } };
                double[,] BB = new double[,] { { 0, 0, 0, 1 } };
                double[,] P_13 = Matrix.Minus(Matrix.MatrixProduct(T_14, AA.Transpose()), BB.Transpose());
                //theta2
                double norm = Math.Sqrt(P_13[0, 0] * P_13[0, 0] + P_13[1, 0] * P_13[1, 0] + P_13[2, 0] * P_13[2, 0] + P_13[3, 0] * P_13[3, 0]);
                th[1, c] = -Math.Atan2(P_13[1, 0], -P_13[0, 0]) + Math.Asin(DHtable.a3 * Math.Sin(th[2, c]) / norm);
                //theta2        
                double[,] T_32 = AH(3, th, c).Inverse();
                double[,] T_21 = AH(2, th, c).Inverse();
                double[,] T_34 = Matrix.MatrixProduct(Matrix.MatrixProduct(T_32, T_21), T_14);
                th[3, c] = Math.Atan2(T_34[1, 0], T_34[0, 0]);
            }

            th = th.Transpose();

            URJoint[] rtn = new URJoint[8];
            for (int i = 0; i < 8; i++)
            {
                rtn[i] = new URJoint(th[i, 0].rad(), th[i, 1].rad(), th[i, 2].rad(), th[i, 3].rad(), th[i, 4].rad(), th[i, 5].rad());
            }


            return rtn;

            double[,] AH(int n, double[,] _th, int c)
            {//th = new double[6, 8];
                double[,] T_a = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
                T_a[0, 3] = DHtable.A[n - 1];
                double[,] T_d = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
                T_d[2, 3] = DHtable.D[n - 1];
                double[,] Rzt = new double[,] { { Math.Cos(_th[n - 1, c]), -Math.Sin(_th[n - 1, c]), 0, 0 }, { Math.Sin(_th[n - 1, c]), Math.Cos(_th[n - 1, c]), 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
                double[,] Rxa = new double[,] { { 1, 0, 0, 0 }, { 0, Math.Cos(DHtable.alpha[n - 1]), -Math.Sin(DHtable.alpha[n - 1]), 0 }, { 0, Math.Sin(DHtable.alpha[n - 1]), Math.Cos(DHtable.alpha[n - 1]), 0 }, { 0, 0, 0, 1 } };

                return Matrix.MatrixProduct(Matrix.MatrixProduct(Matrix.MatrixProduct(T_d, Rzt), T_a), Rxa);
            }
        }
        public static URJoint Select(URJoint[] candidate, URJoint nowJoint)
        {
            double[] delta = new double[8];

            for (int i = 0; i < 8; i++)
            {
                delta[i] += Math.Abs(candidate[i].J1.rad - nowJoint.J1.rad);
                delta[i] += Math.Abs(candidate[i].J2.rad - nowJoint.J2.rad);
                delta[i] += Math.Abs(candidate[i].J3.rad - nowJoint.J3.rad);
                delta[i] += Math.Abs(candidate[i].J4.rad - nowJoint.J4.rad);
                delta[i] += Math.Abs(candidate[i].J5.rad - nowJoint.J5.rad);
                delta[i] += Math.Abs(candidate[i].J6.rad - nowJoint.J6.rad);
            }
            double minValue = delta.Min();
            int minIndex = delta.ToList().IndexOf(minValue);
            return candidate[minIndex];


        }
    }
    static class Matrix
    {
        static double[][] MatrixCreate(int rows, int cols)
        {
            // creates a matrix initialized to all 0.0s
            // do error checking here?
            double[][] result = new double[rows][];
            for (int i = 0; i < rows; ++i)
                result[i] = new double[cols]; // auto init to 0.0
            return result;
        }
        static double[][] MatrixProduct(double[][] matrixA, double[][] matrixB)
        {
            int aRows = matrixA.Length; int aCols = matrixA[0].Length;
            int bRows = matrixB.Length; int bCols = matrixB[0].Length;
            if (aCols != bRows)
                throw new Exception("Non-conformable matrices in MatrixProduct");
            double[][] result = MatrixCreate(aRows, bCols);
            for (int i = 0; i < aRows; ++i) // each row of A
                for (int j = 0; j < bCols; ++j) // each col of B
                    for (int k = 0; k < aCols; ++k)
                        result[i][j] += matrixA[i][k] * matrixB[k][j];
            return result;
        }
        public static double[,] MatrixProduct(double[,] matrixA, double[,] matrixB)
        {
            int aRows = matrixA.GetLength(0); int aCols = matrixA.GetLength(1);
            int bRows = matrixB.GetLength(0); int bCols = matrixB.GetLength(1);
            if (aCols != bRows)
                throw new Exception("Non-conformable matrices in MatrixProduct");
            double[,] result = new double[aRows, bCols];
            for (int i = 0; i < aRows; ++i) // each row of A
                for (int j = 0; j < bCols; ++j) // each col of B
                    for (int k = 0; k < aCols; ++k)
                        result[i, j] += matrixA[i, k] * matrixB[k, j];
            return result;
        }
        public static double[,] Transpose(this double[,] matrix)
        {
            int w = matrix.GetLength(0);
            int h = matrix.GetLength(1);
            double[,] result = new double[h, w];
            for (int i = 0; i < w; i++)
                for (int j = 0; j < h; j++)
                    result[j, i] = matrix[i, j];

            return result;
        }
        public static double[,] Minus(double[,] matrixA, double[,] matrixB)
        {
            double[,] result = new double[matrixA.GetLength(0), matrixA.GetLength(1)];
            for (int i = 0; i < matrixA.GetLength(0); i++)
                for (int j = 0; j < matrixA.GetLength(1); j++)
                    result[i, j] = matrixA[i, j] - matrixB[i, j];
            return result;
        }

        public static double[,] toRotateMatrix(double[] M)
        {
            //R = Rx * Ry * Rz
            double x = M[0];
            double y = M[1];
            double z = M[2];
            double Rx = M[3];
            double Ry = M[4];
            double Rz = M[5];

            double[,] T = new double[,] { { 1, 0, 0, x }, { 0, 1, 0, y }, { 0, 0, 1, z }, { 0, 0, 0, 1 } };
            double[,] TRx = new double[,] { { 1, 0, 0, 0 }, { 0, Math.Cos(Rx), -Math.Sin(Rx), 0 }, { 0, Math.Sin(Rx), Math.Cos(Rx), 0 }, { 0, 0, 0, 1 } };
            double[,] TRy = new double[,] { { Math.Cos(Ry), 0, Math.Sin(Ry), 0 }, { 0, 1, 0, 0 }, { -Math.Sin(Ry), 0, Math.Cos(Ry), 0 }, { 0, 0, 0, 1 } };
            double[,] TRz = new double[,] { { Math.Cos(Rz), -Math.Sin(Rz), 0, 0 }, { Math.Sin(Rz), Math.Cos(Rz), 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };

            return MatrixProduct(MatrixProduct(MatrixProduct(T, TRx), TRy), TRz);
        }

        public static double[,] toRotateMatrix2(double[] M)
        {
            //General rotations
            //R = Rz * Ry * Rx

            double x = M[0];
            double y = M[1];
            double z = M[2];
            double Rx = M[3];
            double Ry = M[4];
            double Rz = M[5];

            double[,] T = new double[,] { { 1, 0, 0, x }, { 0, 1, 0, y }, { 0, 0, 1, z }, { 0, 0, 0, 1 } };
            double[,] TRx = new double[,] { { 1, 0, 0, 0 }, { 0, Math.Cos(Rx), -Math.Sin(Rx), 0 }, { 0, Math.Sin(Rx), Math.Cos(Rx), 0 }, { 0, 0, 0, 1 } };
            double[,] TRy = new double[,] { { Math.Cos(Ry), 0, Math.Sin(Ry), 0 }, { 0, 1, 0, 0 }, { -Math.Sin(Ry), 0, Math.Cos(Ry), 0 }, { 0, 0, 0, 1 } };
            double[,] TRz = new double[,] { { Math.Cos(Rz), -Math.Sin(Rz), 0, 0 }, { Math.Sin(Rz), Math.Cos(Rz), 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };

            return MatrixProduct(MatrixProduct(MatrixProduct(T, TRz), TRy), TRx);
        }

        public static double[,] Inverse(this double[,] Array)
        {
            int m = 0;
            int n = 0;
            m = Array.GetLength(0);
            n = Array.GetLength(1);
            double[,] array = new double[2 * m + 1, 2 * n + 1];
            for (int k = 0; k < 2 * m + 1; k++)  //初始化數組
            {
                for (int t = 0; t < 2 * n + 1; t++)
                {
                    array[k, t] = 0.0;
                }
            }
            for (int i = 0; i < m; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    array[i, j] = Array[i, j];
                }
            }

            for (int k = 0; k < m; k++)
            {
                for (int t = n; t <= 2 * n; t++)
                {
                    if ((t - k) == m)
                    {
                        array[k, t] = 1.0;
                    }
                    else
                    {
                        array[k, t] = 0.0;
                    }
                }
            }
            //得到逆矩陣
            for (int k = 0; k < m; k++)
            {
                if (array[k, k] != 1)
                {
                    double bs = array[k, k];
                    array[k, k] = 1;
                    for (int p = k + 1; p < 2 * n; p++)
                    {
                        if (bs == 0)
                            Console.WriteLine($"k={k},p={p},array[k,p]={array[k, p]}");
                        array[k, p] /= bs;
                    }
                }
                for (int q = 0; q < m; q++)
                {
                    if (q != k)
                    {
                        double bs = array[q, k];
                        for (int p = 0; p < 2 * n; p++)
                        {
                            array[q, p] -= bs * array[k, p];
                        }
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            double[,] NI = new double[m, n];
            for (int x = 0; x < m; x++)
            {
                for (int y = n; y < 2 * n; y++)
                {
                    NI[x, y - n] = array[x, y];
                }
            }
            return NI;
        }
        public static double[,] Inverse2(this double[,] Array)
        {
            double[] m = new double[16];
            int count = 0;
            for (int n = 0; n < 4; n++)
                for (int j = 0; j < 4; j++)
                {
                    m[count] = Array[n, j];
                    count++;
                }

            double[] invOut = new double[16];
            double[] inv = new double[16];
            double det = 0;

            inv[0] = m[5] * m[10] * m[15] -
                     m[5] * m[11] * m[14] -
                     m[9] * m[6] * m[15] +
                     m[9] * m[7] * m[14] +
                     m[13] * m[6] * m[11] -
                     m[13] * m[7] * m[10];

            inv[4] = -m[4] * m[10] * m[15] +
                      m[4] * m[11] * m[14] +
                      m[8] * m[6] * m[15] -
                      m[8] * m[7] * m[14] -
                      m[12] * m[6] * m[11] +
                      m[12] * m[7] * m[10];

            inv[8] = m[4] * m[9] * m[15] -
                     m[4] * m[11] * m[13] -
                     m[8] * m[5] * m[15] +
                     m[8] * m[7] * m[13] +
                     m[12] * m[5] * m[11] -
                     m[12] * m[7] * m[9];

            inv[12] = -m[4] * m[9] * m[14] +
                       m[4] * m[10] * m[13] +
                       m[8] * m[5] * m[14] -
                       m[8] * m[6] * m[13] -
                       m[12] * m[5] * m[10] +
                       m[12] * m[6] * m[9];

            inv[1] = -m[1] * m[10] * m[15] +
                      m[1] * m[11] * m[14] +
                      m[9] * m[2] * m[15] -
                      m[9] * m[3] * m[14] -
                      m[13] * m[2] * m[11] +
                      m[13] * m[3] * m[10];

            inv[5] = m[0] * m[10] * m[15] -
                     m[0] * m[11] * m[14] -
                     m[8] * m[2] * m[15] +
                     m[8] * m[3] * m[14] +
                     m[12] * m[2] * m[11] -
                     m[12] * m[3] * m[10];

            inv[9] = -m[0] * m[9] * m[15] +
                      m[0] * m[11] * m[13] +
                      m[8] * m[1] * m[15] -
                      m[8] * m[3] * m[13] -
                      m[12] * m[1] * m[11] +
                      m[12] * m[3] * m[9];

            inv[13] = m[0] * m[9] * m[14] -
                      m[0] * m[10] * m[13] -
                      m[8] * m[1] * m[14] +
                      m[8] * m[2] * m[13] +
                      m[12] * m[1] * m[10] -
                      m[12] * m[2] * m[9];

            inv[2] = m[1] * m[6] * m[15] -
                     m[1] * m[7] * m[14] -
                     m[5] * m[2] * m[15] +
                     m[5] * m[3] * m[14] +
                     m[13] * m[2] * m[7] -
                     m[13] * m[3] * m[6];

            inv[6] = -m[0] * m[6] * m[15] +
                      m[0] * m[7] * m[14] +
                      m[4] * m[2] * m[15] -
                      m[4] * m[3] * m[14] -
                      m[12] * m[2] * m[7] +
                      m[12] * m[3] * m[6];

            inv[10] = m[0] * m[5] * m[15] -
                      m[0] * m[7] * m[13] -
                      m[4] * m[1] * m[15] +
                      m[4] * m[3] * m[13] +
                      m[12] * m[1] * m[7] -
                      m[12] * m[3] * m[5];

            inv[14] = -m[0] * m[5] * m[14] +
                       m[0] * m[6] * m[13] +
                       m[4] * m[1] * m[14] -
                       m[4] * m[2] * m[13] -
                       m[12] * m[1] * m[6] +
                       m[12] * m[2] * m[5];

            inv[3] = -m[1] * m[6] * m[11] +
                      m[1] * m[7] * m[10] +
                      m[5] * m[2] * m[11] -
                      m[5] * m[3] * m[10] -
                      m[9] * m[2] * m[7] +
                      m[9] * m[3] * m[6];

            inv[7] = m[0] * m[6] * m[11] -
                     m[0] * m[7] * m[10] -
                     m[4] * m[2] * m[11] +
                     m[4] * m[3] * m[10] +
                     m[8] * m[2] * m[7] -
                     m[8] * m[3] * m[6];

            inv[11] = -m[0] * m[5] * m[11] +
                       m[0] * m[7] * m[9] +
                       m[4] * m[1] * m[11] -
                       m[4] * m[3] * m[9] -
                       m[8] * m[1] * m[7] +
                       m[8] * m[3] * m[5];

            inv[15] = m[0] * m[5] * m[10] -
                      m[0] * m[6] * m[9] -
                      m[4] * m[1] * m[10] +
                      m[4] * m[2] * m[9] +
                      m[8] * m[1] * m[6] -
                      m[8] * m[2] * m[5];

            det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

            det = 1.0 / det;

            for (int n = 0; n < 16; n++)
                invOut[n] = inv[n] * det;

            double[,] inAns = new double[4, 4];
            count = 0;
            for (int n = 0; n < 4; n++)
                for (int j = 0; j < 4; j++)
                {
                    inAns[n, j] = invOut[count];
                    count++;
                }

            return inAns;
        }

    }
    static public class Math2
    {
        static public double Acos(double d)
        {
            if (d < -1) d = -1;
            else if (d > 1) d = 1;
            return Math.Acos(d);
        }
        static public double Atan2(double y, double x)
        {
            if (double.IsInfinity(x) || double.IsInfinity(y))
                return 0;
            else
                return Math.Atan2(y, x);
        }


    }
}
