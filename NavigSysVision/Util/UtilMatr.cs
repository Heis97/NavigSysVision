using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace opengl3
{
    static public class UtilMatr
    {
        

        /// <summary>
        /// 
        /// </summary>
        /// <param name="ps"></param>
        /// <returns>
        /// Center sphere XYZ, radius RRR
        /// </returns>
        static public Point3d_GL[] findSphereFrom4Points(Point3d_GL[] ps)
        {
            var x1 = ps[0].x; var y1 = ps[0].y; var z1 = ps[0].z;
            var x2 = ps[1].x; var y2 = ps[1].y; var z2 = ps[1].z;
            var x3 = ps[2].x; var y3 = ps[2].y; var z3 = ps[2].z;
            var x4 = ps[3].x; var y4 = ps[3].y; var z4 = ps[3].z;
            var U = (z1 - z2) * (x3 * y4 - x4 * y3) - (z2 - z3) * (x4 * y1 - x1 * y4);
            var V = (z3 - z4) * (x1 * y2 - x2 * y1) - (z4 - z1) * (x2 * y3 - x3 * y2);
            var W = (z1 - z3) * (x4 * y2 - x2 * y4) - (z2 - z4) * (x1 * y3 - x3 * y1);

            var Ax = (x1 * x1 + y1 * y1 + z1 * z1) * (y2 * (z3 - z4) + y3 * (z4 - z2) + y4 * (z2 - z3));
            var Bx = (x2 * x2 + y2 * y2 + z2 * z2) * (y3 * (z4 - z1) + y4 * (z1 - z3) + y1 * (z3 - z4));
            var Cx = (x3 * x3 + y3 * y3 + z3 * z3) * (y4 * (z1 - z2) + y1 * (z2 - z4) + y2 * (z4 - z1));
            var Dx = (x4 * x4 + y4 * y4 + z4 * z4) * (y1 * (z2 - z3) + y2 * (z3 - z1) + y3 * (z1 - z2));

            var Ay = (x1 * x1 + y1 * y1 + z1 * z1) * (x2 * (z3 - z4) + x3 * (z4 - z2) + x4 * (z2 - z3));
            var By = (x2 * x2 + y2 * y2 + z2 * z2) * (x3 * (z4 - z1) + x4 * (z1 - z3) + x1 * (z3 - z4));
            var Cy = (x3 * x3 + y3 * y3 + z3 * z3) * (x4 * (z1 - z2) + x1 * (z2 - z4) + x2 * (z4 - z1));
            var Dy = (x4 * x4 + y4 * y4 + z4 * z4) * (x1 * (z2 - z3) + x2 * (z3 - z1) + x3 * (z1 - z2));

            var Az = (x1 * x1 + y1 * y1 + z1 * z1) * (x2 * (y3 - y4) + x3 * (y4 - y2) + x4 * (y2 - y3));
            var Bz = (x2 * x2 + y2 * y2 + z2 * z2) * (x3 * (y4 - y1) + x4 * (y1 - y3) + x1 * (y3 - y4));
            var Cz = (x3 * x3 + y3 * y3 + z3 * z3) * (x4 * (y1 - y2) + x1 * (y2 - y4) + x2 * (y4 - y1));
            var Dz = (x4 * x4 + y4 * y4 + z4 * z4) * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));

            var x0 = 0.5 * (Ax - Bx + Cx - Dx) / (U + V + W);
            var y0 = -0.5 * (Ay - By + Cy - Dy) / (U + V + W);
            var z0 = 0.5 * (Az - Bz + Cz - Dz) / (U + V + W);
            var R = Math.Sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
            return new Point3d_GL[] { new Point3d_GL(x0, y0, z0),new Point3d_GL(R,R,R) };

        }


        static float PI = (float)Math.PI;
        static public float toRad(float degrees)
        {
            return degrees * PI / 180;
        }

        static public float toDegrees(float rad)
        {
            return rad * 180 / PI;
        }
      


        public static Matrix<double> to_inv_rot_matrix(Matrix<double> m)
        {
            var matr = new Matrix<double>(4, 4);
            var x = m[0, 3];
            var y = m[1, 3];
            var z = m[2, 3];
            matr = m.Transpose();
            matr[0, 3] = x;
            matr[1, 3] = y;
            matr[2, 3] = z;

            matr[3, 0] = 0;
            matr[3, 1] = 0;
            matr[3, 2] = 0;
            return matr;
        }

        public static Matrix<double> to_inv_trns_matrix(Matrix<double> m)
        {
            var matr = m.Clone();
            var matr_inv = m.Clone();
            CvInvoke.Invert(matr_inv, matr_inv, DecompMethod.LU);
            var x = matr_inv[0, 3];
            var y = matr_inv[1, 3];
            var z = matr_inv[2, 3];
            matr[0, 3] = x;
            matr[1, 3] = y;
            matr[2, 3] = z;

            return matr;
        }

        static public Matrix<double> toMatrix(Mat mat)
        {
            var arr = mat.GetData();
            var v = (double[,])arr;
            return new Matrix<double>(v);
        }
        static public System.Drawing.PointF[] toPointF(Mat corn)
        {

            var arr = corn.GetData();
            var flarr = (float[,,])arr;
            var points = new System.Drawing.PointF[flarr.Length / 2];
            //  Console.WriteLine(flarr.Length);
            for (int i = 0; i < flarr.Length / 2; i++)
            {
                points[i] = new System.Drawing.PointF(flarr[i, 0, 0], flarr[i, 0, 1]);
                // Console.WriteLine(flarr[i, 0, 0] + " " + flarr[i, 0, 1]);
            }
            return points;
        }
        static public System.Drawing.PointF[] toPointF(VectorOfPointF corn)
        {
            var points = new System.Drawing.PointF[corn.Size];
            //  Console.WriteLine(flarr.Length);
            for (int i = 0; i < corn.Size; i++)
            {
                points[i] = new System.Drawing.PointF(corn[i].X, corn[i].Y);
                // Console.WriteLine(flarr[i, 0, 0] + " " + flarr[i, 0, 1]);
            }
            return points;
        }
        static public System.Drawing.PointF[] toPointF(MCvPoint3D32f[] corn)
        {
            var points = new System.Drawing.PointF[corn.Length];
            //  Console.WriteLine(flarr.Length);
            for (int i = 0; i < corn.Length; i++)
            {
                points[i] = new System.Drawing.PointF(corn[i].X, corn[i].Y);
                // Console.WriteLine(flarr[i, 0, 0] + " " + flarr[i, 0, 1]);
            }
            return points;
        }
        static public double[,] convToDouble(float[,] f_mass)
        {
            var d_mass = new double[f_mass.GetLength(0), f_mass.GetLength(1)];
            for (int i = 0; i < f_mass.GetLength(0); i++)
            {
                for (int j = 0; j < f_mass.GetLength(1); j++)
                {
                    d_mass[i, j] = (double)f_mass[i, j];
                }
            }
            return d_mass;
        }
        static public Point3d_GL[] transfPoints(Point3d_GL[] points, double[,] matr)
        {
            Point3d_GL[] points_trans = new Point3d_GL[points.Length];
            for (int i = 0; i < points.Length; i++)
            {
                points_trans[i] = new Point3d_GL(MatrixMultiplication(matr, points[i].ToDouble()));
            }
            return points_trans;
        }
        static public double cos(double alpha)
        {
            return Math.Cos(alpha);
        }
        static public double sin(double alpha)
        {
            return Math.Sin(alpha);
        }
        static public double[,] RotZmatr(double alpha)
        {
            return new double[,] {
                { cos(alpha), -sin(alpha), 0,0 },
                { sin(alpha), cos(alpha), 0, 0 },
                { 0, 0, 1, 0 },
                { 0, 0, 0, 1 } };
        }
        static public double[,] MatrixMultiplication(double[,] matrixA, Point3d_GL p)
        {
            double[,] matB = new double[,] { { p.x }, { p.y }, { p.z }, { 1 } };
            return MatrixMultiplication(matrixA, matB);
        }
        static public double[,] MatrixMultiplication(double[,] matrixA, double[,] matrixB)
        {
            if (matrixA.GetLength(1) != matrixB.GetLength(0))
            {
                return null;
            }

            var matrixC = new double[matrixA.GetLength(0), matrixB.GetLength(1)];


            for (var i = 0; i < matrixA.GetLength(0); i++)
            {
                for (var j = 0; j < matrixB.GetLength(1); j++)
                {

                    matrixC[i, j] = 0;
                    for (var k = 0; k < matrixB.GetLength(0); k++)
                    {

                        matrixC[i, j] += matrixA[i, k] * matrixB[k, j];
                    }
                }
            }
            return matrixC;
        }
        /// <summary>
        /// Matrix basis1->basis2
        /// </summary>
        /// <param name="basis1"></param>
        /// <param name="basis2"></param>
        /// <returns></returns>


        static public Matrix<double> calcTransformMatr_cv(Point3d_GL[] basis1, Point3d_GL[] basis2)
        {
            if (basis1.Length != basis2.Length || basis1.Length<4 || basis2.Length < 4)
            {
                return null;
            }
            var basis1d = new double[4, 4];
            var basis2d = new double[4, 4];
            for (int i = 0; i < 4; i++)
            {
                basis1d[i, 0] = basis1[i].x; basis1d[i, 1] = basis1[i].y; basis1d[i, 2] = basis1[i].z; basis1d[i, 3] = 1;
                basis2d[i, 0] = basis2[i].x; basis2d[i, 1] = basis2[i].y; basis2d[i, 2] = basis2[i].z; basis2d[i, 3] = 1;
            }

            var m1 = new Matrix<double>(basis1d);
            var m2 = new Matrix<double>(basis2d);
            var m1_inv = m1.Clone();
            CvInvoke.Invert(m1, m1_inv, Emgu.CV.CvEnum.DecompMethod.LU);
            prin.t("m1 * m1_inv");
            prin.t(m1 * m1_inv);
            var m12 = m1_inv * m2;
            var m12t = m12.Transpose();
            return m12t;
        }
        static public Mat doubleToMat(double[][] inp, Size size)
        {
            Image<Gray, Byte> im_gray = new Image<Gray, byte>(size);
            for (int x = 0; x < inp.Length; x++)
            {
                //Console.WriteLine("n = "+inp[x][0] + "od = " + inp[x][1]);
                int y = (int)inp[x][1];
                if (y < 0)
                {
                    y = 0;
                }
                if (y >= size.Height)
                {
                    y = size.Height - 1;
                }
                im_gray.Data[y, x, 0] = 255;
            }
            return im_gray.Mat;
        }
        static public PointF[] doubleToPointF(double[][] inp)
        {
            var points = new List<PointF>();
            for (int i = 0; i < inp.Length; i++)
            {
                points.Add(new PointF((float)inp[i][0], (float)inp[i][1]));
            }
            return points.ToArray();
        }

        static public PointF[] doubleToPointF_real(double[][] inp)
        {
            var points = new List<PointF>();
            for (int i = 0; i < inp.Length; i++)
            {
                if (inp[i][2] > 0)
                {
                    points.Add(new PointF((float)inp[i][0], (float)inp[i][1]));
                }

            }
            return points.ToArray();
        }



        static public PointF[] moveToCentr(PointF[] points, Size size)
        {
            var points1 = new PointF[points.Length];
            for (int i = 0; i < points.Length; i++)
            {
                points1[i] = new PointF(points[i].X - size.Width / 2,
                                        points[i].Y - size.Height / 2);
                points1[i].Y *= -1;
            }
            return points1;
        }


        static public Matrix<double> eye_matr(int s)
        {
            var matr = new Matrix<double>(s, s);
            for(int i=0; i<s; i++)
            {
                for (int j = 0; j < s; j++)
                {
                    if(i == j) matr[i, j] = 1;
                    else matr[i, j] = 0;
                }
            }
            return matr;
        }

        static public Matrix<double> zero_matr(int s)
        {
            var matr = new Matrix<double>(s, s);
            for (int i = 0; i < s; i++)
            {
                for (int j = 0; j < s; j++)
                {
                     matr[i, j] = 0;
                }
            }
            return matr;
        }
    }
}
