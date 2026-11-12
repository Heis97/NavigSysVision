using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace opengl3
{
    
    public class StereoCamera
    {
        //base = world, flange, scanner = cam1, model = local cord
        public enum mode {model,world,camera,world_virt };
        public CameraCV[] cameraCVs;
        public Matrix<double> R;//1 * R -> 2
        public Matrix<double> R_inv;//1 * R -> 2

        public Matrix<double> R2;//1 * R -> 3
        public Matrix<double> R2_inv;//1 * R -> 3


        public Matrix<double> Bfs;//Flange->scaner

        public Matrix<double> Bfs_r;//Flange->scaner

        public Matrix<double> Bbf;//Base->Flange

        public Matrix<double> BS;//Flange->scaner
        public mode scan_coord_sys;

        public StereoCamera(CameraCV[] _cameraCVs,string bfs_file = null)
        {
            cameraCVs = _cameraCVs;  
            if(bfs_file != null)
            {
                /*string file;
                using (StreamReader sr = new StreamReader(bfs_file))
                {
                    file = sr.ReadToEnd();
                }
                string[] lines = file.Split(new char[] { '\n' });
                Bfs = Settings_loader.matrix_load(lines[0]);*/
                Bfs = (Matrix<double>)Settings_loader.load_data(bfs_file)[0];
            }

           
        }
        public static Line3d_GL[] computeTracesCam(PointF[] points_im, CameraCV cameraCV, Matrix<double> matrix = null)
        {
            var lines3d = new Line3d_GL[points_im.Length];
            for (int i = 0; i < lines3d.Length; i++)
            {
                if (matrix == null)
                {
                    lines3d[i] = new Line3d_GL(
                                        cameraCV.point3DfromCam(points_im[i]),
                                        new Point3d_GL(0, 0, 0));
                }
                else
                {
                    lines3d[i] = new Line3d_GL(
                                         matrix * cameraCV.point3DfromCam(points_im[i]),
                                         matrix * new Point3d_GL(0, 0, 0));
                }

            }
            return lines3d;
        }

        public static Line3d_GL[] computeTracesCam(Point3d_GL[] points_im, Matrix<double> matrix = null)
        {
            var lines3d = new Line3d_GL[points_im.Length];
            for (int i = 0; i < lines3d.Length; i++)
            {
                if (matrix != null)
                {
                    lines3d[i] = new Line3d_GL(
                   matrix * points_im[i],
                   matrix * new Point3d_GL(0, 0, 0));
                }
                else
                {
                    lines3d[i] = new Line3d_GL(
                    points_im[i],
                    new Point3d_GL(0, 0, 0));
                }

            }
            return lines3d;
        }

        public Point3d_GL[] comp_points_3d(System.Drawing.PointF[] ps1, System.Drawing.PointF[] ps2,int ind_main=0)
        {
            if (ps1 == null || ps2 == null) return null;

            if (ps1.Length != ps2.Length) return null;
            var lines1 = computeTracesCam(PointF.toPointF(ps1), cameraCVs[0],cameraCVs[0].matrixCS );
            var lines2 = computeTracesCam(PointF.toPointF(ps2), cameraCVs[1], cameraCVs[0].matrixCS*R);

            var ps_comp = new Point3d_GL[lines1.Length];
            for(int i = 0;i<lines1.Length; i++)
            {
                ps_comp[i] = Line3d_GL.point_betw_cross_lines(lines1[i],lines2[i]);
                ps_comp[i].ind = ind_main;
                ps_comp[i].ind_sec = i;
            }
           
            return ps_comp;
        }

        public Point3d_GL[] comp_points_3d_not_matched(System.Drawing.PointF[] ps1, System.Drawing.PointF[] ps2, int ind_main = 0)
        {
            if (ps1 == null || ps2 == null) return null;

            if (ps1.Length==0 ||  ps2.Length==0) return null;

            //Console.Write("ps1.Length: " + ps1.Length + "; ps1.Length: " + ps2.Length);
            var cs0 = new Matrix<double>(new double[4, 4] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } });
            var lines1 = computeTracesCam(PointF.toPointF(ps1), cameraCVs[0], cs0);
            var lines2 = computeTracesCam(PointF.toPointF(ps2), cameraCVs[1], cs0 *R);

            var ps_comp = new List< Point3d_GL>();
            for (int i = 0; i < lines1.Length; i++)
            {
                for (int j = 0; j < lines2.Length; j++)
                {
                    var p3d = Line3d_GL.point_betw_cross_lines(lines1[i], lines2[j],1);
                    if(p3d.exist)
                    {
                        ps_comp.Add(p3d);
                    }
                }
            }
            //Console.WriteLine("; ps_comp.Length: " + ps_comp.Count);

            //Console.WriteLine("");
            return ps_comp.ToArray();
        }
        public Point3d_GL[] comp_points_3d_3cam(System.Drawing.PointF[] ps1, System.Drawing.PointF[] ps2, System.Drawing.PointF[] ps3, int ind_main = 0)
        {
            if (ps1 == null || ps2 == null || ps3 == null) return null;

            if (ps1.Length != ps2.Length && ps1.Length != ps3.Length) return null;
            var lines1 =computeTracesCam(PointF.toPointF(ps1), cameraCVs[0] );//cameraCVs[0].matrixCS
            var lines2 = computeTracesCam(PointF.toPointF(ps2), cameraCVs[1], R);
            var lines3 = computeTracesCam(PointF.toPointF(ps3), cameraCVs[2],  R2);

            var ps_comp = new Point3d_GL[lines1.Length];
            for (int i = 0; i < lines1.Length; i++)
            {
                ps_comp[i] = Line3d_GL.point_betw_cross_lines(lines1[i], lines2[i], lines3[i]);// Line3d_GL.point_betw_cross_lines(lines1[i], lines2[i], lines3[i]);
                ps_comp[i].ind = ind_main;
                ps_comp[i].ind_sec = i;
            }

            return ps_comp;
        }

        public void calibrate(Mat[] mats,PatternType patternType,System.Drawing.Size pattern_size, float marksize)
        {
            if(mats.Length == cameraCVs.Length)
            {
               
                bool comp_pos = true;
                for(int i = 0; i < mats.Length; i++)
                {
                    comp_pos &= cameraCVs[i].compPos(mats[i], patternType, pattern_size,marksize);
                    //Console.WriteLine(comp_pos);
                }
                if(mats.Length==2 && comp_pos)
                {
                    var inv_cs1 = new Matrix<double>(4,4);
                    CvInvoke.Invert(cameraCVs[0].matrixCS, inv_cs1, DecompMethod.LU);
                    R = inv_cs1 * cameraCVs[1].matrixCS;
                    /*prin.t("stereo calib R_cs:");
                    prin.t(R);
                    R[0, 0] = 0.572; R[2, 2] = 0.572;
                    R[0, 3] = 114.2;
                    R[1, 3] = 4.8;
                    R[2, 3] = 61.8;*/

                    // Console.WriteLine(R[0, 3] + " " + R[1, 3] + " " + R[2, 3] + " " + R[0, 0] + " " + R[0, 1] + " " + R[0, 2]);

                    inv_cs1 = new Matrix<double>(4, 4);
                    CvInvoke.Invert(cameraCVs[1].matrixSC, inv_cs1, DecompMethod.LU);
                    R = cameraCVs[0].matrixSC* inv_cs1  ;

                    R_inv = R.Clone() ;
                    CvInvoke.Invert(R_inv,R_inv, DecompMethod.LU);
                    Settings_loader.save_file("stereo_cal.txt", new object[] { R });
                    prin.t("stereo calib R_sc:");
                    prin.t(R);
                }

                if (mats.Length ==3 && comp_pos)
                {
                    var inv_cs1 = new Matrix<double>(4, 4);
                    CvInvoke.Invert(cameraCVs[1].matrixSC, inv_cs1, DecompMethod.LU);
                    R = cameraCVs[0].matrixSC * inv_cs1;
                    R_inv = R.Clone();
                    CvInvoke.Invert(R_inv, R_inv, DecompMethod.LU);

                    var inv_cs2 = new Matrix<double>(4, 4);
                    CvInvoke.Invert(cameraCVs[2].matrixSC, inv_cs2, DecompMethod.LU);
                    R2 = cameraCVs[0].matrixSC * inv_cs2;
                    R2_inv = R2.Clone();
                    CvInvoke.Invert(R2_inv, R2_inv, DecompMethod.LU);



                    Settings_loader.save_file("stereo_cal3.txt", new object[] { R,R2 });
                    prin.t("stereo calib R_sc:");
                    prin.t(R);
                    prin.t("stereo calib R2_sc:");
                    prin.t(R2);
                }
            }
        }
        public void calibrate_stereo(Frame[] frames, PatternType patternType, System.Drawing.Size pattern_size, float mark_size)
        {
            
            if (cameraCVs.Length==2)
            {
                for (int i = 0; i < frames.Length; i++)
                {
                    var pos1 = cameraCVs[0].compPos(frames[i].im, patternType, pattern_size, mark_size);
                    var pos2 = cameraCVs[1].compPos(frames[i].im_sec, patternType, pattern_size, mark_size);
                    if(pos1&&pos2)
                    {
                        var inv_cs1 = new Matrix<double>(4, 4);
                        CvInvoke.Invert(cameraCVs[0].matrixCS, inv_cs1, DecompMethod.LU);

                        R = inv_cs1 * cameraCVs[1].matrixCS;
                        R_inv = R.Clone();
                        CvInvoke.Invert(R_inv, R_inv, DecompMethod.LU);
                        var c1 = cameraCVs[0].matrixCS;
                        Console.WriteLine(i + " " + Math.Round( R[0, 3],3) + " " + Math.Round(R[1, 3], 3) + " " + Math.Round(R[2, 3], 3) + " " + " " + R[0, 2] + " " + R[1, 2] + " " + R[2, 2] + " "//
                        + c1[0, 3] + " " + c1[1, 3] + " " + c1[2, 3] + " " + c1[2, 0] + " " + c1[2, 1] + " " + c1[2, 2]
                        + " reproj: " + Math.Round(cameraCVs[0].last_reproj_err, 3) + " " + Math.Round(cameraCVs[1].last_reproj_err, 3));
                        GC.Collect();

                        
                    }

                   // Console.WriteLine(comp_pos);
                }
            }
        }

       
        

       


        static List<Matrix<double>> get_matrices_filter(List<Matrix<double>> matrices,int k)
        {
            var ms_filt = new List<Matrix<double>>();
            for (int i = 0; i < matrices.Count; i+=k)
            {
                ms_filt.Add(matrices[i]);
            }
            return ms_filt;
        }

        

        

        


        /// <summary>
        /// For make housing
        /// </summary>
        /// <param name="alpha">fov x, degree</param>
        /// <param name="beta">xy Norm ^ camera Norm ,degree</param>
        /// <param name="L">size of scan, mm</param>
        static public void calcSizesScanner(double alpha, double beta, double L)
        {
            var gamma = (180 - alpha) / 2;
            var tetta = 180 - beta - gamma;
            var m1 = (L * sin(tetta) )/ (2 * sin(gamma));
            var m2 = m1 * tg(beta);
            var k1 = m1 / cos(beta);
            var k2 = m1 * (tg(gamma) - tg(beta));
            var k = (tg(gamma) - tg(beta)) * cos(beta);
            var y = k * m1;
            var x = k * m2 + k1 - L / 2;
            Console.WriteLine("X: "+x + "; Y: " + y);
        }
        static public void calcSizesScanner_v3(double alpha, double beta, double L)//alpha - fov, beta - cam_z^surf_n
        {
            var gamma = (180 - alpha) / 2;
            var tetta = 180 - beta - gamma;
            var omega = 90 - beta;
            var m1 = (L * sin(tetta)) / (2 * sin(gamma));
            var m2 = m1 * tg(beta);
            var k1 = m1 / cos(beta);
            var k2 = m1 * (tg(gamma) - tg(beta));
            // var k = (tg(gamma) - tg(beta)) * cos(beta);
            var y = k2 * cos(omega) - L / 2 + k1;

            var z = k2 * sin(omega);
            Console.WriteLine("X: " + y + "; Y: " + z);
        }
        public static double calcFov(double size, double f)
        {
            return 2 * arctg(size / (2 * f));
        }
        static double toRad(double degree)
        {
            return degree * Math.PI / 180;
        }
        static double toDegree(double rad)
        {
            return rad * 180 / Math.PI  ;
        }
        static double sin(double degree)
        {
            return Math.Sin(toRad(degree));
        }
        static double arctg(double val)
        {
            return  toDegree( Math.Atan(val));
        }
        static double cos(double degree)
        {
            return Math.Cos(toRad(degree));
        }

        static double tg(double degree)
        {
            return Math.Tan(toRad(degree));
        }

    }   
}
