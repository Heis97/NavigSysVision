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

        public Point3d_GL[] comp_points_3d_not_matched(System.Drawing.PointF[] ps1, System.Drawing.PointF[] ps2)
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
                    var p3d = Line3d_GL.point_betw_cross_lines(lines1[i], lines2[j],0.7);
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
        /// <summary>
        /// true: при стереокалибровке внутренние параметры обеих камер уточняются совместно по всем кадрам;
        /// false: внутренние параметры остаются как загружены, ищется только взаимное положение камер
        /// </summary>
        public static bool refine_intrinsics = true;

        public void calibrate_stereo(Frame[] frames, PatternType patternType, System.Drawing.Size pattern_size, float mark_size)
        {
            
            if (cameraCVs.Length==2)
            {
                var objps = new List<MCvPoint3D32f[]>();
                var ps1 = new List<System.Drawing.PointF[]>();
                var ps2 = new List<System.Drawing.PointF[]>();
                var frame_inds = new List<int>();
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

                        // точки этого кадра для совместной калибровки по всем кадрам
                        objps.Add(cameraCVs[0].last_obp);
                        ps1.Add(cameraCVs[0].last_corners);
                        ps2.Add(cameraCVs[1].last_corners);
                        frame_inds.Add(i);
                        GC.Collect();

                        
                    }

                   // Console.WriteLine(comp_pos);
                }

                // Покадровые R выше - только для контроля: одна плоская мишень плохо разделяет поворот и сдвиг,
                // поэтому они "гуляют". Итоговое R считается одно на все кадры.
                if (objps.Count >= 3)
                {
                    calibrate_stereo_joint(objps.ToArray(), ps1.ToArray(), ps2.ToArray());
                    check_stereo_distances(objps.ToArray(), ps1.ToArray(), ps2.ToArray(), frame_inds.ToArray(), pattern_size);
                }
                else
                {
                    Console.WriteLine("stereo joint: мало кадров с найденным шаблоном (" + objps.Count + "), R взято из последнего кадра");
                }
            }
        }

        static double dist3(MCvPoint3D32f a, MCvPoint3D32f b)
        {
            double dx = a.X - b.X, dy = a.Y - b.Y, dz = a.Z - b.Z;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        /// <summary>
        /// Проверка стереокалибровки по известным размерам шаблона: точки каждого кадра восстанавливаются в 3d
        /// (триангуляция по итоговому R) и измеренные расстояния сравниваются с номинальными.
        /// Выводятся: две стороны сетки (катеты), диагональ, СКО ошибки шага между соседними точками
        /// и средний зазор между скрещивающимися лучами двух камер.
        /// </summary>
        void check_stereo_distances(MCvPoint3D32f[][] objps, System.Drawing.PointF[][] ps1, System.Drawing.PointF[][] ps2, int[] frame_inds, System.Drawing.Size pattern_size)
        {
            if (R == null) return;
            int w = pattern_size.Width;
            int h = pattern_size.Height;
            int len = w * h;
            if (w < 2 || h < 2) return;
            var cs0 = new Matrix<double>(new double[4, 4] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } });
            // пары точек: сторона вдоль Width, сторона вдоль Height, диагональ
            var pairs = new int[][] { new int[] { 0, w - 1 }, new int[] { 0, len - w }, new int[] { 0, len - 1 } };
            var names = new string[] { "side_w", "side_h", "diag" };
            var nominal = new double[3];
            var sum_err = new double[3];
            var sum_sq = new double[3];
            var max_err = new double[3];
            double step_sq_all = 0;
            int step_n_all = 0;
            double gap_all = 0;
            int n_fr = 0;

            Console.WriteLine("stereo check, mm: frame | side_w (err) | side_h (err) | diag (err) | step rms err | ray gap");
            for (int f = 0; f < objps.Length; f++)
            {
                if (objps[f] == null || ps1[f] == null || ps2[f] == null) continue;
                if (objps[f].Length != len || ps1[f].Length != len || ps2[f].Length != len) continue;

                var lines1 = computeTracesCam(PointF.toPointF(ps1[f]), cameraCVs[0], cs0);
                var lines2 = computeTracesCam(PointF.toPointF(ps2[f]), cameraCVs[1], cs0 * R);
                var p3 = new Point3d_GL[len];
                double gap = 0;
                for (int k = 0; k < len; k++)
                {
                    p3[k] = Line3d_GL.point_betw_cross_lines(lines1[k], lines2[k]);
                    gap += Math.Abs(Line3d_GL.dist_cross_lines(lines1[k], lines2[k]));
                }
                gap /= len;

                var line = frame_inds[f].ToString();
                for (int d = 0; d < 3; d++)
                {
                    int a = pairs[d][0], b = pairs[d][1];
                    double nom = dist3(objps[f][a], objps[f][b]);
                    double meas = (p3[a] - p3[b]).magnitude();
                    double err = meas - nom;
                    nominal[d] = nom;
                    sum_err[d] += err;
                    sum_sq[d] += err * err;
                    if (Math.Abs(err) > Math.Abs(max_err[d])) max_err[d] = err;
                    line += " | " + Math.Round(meas, 3) + " (" + Math.Round(err, 3) + ")";
                }

                // шаг между соседними точками сетки (по строкам и по столбцам)
                double step_sq = 0;
                int step_n = 0;
                for (int j = 0; j < h; j++)
                {
                    for (int i = 0; i < w; i++)
                    {
                        int k = j * w + i;
                        if (i + 1 < w)
                        {
                            double e = (p3[k] - p3[k + 1]).magnitude() - dist3(objps[f][k], objps[f][k + 1]);
                            step_sq += e * e; step_n++;
                        }
                        if (j + 1 < h)
                        {
                            double e = (p3[k] - p3[k + w]).magnitude() - dist3(objps[f][k], objps[f][k + w]);
                            step_sq += e * e; step_n++;
                        }
                    }
                }
                step_sq_all += step_sq;
                step_n_all += step_n;
                gap_all += gap;
                n_fr++;
                line += " | " + Math.Round(Math.Sqrt(step_sq / step_n), 3) + " | " + Math.Round(gap, 3);
                Console.WriteLine(line);
            }
            if (n_fr == 0) return;

            Console.WriteLine("stereo check summary, frames " + n_fr + ":");
            for (int d = 0; d < 3; d++)
            {
                Console.WriteLine("  " + names[d] + ": nominal " + Math.Round(nominal[d], 3)
                    + "; mean err " + Math.Round(sum_err[d] / n_fr, 3)
                    + "; rms err " + Math.Round(Math.Sqrt(sum_sq[d] / n_fr), 3)
                    + "; max err " + Math.Round(max_err[d], 3));
            }
            Console.WriteLine("  step: rms err " + Math.Round(Math.Sqrt(step_sq_all / step_n_all), 3)
                + "; ray gap mean " + Math.Round(gap_all / n_fr, 3));
        }

        /// <summary>
        /// Совместная стереокалибровка по всем кадрам: одно взаимное положение камер,
        /// минимизирующее ошибку репроекции в обеих камерах сразу.
        /// </summary>
        void calibrate_stereo_joint(MCvPoint3D32f[][] objps, System.Drawing.PointF[][] ps1, System.Drawing.PointF[][] ps2)
        {
            var k1 = cameraCVs[0].cameramatrix.Clone();
            var d1 = cameraCVs[0].distortmatrix.Clone();
            var k2 = cameraCVs[1].cameramatrix.Clone();
            var d2 = cameraCVs[1].distortmatrix.Clone();
            var r = new Mat();
            var t = new Mat();
            var e = new Mat();
            var f = new Mat();
            var flags = refine_intrinsics ? CalibType.UseIntrinsicGuess : CalibType.FixIntrinsic;

            double err = CvInvoke.StereoCalibrate(objps, ps1, ps2, k1, d1, k2, d2, cameraCVs[0].image_size,
                r, t, e, f, flags, new MCvTermCriteria(200, 1e-7));

            // OpenCV: X2 = r * X1 + t (камера 1 -> камера 2); в проекте R - камера 2 -> камера 1
            var r_data = (double[,])r.GetData();
            var t_data = (double[,])t.GetData();
            var m12 = new Matrix<double>(new double[4, 4]
            {
                { r_data[0, 0], r_data[0, 1], r_data[0, 2], t_data[0, 0] },
                { r_data[1, 0], r_data[1, 1], r_data[1, 2], t_data[1, 0] },
                { r_data[2, 0], r_data[2, 1], r_data[2, 2], t_data[2, 0] },
                { 0, 0, 0, 1 }
            });
            R = new Matrix<double>(4, 4);
            CvInvoke.Invert(m12, R, DecompMethod.LU);
            R_inv = m12;

            if (refine_intrinsics)
            {
                cameraCVs[0].set_intrinsics(k1, d1);
                cameraCVs[1].set_intrinsics(k2, d2);
            }

            double baseline = Math.Sqrt(R[0, 3] * R[0, 3] + R[1, 3] * R[1, 3] + R[2, 3] * R[2, 3]);
            Console.WriteLine("stereo joint: frames " + objps.Length + "; reproj rms " + Math.Round(err, 4)
                + "; t " + Math.Round(R[0, 3], 3) + " " + Math.Round(R[1, 3], 3) + " " + Math.Round(R[2, 3], 3)
                + "; baseline " + Math.Round(baseline, 3));
            prin.t("stereo joint R:");
            prin.t(R);
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
