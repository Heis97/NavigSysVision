using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using Emgu.CV;
using Emgu.CV.Aruco;
using System.Drawing;
using System.ComponentModel;
using System.Drawing.Design;

using System.Text.Json;


using Emgu.CV.Dnn;
using Accord.Math;
using System.Reflection;

namespace opengl3
{



    public class NavigMarker
    {
        public int ind = -1;
        public double marker_size_mm = -1;
        public double near_dist = 1;
        public double far_dist = 1;
        public int size_circle = 1;
        public Point3d_GL[] ps_near;
        public Point3d_GL[] ps_far;
        
        static public  NavigMarker get_marker_p1v2(int ind)
        {
            var marker = new NavigMarker();

            marker.ind = ind;
            marker.marker_size_mm = 10.8;
            marker.near_dist = 13.6;
            marker.far_dist = 100;
            marker.size_circle = 50;

            return marker;
        }

    }


    public class NavigTool
    {
        public NavigMarker marker;
        public enum ToolType { tp4_v1, tp1_v1 };
        public enum ToolFuncType { Registr, Model , Other};
        public Point3d_GL[] ps;
        public List<Point3d_GL> trace_tcp = new List<Point3d_GL>();
        int[] inds;
        int[] ind_aruco;
        int aruco_corner_num=4;
        int aruco_number = 0;
        public ToolFuncType toolfunc = ToolFuncType.Other;

        Point3d_GL tcp;
        Point3d_GL rotate;

        public Matrix<double> matrix_frame = UtilMatr.eye_matr(4);
        public Matrix<double> matrix_frame_inv;
        public Matrix<double> matrix_model;
        public Matrix<double> matrix_model_inv;

        public Matrix<double> matrix_model_debug = UtilMatr.eye_matr(4);

        public string name_3d_model = "new_tool";
        public string path_3d_model = null;
        public string name_3d_model_debug = "new_debug_tool";
        public string path_3d_model_debug = null;
        public string name_3d_model_trace_tcp = "new_trace_tcp";
        
        public Matrix<double> matrix_tcp = UtilMatr.eye_matr(4);
        public ToolType tool_type;
        public List<List<Point3d_GL>> ps_for_registr = new List<List<Point3d_GL>>();

        /*
         * x down
         * y right 
         * 
         * y-> 
         * p1----------p0
         * 
         * x
         * ||    
         * \/
         * 
         * p2-----------p3

         * */

        public NavigTool(int[] _inds, ToolType _tool_type,string _name_3d_model=null, string _path_3d_model = null, Matrix<double> _matrix_model_debug = null, string _path_3d_model_debug = null) 
        {
            if (_inds == null) { Console.WriteLine("NavigTool _inds == null"); return; }
            aruco_number = _inds.Length;
            inds = new int[aruco_number];
            for (int i =0; i<_inds.Length;i++)
            {
                inds[i] = _inds[i];
            }
            tool_type = _tool_type;
            if (_tool_type == ToolType.tp4_v1)
            {
                if (aruco_number != 4)  Console.WriteLine("NavigTool inds.Length != 4 for ToolType.tp4_v1");
            }
            if (_tool_type == ToolType.tp1_v1)
            {
                if (aruco_number != 1) Console.WriteLine("NavigTool inds.Length != 1 for ToolType.tp1_v1");
            }
            if (_name_3d_model != null)
            {
                name_3d_model = _name_3d_model;
            }
            if (_path_3d_model != null)
            {
                path_3d_model = _path_3d_model;
            }
            if (_matrix_model_debug != null && _path_3d_model_debug != null)
            {
                matrix_model_debug = _matrix_model_debug;
                path_3d_model_debug = _path_3d_model_debug;
            }
            ind_aruco = comp_ind_table(inds);
        }
        public NavigTool(NavigMarker nmarker, ToolType _tool_type, string _name_3d_model = null, string _path_3d_model = null, Matrix<double> _matrix_model_debug = null, string _path_3d_model_debug = null)
        {
            if (nmarker == null) { Console.WriteLine("nmarker == null"); return; }

            marker = nmarker;
            aruco_number = 1;
            inds = new int[1];
            inds[0] = nmarker.ind;
            tool_type = _tool_type;
            if (_tool_type == ToolType.tp4_v1)
            {
                if (aruco_number != 4) Console.WriteLine("NavigTool inds.Length != 4 for ToolType.tp4_v1");
            }
            if (_tool_type == ToolType.tp1_v1)
            {
                if (aruco_number != 1) Console.WriteLine("NavigTool inds.Length != 1 for ToolType.tp1_v1");
            }
            if (_name_3d_model != null)
            {
                name_3d_model = _name_3d_model;
            }
            if (_path_3d_model != null)
            {
                path_3d_model = _path_3d_model;
            }
            if (_matrix_model_debug != null && _path_3d_model_debug != null)
            {
                matrix_model_debug = _matrix_model_debug;
                path_3d_model_debug = _path_3d_model_debug;
            }
            ind_aruco = comp_ind_table(inds);
        }
        static int[] comp_ind_table(int[] inds)
        {
            var max_ind = int.MinValue;
            for (int i = 0; i < inds.Length; i++)
            {
                max_ind = Math.Max(max_ind, inds[i]);
            }
            var table = new int [max_ind+1];
            for (int i = 0; i < inds.Length; i++)
            {
                table[inds[i]] = i;
            }
            return table;
        }
        public double calibrate_markers(Point3d_GL[][][] ps_cal)//detect markerspos relative p1  //ps not filterd
        {
            var ps = filter_ps_array(ps_cal);
            if(ps == null ) return -1;
            if(ps.Length <4 ) return -1;


            return 0;
        }
        public Point3d_GL calibrate_tcp_4p(Point3d_GL[][][] ps_cal)//ps filterd
        {
            var ps = ps_cal;
            if (ps == null) return Point3d_GL.notExistP();
            if (ps.Length < 5) return Point3d_GL.notExistP();
            var ms = new List<Matrix<double>>();
            var ps_calc = new List<Point3d_GL>();
            for (int i = 0; i < 5; i++)
            {
                var m = get_frame(ps[i]);
                ms.Add(m);
                ps_calc.Add(new Point3d_GL(m[0, 3], m[1, 3], m[2, 3]));
            }


            var p_cenr = find_center_sphere_4p(ps_calc)[0];

            var p_cenr_2 = find_center_sphere_4p(ps_calc.GetRange(1,4))[0];


            //Console.WriteLine("p_cenr: " + p_cenr);
            //Console.WriteLine("p_cenr2: " + p_cenr_2);

            var tcp_aver = new Matrix<double>(new double[,] {
                {0,0,0,0 },
                {0,0,0,0 },
                {0,0,0,0 },
                {0,0,0,0 }});
            int ps_count = 5;

            var tcp_list = new List<Matrix<double>>();
            for (int i = 0; i < ps_count; i++)
            {
                var m_inv = ms[i].Clone();

                CvInvoke.Invert(m_inv, m_inv, DecompMethod.Svd);

                var m_test = m_inv * ms[i];
                var tcp = new Matrix<double>(new double[,] {
                {1,0,0,p_cenr.x },
                {0,1,0,p_cenr.y },
                {0,0,1,p_cenr.z },
                {0,0,0,1 }});
                var m_tcp = m_inv * tcp;
                tcp_list.Add(m_tcp);
                tcp_aver += m_tcp;// m_inv* tcp
                //Console.WriteLine("m_inv * tcp");
                //prin.t(m_inv * tcp);

            }
            tcp_aver = tcp_aver / ps_count;
            var p_err = new Point3d_GL();

            for (int i = 0;i < ps_count;i++)
            {
                var p_i = new Point3d_GL(
                    Math.Abs(tcp_aver[0, 3] - tcp_list[i][0, 3]),
                    Math.Abs(tcp_aver[1, 3] - tcp_list[i][1, 3]),
                    Math.Abs(tcp_aver[2, 3] - tcp_list[i][2, 3]));
                p_err += p_i;
            }


            Console.WriteLine("Navig_calib p_err x y z: " + p_err/ps_count);



            this.matrix_tcp = tcp_aver;
            //Console.WriteLine("tcp_aver");
            //prin.t(tcp_aver);

            this.tcp = new Point3d_GL(tcp_aver[0, 3], tcp_aver[1, 3], tcp_aver[2, 3]);

            return this.tcp;
        }
        public Point3d_GL calibrate_tool_tcp_4p(Point3d_GL[][][] ps_cal)//ps filterd
        {
            var ps = ps_cal;
            if (ps == null) return Point3d_GL.notExistP();
            if (ps.Length < 5) return Point3d_GL.notExistP();
            var ms = new List<Matrix<double>>();
            var ps_calc = new List<Point3d_GL>();
            for (int i = 0; i < 5; i++)
            {
                var m = get_frame(ps[i]);
                if (m == null) continue;
                ms.Add(m);
                ps_calc.Add(new Point3d_GL(m[0, 3], m[1, 3], m[2, 3]));
            }


            var p_cenr = find_center_sphere_4p(ps_calc)[0];

            var p_cenr_2 = find_center_sphere_4p(ps_calc.GetRange(1, 4))[0];


            //Console.WriteLine("p_cenr: " + p_cenr);
            //Console.WriteLine("p_cenr2: " + p_cenr_2);

            var tcp_aver = new Matrix<double>(new double[,] {
                {0,0,0,0 },
                {0,0,0,0 },
                {0,0,0,0 },
                {0,0,0,0 }});
            int ps_count = 5;

            var tcp_list = new List<Matrix<double>>();
            for (int i = 0; i < ps_count; i++)
            {
                var m_inv = ms[i].Clone();

                CvInvoke.Invert(m_inv, m_inv, DecompMethod.LU);

                var m_test = m_inv * ms[i];
                var tcp = new Matrix<double>(new double[,] {
                {1,0,0,p_cenr.x },
                {0,1,0,p_cenr.y },
                {0,0,1,p_cenr.z },
                {0,0,0,1 }});
                var m_tcp = m_inv * tcp;
                tcp_list.Add(m_tcp);
                tcp_aver += m_tcp;// m_inv* tcp
                //Console.WriteLine("m_inv * tcp");
                //prin.t(m_inv * tcp);

            }
            tcp_aver = tcp_aver / ps_count;
            var p_err = new Point3d_GL();

            for (int i = 0; i < ps_count; i++)
            {
                var p_i = new Point3d_GL(
                    Math.Abs(tcp_aver[0, 3] - tcp_list[i][0, 3]),
                    Math.Abs(tcp_aver[1, 3] - tcp_list[i][1, 3]),
                    Math.Abs(tcp_aver[2, 3] - tcp_list[i][2, 3]));
                p_err += p_i;
            }


            Console.WriteLine("Navig_calib p_err x y z: " + p_err / ps_count);



            this.matrix_tcp = tcp_aver;
            //Console.WriteLine("tcp_aver");
            //prin.t(tcp_aver);
            this.tcp = new Point3d_GL(tcp_aver[0, 3], tcp_aver[1, 3], tcp_aver[2, 3]);
            this.matrix_tcp = new Matrix<double>(new double[,] {
                {1,0,0,tcp.x },
                {0,1,0,tcp.y },
                {0,0,1,tcp.z },
                {0,0,0,1 }});
            this.tcp = new Point3d_GL(tcp_aver[0, 3], tcp_aver[1, 3], tcp_aver[2, 3]);

            return this.tcp;
        }
        public static Point3d_GL[] find_center_sphere_4p(List<Point3d_GL> points)
        {
            if (points == null || points.Count < 4)
                throw new ArgumentException("Необходимо минимум 4 точки.");

            double x1 = points[0].x, y1 = points[0].y, z1 = points[0].z;
            double x2 = points[1].x, y2 = points[1].y, z2 = points[1].z;
            double x3 = points[2].x, y3 = points[2].y, z3 = points[2].z;
            double x4 = points[3].x, y4 = points[3].y, z4 = points[3].z;

            // Вспомогательные величины U, V, W
            double U = (z1 - z2) * (x3 * y4 - x4 * y3) - (z2 - z3) * (x4 * y1 - x1 * y4);
            double V = (z3 - z4) * (x1 * y2 - x2 * y1) - (z4 - z1) * (x2 * y3 - x3 * y2);
            double W = (z1 - z3) * (x4 * y2 - x2 * y4) - (z2 - z4) * (x1 * y3 - x3 * y1);

            double denominator = U + V + W;
            if (Math.Abs(denominator) < 1e-12)
                throw new InvalidOperationException("Точки компланарны или сфера не определена.");

            // Вычисление компонент центра (Ax, Bx, Cx, Dx) и т.д.
            double squared1 = x1 * x1 + y1 * y1 + z1 * z1;
            double squared2 = x2 * x2 + y2 * y2 + z2 * z2;
            double squared3 = x3 * x3 + y3 * y3 + z3 * z3;
            double squared4 = x4 * x4 + y4 * y4 + z4 * z4;

            double Ax = squared1 * (y2 * (z3 - z4) + y3 * (z4 - z2) + y4 * (z2 - z3));
            double Bx = squared2 * (y3 * (z4 - z1) + y4 * (z1 - z3) + y1 * (z3 - z4));
            double Cx = squared3 * (y4 * (z1 - z2) + y1 * (z2 - z4) + y2 * (z4 - z1));
            double Dx = squared4 * (y1 * (z2 - z3) + y2 * (z3 - z1) + y3 * (z1 - z2));

            double Ay = squared1 * (x2 * (z3 - z4) + x3 * (z4 - z2) + x4 * (z2 - z3));
            double By = squared2 * (x3 * (z4 - z1) + x4 * (z1 - z3) + x1 * (z3 - z4));
            double Cy = squared3 * (x4 * (z1 - z2) + x1 * (z2 - z4) + x2 * (z4 - z1));
            double Dy = squared4 * (x1 * (z2 - z3) + x2 * (z3 - z1) + x3 * (z1 - z2));

            double Az = squared1 * (x2 * (y3 - y4) + x3 * (y4 - y2) + x4 * (y2 - y3));
            double Bz = squared2 * (x3 * (y4 - y1) + x4 * (y1 - y3) + x1 * (y3 - y4));
            double Cz = squared3 * (x4 * (y1 - y2) + x1 * (y2 - y4) + x2 * (y4 - y1));
            double Dz = squared4 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));

            double x0 = 0.5 * (Ax - Bx + Cx - Dx) / denominator;
            double y0 = -0.5 * (Ay - By + Cy - Dy) / denominator;
            double z0 = 0.5 * (Az - Bz + Cz - Dz) / denominator;

            double radius = Math.Sqrt((x1 - x0) * (x1 - x0) +
                                      (y1 - y0) * (y1 - y0) +
                                      (z1 - z0) * (z1 - z0));

            return new Point3d_GL[]
            {
            new Point3d_GL(x0, y0, z0),
            new Point3d_GL(radius, radius, radius)
            };
        }

        public double calibrate_rot(Point3d_GL[][][] ps_cal)//ps not filterd
        {
            var ps = filter_ps_array(ps_cal);

            return 0;
        }

        public Matrix<double> get_frame(Point3d_GL[][] ps_finded,bool filtered = false)//ps not filterd
        {
            var ps = filter_ps( ps_finded);
            if (!check_aruko_ps(ps)) { return null; } 
            if (tool_type == ToolType.tp4_v1)
            {
                return get_frame_tr4_v1(ps);
            }
            else if (tool_type == ToolType.tp1_v1)
            {
                return get_frame_tr1_v1(ps, filtered);
            }
            return null;
        }
        public Matrix<double> get_frame_tcp(Point3d_GL[][] ps_finded)//ps not filterd
        {
            var ps = filter_ps(ps_finded);
            if (!check_aruko_ps(ps)) { return null; }
            if (tool_type == ToolType.tp4_v1)
            {
                get_frame_tr4_v1(ps);
                return  matrix_model;
            }
            if (tool_type == ToolType.tp1_v1)
            {
                get_frame_tr1_v1(ps);
                return matrix_model;
            }
            return null;
        }
        /*
         * x down
         * y right 
         * 
         * x - true axis
         * y-> 
         * p1----------p0
         * 
         * x
         * ||    
         * \/
         * 
         * p2-----------p3
         * */
        public Matrix<double> get_frame_tr4_v1(Point3d_GL[][] ps_finded)//ps filterd
        {
            var p0 = Point3d_GL.centr_mass(ps_finded[0]);
            var p1 = Point3d_GL.centr_mass(ps_finded[1]);
            var p2 = Point3d_GL.centr_mass(ps_finded[2]);
            var p3 = Point3d_GL.centr_mass(ps_finded[3]);

            var vx = (p2 - p1).normalize();
            var vyx = (p0 - p1).normalize();

            var vz = (vx | vyx).normalize();
            var vy = (vz | vx).normalize();
            matrix_frame = matrix_basis_from_ps(new Point3d_GL[] { p0, p1, p2 });
            matrix_frame_inv = matrix_frame.Clone();
            CvInvoke.Invert(matrix_frame, matrix_frame_inv, DecompMethod.LU);

            if (tool_type == ToolType.tp4_v1)
            {
                matrix_model = matrix_frame * matrix_tcp ;
            }
            matrix_model_inv = matrix_model.Clone();
            CvInvoke.Invert(matrix_model, matrix_model_inv, DecompMethod.LU);

            ps = new Point3d_GL[] { p0, p1, p2, p3,new Point3d_GL(matrix_model[0,3], matrix_model[1, 3], matrix_model[2, 3]) };
            return matrix_frame;
        }
        public static Matrix<double> matrix_basis_from_ps(Point3d_GL[] ps)//p2 x; p1 o, p0 y
        {
            if (ps == null) return null;
            if (ps.Length < 3) return null;
            var p0 = ps[0];
            var p1 = ps[1];
            var p2 = ps[2];

            var vx = (p2 - p1).normalize();
            var vyx = (p0 - p1).normalize();

            var vz = (vx | vyx).normalize();
            var vy = (vz | vx).normalize();
            return matrix_assemble(vx, vy, vz, p1);
        }
        static public Matrix<double> matrix_assemble(Point3d_GL vx, Point3d_GL vy, Point3d_GL vz, Point3d_GL p)
        {
            /*  return new Matrix<double>(new double[,] {
              {vx.x,vx.y,vx.z,p.x },
              {vy.x,vy.y,vy.z,p.y },
              {vz.x,vz.y,vz.z,p.z },
              {   0,   0,   0,  1 },
              });*/

            return new Matrix<double>(new double[,] {
            {vx.x,vy.x,vz.x,p.x },
            {vx.y,vy.y,vz.y,p.y },
            {vx.z,vy.z,vz.z,p.z },
            {   0,   0,   0,  1 },
            });
        }
        int problem_counter = 0;
        int problem_max_count = 10;
        public Matrix<double> get_frame_tr1_v1(Point3d_GL[][] ps_finded,bool filtered = false)//ps filterd
        {
            var p0 = ps_finded[0][0];
            var p1 = ps_finded[0][3];
            var p2 = ps_finded[0][2];
            var p3 = ps_finded[0][1];


            /*var matrix_shift = UtilMatr.matrix_cv(new Point3d_GL(0, 0, -36.1), new Point3d_GL());

            var matrix_cur =  matrix_basis_from_ps_frame(new Point3d_GL[] { p0, p1, p3,( p1 + p3) / 2 })* matrix_shift;

           // Console.WriteLine("dist: " + (p0 - p2).magnitude() + " " + (p0 - p1).magnitude() + " " + (p1 - p2).magnitude());
            //gauss_matrix(matrix_cur,filtered);
            matrix_frame = matrix_cur;
            matrix_frame_inv = matrix_frame.Clone();
            CvInvoke.Invert(matrix_frame, matrix_frame_inv, DecompMethod.LU); 
            matrix_model = matrix_frame * matrix_tcp;
            matrix_model_inv = matrix_model.Clone();
            CvInvoke.Invert(matrix_model, matrix_model_inv, DecompMethod.LU);
           
            ps = new Point3d_GL[] { p0, p1, p2, p3,
                new Point3d_GL(matrix_model[0, 3], matrix_model[1, 3], matrix_model[2, 3]),
                new Point3d_GL(matrix_frame[0, 3], matrix_frame[1, 3], matrix_frame[2, 3]),
                new Point3d_GL((p0 - p1).magnitude(), (p0 - p2).magnitude(), (p2 - p1).magnitude())
            };

            /*var ps_ext = gen_3d_ps_other_corners(new Point3d_GL(9, 9, 0), 50, 100, matrix_frame);
            var ps_l = ps.ToList();
            ps_l.AddRange(ps_ext);
            ps = ps_l.ToArray();*/

            

            return matrix_frame;
        }
        public static Matrix<double> matrix_basis_from_ps_frame(Point3d_GL[] ps)//
        {
            if (ps == null) return null;
            if (ps.Length < 3) return null;
            var p0 = ps[0];
            var p1 = ps[1];
            var p2 = ps[2];
            var p3 = ps[3];

            var vx = (p2 - p0).normalize();
            var vyx = (p0 - p1).normalize();

            var vz = (vx | vyx).normalize();
            var vy = (vz | vx).normalize();
            return matrix_assemble(vx, vy, vz, p3);
        }
        void gauss_matrix(Matrix<double> matrix_cur,bool filtered)
        {
            if (matrix_frame != null && filtered && ps != null && ps.Length > 4)
            {
                var matrix_prev = matrix_frame.Clone();
                var p_cur = new Point3d_GL(matrix_cur[0, 3], matrix_cur[1, 3], matrix_cur[2, 3]);

                var dist_prev_cur = (p_cur - ps[5]).magnitude();
                //Console.WriteLine(dist_prev_cur);
                //Console.WriteLine(problem_counter);
                if (dist_prev_cur < 20 && problem_counter < problem_max_count)
                {
                    matrix_frame = matrix_frame + 0.5 * (matrix_cur - matrix_frame);
                    problem_counter = 0;

                }
                else
                {

                    problem_counter++;
                    if (problem_counter > problem_max_count)
                    {
                        problem_counter = 0;
                        matrix_frame = matrix_cur;

                    }

                }
            }
            else
            {
                matrix_frame = matrix_cur;
            }

        }

        static Point3d_GL[] gen_3d_ps_other_corners(Point3d_GL pc, double marker_size, double corner_lin_dist, Matrix<double> matrix_frame)
        {
            var ps_marker = new Point3d_GL[] { new Point3d_GL(0, 0, 0), new Point3d_GL(marker_size, 0, 0), new Point3d_GL(marker_size, marker_size, 0), new Point3d_GL(0, marker_size, 0) };
            var pc_marker = new Point3d_GL(marker_size/2, marker_size / 2, 0);
            var ps_corn = Point3d_GL.add_arr(ps_marker, pc - pc_marker);

            var ps_corn_x = Point3d_GL.add_arr(ps_corn, new Point3d_GL(corner_lin_dist,0,0));
            var ps_corn_y = Point3d_GL.add_arr(ps_corn, new Point3d_GL( 0, corner_lin_dist, 0));
            var ps_corn_xy = Point3d_GL.add_arr(ps_corn, new Point3d_GL(corner_lin_dist, corner_lin_dist, 0));

            var ps_l = new List<Point3d_GL>();
            ps_l.AddRange(ps_corn_x);
            ps_l.AddRange(ps_corn_y);
            ps_l.AddRange(ps_corn_xy);
            var ps_arr = Point3d_GL.multMatr( ps_l.ToArray() , matrix_frame);
            return ps_arr;
        }

        Point3d_GL[][][] filter_ps_array(Point3d_GL[][][] ps)
         {
            var ps_filtr = new List<Point3d_GL[][]>();
            for(int i = 0; i < ps.Length; i++)
            {
                var frame_ps_filtr = filter_ps(ps[i]);

                if(check_aruko_ps(frame_ps_filtr))
                {
                    ps_filtr.Add(frame_ps_filtr);
                }                
            }
            return ps_filtr.ToArray();
        }

        Point3d_GL[][] filter_ps(Point3d_GL[][] ps)
        {
            if(ps==null) return null;
            var ps_filtr = new Point3d_GL[aruco_number][];
            for (int i = 0; i < ps_filtr.Length; i++)
            {
                ps_filtr[i] = new Point3d_GL[aruco_corner_num];
                for (int j = 0; j < ps_filtr[i].Length; j++)
                {

                    ps_filtr[i][j] = Point3d_GL.notExistP();
                }

            }

            for (int i = 0; i< ps.Length; i++)
            {
                if (ps[i]!=null && ps[i].Length!=0)
                {
                    var cur_aruco_ind = ps[i][0].ind;
                    if (inds.Contains(cur_aruco_ind))
                    {
                        for (int j = 0; j < ps[i].Length; j++)
                        {
                            if (ps[i][j].ind_sec < aruco_corner_num)
                            {
                                ps_filtr[ind_aruco[cur_aruco_ind]][ps[i][j].ind_sec] = ps[i][j];
                            }
                        }
                    }
                }
                
            }



            return ps_filtr;
        }

        static bool check_aruko_ps(Point3d_GL[][] ps)
        {
            if(ps==null) return false;
            bool check_done = true;
            for (int i = 0; i < ps.Length; i++)
            {
                if(ps[i]!=null)
                {
                    for (int j = 0; j < ps[i].Length; j++)
                    {

                        if (!ps[i][j].exist) check_done = false;
                    }
                }
                else
                {
                    check_done = false;
                }

            }

            return check_done;
        }


        public void add_point_for_registr(int ind_p,Point3d_GL p)
        {
            if(ind_p<ps_for_registr.Count)
            {
                ps_for_registr[ind_p].Add(p * matrix_frame_inv);
            }
        }

        public void init_points_for_registr(int count_p)
        {
            ps_for_registr = new List<List<Point3d_GL>>();
            for (int i = 0; i < count_p; i++)
            {
                ps_for_registr.Add(new List<Point3d_GL>());
            }
        }

        public Point3d_GL[] get_points_for_registr_draw()
        {
          return Point3d_GL.multMatr(Point3d_GL.unifPoints2d(ps_for_registr).ToArray(), matrix_model);
        }
        List<string> models_name = new List<string>();

    }



    public class NavigVision
    {
        VideoCapture[] videoCaptures = new VideoCapture[4];
        volatile IntPtr[] camera_ind_ptr = new IntPtr[] { (IntPtr)0, (IntPtr)0, (IntPtr)0, (IntPtr)0, (IntPtr)0, (IntPtr)0, (IntPtr)0 };
        private Size cameraSize = new Size(1280, 720);
        volatile Mat[] mat_global = new Mat[6];
        int cam1_ind = 0;
        int cam2_ind = 1;
        int cams_num = 2;
        public NavigVision()
        {

        }
        private void videoStart_sam(int number, int num_imb = -1)
        {
            var capture = new VideoCapture(number, VideoCapture.API.DShow);

            var num_vid = num_imb;
            if (num_vid < 0) num_vid = number;
            videoCaptures[num_vid] = capture;
            capture.Set(CapProp.FrameWidth, cameraSize.Width);
            capture.Set(CapProp.FrameHeight, cameraSize.Height);
            capture.Set(CapProp.Exposure, -7);
            capture.Set(CapProp.FourCC, VideoWriter.Fourcc('M', 'J', 'P', 'G'));
            //capture.Set(CapProp.Fps, 30);
            cameraSize.Width = (int)capture.Get(CapProp.FrameWidth);
            cameraSize.Height = (int)capture.Get(CapProp.FrameHeight);
            //videoCaptures[scanner_config.Cam1_ind]?.Set(CapProp.AutoExposure, 1);
            var fps = capture.Get(CapProp.Fps);
            Console.WriteLine("number: " + number + "; num_vid: " + num_vid + "; " + cameraSize.Width.ToString() + " " + cameraSize.Height.ToString() + " " + fps);
            //capture.SetCaptureProperty(CapProp.Contrast, 30);
            camera_ind_ptr[num_vid] = capture.Ptr;

            capture.ImageGrabbed += capturingVideo_sam;
            capture.Start();
        }

        void drawCameras_sam(VideoCapture cap)
        {
            //Console.WriteLine("dr_cam");
            var ind_cam = camera_ind_ptr.IndexOf(cap.Ptr);
            if (ind_cam < 10 && ind_cam >= 0)
            {
                cap.Retrieve(mat_global[ind_cam]);
                for (int i = 0; i < cams_num; i++)
                {
                    if (ind_cam == 0)
                    {
                        //if (NavigSys.cam_numbers == 3)
                        {
                           // navig_system.points2d_cams[i] = navig_system.navigation_processing_get_points2d(ref mat_global[ind_cam], ind_cam);
                        }

                    }
                }
            }

        }
        void capturingVideo_sam(object sender, EventArgs e)
        {
            drawCameras_sam((VideoCapture)sender);
        }
        private void con_navig_sys()
        {

            videoStart_sam(cam1_ind, 0);
            videoStart_sam(cam2_ind, 1);
            System.Timers.Timer timer = new System.Timers.Timer(10);
            
        }

        private void timer_navig_processing_Tick(object sender, EventArgs e)
        {
            //var ps3d = navig_system.navigation_processing_get_points3d_3cam(navig_system.points2d_cams[0], navig_system.points2d_cams[1], navig_system.points2d_cams[2]);    //navyg_sys_info(camera_only)
            //navig_system.navigation_processing_get_scene(ps3d, true);
        }
    }

}
