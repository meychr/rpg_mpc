/*    rpg_quadrotor_mpc
 *    A model predictive control implementation for quadrotors.
 *    Copyright (C) 2017-2018 Philipp Foehn, 
 *    Robotics and Perception Group, University of Zurich
 * 
 *    Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
 *    https://github.com/uzh-rpg/rpg_quadrotor_control
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include <memory>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

// Standalone code generation for a parameter-free quadrotor model
// with thrust and rates input. 

int main( ){
  // Use Acado
  USING_NAMESPACE_ACADO

  /*
  Switch between code generation and analysis.

  If CODE_GEN is true the system is compiled into an optimizaiton problem
  for real-time iteration and all code to run it online is generated.
  Constraints and reference structure is used but the values will be set on
  runtinme.

  If CODE_GEN is false, the system is compiled into a standalone optimization
  and solved on execution. The reference and constraints must be set in here.
  */
  const bool CODE_GEN = false;

  // System variables
  DifferentialState     p_x, p_y, p_z;
  DifferentialState     q_w, q_x, q_y, q_z;
  DifferentialState     v_x, v_y, v_z;
  DifferentialState     theta_0, theta_1, theta_2, theta_3;         // rotor arm servo angles
  Control               T, w_x, w_y, w_z;
  Control               theta_dot_0, theta_dot_1, theta_dot_2, theta_dot_3;
  DifferentialEquation  f;
  Function              h, hN;
  OnlineData            p_F_x, p_F_y, p_F_z;
  OnlineData            t_B_C_x, t_B_C_y, t_B_C_z;
  OnlineData            q_B_C_w, q_B_C_x, q_B_C_y, q_B_C_z;

  // Parameters with exemplary values. These are set/overwritten at runtime.
  const double t_start = 0.0;     // Initial time [s]
  const double t_end = 2.0;       // Time horizon [s]
  const double dt = 0.1;          // Discretization time [s]
  const int N = round(t_end/dt);  // Number of nodes
  const double g_z = 9.8066;      // Gravity is everywhere [m/s^2]
  const double w_max_yaw = 1;     // Maximal yaw rate [rad/s]
  const double w_max_xy = 3;      // Maximal pitch and roll rate [rad/s]
  const double T_min = 2;         // Minimal thrust [N]
  const double T_max = 20;        // Maximal thrust [N]
  const double theta_min = 0.0;
  const double theta_max = 1.57;  // M_PI/2
  const double theta_dot_min = -20.0; // change per 1 sec => total change defined by dt => theta_dot_max * dt = delta_theta_max
  const double theta_dot_max = 20.0;

  // Bias to prevent division by zero.
  const double epsilon = 0.1;     // Camera projection recover bias [m]

  // Proxy function for mlp compensation => optimization exploits it to brake as well!!!
  const double k_aero = 0.1;
  IntermediateState resistance_x = v_x * k_aero * (theta_0 + (1.57 - theta_1) + theta_2 + (1.57 - theta_3));
  IntermediateState resistance_y = v_y * k_aero * ((1.57 - theta_0) + theta_1 + (1.57 - theta_2) + theta_3);

  // System Dynamics
  f << dot(p_x) ==  v_x;
  f << dot(p_y) ==  v_y;
  f << dot(p_z) ==  v_z;
  f << dot(q_w) ==  0.5 * ( - w_x * q_x - w_y * q_y - w_z * q_z);
  f << dot(q_x) ==  0.5 * ( w_x * q_w + w_z * q_y - w_y * q_z);
  f << dot(q_y) ==  0.5 * ( w_y * q_w - w_z * q_x + w_x * q_z);
  f << dot(q_z) ==  0.5 * ( w_z * q_w + w_y * q_x - w_x * q_y);
  f << dot(v_x) ==  2 * ( q_w * q_y + q_x * q_z ) * T - resistance_x;
  f << dot(v_y) ==  2 * ( q_y * q_z - q_w * q_x ) * T - resistance_y;
  f << dot(v_z) ==  ( 1 - 2 * q_x * q_x - 2 * q_y * q_y ) * T - g_z;
  f << dot(theta_0) == theta_dot_0;
  f << dot(theta_1) == theta_dot_1;
  f << dot(theta_2) == theta_dot_2;
  f << dot(theta_3) == theta_dot_3;

  // Intermediate states to calculate point of interest projection!
  IntermediateState intSx = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F_x-p_x)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F_y-p_y)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F_z-p_z));
  IntermediateState intSy = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F_y-p_y)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w))*(p_F_z-p_z)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y))*(p_F_x-p_x));
  IntermediateState intSz = ((((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*(-(-q_x)*q_B_C_z-q_B_C_w*q_y-q_B_C_x*q_z-q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F_z-p_z)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)+((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)*((-q_z)*q_B_C_y+q_B_C_w*q_x+q_B_C_x*q_w+q_B_C_z*q_y)+(-(-q_y)*q_B_C_x-q_B_C_w*q_z-q_B_C_y*q_x-q_B_C_z*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y))*(p_F_x-p_x)+(((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y)+((-q_x)*q_B_C_x+(-q_y)*q_B_C_y+(-q_z)*q_B_C_z+q_B_C_w*q_w)*(-(-q_z)*q_B_C_y-q_B_C_w*q_x-q_B_C_x*q_w-q_B_C_z*q_y)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w)+((-q_x)*q_B_C_z+q_B_C_w*q_y+q_B_C_x*q_z+q_B_C_y*q_w)*((-q_y)*q_B_C_x+q_B_C_w*q_z+q_B_C_y*q_x+q_B_C_z*q_w))*(p_F_y-p_y));

  // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
  // Running cost vector consists of all states and inputs.
  h << p_x << p_y << p_z
    << q_w << q_x << q_y << q_z
    << v_x << v_y << v_z
    << theta_0 << theta_1 << theta_2 << theta_3
    << intSx/(intSz + epsilon) << intSy/(intSz + epsilon) 
    << T << w_x << w_y << w_z
    << theta_dot_0 << theta_dot_1 << theta_dot_2 << theta_dot_3;

  // End cost vector consists of all states (no inputs at last state).
  hN << p_x << p_y << p_z
    << q_w << q_x << q_y << q_z
    << v_x << v_y << v_z
    << theta_0 << theta_1 << theta_2 << theta_3
    << intSx/(intSz + epsilon) << intSy/(intSz + epsilon);

  // Running cost weight matrix
  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();
  Q(0,0) = 100;     // x
  Q(1,1) = 100;     // y
  Q(2,2) = 100;     // z
  Q(3,3) = 100;     // qw
  Q(4,4) = 100;     // qx
  Q(5,5) = 100;     // qy
  Q(6,6) = 100;     // qz
  // Increased weights for testing proxy function, otherwise final position dominates => emphasis on breaking not keeping velocity
  Q(7,7) = 100;     // vx
  Q(8,8) = 100;     // vy
  Q(9,9) = 10;      // vz
  // Use optionally to set desired standard config with low weights
  Q(10,10) = 0.01;   // theta_0
  Q(11,11) = 0.01;   // theta_1
  Q(12,12) = 0.01;   // theta_2
  Q(13,13) = 0.01;   // theta_3
  // Turn off perception cost => leads to overhead???
  Q(14,14) = 0;     // Cost on perception
  Q(15,15) = 0;     // Cost on perception
  // Control inputs
  Q(16,16) = 1;     // T
  Q(17,17) = 1;     // wx
  Q(18,18) = 1;     // wy
  Q(19,19) = 1;     // wz
  Q(20,20) = 0.1;   // theta_dot_0
  Q(21,21) = 0.1;   // theta_dot_1
  Q(22,22) = 0.1;   // theta_dot_2
  Q(23,23) = 0.1;   // theta_dot_3

  // End cost weight matrix
  DMatrix QN(hN.getDim(), hN.getDim());
  QN.setIdentity();
  QN(0,0) = Q(0,0);   // x
  QN(1,1) = Q(1,1);   // y
  QN(2,2) = Q(2,2);   // z
  QN(3,3) = Q(3,3);   // qw
  QN(4,4) = Q(4,4);   // qx
  QN(5,5) = Q(5,5);   // qy
  QN(6,6) = Q(6,6);   // qz
  QN(7,7) = Q(7,7);   // vx
  QN(8,8) = Q(8,8);   // vy
  QN(9,9) = Q(9,9);   // vz
  // Use optionally to set desired standard config with low weights
  QN(10,10) = Q(10,10);   // theta_0
  QN(11,11) = Q(11,11);   // theta_1
  QN(12,12) = Q(12,12);   // theta_2
  QN(13,13) = Q(13,13);   // theta_3
  // Turn off perception cost => leads to overhead???
  QN(14,14) = Q(14,14);  // Cost on perception
  QN(15,15) = Q(15,15);  // Cost on perception

  // Set a reference for the analysis (if CODE_GEN is false).
  // Reference is at x = 2.0m in hover (qw = 1).
  DVector r(h.getDim());    // Running cost reference
  r.setZero();
  r(0) = 0.0; // px
  r(1) = 2.0; // py
  r(3) = 1.0; // qw
  r(7) = 0.0; // vx
  r(8) = 2.5; // vy
  r(10) = 0.785; // theta_0
  r(11) = 0.785; // theta_1
  r(12) = 0.785; // theta_2
  r(13) = 0.785; // theta_3
  r(16) = g_z;  // T

  DVector rN(hN.getDim());   // End cost reference
  rN.setZero();
  rN(0) = r(0); // px
  rN(1) = r(1); // py
  rN(3) = r(3); // qw
  rN(7) = r(7); // vx
  rN(8) = r(8); // vy
  rN(10) = r(10); // theta_0
  rN(11) = r(11); // theta_1
  rN(12) = r(12); // theta_2
  rN(13) = r(13); // theta_3

  // DEFINE AN OPTIMAL CONTROL PROBLEM:
  // ----------------------------------
  OCP ocp( t_start, t_end, N );
  if(!CODE_GEN)
  {
    // For analysis, set references.
    ocp.minimizeLSQ( Q, h, r );
    ocp.minimizeLSQEndTerm( QN, hN, rN );
  }else{
    // For code generation, references are set during run time.
    BMatrix Q_sparse(h.getDim(), h.getDim());
    Q_sparse.setIdentity();
    BMatrix QN_sparse(hN.getDim(), hN.getDim());
    QN_sparse.setIdentity();
    ocp.minimizeLSQ( Q_sparse, h);
    ocp.minimizeLSQEndTerm( QN_sparse, hN );
  }

  // Add system dynamics
  ocp.subjectTo( f );
  // Add constraints
  ocp.subjectTo(-w_max_xy <= w_x <= w_max_xy);
  ocp.subjectTo(-w_max_xy <= w_y <= w_max_xy);
  ocp.subjectTo(-w_max_yaw <= w_z <= w_max_yaw);
  ocp.subjectTo( T_min <= T <= T_max);
  ocp.subjectTo( theta_min <= theta_0 <= theta_max);
  ocp.subjectTo( theta_min <= theta_1 <= theta_max);
  ocp.subjectTo( theta_min <= theta_2 <= theta_max);
  ocp.subjectTo( theta_min <= theta_3 <= theta_max);
  ocp.subjectTo( theta_dot_min <= theta_dot_0 <= theta_dot_max);
  ocp.subjectTo( theta_dot_min <= theta_dot_1 <= theta_dot_max);
  ocp.subjectTo( theta_dot_min <= theta_dot_2 <= theta_dot_max);
  ocp.subjectTo( theta_dot_min <= theta_dot_3 <= theta_dot_max);

  // number of online data
  ocp.setNOD(10);

  if(!CODE_GEN)
  {
    // Set initial state
    ocp.subjectTo( AT_START, p_x ==  0.0 );
    ocp.subjectTo( AT_START, p_y ==  0.0 );
    ocp.subjectTo( AT_START, p_z ==  0.0 );
    ocp.subjectTo( AT_START, q_w ==  1.0 );
    ocp.subjectTo( AT_START, q_x ==  0.0 );
    ocp.subjectTo( AT_START, q_y ==  0.0 );
    ocp.subjectTo( AT_START, q_z ==  0.0 );
    ocp.subjectTo( AT_START, v_x ==  0.0 );
    ocp.subjectTo( AT_START, v_y ==  0.0 );
    ocp.subjectTo( AT_START, v_z ==  0.0 );
    ocp.subjectTo( AT_START, w_x ==  0.0 );
    ocp.subjectTo( AT_START, w_y ==  0.0 );
    ocp.subjectTo( AT_START, w_z ==  0.0 );

    ocp.subjectTo( AT_START, theta_0 ==  0.785 );
    ocp.subjectTo( AT_START, theta_1 ==  0.785 );
    ocp.subjectTo( AT_START, theta_2 ==  0.785 );
    ocp.subjectTo( AT_START, theta_3 ==  0.785 );
    ocp.subjectTo( AT_START, theta_dot_0 ==  0.0 );
    ocp.subjectTo( AT_START, theta_dot_1 ==  0.0 );
    ocp.subjectTo( AT_START, theta_dot_2 ==  0.0 );
    ocp.subjectTo( AT_START, theta_dot_3 ==  0.0 );

    // Setup some visualization
    GnuplotWindow window1( PLOT_AT_END );
//    window1.addSubplot( p_x,"position x" );
//    window1.addSubplot( p_y,"position y" );
//    window1.addSubplot( p_z,"position z" );
    window1.addSubplot( v_x,"velocity x" );
    window1.addSubplot( v_y,"velocity y" );
    window1.addSubplot( v_z,"velocity z" );
    // radians to degrees causes problems with plotting
    window1.addSubplot( theta_0 * 180.0 / 3.141,"theta 0" );
    window1.addSubplot( theta_1 * 180.0 / 3.141,"theta 1" );
    window1.addSubplot( theta_2 * 180.0 / 3.141,"theta 2" );
    window1.addSubplot( theta_3 * 180.0 / 3.141,"theta 3" );

    GnuplotWindow window2( PLOT_AT_END);
    window2.addSubplot( w_x,"rotation-acc x" );
    window2.addSubplot( w_y,"rotation-acc y" );
    window2.addSubplot( w_z,"rotation-acc z" );
    window2.addSubplot( T,"Thrust" );

    GnuplotWindow window3( PLOT_AT_END );
    window3.addSubplot( theta_0,"theta 0" );
    window3.addSubplot( theta_1,"theta 1" );
    window3.addSubplot( theta_2,"theta 2" );
    window3.addSubplot( theta_3,"theta 3" );

    GnuplotWindow window4( PLOT_AT_END );
    window4.addSubplot( theta_dot_0,"theta dot 0" );
    window4.addSubplot( theta_dot_1,"theta dot 1" );
    window4.addSubplot( theta_dot_2,"theta dot 2" );
    window4.addSubplot( theta_dot_3,"theta dot 3" );

    // Define an algorithm to solve it.
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set( INTEGRATOR_TOLERANCE, 1e-6 );
    algorithm.set( KKT_TOLERANCE, 1e-3 );

    // Not possible to use more than two windows???
    algorithm << window1;
    //algorithm << window4;

    algorithm.solve();

  }else{
    // For code generation, we can set some properties.
    // The main reason for a setting is given as comment.
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION,  GAUSS_NEWTON);        // is robust, stable
    mpc.set(DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING);   // good convergence
    mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);  // due to qpOASES
    mpc.set(INTEGRATOR_TYPE,        INT_IRK_GL4);         // accurate
    mpc.set(NUM_INTEGRATOR_STEPS,   N);
    mpc.set(QP_SOLVER,              QP_QPOASES);          // free, source code
    mpc.set(HOTSTART_QP,            YES);
    mpc.set(CG_USE_OPENMP,                    YES);       // paralellization
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);        // set on runtime
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);       // time-varying costs
    mpc.set( USE_SINGLE_PRECISION,        YES);           // Single precision

    // Do not generate tests, makes or matlab-related interfaces.
    mpc.set( GENERATE_TEST_FILE,          NO);
    mpc.set( GENERATE_MAKE_FILE,          NO);
    mpc.set( GENERATE_MATLAB_INTERFACE,   NO);
    mpc.set( GENERATE_SIMULINK_INTERFACE, NO);

    // Finally, export everything.
    if(mpc.exportCode("quadrotor_mpc_codegen") != SUCCESSFUL_RETURN)
      exit( EXIT_FAILURE );
    mpc.printDimensionsQP( );
  }

  return EXIT_SUCCESS;
}