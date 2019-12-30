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
  DifferentialState     theta_0, theta_1, theta_2, theta_3;         // rotor arm servo angles
  Control               theta_dot_0, theta_dot_1, theta_dot_2, theta_dot_3;
  DifferentialEquation  f;
  Function              h, hN;

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
  const double theta_dot_min = -1.0; // change per 1 sec => total change defined by dt => theta_dot_max * dt = delta_theta_max
  const double theta_dot_max = 1.0;

  // Bias to prevent division by zero.
  const double epsilon = 0.1;     // Camera projection recover bias [m]

  // System Dynamics
  f << dot(theta_0) == theta_dot_0;
  f << dot(theta_1) == theta_dot_1;
  f << dot(theta_2) == theta_dot_2;
  f << dot(theta_3) == theta_dot_3;

  // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
  // Running cost vector consists of all states and inputs.
  h << theta_0 << theta_1 << theta_2 << theta_3
    << theta_dot_0 << theta_dot_1 << theta_dot_2 << theta_dot_3;

  // End cost vector consists of all states (no inputs at last state).
  hN << theta_0 << theta_1 << theta_2 << theta_3;

  // Zero cost on perception triggers error: matrix must be positive definite

  // Running cost weight matrix
  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();
  Q(0,0) = 10;   // theta_0
  Q(1,1) = 10;   // theta_1
  Q(2,2) = 10;   // theta_2
  Q(3,3) = 10;   // theta_3
  Q(4,4) = 1;   // theta_0_dot
  Q(5,5) = 1;   // theta_1_dot
  Q(6,6) = 1;   // theta_2_dot
  Q(7,7) = 1;   // theta_3_dot

  // End cost weight matrix
  DMatrix QN(hN.getDim(), hN.getDim());
  QN.setIdentity();
    Q(0,0) = 10;   // theta_0
    Q(1,1) = 10;   // theta_1
    Q(2,2) = 10;   // theta_2
    Q(3,3) = 10;   // theta_3

  // Set a reference for the analysis (if CODE_GEN is false).
  // Reference is at x = 2.0m in hover (qw = 1).
  // Goal is to move from x = 0.0m to x = 2.0m
  DVector r(h.getDim());    // Running cost reference
  r.setZero();
  r(0) = 0.785;
  r(1) = 0.785;
  r(2) = 0.785;
  r(3) = 0.785;

    DVector rN(hN.getDim());   // End cost reference
  rN.setZero();
  r(0) = 0.785;
  r(1) = 0.785;
  r(2) = 0.785;
  r(3) = 0.785;

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
  ocp.subjectTo( theta_min <= theta_0 <= theta_max);
  ocp.subjectTo( theta_min <= theta_1 <= theta_max);
  ocp.subjectTo( theta_min <= theta_2 <= theta_max);
  ocp.subjectTo( theta_min <= theta_3 <= theta_max);
  ocp.subjectTo( theta_dot_min <= theta_dot_0 <= theta_dot_max);
  ocp.subjectTo( theta_dot_min <= theta_dot_1 <= theta_dot_max);
  ocp.subjectTo( theta_dot_min <= theta_dot_2 <= theta_dot_max);
  ocp.subjectTo( theta_dot_min <= theta_dot_3 <= theta_dot_max);



  if(!CODE_GEN)
  {
    // Set initial state
    ocp.subjectTo( AT_START, theta_0 ==  0.0 );
    ocp.subjectTo( AT_START, theta_1 ==  0.0 );
    ocp.subjectTo( AT_START, theta_2 ==  0.0 );
    ocp.subjectTo( AT_START, theta_3 ==  0.0 );
    ocp.subjectTo( AT_START, theta_dot_0 ==  0.0 );
    ocp.subjectTo( AT_START, theta_dot_1 ==  0.0 );
    ocp.subjectTo( AT_START, theta_dot_2 ==  0.0 );
    ocp.subjectTo( AT_START, theta_dot_3 ==  0.0 );

//      ocp.subjectTo( AT_END, theta_0 ==  0.785 );
//      ocp.subjectTo( AT_END, theta_1 ==  0.785 );
//      ocp.subjectTo( AT_END, theta_2 ==  0.785 );
//      ocp.subjectTo( AT_END, theta_3 ==  0.785 );
//
//      ocp.subjectTo( AT_END, theta_dot_0 ==  0. );
//      ocp.subjectTo( AT_END, theta_dot_1 ==  0. );
//      ocp.subjectTo( AT_END, theta_dot_2 ==  0. );
//      ocp.subjectTo( AT_END, theta_dot_3 ==  0. );

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
    algorithm << window3;
    algorithm << window4;
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