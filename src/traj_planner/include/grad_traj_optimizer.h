#ifndef _GRAD_TRAJ_OPTIMIZER_H_
#define _GRAD_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <nlopt.hpp>

// just use to get signed distance field
#include <ros/ros.h>

// sdf_tools
#include "sdf_tools/collision_map.hpp"
#include "sdf_tools/sdf.hpp"

#include "qp_generator.h"

#define GDTB getDistanceToBoundary
#define OPT_INITIAL_TRY 0
#define OPT_FIRST_STEP 1
#define OPT_SECOND_STEP 2

using namespace std;

class GradTrajOptimizer
{
public:
  GradTrajOptimizer(const vector<Eigen::Vector3d> &way_points);
  GradTrajOptimizer(ros::NodeHandle &node, const vector<Eigen::Vector3d> &waypoints);

  bool optimizeTrajectory(int step);

  void getCoefficient(Eigen::MatrixXd &coeff);

  void getSegmentTime(Eigen::VectorXd &T);

  void setSignedDistanceField(sdf_tools::SignedDistanceField *sdf, double resolution);

private:
  /** signed distance field */
  sdf_tools::SignedDistanceField *sdf;

  /** virtual boundary, 6x1 vector, for min x max x... min z,max z */
  mutable Eigen::VectorXd boundary;

  /** coefficient of polynomials*/
  Eigen::MatrixXd coeff; // 7*18 每一段的轨迹参数构成的矩阵，其中第i列代表第i段，每一行依次分别为x6个,y6个,z6个

  /** important matrix and  variables*/
  Eigen::MatrixXd A; // Mapping matrix，_A矩阵为Ab矩阵构成，A(6m*6m)*Px(6m*1)=Dx(6m*1)
  Eigen::MatrixXd C; // Selection matrix,((3m+3)*6m)
  Eigen::MatrixXd L; // A.inv() * C.transpose()
  Eigen::MatrixXd R; // (3m+3)*(3m+3)
  Eigen::MatrixXd Rff; // 6*6
  Eigen::MatrixXd Rpp; // (3m-3)*(3m-3)
  Eigen::MatrixXd Rpf; // (3m-3)*6
  Eigen::MatrixXd Rfp; // 6*(3m-3)
  Eigen::VectorXd Time; // 6*1 走每段轨迹的时间
  Eigen::VectorXd origin_time;
  Eigen::MatrixXd V;  // 6*6
  Eigen::MatrixXd Df; // 3*6 固定约束量，3个方向xyz*起pva+终pva共6个
  Eigen::MatrixXd Dp; // 3*3(m-1) 3个方向xyz*未知约束量的其他pva
  Eigen::MatrixXd origin_dp; // 3*3(m-1) 初始化 = Dp
  Eigen::MatrixXd initial_dp; // 1*3(m-1) 初始化 = Dp
  Eigen::MatrixXd path; // 7*3 所选7个轨迹点构成的矩阵，每一列代表x,y,z
  int num_dp; // 3(m-1)，所求变量
  int num_df; // 6
  int num_point; // 7
  mutable int iter_num = 0; //迭代次数
  mutable double total_time = 0.0;
  int step = 1; // 优化阶段
  double resolution; // 定义每个方块（栅格）大小
  int algorithm;  // int(24)，选择nlopt的 NLOPT_LD_MMA 算法
  double time_limit_1;  // 第一步迭代时间限制
  double time_limit_2;  // 第二步迭代时间限制
  double try_limit;
  double offset;
  double deltat;
  double bos; // 3.0
  double vos; // 8.0
  double aos; // 10.0
  double gd_value;
  int gd_type;
  double retry_offset;

  /** dynamics  parameter   from param server*/
  double w_smooth;  // 20，minimum snap 代价函数的权重系数
  double w_collision; // 0.1, 障碍代价函数的权重系数
  double w_collision_temp; // 0.1
  double d0;    // 障碍代价函数中的距离障碍物阈值，d0 = 0.7
  double r;     // 障碍代价函数的上降速率，r = 0.5 
  double alpha; // 障碍代价函数的惩罚因子, alpha = 10

  double v0;
  double rv;
  double alphav;
  double a0;
  double ra;
  double alphaa;

  double sgm_time;  // 每段轨迹的最小阈值时间
  double init_time;   // 起步和停止的时间
  double mean_v;  // 平均速度

  /** optimizer*/
  nlopt::opt optimizer;

  /** main computation function,get smoothness, collision ,velocity ,accleration cost and gradient*/
  void getCostAndGradient(std::vector<double> dp, double &cost, std::vector<double> &grad) const;

  /** cost function of optimization */
  static double costFunc(const std::vector<double> &x, std::vector<double> &grad, void *func_data);

  /** convert derivatives of end points to polynomials coefficient */
  void getCoefficientFromDerivative(Eigen::MatrixXd &coeff, const std::vector<double> &_dp) const;

  /** get distance and gradient in signed distance field ,by position query*/
  void getDistanceAndGradient(Eigen::Vector3d &pos, double &dist, Eigen::Vector3d &grad) const;

  void getPositionFromCoeff(Eigen::Vector3d &pos, const Eigen::MatrixXd &coeff, const int &index,
                            const double &time) const;

  void getVelocityFromCoeff(Eigen::Vector3d &vel, const Eigen::MatrixXd &coeff, const int &index,
                            const double &time) const;

  void getAccelerationFromCoeff(Eigen::Vector3d &acc, const Eigen::MatrixXd &coeff,
                                const int &index, const double &time) const;

  /** penalty and gradient */
  void getDistancePenalty(const double &distance, double &cost) const;
  void getDistancePenaltyGradient(const double &distance, double &grad) const;
  void getVelocityPenalty(const double &distance, double &cost) const;
  void getVelocityPenaltyGradient(const double &vel, double &grad) const;
  void getAccelerationPenalty(const double &distance, double &cost) const;
  void getAccelerationPenaltyGradient(const double &acc, double &grad) const;

  void getTimeMatrix(const double &t, Eigen::MatrixXd &T) const;
  void constrains(double &n, double min, double max) const;
  bool pathOutsideBoundary() const;
  void createNewVirtualBoundary() const;
  double getDistanceToBoundary(const double &x, const double &y, const double &z) const;
  void recaluculateGradient(const double &x, const double &y, const double &z,
                            Eigen ::Vector3d &grad) const;

  void tryDifferentParameter();
};

GradTrajOptimizer::GradTrajOptimizer(const vector<Eigen::Vector3d> &way_points)
{
  boundary = Eigen::VectorXd::Zero(6);  // 初始化全为0

  //-------------------------get parameter from server--------------------
  ros::param::get("/traj_generate/alg", this->algorithm);
  ros::param::get("/traj_generate/time_limit_1", this->time_limit_1);
  ros::param::get("/traj_generate/time_limit_2", this->time_limit_2);
  ros::param::get("/traj_generate/try_limit", this->try_limit);
  ros::param::get("/traj_generate/offset", this->offset);
  ros::param::get("/traj_generate/dt", this->deltat);
  ros::param::get("/traj_generate/retry_offset", this->retry_offset);

  ros::param::get("/traj_generate/ws", this->w_smooth);
  ros::param::get("/traj_generate/wc", this->w_collision);
  ros::param::get("/traj_generate/wc", this->w_collision_temp);

  ros::param::get("/traj_generate/alpha", this->alpha);
  ros::param::get("/traj_generate/r", this->r);
  ros::param::get("/traj_generate/d0", this->d0);

  ros::param::get("/traj_generate/alphav", this->alphav);
  ros::param::get("/traj_generate/rv", this->rv);
  ros::param::get("/traj_generate/v0", this->v0);

  ros::param::get("/traj_generate/alphaa", this->alphaa);
  ros::param::get("/traj_generate/ra", this->ra);
  ros::param::get("/traj_generate/a0", this->a0);

  ros::param::get("/traj_generate/bos", this->bos);
  ros::param::get("/traj_generate/vos", this->vos);
  ros::param::get("/traj_generate/aos", this->aos);

  ros::param::get("/traj_generate/gd_value", this->gd_value);
  ros::param::get("/traj_generate/gd_type", this->gd_type);

  ros::param::get("/traj_generate/segment_time", sgm_time);
  ros::param::get("/traj_generate/mean_v", mean_v); 
  ros::param::get("/traj_generate/init_time", init_time);

  //------------------------generate optimization dependency------------------

  path = Eigen::MatrixXd::Zero(way_points.size(), 3);   
  for(int i = 0; i < way_points.size(); ++i)
    path.row(i) = way_points[i].transpose();

  Time = Eigen::VectorXd::Zero(way_points.size() - 1);
  for(int i = 0; i < (way_points.size() - 1); ++i)
  {
    // double len= sqrt(pow(path(i, 0) - path(i + 1, 0), 2) + pow(path(i, 1) - path(i + 1, 1), 2) +
    //                  pow(path(i, 2) - path(i + 1, 2), 2));
    double len = (path.row(i) - path.row(i + 1)).norm();
    Time(i) = max(len / mean_v, sgm_time);
    if(i == 0 || i == way_points.size() - 2)
      Time(i) += init_time;
  }
  origin_time = Time;

  Eigen::Vector3d vel, acc;
  vel.setZero();
  acc.setZero();
  int type = 2;

  TrajectoryGenerator generator;

  coeff = generator.PolyQPGeneration(path, vel, acc, Time, type); // 得到各段轨迹参数
  generator.StackOptiDep();
  R = generator.getR();
  Rff = generator.getRff();
  Rpp = generator.getRpp();
  Rpf = generator.getRpf();
  Rfp = generator.getRfp();
  L = generator.getL();
  A = generator.getA();
  C = generator.getC();

  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> d = generator.getInitialD(); //[Dp,Df](3*3(m+1)) row(0) -> X | row(1) -> Y | row(2) -> Z
  initial_dp = origin_dp = Dp = d.first;
  Df = d.second;

  V.resize(6, 6);
  for(int i = 0; i < 5; ++i)
    V(i, i + 1) = i + 1;

  num_dp = Dp.cols();
  num_df = Df.cols();
  num_point = Time.rows() + 1;
}

void GradTrajOptimizer::setSignedDistanceField(sdf_tools::SignedDistanceField *s, double res)
{
  this->sdf = s;
  this->resolution = res;
}

// 判断轨迹各段起点和终点在虚拟边界外面吗
bool GradTrajOptimizer::pathOutsideBoundary() const
{
  for(int i = 0; i < path.rows(); ++i)
  {
    if(path(i, 0) <= boundary(0) || path(i, 0) >= boundary(1))
      return true;
    else if(path(i, 1) <= boundary(2) || path(i, 1) >= boundary(3))
      return true;
    else if(path(i, 2) <= boundary(4) || path(i, 2) >= boundary(5))
      return true;
  }
  return false;
}

// 创建虚拟边界
void GradTrajOptimizer::createNewVirtualBoundary() const
{
  // find the max and min x,y,z of path
  double max_x = -1000.0, max_y = -1000.0, max_z = -1000.0, min_x = 1000.0, min_y = 1000.0,
         min_z = 1000.0;
  for(int i = 0; i < path.rows(); ++i)
  {
    // max x
    if(path(i, 0) > max_x)
    {
      max_x = path(i, 0);
    }
    // max y
    if(path(i, 1) > max_y)
    {
      max_y = path(i, 1);
    }
    // max z
    if(path(i, 2) > max_z)
    {
      max_z = path(i, 2);
    }
    // min x
    if(path(i, 0) < min_x)
    {
      min_x = path(i, 0);
    }
    // min y
    if(path(i, 1) < min_y)
    {
      min_y = path(i, 1);
    }
    // min z
    if(path(i, 2) < min_z)
    {
      min_z = path(i, 2);
    }
  }

  // offset the min max box to create new boundary
  static double os = this->offset;
  boundary(0) = min_x - os;
  boundary(1) = max_x + os;
  boundary(2) = min_y - os;
  boundary(3) = max_y + os;
  boundary(4) = min_z - os;
  boundary(5) = max_z + os;

  // std::cout << "boundary:" << boundary << std::endl;
}

void GradTrajOptimizer::constrains(double &n, double min, double max) const
{
  if(n > max)
    n = max;
  else if(n < min)
    n = min;
}

void GradTrajOptimizer::tryDifferentParameter()
{
  srand(ros::Time::now().toNSec() % int(2e7));

  //--------------------- adjust polynomials segment time ----------------
  double rdt = -0.5 + 1.0 * rand() / double(RAND_MAX);
  for(int i = 0; i < this->Time.rows(); ++i)
  {
    Time(i) = origin_time(i) + rdt;
  }

  //--------------------- adjust optimization inital value ----------------
  for(int i = 0; i < num_dp; ++i)
  {
    if(i % 3 == 0)
    {
      Dp(0, i) = origin_dp(0, i) - retry_offset + 2 * retry_offset * rand() / double(RAND_MAX);
      Dp(1, i) = origin_dp(1, i) - retry_offset + 2 * retry_offset * rand() / double(RAND_MAX);
      Dp(2, i) = origin_dp(2, i) - retry_offset + 2 * retry_offset * rand() / double(RAND_MAX);

      initial_dp(0, i) = Dp(0, i);
      initial_dp(1, i) = Dp(1, i);
      initial_dp(2, i) = Dp(2, i);
    }
  }

  //-----------------------adjust cost weight----------------------------
  w_collision_temp = w_collision;
  w_collision = 0.0;
}

// 基于软约束的轨迹
bool GradTrajOptimizer::optimizeTrajectory(int step)
{
  if(step != 0 && step != 1 && step != 2)
  {
    cout << "step number error, step should be 0, 1 or 2" << endl;
  }
  this->step = step;

  // ---------------------create a virtual boundary, in avoidance of pushing the trajectory
  // ---------------------infinitely far----------------------------------------
  if(step == 1 && pathOutsideBoundary())
  {
    createNewVirtualBoundary();
  }

  // --------------------------initilize NLopt----------------------------------------
  // nlopt::srand_time();
  int seed = ros::Time::now().toNSec() % 65536;
  nlopt::srand(seed);
  nlopt::opt opt(nlopt::algorithm(this->algorithm), 3 * num_dp);  // x,y,z (3*m-3) x 3
  optimizer = opt;
  optimizer.set_min_objective(GradTrajOptimizer::costFunc, this); // 二次规划主要函数
  // optimizer.set_xtol_abs(1e-7);

  // --------------------------step specific options-----------------------------
  if(step == OPT_INITIAL_TRY)
  {
    optimizer.set_maxtime(try_limit);
  }
  else if(step == OPT_FIRST_STEP)
  {
    optimizer.set_maxtime(time_limit_1);
  }
  else if(step == OPT_SECOND_STEP)
  {
    optimizer.set_maxtime(time_limit_2);
  }

  // ---------------------------set upper and lower bound for dp--------------------
  vector<double> lb, ub;
  lb.resize(3 * num_dp);
  ub.resize(3 * num_dp);
  for(int i = 0; i < num_dp; ++i)
  {
    if(i % 3 == 0)  // 位置上下边界
    {
      lb[i] = path(i / 3 + 1, 0) - bos;
      lb[i + num_dp] = path(i / 3 + 1, 1) - bos;
      lb[i + num_dp * 2] = path(i / 3 + 1, 2) - bos;
      ub[i] = path(i / 3 + 1, 0) + bos;
      ub[i + num_dp] = path(i / 3 + 1, 1) + bos;
      ub[i + num_dp * 2] = path(i / 3 + 1, 2) + bos;
    }
    else if(i % 3 == 1) // 速度三下边界
    {
      lb[i] = -vos;
      lb[i + num_dp] = -vos;
      lb[i + 2 * num_dp] = -vos;
      ub[i] = vos;
      ub[i + num_dp] = vos;
      ub[i + num_dp * 2] = vos;
    }
    else // 加速度上下边界
    {
      lb[i] = -aos;
      lb[i + num_dp] = -aos;
      lb[i + 2 * num_dp] = -aos;
      ub[i] = aos;
      ub[i + num_dp] = aos;
      ub[i + num_dp * 2] = aos;
    }
  }
  optimizer.set_lower_bounds(lb);
  optimizer.set_upper_bounds(ub);

  // ---------------------------set initial value---------------------------
  std::vector<double> _dp(3 * num_dp);
  for(int i = 0; i < num_dp; ++i)
  {
    _dp[i] = Dp(0, i);
    _dp[i + num_dp] = Dp(1, i);
    _dp[i + 2 * num_dp] = Dp(2, i);
  }
  double min_f;

  // ---------------------------optimize ---------------------------
  cout << "-------------------begin optimization-------------------" << endl;
  nlopt::result result = optimizer.optimize(_dp, min_f);

  // ---------------------------display the result---------------------------
  cout << "Optimized result is:" << result << endl;

  // ---------------------------update optimized derivative---------------------------
  Dp.setZero();
  for(int i = 0; i < num_dp; ++i)
  {
    Dp(0, i) = _dp[i];
    Dp(1, i) = _dp[i + num_dp];
    Dp(2, i) = _dp[i + 2 * num_dp];
  }

  //----------------------------reallocate segment time--------------------------------
  for(int i = 0; i < Time.size(); ++i)
  {
    double len = 0.0;
    // head and tail segment length
    if(i == 0)
    {
      len = sqrt(pow(Df(0, 0) - Dp(0, 0), 2) + pow(Df(1, 0) - Dp(1, 0), 2) +
                 pow(Df(2, 0) - Dp(2, 0), 2));
    }
    else if(i == Time.size() - 1)
    {
      len = sqrt(pow(Df(0, 3) - Dp(0, 3 * (i - 1)), 2) + pow(Df(1, 3) - Dp(1, 3 * (i - 1)), 2) +
                 pow(Df(2, 3) - Dp(2, 3 * (i - 1)), 2));
    }
    else
    // median segment length
    {
      len = sqrt(pow(Dp(0, 3 * (i - 1)) - Dp(0, 3 * i), 2) +
                 pow(Dp(1, 3 * (i - 1)) - Dp(1, 3 * i), 2) +
                 pow(Dp(2, 3 * (i - 1)) - Dp(2, 3 * i), 2));
    }
    Time(i) = max(len / mean_v, sgm_time);
    if(i == 0 || i == Time.size() - 1)
      Time(i) += init_time;
  }

  // ---------------------------update optimized coefficient---------------------------
  getCoefficientFromDerivative(this->coeff, _dp);

  //---------------------------show optimization time of two step---------------------------
  if(step == 1 || step == 2)
  {
    cout << "total time:" << total_time << endl << "iterative num:" << iter_num << endl;
    if(step == 2)
    {
      total_time = 0;
      iter_num = 0;
    }
  }

  return true;
}

// 得到各段的轨迹参数
void GradTrajOptimizer::getCoefficient(Eigen::MatrixXd &coe)
{
  coe = this->coeff;
}

// 得到轨迹各段的时间
void GradTrajOptimizer::getSegmentTime(Eigen::VectorXd &T)
{
  T = this->Time;
}

void GradTrajOptimizer::getCoefficientFromDerivative(Eigen::MatrixXd &coefficient,
                                                     const std::vector<double> &_dp) const
{
  coefficient.resize(num_point - 1, 18);

  for(int i = 0; i < 3; ++i)
  {
    //-----------------------merge df and dp -> d(df,dp)-----------------------
    Eigen::VectorXd df(num_df);
    Eigen::VectorXd dp(num_dp);
    Eigen::VectorXd d(num_df + num_dp);

    df = Df.row(i);
    for(int j = 0; j < num_dp; j++)
    {
      dp(j) = _dp[j + num_dp * i];
    }

    d.segment(0, 6) = df;
    d.segment(6, num_dp) = dp;

    // ----------------------convert derivative to coefficient------------------
    Eigen::VectorXd coe(6 * (num_point - 1));
    coe = L * d;

    for(int j = 0; j < (num_point - 1); j++)
    {
      coefficient.block(j, 6 * i, 1, 6) = coe.segment(6 * j, 6).transpose();
    }
  }
}

void GradTrajOptimizer::getCostAndGradient(std::vector<double> dp, double &cost,
                                           std::vector<double> &_grad) const
{
  // get total iterative number and time
  iter_num++;
  ros::Time tb1 = ros::Time::now();

  // --------------------------initialize---------------------------------
  cost = 0;
  double cost_smooth = 0;
  double cost_colli = 0;
  double cost_vel = 0;
  double cost_acc = 0;

  Eigen::MatrixXd gradient = Eigen::MatrixXd::Zero(3, num_dp);
  Eigen::MatrixXd g_smooth = Eigen::MatrixXd::Zero(3, num_dp);
  Eigen::MatrixXd g_colli = Eigen::MatrixXd::Zero(3, num_dp); // 障碍物代价函数的梯度
  Eigen::MatrixXd g_vel = Eigen::MatrixXd::Zero(3, num_dp);
  Eigen::MatrixXd g_acc = Eigen::MatrixXd::Zero(3, num_dp);

  // -----------get smoothness cost---------------------------------------

  //-------------merge df and dp into d(df,dp)-----------------------------
  // #pragma omp parallel sections
  {
    // #pragma omp section
    {
      Eigen::VectorXd dfx = Df.block(0, 0, 1, 6).transpose();
      Eigen::VectorXd dfy = Df.block(1, 0, 1, 6).transpose();
      Eigen::VectorXd dfz = Df.block(2, 0, 1, 6).transpose();

      Eigen::VectorXd dpx = Eigen::VectorXd::Zero(num_dp);
      Eigen::VectorXd dpy = Eigen::VectorXd::Zero(num_dp);
      Eigen::VectorXd dpz = Eigen::VectorXd::Zero(num_dp);
      for(int i = 0; i < num_dp; ++i) // 第一轮迭代用minimum snap的dp值作为初值，将上一轮迭代的dp作为这一轮迭代的初值
      {
        dpx(i) = dp[i];
        dpy(i) = dp[i + num_dp];
        dpz(i) = dp[i + 2 * num_dp];
      }

      Eigen::VectorXd dx = Eigen::VectorXd::Zero(num_dp + num_df);
      Eigen::VectorXd dy = Eigen::VectorXd::Zero(num_dp + num_df);
      Eigen::VectorXd dz = Eigen::VectorXd::Zero(num_dp + num_df);
      dx.segment(0, 6) = dfx;
      dx.segment(6, num_dp) = dpx;
      dy.segment(0, 6) = dfy;
      dy.segment(6, num_dp) = dpy;
      dz.segment(0, 6) = dfz;
      dz.segment(6, num_dp) = dpz;

      // -------------------get smoothness cost,fs= d'Rd-----------------------
      cost_smooth = double(dx.transpose() * R * dx) + double(dy.transpose() * R * dy) +
                    (dz.transpose() * R * dz);

      //-------------------- get smoothness gradient---------------------------
      Eigen::MatrixXd gx_smooth = 2 * Rfp.transpose() * dfx + 2 * Rpp * dpx;
      Eigen::MatrixXd gy_smooth = 2 * Rfp.transpose() * dfy + 2 * Rpp * dpy;
      Eigen::MatrixXd gz_smooth = 2 * Rfp.transpose() * dfz + 2 * Rpp * dpz;

      g_smooth.row(0) = gx_smooth.transpose();
      g_smooth.row(1) = gy_smooth.transpose();
      g_smooth.row(2) = gz_smooth.transpose();
    }

    // ---------------------get polynomials coefficient, for evaluating penalty----------------
    // #pragma omp section
    {
      Eigen::MatrixXd coe; // 6*18 6段轨迹*(x,y,z分别6个多项式系数)
      getCoefficientFromDerivative(coe, dp);

      // ------------------get coolision, velocity and acceleration cost and gradient by integrate
      // -------------------along the trajectory-------------------------
      Eigen::MatrixXd Ldp(6, num_dp); // 6段轨迹 * 未知变量个数 num_dp

      // #pragma omp parallel for
      for(int s = 0; s < Time.size(); s++)
      {
        if(fabs(w_collision) < 1e-4)
          break;
        // ------------------------get matrix Ldp-----------------------------------
        Ldp = L.block(6 * s, 6, 6, num_dp);

        //------------------------- discrete time step------------------------------
        double dt = Time(s) / 15.0; // 每一段的segment再离散成15段

        for(double t = 1e-3; t < Time(s); t += dt)
        {
          // ------------------------get position,velocity------------------------
          Eigen::Vector3d pos, vel; // 根据当前时间和多项式系数计算出位置和速度
          getPositionFromCoeff(pos, coe, s, t);
          getVelocityFromCoeff(vel, coe, s, t);
          double vel_norm = vel.norm() + 1e-5;

          // ------------------------get information from signed distance field----------
          double dist = 0, gd = 0, cd = 0;
          Eigen::Vector3d grad; // 在sdf地图中的距离障碍物的梯度grad
          getDistanceAndGradient(pos, dist, grad);  // 在sdf地图中的距离障碍物dist和grad

          getDistancePenalty(dist, cd); // 计算障碍物距离惩罚cost
          getDistancePenaltyGradient(dist, gd); // 计算障碍物距离的梯度
          if(gd_type == 1)
          {
          }
          else if(gd_type == 2)
          {
            gd = -gd_value;
          }
          //------------------------time Matrix T------------------------
          Eigen::MatrixXd T(1, 6);
          getTimeMatrix(t, T);

          // #pragma omp atomic
          // ------------------------ collision cost------------------------
          cost_colli += cd * vel_norm * dt;

          // #pragma omp critical
          // ------------------------ gradient of collision cost------------------------
          {
            for(int k = 0; k < 3; k++)
            {
              g_colli.row(k) = g_colli.row(k) + (gd * grad(k) * cd * vel_norm * T * Ldp +
                                                 cd * (vel(k) / vel_norm) * T * V * Ldp) *
                                                    dt;
            }
          }
          // ------------------------only in second step optimization ------------------------
          //------------------------get velocity and accleration cost------------------------
          if(step == 2)
          {
            double cv = 0, ca = 0, gv = 0, ga = 0;
            Eigen::Vector3d acc;
            getAccelerationFromCoeff(acc, coe, s, t);
            // #pragma omp critical
            {
              for(int k = 0; k < 3; k++)
              {
                // ------------------------get velocity cost------------------------
                getVelocityPenalty(vel(k), cv);
                cost_vel += cv * vel_norm * dt;

                //------------------------ get acceleration cost------------------------
                getAccelerationPenalty(acc(k), ca);
                cost_acc += ca * vel_norm * dt;
              }
            }

            // #pragma omp critical
            {
              for(int k = 0; k < 3; k++)
              {
                // ------------------------get velocity gradient------------------------
                getVelocityPenaltyGradient(vel(k), gv);
                g_vel.row(k) =
                    g_vel.row(k) +
                    (gv * vel_norm * T * V * Ldp + cv * (vel(k) / vel_norm) * T * V * Ldp) * dt;

                // ------------------------get acceleration gradient------------------------
                getAccelerationPenaltyGradient(acc(k), ga);
                g_acc.row(k) =
                    g_acc.row(k) +
                    (ga * vel_norm * T * V * V * Ldp + ca * (vel(k) / vel_norm) * T * V * Ldp) * dt;
              }
            }
          }
        }
      }
    }
  }

  //------------------------ sum up all cost------------------------
  double ws = this->w_smooth, wc = this->w_collision, wv = 1.0, wa = 1.0;
  if(step == OPT_INITIAL_TRY)
  {
    // wc= 0.0;
  }
  else if(step == OPT_FIRST_STEP)
  {
    ws = 0.0;
    // wc= 1.0;
  }
  else if(step == OPT_SECOND_STEP)
  {
    // ws= 4.0;
    // wc= 1.0;
  }

  cost = ws * cost_smooth + wc * cost_colli + wv * cost_vel + wa * cost_acc + 1e-3;

  // cout << "smooth cost:" << ws * cost_smooth << "  collision cost" << wc * cost_colli
  //      << " vel cost " << wv * cost_vel << "  acc cost: " << wa * cost_acc << "  total:" << cost
  //      << endl;

  // ------------------------sum up all gradient and convert ------------------------
  gradient = ws * g_smooth + wc * g_colli + wv * g_vel + wa * g_acc;
  _grad.resize(num_dp * 3);

  for(int i = 0; i < num_dp; ++i)
  {
    _grad[i] = gradient(0, i) + 1e-5;
    _grad[i + num_dp] = gradient(1, i) + 1e-5;
    _grad[i + 2 * num_dp] = gradient(2, i) + 1e-5;
  }

  // ------------------------get total time------------------------
  ros::Time te1 = ros::Time::now();
  total_time += (te1.toSec() - tb1.toSec());
}

// get position from coefficient
void GradTrajOptimizer::getPositionFromCoeff(Eigen::Vector3d &pos, const Eigen::MatrixXd &coeff,
                                             const int &index, const double &time) const
{
  int s = index;
  double t = time;
  float x = coeff(s, 0) + coeff(s, 1) * t + coeff(s, 2) * pow(t, 2) + coeff(s, 3) * pow(t, 3) +
            coeff(s, 4) * pow(t, 4) + coeff(s, 5) * pow(t, 5);
  float y = coeff(s, 6) + coeff(s, 7) * t + coeff(s, 8) * pow(t, 2) + coeff(s, 9) * pow(t, 3) +
            coeff(s, 10) * pow(t, 4) + coeff(s, 11) * pow(t, 5);
  float z = coeff(s, 12) + coeff(s, 13) * t + coeff(s, 14) * pow(t, 2) + coeff(s, 15) * pow(t, 3) +
            coeff(s, 16) * pow(t, 4) + coeff(s, 17) * pow(t, 5);

  pos(0) = x;
  pos(1) = y;
  pos(2) = z;
}

// get velocity from cofficient
void GradTrajOptimizer::getVelocityFromCoeff(Eigen::Vector3d &vel, const Eigen::MatrixXd &coeff,
                                             const int &index, const double &time) const
{
  int s = index;
  double t = time;
  float vx = coeff(s, 1) + 2 * coeff(s, 2) * pow(t, 1) + 3 * coeff(s, 3) * pow(t, 2) +
             4 * coeff(s, 4) * pow(t, 3) + 5 * coeff(s, 5) * pow(t, 4);
  float vy = coeff(s, 7) + 2 * coeff(s, 8) * pow(t, 1) + 3 * coeff(s, 9) * pow(t, 2) +
             4 * coeff(s, 10) * pow(t, 3) + 5 * coeff(s, 11) * pow(t, 4);
  float vz = coeff(s, 13) + 2 * coeff(s, 14) * pow(t, 1) + 3 * coeff(s, 15) * pow(t, 2) +
             4 * coeff(s, 16) * pow(t, 3) + 5 * coeff(s, 17) * pow(t, 4);

  vel(0) = vx;
  vel(1) = vy;
  vel(2) = vz;
}

// get acceleration from coefficient
void GradTrajOptimizer::getAccelerationFromCoeff(Eigen::Vector3d &acc, const Eigen::MatrixXd &coeff,
                                                 const int &index, const double &time) const
{
  int s = index;
  double t = time;
  float ax = 2 * coeff(s, 2) + 6 * coeff(s, 3) * pow(t, 1) + 12 * coeff(s, 4) * pow(t, 2) +
             20 * coeff(s, 5) * pow(t, 3);
  float ay = 2 * coeff(s, 8) + 6 * coeff(s, 9) * pow(t, 1) + 12 * coeff(s, 10) * pow(t, 2) +
             20 * coeff(s, 11) * pow(t, 3);
  float az = 2 * coeff(s, 14) + 6 * coeff(s, 15) * pow(t, 1) + 12 * coeff(s, 16) * pow(t, 2) +
             20 * coeff(s, 17) * pow(t, 3);

  acc(0) = ax;
  acc(1) = ay;
  acc(2) = az;
}

inline void GradTrajOptimizer::getDistancePenalty(const double &d, double &cost) const
{
  cost = this->alpha * exp(-(d - this->d0) / this->r);
}

inline void GradTrajOptimizer::getDistancePenaltyGradient(const double &d, double &grad) const
{
  grad = -(this->alpha / this->r) * exp(-(d - this->d0) / this->r);
}

inline void GradTrajOptimizer::getVelocityPenalty(const double &v, double &cost) const
{
  cost = alphav * exp((abs(v) - v0) / rv);
}

inline void GradTrajOptimizer::getVelocityPenaltyGradient(const double &v, double &grad) const
{
  grad = (alphav / rv) * exp((abs(v) - v0) / rv);
}

inline void GradTrajOptimizer::getAccelerationPenalty(const double &a, double &cost) const
{
  cost = alphaa * exp((abs(a) - a0) / ra);
}

inline void GradTrajOptimizer::getAccelerationPenaltyGradient(const double &a, double &grad) const
{
  grad = (alphaa / ra) * exp((abs(a) - a0) / ra);
}

// get distance in signed distance field ,by position query
void GradTrajOptimizer::getDistanceAndGradient(Eigen::Vector3d &pos, double &dist,
                                               Eigen::Vector3d &grad) const
{
  // get sdf directly from sdf_tools
  Eigen::Vector3d ori_pos = pos;
  constrains(pos(0), -9.8, 9.8);
  constrains(pos(1), -9.8, 9.8);
  constrains(pos(2), 0.2, 4.8);
  std::vector<double> location_gradient_query =
      this->sdf->GetGradient(pos(0), pos(1), pos(2), true);
  grad(0) = location_gradient_query[0];
  grad(1) = location_gradient_query[1];
  grad(2) = location_gradient_query[2];
  std::pair<float, bool> location_sdf_query = this->sdf->GetSafe(pos(0), pos(1), pos(2));
  dist = location_sdf_query.first;

  // if(dist < 0)
  //   cout << "pos:" << pos << "dist:" << dist << "grad:" << grad << endl;

  // update distance and gradient using boundary
  double dtb = getDistanceToBoundary(ori_pos(0), ori_pos(1), ori_pos(2));

  if(dtb < dist)
  {
    dist = dtb;
    recaluculateGradient(ori_pos(0), ori_pos(1), ori_pos(2), grad);
  }
}

double GradTrajOptimizer::getDistanceToBoundary(const double &x, const double &y,
                                                const double &z) const
{
  double dist_x = min(x - boundary(0), boundary(1) - x);
  double dist_y = min(y - boundary(2), boundary(3) - y);
  double dist_z = min(z - boundary(4), boundary(5) - z);
  double dtb = min(dist_x, dist_y);
  dtb = min(dtb, dist_z);

  return dtb;
}

void GradTrajOptimizer::recaluculateGradient(const double &x, const double &y, const double &z,
                                             Eigen::Vector3d &grad) const
{
  double r = this->resolution;

  grad(0) = (10 * (GDTB(x + r, y, z) - GDTB(x - r, y, z)) +
             3 * (GDTB(x + r, y + r, z) - GDTB(x - r, y + r, z)) +
             3 * (GDTB(x + r, y - r, z) - GDTB(x - r, y - r, z))) /
            (32 * r);
  grad(1) = (10 * (GDTB(x, y + r, z) - GDTB(x, y - r, z)) +
             3 * (GDTB(x + r, y + r, z) - GDTB(x + r, y - r, z)) +
             3 * (GDTB(x - r, y + r, z) - GDTB(x - r, y - r, z))) /
            (32 * r);
  grad(2) = (10 * (GDTB(x, y, z + r) - GDTB(x, y, z - r)) +
             3 * (GDTB(x, y + r, z + r) - GDTB(x, y + r, z - r)) +
             3 * (GDTB(x, y - r, z + r) - GDTB(x, y - r, z - r))) /
            (32 * r);
}

void GradTrajOptimizer::getTimeMatrix(const double &t, Eigen::MatrixXd &T) const
{
  T.resize(1, 6);
  T.setZero();

  for(int i = 0; i < 6; ++i)
  {
    T(0, i) = pow(t, i);
  }
}

/** NLopt format cost function */
double GradTrajOptimizer::costFunc(const std::vector<double> &x, std::vector<double> &grad,
                                   void *func_data)
{
  GradTrajOptimizer *gtop = reinterpret_cast<GradTrajOptimizer *>(func_data);
  double cost;

  gtop->getCostAndGradient(x, cost, grad);

  return cost;
}

#endif