#ifndef _TRAJECTORY_GENERATOR_H_
#define _TRAJECTORY_GENERATOR_H_
/*#include <eigen3/Eigen/Dense>*/
#include <Eigen/Eigen>
#include <vector>


class TrajectoryGenerator {

private:
		std::vector<double> qp_cost;
        Eigen::MatrixXd _A; // Mapping matrix，_A矩阵为Ab矩阵构成，A(6m*6m)*Px(6m*1)=Dx(6m*1)
        Eigen::MatrixXd _Q; // Hessian matrix,(6m*6m),_Q = H
        Eigen::MatrixXd _C; // Selection matrix,((3m+3)*6m)
        Eigen::MatrixXd _L; // A.inv() * C.transpose()

        Eigen::MatrixXd _R; // (3m+3)*(3m+3)
        Eigen::MatrixXd _Rff; // 6*6
        Eigen::MatrixXd _Rpp; // (3m-3)*(3m-3)
        Eigen::MatrixXd _Rpf; // (3m-3)*6
        Eigen::MatrixXd _Rfp; // 6*(3m-3)

        Eigen::VectorXd _Pxi; // 6m*1
        Eigen::VectorXd _Pyi; // 6m*1
        Eigen::VectorXd _Pzi; // 6m*1

        Eigen::VectorXd _Dx; // [p0,p1,v0,v1,a0,a1,p1,p2,v1,v2,a1,a2...](6m*1)
        Eigen::VectorXd _Dy; // [p0,p1,v0,v1,a0,a1,p1,p2,v1,v2,a1,a2...](6m*1)
        Eigen::VectorXd _Dz; // [p0,p1,v0,v1,a0,a1,p1,p2,v1,v2,a1,a2...](6m*1)
public:
        Eigen::MatrixXd _Path;
        Eigen::VectorXd _Time;

        TrajectoryGenerator();

        ~TrajectoryGenerator();

        Eigen::MatrixXd PolyQPGeneration(
            const Eigen::MatrixXd &Path,
            const Eigen::Vector3d &Vel,
            const Eigen::Vector3d &Acc,
            const Eigen::VectorXd &Time,
            const int &type);
        
        Eigen::MatrixXd PloyCoeffGeneration(
            const Eigen::MatrixXd &PathCorridor,
            const Eigen::MatrixXd &PathConnect,
            const Eigen::VectorXd &Radius,
            const Eigen::VectorXd &Path_Radius,
            const Eigen::VectorXd &Time,
            const Eigen::MatrixXd &vel,
            const Eigen::MatrixXd &acc,
            const double maxVel,
            const double maxAcc );

        std::vector<double> getCost();

        void StackOptiDep(); // Stack the optimization's dependency, the intermediate matrix and initial derivatives

        std::pair< Eigen::MatrixXd, Eigen::MatrixXd > getInitialD(); // Initial Derivatives variable for the following optimization 

        Eigen::MatrixXd getA();
        Eigen::MatrixXd getQ();        
        Eigen::MatrixXd getC();
        Eigen::MatrixXd getL();

        Eigen::MatrixXd getR();
        Eigen::MatrixXd getRpp();
        Eigen::MatrixXd getRff();
        Eigen::MatrixXd getRfp();
        Eigen::MatrixXd getRpf();
};

#endif
