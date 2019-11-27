#include "ros/ros.h"
#include "std_msgs/String.h"

#include <vector>
#include <string>
#include <XBotInterface/XBotInterface.h>
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include </opt/ros/kinetic/include/kdl/frames.hpp>
#include </opt/ros/kinetic/include/kdl/jacobian.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames_io.hpp>
#include<Eigen/Core>
#include<Eigen/SVD>
#include <Eigen/LU> 
#include <cmath> 

#define N_TRYTASK 5
#define N_TASK 1
#define N_SAMETASK 5

bool PersonalThirdTrajectory(KDL::Frame xe,
                             KDL::Frame xtraj,
                             const double t,
                             const double tf, 
                             KDL::Frame& xd,
                             KDL::Twist& xd_dot) 
{
    KDL::Vector a0,a2,a3,pdiff,odiff,otraj,oe;
    bool Trajectory_Compl=false;
    double Alfa,Beta,Gamma;

    if(tf > 0 && t<tf)
    {
        // Follow a third order polinomial
        
        //-----------------------Position--------------------///
        
        pdiff=xtraj.p-xe.p;
        a0=xe.p;
        a2=3*pdiff/std::pow(tf,2);
        a3=-2*pdiff/std::pow(tf,3); 
        xd.p.data[0]= a3[0]* std::pow(t,3) + a2[0]* std::pow(t,2) + a0[0];
        xd.p.data[1]= a3[1]* std::pow(t,3) + a2[1]* std::pow(t,2) + a0[1]; 
        xd.p.data[2]= a3[2]* std::pow(t,3) + a2[2]* std::pow(t,2) + a0[2]; 
        
        xd_dot.vel.data[0]   = 3*a3[0]* std::pow(t,2)+2*a2[0]*t;
        xd_dot.vel.data[1]   = 3*a3[1]* std::pow(t,2)+2*a2[1]*t;
        xd_dot.vel.data[2]   = 3*a3[2]* std::pow(t,2)+2*a2[2]*t;
        
        //-----------------------Position--------------------///
        
        //-----------------------Orientation--------------------///
        
        // get ZYX Euler angles from the rotation
        
        xtraj.M.GetEulerZYZ(Alfa,Beta,Gamma);
        otraj.data[0]=Alfa;
        otraj.data[1]=Beta,
        otraj.data[2]=Gamma;
        
        xe.M.GetEulerZYZ(Alfa,Beta,Gamma);
        oe.data[0]=Alfa;
        oe.data[1]=Beta;
        oe.data[2]=Gamma;

        
        odiff=otraj-oe;
        a0=oe;
        a2=3*odiff/std::pow(tf,2);
        a3=-2*odiff/std::pow(tf,3); 
        Alfa= a3[0]* std::pow(t,3) + a2[0]* std::pow(t,2) + a0[0];
        Beta= a3[1]* std::pow(t,3) + a2[1]* std::pow(t,2) + a0[1]; 
        Gamma= a3[2]* std::pow(t,3) + a2[2]* std::pow(t,2) + a0[2]; 
        xd.M=xe.M.EulerZYZ(Alfa,Beta,Gamma);
        
        xd_dot.rot.data[0]   = 3*a3[0]* std::pow(t,2)+2*a2[0]*t;
        xd_dot.rot.data[1]   = 3*a3[1]* std::pow(t,2)+2*a2[1]*t;
        xd_dot.rot.data[2]   = 3*a3[2]* std::pow(t,2)+2*a2[2]*t;
        
        //-----------------------Orientation--------------------///
        
    }
    else
    {
    xd = xe; 
    xd_dot.vel.data[0]   = 0.0;
    xd_dot.vel.data[1]   = 0.0;
    xd_dot.vel.data[2]   = 0.0;
    xd_dot.rot.data[0]   = 0.0;
    xd_dot.rot.data[1]   = 0.0;
    xd_dot.rot.data[2]   = 0.0;
    Trajectory_Compl=true;
    }
    
  
return(Trajectory_Compl);
    
}


template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
        Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
        return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();

// A= V*S*UT  
    
}


int main ( int argc, char **argv ) {
    
    ros::init ( argc, argv, "Pose_Cartesian_Space" );
   
    double time,final_time,Alfa,Beta,Gamma,Kp,Kvp,Ko,Kvo;
    
    bool completed_mov,singlepp_reached;
    double dt = 0.01; // 100 Hz loop
    int countTask=0,countryTask;
    
    // Init Frame Desired
    KDL::Frame xtraj,xdes;
    KDL::Twist xdes_dot,xerror;
    Eigen::VectorXd q_ref, qdot_ref,qref_aux,qdot_auxrefP,qdot_auxref,XdotPlusError(6);
    KDL:: Vector od,oe,xerroro,xerrorp;
    KDL::Frame xe;
    Eigen::MatrixXd JG,JA;
    Eigen::Matrix6d RotJgtoJa;
    KDL::Rotation RotInit;
    
    std::string path_to_config_file = XBot::Utils::getXBotConfig();
    XBot::ModelInterface::Ptr ModelRobot= XBot::ModelInterface::getModel(path_to_config_file); // returns a shared pointer to a model object
    XBot::RobotInterface::Ptr Robot = XBot::RobotInterface::getRobot ( path_to_config_file ); // returns a shared pointer to a robot object
    
    KDL::Frame T_lsole,T_lsole_inv,T_lsole_new,T_LWrMot3;
   
    ModelRobot->syncFrom<>(Robot->model());
    ModelRobot->getPose("l_sole",T_lsole);
    ModelRobot->getPose("LWrMot3",T_LWrMot3);
    
    T_lsole_inv=T_lsole.Inverse();                
    ModelRobot->setFloatingBasePose(T_lsole_inv);     // Move l_sole World Frame
        
    ModelRobot->update();                             // MUST BE CALLED to REFRESH
    
    ModelRobot->getPose("l_sole",T_lsole_new);
    
    ros::Rate loop_rate ( 200 );
    
    
    while ( ros::ok() && (countTask<N_TASK)) {
    
    Robot->sense();                             // SENSE FROM ROBOT
    
    ModelRobot->syncFrom<>(Robot->model());           // SYNC FROM ROBOT
    ModelRobot->setFloatingBasePose(T_lsole_inv);     // Move l_sole World Frame
    ModelRobot->update();
    ModelRobot->getPose("LWrMot3",T_LWrMot3);
 

    switch (countTask)
    {
        case 0: {
                xtraj.p.data[0]=-0.1;
                xtraj.p.data[1]=0.5;
                xtraj.p.data[2]=1.8;
                xtraj.M = KDL::Rotation::EulerZYZ(M_PI/2,M_PI,0);
                final_time=5;
                Kp=1.0,Ko=1.0; // converge coefficient  kp[1/s]*ep[m]  ko[1/s]*eo[rad]
                Kvp=1.0,Kvo=1.0; // velocity reduction factor unitary             
        }break;
        
        case 1: {
                xtraj.p.data[0]=T_LWrMot3.p.x();
                xtraj.p.data[1]=T_LWrMot3.p.y();
                xtraj.p.data[2]=T_LWrMot3.p.z();
                
                T_LWrMot3.M.GetEulerZYZ(Alfa,Beta,Gamma);
                xtraj.M.EulerZYZ(0.0,0.0,0.0);
        }break;
        
        case 2: {
                xtraj.p.data[0]=T_LWrMot3.p.x();
                xtraj.p.data[1]=0.3;
                xtraj.p.data[2]=T_LWrMot3.p.z();
                
                xtraj.M.EulerZYZ(0.0,0.0,0.0);
        }break;

    }
    

    time=0;
    completed_mov = false;

    while ( !completed_mov ) {
      /////----- CREATE TRAJECTORY------------////
      
      completed_mov=PersonalThirdTrajectory(T_LWrMot3,xtraj,time,final_time,xdes,xdes_dot); 
      countryTask=0;
      if(!completed_mov)
      {
        //ROS_INFO("Trajectory Desired into Cartesian Space [%lf][m] [%lf][m] [%lf][m]",xdes.p.x(),xdes.p.y(),xdes.p.z());
        singlepp_reached=false;
        while(!singlepp_reached && countryTask<N_TRYTASK)
        {
/*---------------------------------------------------------------------------------------------------------------------*/
            ModelRobot->update();
            ModelRobot->getPose("LWrMot3",xe);  // GET Forward Kinematic from worl frame to Left Hand Frame (LwrMot3)
            ModelRobot->getJointVelocity(qdot_auxrefP); //GET Joint Velocity From the ModelInterface (m/s prismatic or rad/s rotoidal)
            ModelRobot->getJointPosition(qref_aux);     //GET Joint Position From the ModelInterface (m prismatic or rad rotoidal) 
/*---------------------------------------------------------------------------------------------------------------------*/          
        /////----- GET JACOBIAN-------------////
            ModelRobot->getJacobian("LWrMot3",JG);  // GET JACOBIAN from world frame to Left Hand Frame (LwrMot3)
            JG.block(0,0,6,6) = Eigen::Matrix6d::Zero();// PUT 6X6 World Reference-Virtual Joints to Zero with 6x6
                                                        // set active joint mask (OPENSOT) OPPOSITE: //ModelRobot->maskJacobian("left_arm", JA);
/*---------------------------------------------------------------------------------------------------------------------*/        
            xdes.M.GetEulerZYZ(Alfa,Beta,Gamma);
            od[0]=Alfa;
            od[1]=Beta,
            od[2]=Gamma;
        
            xe.M.GetEulerZYZ(Alfa,Beta,Gamma);
            oe[0]=Alfa;
            oe[1]=Beta,
            oe[2]=Gamma;
            
            RotInit=KDL::Rotation(0,-sin(Alfa),cos(Alfa)*sin(Beta),0,cos(Alfa),sin(Alfa)*sin(Beta),1,0,cos(Beta));
            RotInit=RotInit.Inverse();
/*---------------------------------------------------------------------------------------------------------------------*/  
            Eigen::MatrixXd JGpinv(JG.rows(),JG.cols());      
            JGpinv = pseudoInverse(JG);  //calculates the pseudo inverse
/*---------------------------------------------------------------------------------------------------------------------*/
          
            RotJgtoJa=Eigen::Matrix6d::Zero();
            RotJgtoJa.block(0,0,3,3)=Eigen::Matrix3d::Identity();
            RotJgtoJa.block(3,3,3,3)<<    RotInit.data[0],RotInit.data[1],RotInit.data[2],
                                        RotInit.data[3],RotInit.data[4],RotInit.data[5],
                                        RotInit.data[6],RotInit.data[7],RotInit.data[8];
            JA=RotJgtoJa*JG;
            Eigen::MatrixXd JApinv(JA.cols(),JA.rows());   
            JApinv = pseudoInverse(JA);  //calculates the pseudo inverse
/*---------------------------------------------------------------------------------------------------------------------*/

            xerrorp=xdes.p-xe.p;
            xerror.vel=Kp*(xdes.p-xe.p);                                // error= Kp*(pd-pe) = [m/s] converge cofficient
            XdotPlusError(0)=Kvp*xdes_dot.vel.x()+xerror.vel.x();   
            XdotPlusError(1)=Kvp*xdes_dot.vel.y()+xerror.vel.y();       // Feedforward for the Velocity Fp= pdot+perror [m/s];
            XdotPlusError(2)=Kvp*xdes_dot.vel.z()+xerror.vel.z();
            
            xerroro=od-oe;
            xerror.rot=Ko*(od-oe);
            XdotPlusError(3)=Kvo*xdes_dot.rot.x()+xerror.rot.x();    // error= Ko*(od-oe) = [rad/s]
            XdotPlusError(4)=Kvo*xdes_dot.rot.y()+xerror.rot.y();       // Feedforward for the Velocity Fp= odot+oerror [rad/s];
            XdotPlusError(5)=Kvo*xdes_dot.rot.z()+xerror.rot.z();
                 
/*---------------------------------------------------------------------------------------------------------------------*/
      
            qdot_auxref=JApinv * XdotPlusError;                     // qdot= JPInv*Fp+Fo [m/s or rad/s]  vector of Joints element
            qref_aux+=dt*(qdot_auxrefP+qdot_auxref)/2;              //integration trapezoidal method q=qpast+(sample_time*(qdotp+qdot)/2)

            qdot_ref=qdot_auxref.tail(Robot->getJointNum());       // Delete the 6 joints added for the world frame in setFloatingBasePose
            q_ref=qref_aux.tail(Robot->getJointNum());     
/*---------------------------------------------------------------------------------------------------------------------*/
        
            ModelRobot->setJointPosition(qref_aux);                // Set qdot and q for the model
            ModelRobot->setJointVelocity(qdot_auxref);

            Robot->setPositionReference ( q_ref );                 // Set qdot and q for the robot
            Robot->setVelocityReference ( qdot_ref );
            Robot->move();
    /*---------------------------------------------------------------------------------------------------------------------*/
            //if(fabs((xerrorp.x())<0.3)&&(fabs(xerrorp.y())<0.3)&&(fabs(xerrorp.z())<0.3)&&   /// Position Error of less then 30 Centimetre
            //fabs((xerroro.x())<0.1)&&(fabs(xerroro.y())<0.1)&&(fabs(xerroro.z())<0.1))    /// Orientation Error of less then about 6 degree
                singlepp_reached=true;
            //else
            //{
                //countryTask++;
            //    if(countryTask>=N_TRYTASK/dt)
            //        completed_mov=true;
            //}
        }
      }
      time +=dt;

      usleep ( 1e6*dt );  
    }  
    if(countryTask<N_TRYTASK)
        countTask++; 
    else
        ROS_INFO("I'm goint to perform the past trajectory");
    
    ros::spinOnce();
    
    loop_rate.sleep();
    
 }
    
return 0;
}


/////***************** NOTE****************************

/* Eigen::MatrixXd JP(3,JA.cols());
for (int i = 0; i < JP.rows(); ++i)
    for (int j = 0; j < JP.cols(); ++j)    // SELECT ONLY THE JACOBIAN FOR THE Position part NOT the Orientation
        JP(i,j)=JA(i,j); 
/////----- GET JACOBIAN-------------////
    
Eigen::MatrixXd JPpinv(3,JA.cols());  
JPpinv = pseudoInverse(JP);  //calculates the pseudo inverse
*/

// get the RPY (fixed axis) from the rotation
//T_lsole.M.GetRPY(R,P,Y);  
//ROS_INFO("Super OK [%lf] [deg][%lf] [deg][%lf] [deg]",Rx,Ry,Rz); 
//ROS_INFO("Super OK [%lf] [deg][%lf] [deg][%lf] [deg]",Y,P,R);

/////***************** NOTE****************************



