#include <mujoco/mujoco.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <GLFW/glfw3.h>
#include <fcntl.h> 
#include <cstdio>
#include <iostream>
#include "useful_math.h"
#include "GLFW_callbacks.h"
#include "MJ_Interface.h"
#include "PVT_Ctr.h"
#include "Pin_KinDyn.h"
#include "DataLogger.h"
#include "../ik_7dof_F2O_OLR3/ik_7dof_F2O_OLR3.c"

// MuJoCo load and compile model
char error[1000] = "Could not load binary model";
mjModel* mj_model = mj_loadXML("../Models/scene_float_new.xml", 0, error, 1000);
mjData* mj_data = mj_makeData(mj_model);


constexpr int PORT = 5005;
constexpr int BUFFER_SIZE = 1024;

// 全局原子变量，用于控制线程何时停止
std::atomic<bool> running(true);

// 用于存储接收到的数据的全局向量
std::vector<float> receivedData;

// 修正的打印关节名称的函数
void printJointNames(const mjModel* m) {
    std::cout << "Listing all joint names in the model:" << std::endl;
    const char* name = m->names;  // 指向名称字符串的开始
    for (int i = 0; i < m->njnt; ++i) {
        std::cout << "Joint " << i << " name: " << name << std::endl;
        while (*name++);  // 移动到下一个名称
    }
}

std::vector<double> ik_left_arm(double z_alpha, double y_beta, double x_gamma,
  double p_x, double p_y, double p_z, double bet, double cur_theta[7])
{

    cur_theta[1] = -cur_theta[1];
    cur_theta[5] = -cur_theta[5];

    const double current_ddd[7]={cur_theta[0], cur_theta[1], cur_theta[2], cur_theta[3], cur_theta[4], cur_theta[5], cur_theta[6]};
    double theta[8]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};



    ik_7dof_F2O_OLR3(z_alpha, y_beta, x_gamma, p_x, p_y, p_z, bet, current_ddd, theta);
    std::vector<double> left_dof(8,0.0);
    for (size_t i = 0; i < 8; i++)
    {
        left_dof[i] = (double)theta[i];
    }
    left_dof[1] = -left_dof[1];
    left_dof[5] = -left_dof[5];
    
    
    return left_dof;


}

std::vector<double> ik_right_arm(double z_alpha, double y_beta, double x_gamma,
  double p_x, double p_y, double p_z, double bet, double cur_theta[7])
{
    // cur_theta[5] = -cur_theta[5];
    cur_theta[0] = -cur_theta[0];
    cur_theta[1] = -cur_theta[1];
    cur_theta[2] = -cur_theta[2];
    cur_theta[4] = -cur_theta[4];
    cur_theta[6] = -cur_theta[6];

    cur_theta[1] = -cur_theta[1];
    cur_theta[5] = -cur_theta[5];

    const double current_ddd[7]={cur_theta[0], cur_theta[1], cur_theta[2], cur_theta[3], cur_theta[4], cur_theta[5], cur_theta[6]};
    double theta[8]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    ik_7dof_F2O_OLR3(-z_alpha, y_beta, -x_gamma, p_x, -p_y, p_z, bet, current_ddd, theta);
    std::vector<double> right_dof(8,0.0);
    for (size_t i = 0; i < 8; i++)
    {
        right_dof[i] = (double)theta[i];
    }
    // right_dof[5] = -right_dof[5];

    right_dof[0] = -right_dof[0];
    right_dof[1] = -right_dof[1];
    right_dof[2] = -right_dof[2];
    right_dof[4] = -right_dof[4];
    right_dof[6] = -right_dof[6];

    right_dof[1] = -right_dof[1];
    right_dof[5] = -right_dof[5];
    return right_dof;



}

double left_last_dof[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double right_last_dof[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// for(size_t i = 0; i < 7; i++)
// {
//     left_last_dof[i]=0.0;
//     right_last_dof[i]=0.0;
// }

double left_bet=0.5233;
double right_bet=0.5233;
// double left_p_a[6];
// double right_p_a[6];


    double left_p_a[6] = {-1.5708, 1.5708, 0.0, 0.0 ,500, 0.0};
    double right_p_a[6] = {1.5708, 1.5708, 0.0, 0.0 ,-500, 0.0};


// 接收数据的函数
void receiveData(int sockfd) {

    receivedData.clear();
    struct sockaddr_in cliaddr;
    char buffer[BUFFER_SIZE];

    while (running) {
        socklen_t len = sizeof(cliaddr);
        // std::cout<<"fuck:"<<len<<std::endl;
        ssize_t n = recvfrom(sockfd, buffer, BUFFER_SIZE, 0, (struct sockaddr*)&cliaddr, &len);
        if (n == -1) {
            std::cerr << "接收数据时发生错误\n";
            continue;
        }

        // 假设每个浮点数占4字节
        if (n % 4 != 0) {
            std::cerr << "接收到的数据长度不是浮点数的整数倍\n";
            continue;
        }


        for (int i = 0; i < n; i += 4) {
            if (receivedData.size() >= 14) {
                // 如果已有12个数据，删除第一个以保持数量为12
                receivedData.erase(receivedData.begin());
            }
            
            uint32_t byteRepresentation = (static_cast<uint32_t>(static_cast<unsigned char>(buffer[i])) << 24) |
                                          (static_cast<uint32_t>(static_cast<unsigned char>(buffer[i + 1])) << 16) |
                                          (static_cast<uint32_t>(static_cast<unsigned char>(buffer[i + 2])) << 8) |
                                          static_cast<uint32_t>(static_cast<unsigned char>(buffer[i + 3]));
            float value;
            memcpy(&value, &byteRepresentation, sizeof(float));
            receivedData.push_back(value);
        }

        for (size_t i = 0; i < 6; i++)
{
    left_p_a[i] = (double)receivedData[i];
    right_p_a[i] = (double)receivedData[i+7];
}
left_bet = (double)receivedData[6];
right_bet = (double)receivedData[13];
        // 每次解析完一组浮点数后打印
        // std::cout << "接收到最新的 24 个浮点数：";
        // for (float val : receivedData) {
        //     std::cout << val << " ";
        // }
        // for (size_t i = 0; i < 26; i++)
        // {
        //     std::cout << receivedData[i] << " ";
        // }
        
        // std::cout << std::endl;
    }
}




//************************
// main function
int main(int argc, const char** argv)
{
// for (size_t i = 0; i < 12; i++)
// {
// }
    receivedData.push_back(-1.5708);
    receivedData.push_back(1.5708);
    receivedData.push_back(0.0);
    receivedData.push_back(0.0);
    receivedData.push_back(500.0);
    receivedData.push_back(0.0);
    receivedData.push_back(0.5233);
    receivedData.push_back(1.5708);
    receivedData.push_back(1.5708);
    receivedData.push_back(0.0);
    receivedData.push_back(0.0);
    receivedData.push_back(500.0);
    receivedData.push_back(0.0);
    receivedData.push_back(0.5233);

    // receivedData.push_back(-0.019);
    // receivedData.push_back(-0.32);
    // receivedData.push_back(-0.159);
    // receivedData.push_back(-0.961012);
    // receivedData.push_back(-0.238101);
    // receivedData.push_back(-0.140587);
    // receivedData.push_back(-0.178345);
    // receivedData.push_back(0.145203);
    // receivedData.push_back(0.973195);
    // receivedData.push_back(-0.211305);
    // receivedData.push_back(0.960325);
    // receivedData.push_back(-0.182006);

    // receivedData.push_back(-0.018);
    // receivedData.push_back(0.113);
    // receivedData.push_back(-1.01);
    // receivedData.push_back(0.0);
    // receivedData.push_back(0.0);
    // receivedData.push_back(0.0);
    // receivedData.push_back(-0.018);
    // receivedData.push_back(-0.116);
    // receivedData.push_back(-1.01);
    // receivedData.push_back(0.0);
    // receivedData.push_back(0.0);
    // receivedData.push_back(0.0);

        int sockfd;
    struct sockaddr_in servaddr;

    // 创建UDP套接字
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
        std::cerr << "无法创建套接字\n";
        return 1;
    }

    // 设置地址和端口
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = INADDR_ANY;

    // 绑定到端口
    if (bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1) {
        std::cerr << "无法绑定套接字\n";
        close(sockfd);
        return 1;
    }

    // 创建并启动接收数据线程
    std::thread receiverThread(receiveData, sockfd);


    UIctr uiController(mj_model,mj_data);   // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data); // data interface for Mujoco
    Pin_KinDyn kinDynSolver("../Models/AzureDragon.urdf"); // kinematics and dynamics solver
    DataBus RobotState(kinDynSolver.model_nv); // data bus
    PVT_Ctr pvtCtr(mj_model->opt.timestep,"../Common/JointCtrConfig.json");// PVT joint control
    DataLogger logger("../RecordedData/DataLogger.log"); // data logger



    std::vector<double> left_angles(7,0.0);
    std::vector<double> right_angles(7,0.0);



    // variables ini
    double stand_legLength = 1.01; //-0.95; // desired baselink height
    double foot_height =0.07; // distance between the foot ankel joint and the bottom
    double  xv_des = 0.7;  // desired velocity in x direction

    RobotState.width_hips = 0.5;
    //mju_copy(mj_data->qpos, mj_model->key_qpos, mj_model->nq*1); // set ini pos in Mujoco
    int model_nv=kinDynSolver.model_nv;



    // ini position and posture for foot-end and hand
    std::vector<double> motors_pos_des(model_nv-6,0);
    std::vector<double> motors_pos_cur(model_nv-6,0);
    std::vector<double> motors_vel_des(model_nv-6,0);
    std::vector<double> motors_vel_cur(model_nv-6,0);
    std::vector<double> motors_tau_des(model_nv-6,0);
    std::vector<double> motors_tau_cur(model_nv-6,0);
    Eigen::Vector3d fe_l_pos_L_des={-0.018, 0.113, -stand_legLength};
    Eigen::Vector3d fe_r_pos_L_des={-0.018, -0.116, -stand_legLength};
    Eigen::Vector3d fe_l_eul_L_des={-0.000, -0.008, -0.000};
    Eigen::Vector3d fe_r_eul_L_des={0.000, -0.008, 0.000};
    Eigen::Matrix3d fe_l_rot_des= eul2Rot(fe_l_eul_L_des(0),fe_l_eul_L_des(1),fe_l_eul_L_des(2));
    Eigen::Matrix3d fe_r_rot_des= eul2Rot(fe_r_eul_L_des(0),fe_r_eul_L_des(1),fe_r_eul_L_des(2));



    Eigen::Vector3d hd_l_pos_L_des={-0.019, 0.32, -0.159};
    Eigen::Vector3d hd_l_eul_L_des={-1.7581, 0.2129, 2.9581};
    Eigen::Vector3d hd_r_pos_L_des={-0.019, -0.32, -0.159};
    Eigen::Vector3d hd_r_eul_L_des={1.7581, 0.21291, -2.9581};
    Eigen::Matrix3d hd_l_rot_des= eul2Rot(hd_l_eul_L_des(0),hd_l_eul_L_des(1),hd_l_eul_L_des(2));
    Eigen::Matrix3d hd_r_rot_des= eul2Rot(hd_r_eul_L_des(0),hd_r_eul_L_des(1),hd_r_eul_L_des(2));

    // std::cout<<"left"<<std::endl;
    // std::cout<<hd_l_rot_des<<std::endl;

    // std::cout<<"right"<<std::endl;
    // std::cout<<hd_r_rot_des<<std::endl;

    auto resLeg=kinDynSolver.computeInK_Leg(fe_l_rot_des,fe_l_pos_L_des,fe_r_rot_des,fe_r_pos_L_des);
    auto resHand=kinDynSolver.computeInK_Hand(hd_l_rot_des,hd_l_pos_L_des,hd_r_rot_des,hd_r_pos_L_des);



    // register variable name for data logger
    logger.addIterm("simTime", 1);
    // logger.addIterm("pos_eul_bet",7);
    // logger.addIterm("motors_pos",7);
    // logger.addIterm("motors_pos_cur",model_nv-6);
    // logger.addIterm("motors_pos_des",model_nv-6);
    // logger.addIterm("motors_tau_cur",model_nv-6);
    // logger.addIterm("motors_vel_des",model_nv-6);
    // logger.addIterm("motors_vel_cur",model_nv-6);
    // logger.addIterm("left_arm_pos",3);
    // logger.addIterm("left_m",9);
    // logger.addIterm("right_arm_pos",3);
    // logger.addIterm("right_m",9);
    logger.finishItermAdding();
    // std::cout<<model_nv<<std::endl;

    /// ----------------- sim Loop ---------------
    double simEndTime=3600;
    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;
    double startSteppingTime=3;
    double startWalkingTime=5;

    // init UI: GLFW
    uiController.iniGLFW();
    uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo",false);

    while( !glfwWindowShouldClose(uiController.window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        simstart=mj_data->time;
        // mj_data->time;
        while( mj_data->time - simstart < 1.0/60.0 )
        // while(true)
        {


// mj_data->time - simstart
    Eigen::VectorXd pos_eul_bet(7);
    Eigen::VectorXd motors_pos(7);


            mj_step(mj_model, mj_data);

            simTime=mj_data->time;
            // printf("-------------%.3f s------------\n",simTime);
            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(RobotState);



            // inverse kinematics
            // fe_l_pos_L_des<<receivedData[14], receivedData[15], receivedData[16];
            // fe_l_eul_L_des<<receivedData[17], receivedData[18], receivedData[19];
            // fe_r_pos_L_des<<receivedData[20], receivedData[21], receivedData[22];
            // fe_r_eul_L_des<<receivedData[23], receivedData[24], receivedData[25];
            fe_l_rot_des= eul2Rot(fe_l_eul_L_des(0),fe_l_eul_L_des(1),fe_l_eul_L_des(2));  // roll, pitch, yaw
            fe_r_rot_des= eul2Rot(fe_r_eul_L_des(0),fe_r_eul_L_des(1),fe_r_eul_L_des(2));

            // hd_l_pos_L_des<<receivedData[0], receivedData[1], receivedData[2];
            // hd_r_pos_L_des<<receivedData[12], receivedData[13], receivedData[14];
            // // hd_l_eul_L_des<<receivedData[3], receivedData[4], receivedData[5];
            // // hd_r_eul_L_des<<receivedData[9], receivedData[10], receivedData[11];



            // // hd_l_pos_L_des<<-0.02, 0.32, -0.159;
            // // hd_r_pos_L_des<<-0.02, -0.32, -0.159;
            // // hd_l_eul_L_des<<-1.7581, 0.2129, 2.9581;
            // // hd_r_eul_L_des<<1.7581, 0.21291, -2.9581;
            // hd_l_rot_des<<receivedData[3], receivedData[4], receivedData[5],
            //               receivedData[6], receivedData[7], receivedData[8],
            //               receivedData[9], receivedData[10], receivedData[11];

            // left_m<<receivedData[3], receivedData[4], receivedData[5],
            //               receivedData[6], receivedData[7], receivedData[8],
            //               receivedData[9], receivedData[10], receivedData[11];


            // hd_r_rot_des<<receivedData[15], receivedData[16], receivedData[17],
            //               receivedData[18], receivedData[19], receivedData[20],
            //               receivedData[21], receivedData[22], receivedData[23];

            // left_m<<receivedData[15], receivedData[16], receivedData[17],
            //               receivedData[18], receivedData[19], receivedData[20],
            //               receivedData[21], receivedData[22], receivedData[23];

            // hd_l_rot_des= eul2Rot(hd_l_eul_L_des(0),hd_l_eul_L_des(1),hd_l_eul_L_des(2));
            // hd_r_rot_des= eul2Rot(hd_r_eul_L_des(0),hd_r_eul_L_des(1),hd_r_eul_L_des(2));

            std::vector<double> left_angles_tested(8,0.0);
            std::vector<double> right_angles_tested(8,0.0);


            left_angles_tested = ik_left_arm(left_p_a[0], left_p_a[1], left_p_a[2], left_p_a[3], 
                                    left_p_a[4], left_p_a[5], left_bet, left_last_dof);


            if (left_angles_tested[7] == 0.0)
            {
                for (size_t i = 0; i < 7; i++)
                {
                    left_angles[i] = left_angles_tested[i];
                }
                
            }
            else
            {
                std::cout<<"left error ::"<<simTime<<" "<<left_angles_tested[7]<<std::endl;
            }
            // for (size_t i = 0; i < 6; i++)
            // {
            //     std::cout<<left_p_a[i]<<"   ";
            // }
            // std::cout<<std::endl;

            // for (size_t i = 0; i < 7; i++)
            // {
            //     std::cout<<left_angles_tested[i]<<"   ";
            // }
            // std::cout<<std::endl;
            
            

            pos_eul_bet<<left_p_a[0], left_p_a[1], left_p_a[2], left_p_a[3], 
                                    left_p_a[4], left_p_a[5], left_bet;

            motors_pos<<left_angles[0],  -left_angles[1],  left_angles[2],  left_angles[3],  left_angles[4],  -left_angles[5],  left_angles[6];



            right_angles_tested = ik_right_arm(right_p_a[0], right_p_a[1], right_p_a[2], right_p_a[3], 
                                    right_p_a[4], right_p_a[5], right_bet, right_last_dof);


            if (right_angles_tested[7] ==0.0)
            {
                for (size_t i = 0; i < 7; i++)
                {
                    right_angles[i] = right_angles_tested[i];
                }
                
            }
            else
            {
                std::cout<<"right error::"<<simTime<<" "<<right_angles_tested[7]<<std::endl;
            }
            



            resLeg=kinDynSolver.computeInK_Leg(fe_l_rot_des,fe_l_pos_L_des,fe_r_rot_des,fe_r_pos_L_des);
            // resHand=kinDynSolver.computeInK_Hand(hd_l_rot_des,hd_l_pos_L_des,hd_r_rot_des,hd_r_pos_L_des);
            std::vector<double> except_arm(17,0.0);
            std::vector<double> except_all(31,0.0);
            except_all = eigen2std(resLeg.jointPosRes);

            for (size_t i = 0; i < 17; i++)
            {
                except_arm[i] = except_all[i+14];
            }
            

            // std::cout<<resLeg.jointPosRes<<std::endl;
            std::vector<double> whole_body(31,0.0);
            for (size_t i = 0; i < 7; i++)
            {
                whole_body[i] = left_angles[i];
                whole_body[i+7] = right_angles[i];
            }
            // std::cout<<except_arm<<std::endl;
            // for (size_t i = 0; i < 17; i++)
            // {
            //     std::cout<<except_arm[i]<<" ";
            // }
            // std::cout<<std::endl;
            

            for (size_t i = 0; i < 17; i++)
            {
                whole_body[i+14] = except_arm[i];
            }


            // for (size_t i = 0; i < 31; i++)
            // {
            //     std::cout<<whole_body[i]<<" ";
            // }
            // std::cout<<std::endl;
            
            
            // Enter here functions to send actuator commands, like:
            // arm-l: 0-6, arm-r: 7-13, head: 14,15 waist: 16-18, leg-l: 19-24, leg-r: 25-30
            // get the final joint command
            RobotState.motors_pos_des= whole_body;
            RobotState.motors_vel_des=motors_vel_des;
            RobotState.motors_tor_des=motors_tau_des;
//            Eigen::VectorXd tmp=resLeg.jointPosRes+resHand.jointPosRes;
//            std::cout<<tmp.transpose()<<std::endl;
//            std::cout<<resHand.itr<<std::endl;
//            std::cout<<resHand.err.transpose()<<std::endl;

            pvtCtr.dataBusRead(RobotState);
            if (simTime<=3)
            {
                pvtCtr.calMotorsPVT(100.0/1000.0/180.0*3.1415);  // limit velocity
            }
            else
            {
//                pvtCtr.setJointPD(100,10,"Joint-ankel-l-pitch");
//                pvtCtr.setJointPD(100,10,"Joint-ankel-l-roll");
//                pvtCtr.setJointPD(100,10,"Joint-ankel-r-pitch");
//                pvtCtr.setJointPD(100,10,"Joint-ankel-r-roll");
//                pvtCtr.setJointPD(1000,100,"Joint-knee-l-pitch");
//                pvtCtr.setJointPD(1000,100,"Joint-knee-r-pitch");
                pvtCtr.calMotorsPVT();
            }
            pvtCtr.dataBusWrite(RobotState);

            mj_interface.setMotorsTorque(RobotState.motors_tor_out);

            logger.startNewLine();
            logger.recItermData("simTime", simTime);
            // logger.recItermData("pos_eul_bet", pos_eul_bet);
            // logger.recItermData("motors_pos", motors_pos);
            // logger.recItermData("motors_pos_cur",RobotState.motors_pos_cur);
            // // logger.recItermData("left_arm_pos",hd_l_pos_L_des);
            // // logger.recItermData("left_m",hd_l_eul_L_des);
            // // logger.recItermData("right_arm_pos",hd_r_pos_L_des);
            // // logger.recItermData("right_m",hd_r_eul_L_des);
            // logger.recItermData("motors_pos_des",RobotState.motors_pos_des);
            // logger.recItermData("motors_tau_cur",RobotState.motors_tor_out);
            // logger.recItermData("motors_vel_cur",RobotState.motors_vel_cur);
            // logger.recItermData("motors_vel_des",RobotState.motors_vel_des);
            logger.finishLine();

            // std::cout<<RobotState.
        // 清空已解析的浮点数以准备下一组
        // values.clear();
        for (size_t i = 0; i < 7; i++)
        {
            left_last_dof[i] = (double)RobotState.motors_pos_cur[i];
        }

        for (size_t i = 0; i < 7; i++)
        {
            right_last_dof[i] = (double)RobotState.motors_pos_cur[i+7];
        }


        }

        

        if (mj_data->time>=simEndTime)
        {
            break;
        }

        uiController.updateScene();
    }

        running = false;
    receiverThread.join();


close(sockfd);
//    // free visualization storage
    uiController.Close();

    // free MuJoCo model and data, deactivate
    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);

    return 0;
}