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


// MuJoCo load and compile model
char error[1000] = "Could not load binary model";
mjModel* mj_model = mj_loadXML("../Models/scene_float.xml", 0, error, 1000);
mjData* mj_data = mj_makeData(mj_model);


constexpr int PORT = 5005;
constexpr int BUFFER_SIZE = 1024;

// 全局原子变量，用于控制线程何时停止
std::atomic<bool> running(true);

// 用于存储接收到的数据的全局向量
std::vector<float> receivedData;


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
            if (receivedData.size() >= 24) {
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

        // 每次解析完一组浮点数后打印
        std::cout << "接收到最新的 24 个浮点数：";
        for (float val : receivedData) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }
}




//************************
// main function
int main(int argc, const char** argv)
{
// receivedData << float(-0.019), float(0.32), float(-0.159), float(-1.7581), float(0.2129), float(2.9581), float(-0.019), float(-0.32), float(-0.159), float(1.7581), float(0.21291), float(-2.9581);
// for (size_t i = 0; i < 12; i++)
// {
// }
    receivedData.push_back(-0.019);
    receivedData.push_back(0.32);
    receivedData.push_back(-0.159);
    receivedData.push_back(-1.7581);
    receivedData.push_back(0.2129);
    receivedData.push_back(2.9581);
    receivedData.push_back(-0.019);
    receivedData.push_back(-0.32);
    receivedData.push_back(-0.159);
    receivedData.push_back(1.7581);
    receivedData.push_back(0.21291);
    receivedData.push_back(-2.9581);

    receivedData.push_back(-0.018);
    receivedData.push_back(0.113);
    receivedData.push_back(-1.01);
    receivedData.push_back(0.0);
    receivedData.push_back(0.0);
    receivedData.push_back(0.0);
    receivedData.push_back(-0.018);
    receivedData.push_back(-0.116);
    receivedData.push_back(-1.01);
    receivedData.push_back(0.0);
    receivedData.push_back(0.0);
    receivedData.push_back(0.0);


    // Eigen::Vector3d hd_l_pos_L_des={-0.019, 0.32, -0.159};
    // Eigen::Vector3d hd_r_pos_L_des={-0.019, -0.32, -0.159};
    // Eigen::Vector3d hd_l_eul_L_des={-1.7581, 0.2129, 2.9581};
    // Eigen::Vector3d hd_r_eul_L_des={1.7581, 0.21291, -2.9581};



    // Eigen::Vector3d fe_l_pos_L_des={-0.018, 0.113, -stand_legLength};
    // Eigen::Vector3d fe_l_eul_L_des={-0.000, -0.008, -0.000};
    // Eigen::Vector3d fe_r_pos_L_des={-0.018, -0.116, -stand_legLength};
    // Eigen::Vector3d fe_r_eul_L_des={0.000, -0.008, 0.000};


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




    // ini classes
    UIctr uiController(mj_model,mj_data);   // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data); // data interface for Mujoco
    Pin_KinDyn kinDynSolver("../Models/AzureDragon.urdf"); // kinematics and dynamics solver
    DataBus RobotState(kinDynSolver.model_nv); // data bus
    PVT_Ctr pvtCtr(mj_model->opt.timestep,"../Common/JointCtrConfig.json");// PVT joint control
    DataLogger logger("../RecordedData/DataLogger.log"); // data logger


    // int sockfd;
    // struct sockaddr_in servaddr, cliaddr;
    // char buffer[BUFFER_SIZE];


    //     int flags = fcntl(sockfd, F_GETFL, 0);
    // fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);



    // // 创建UDP套接字
    // sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    // if (sockfd == -1) {
    //     std::cerr << "无法创建套接字\n";
    //     return 1;
    // }

    //  // 清零servaddr
    // memset(&servaddr, 0, sizeof(servaddr));

    // // 设置地址和端口
    // servaddr.sin_family = AF_INET;
    // servaddr.sin_port = htons(PORT);
    // servaddr.sin_addr.s_addr = INADDR_ANY; // 绑定到所有可用地址

    // // 绑定到端口
    // if (bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1) {
    //     std::cerr << "无法绑定套接字\n";
    //     close(sockfd);
    //     return 1;
    // }
    
    // std::cout << "服务器监听端口 " << PORT << std::endl;

    // std::vector<float> values; // 存储解析得到的浮点数

    // 接收数据循环
    // while (true) {
    //     socklen_t len = sizeof(cliaddr);
    //     ssize_t n = recvfrom(sockfd, buffer, BUFFER_SIZE, 0, (struct sockaddr*)&cliaddr, &len);
    //     if (n == -1) {
    //         std::cerr << "接收数据时发生错误\n";
    //         continue;
    //     }

    //     // 处理接收到的数据
    //     // 假设每个浮点数占4字节
    //     if (n % 4 != 0) {
    //         std::cerr << "接收到的数据长度不是浮点数的整数倍\n";
    //         continue;
    //     }

    //     // 逐个解析浮点数
    //     for (int i = 0; i < n; i += 4) {
    //         uint32_t byteRepresentation = (static_cast<uint32_t>(static_cast<unsigned char>(buffer[i])) << 24) |
    //                                       (static_cast<uint32_t>(static_cast<unsigned char>(buffer[i + 1])) << 16) |
    //                                       (static_cast<uint32_t>(static_cast<unsigned char>(buffer[i + 2])) << 8) |
    //                                       static_cast<uint32_t>(static_cast<unsigned char>(buffer[i + 3]));
    //         float value;
    //         memcpy(&value, &byteRepresentation, sizeof(float));
    //         values.push_back(value);
    //     }

    //     // 每次解析完一组浮点数后打印
    //     std::cout << "接收到 " << values.size() << " 个浮点数：";
    //     for (float val : values) {
    //         std::cout << val << " ";
    //     }
    //     std::cout << std::endl;

    //     // 清空已解析的浮点数以准备下一组
    //     values.clear();
    // }

    // close(sockfd);




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

    auto resLeg=kinDynSolver.computeInK_Leg(fe_l_rot_des,fe_l_pos_L_des,fe_r_rot_des,fe_r_pos_L_des);
    auto resHand=kinDynSolver.computeInK_Hand(hd_l_rot_des,hd_l_pos_L_des,hd_r_rot_des,hd_r_pos_L_des);



    // register variable name for data logger
    logger.addIterm("simTime", 1);
    logger.addIterm("motors_pos_cur",model_nv-6);
    // logger.addIterm("motors_pos_des",model_nv-6);
    // logger.addIterm("motors_tau_cur",model_nv-6);
    // logger.addIterm("motors_vel_des",model_nv-6);
    // logger.addIterm("motors_vel_cur",model_nv-6);
    logger.addIterm("left_arm_pos",3);
    logger.addIterm("left_arm_eul",3);
    logger.addIterm("right_arm_pos",3);
    logger.addIterm("right_arm_eul",3);
    logger.finishItermAdding();

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


            mj_step(mj_model, mj_data);

            simTime=mj_data->time;
            // printf("-------------%.3f s------------\n",simTime);
            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(RobotState);


        // std::cout<<"len"<<std::endl;
        // socklen_t len = sizeof(cliaddr);
        // ssize_t n = recvfrom(sockfd, buffer, BUFFER_SIZE, 0, (struct sockaddr*)&cliaddr, &len);
        // if (n == -1) {
        //     std::cerr << "接收数据时发生错误: " << strerror(errno) << "\n";
        //     continue;
        // }

        // // 处理接收到的数据
        // // 假设每个浮点数占4字节
        // if (n % 4 != 0) {
        //     std::cerr << "接收到的数据长度不是浮点数的整数倍\n";
        //     continue;
        // }

        // // 逐个解析浮点数
        // for (int i = 0; i < n; i += 4) {
        //     uint32_t byteRepresentation = (static_cast<uint32_t>(static_cast<unsigned char>(buffer[i])) << 24) |
        //                                   (static_cast<uint32_t>(static_cast<unsigned char>(buffer[i + 1])) << 16) |
        //                                   (static_cast<uint32_t>(static_cast<unsigned char>(buffer[i + 2])) << 8) |
        //                                   static_cast<uint32_t>(static_cast<unsigned char>(buffer[i + 3]));
        //     float value;
        //     memcpy(&value, &byteRepresentation, sizeof(float));
        //     values.push_back(value);
        // }

        // // 每次解析完一组浮点数后打印
        // std::cout << "接收到 " << values.size() << " 个浮点数：";
        // for (float val : values) {
        //     std::cout << val << " ";
        // }
        // std::cout << std::endl;

            // inverse kinematics
            fe_l_pos_L_des<<receivedData[12], receivedData[13], receivedData[14];
            fe_r_pos_L_des<<receivedData[18], receivedData[19], receivedData[20];
            fe_l_eul_L_des<<receivedData[15], receivedData[16], receivedData[17];
            fe_r_eul_L_des<<receivedData[21], receivedData[22], receivedData[23];
            fe_l_rot_des= eul2Rot(fe_l_eul_L_des(0),fe_l_eul_L_des(1),fe_l_eul_L_des(2));  // roll, pitch, yaw
            fe_r_rot_des= eul2Rot(fe_r_eul_L_des(0),fe_r_eul_L_des(1),fe_r_eul_L_des(2));

            hd_l_pos_L_des<<receivedData[0], receivedData[1], receivedData[2];
            hd_r_pos_L_des<<receivedData[6], receivedData[7], receivedData[8];
            hd_l_eul_L_des<<receivedData[3], receivedData[4], receivedData[5];
            hd_r_eul_L_des<<receivedData[9], receivedData[10], receivedData[11];



            // hd_l_pos_L_des<<-0.02, 0.32, -0.159;
            // hd_r_pos_L_des<<-0.02, -0.32, -0.159;
            // hd_l_eul_L_des<<-1.7581, 0.2129, 2.9581;
            // hd_r_eul_L_des<<1.7581, 0.21291, -2.9581;
            hd_l_rot_des= eul2Rot(hd_l_eul_L_des(0),hd_l_eul_L_des(1),hd_l_eul_L_des(2));
            hd_r_rot_des= eul2Rot(hd_r_eul_L_des(0),hd_r_eul_L_des(1),hd_r_eul_L_des(2));

            resLeg=kinDynSolver.computeInK_Leg(fe_l_rot_des,fe_l_pos_L_des,fe_r_rot_des,fe_r_pos_L_des);
            resHand=kinDynSolver.computeInK_Hand(hd_l_rot_des,hd_l_pos_L_des,hd_r_rot_des,hd_r_pos_L_des);

            // Enter here functions to send actuator commands, like:
            // arm-l: 0-6, arm-r: 7-13, head: 14,15 waist: 16-18, leg-l: 19-24, leg-r: 25-30
            // get the final joint command
            RobotState.motors_pos_des= eigen2std(resLeg.jointPosRes+resHand.jointPosRes);
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
            logger.recItermData("motors_pos_cur",RobotState.motors_pos_cur);
            logger.recItermData("left_arm_pos",hd_l_pos_L_des);
            logger.recItermData("left_arm_eul",hd_l_eul_L_des);
            logger.recItermData("right_arm_pos",hd_r_pos_L_des);
            logger.recItermData("right_arm_eul",hd_r_eul_L_des);
            // logger.recItermData("motors_pos_des",RobotState.motors_pos_des);
            // logger.recItermData("motors_tau_cur",RobotState.motors_tor_out);
            // logger.recItermData("motors_vel_cur",RobotState.motors_vel_cur);
            // logger.recItermData("motors_vel_des",RobotState.motors_vel_des);
            logger.finishLine();


        // 清空已解析的浮点数以准备下一组
        // values.clear();


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