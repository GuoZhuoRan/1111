#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <tinyxml2.h>
#include <map>

using namespace tinyxml2;

class KinematicChain {
public:
    std::vector<std::string> chain_l;
    std::vector<std::string> chain_r;
    std::vector<std::string> chain_waist;
    std::map<std::string, Eigen::Matrix4d> transformations;
    std::map<std::string, Eigen::Vector3d> axes;

    Eigen::Matrix4d left_base_to_Link_arm_l_01;
    Eigen::Matrix4d right_base_to_Link_arm_r_01;
    Eigen::Matrix4d Link_arm_l_01_to_left_tool;
    Eigen::Matrix4d Link_arm_r_01_to_right_tool;
    Eigen::Matrix4d Link_arm_l_01_to_left_tool_only;
    Eigen::Matrix4d Link_arm_r_01_to_right_tool_only;
    Eigen::Matrix4d world_to_Link_waist_yaw_only;

    KinematicChain(const std::string& xml_path) {
        initializeChains();
        parseJointTransformationsAndAxes(xml_path);
    }

    void initializeChains() {

        left_base_to_Link_arm_l_01 << 0.0, 1.0, 0.0, 0.004,
                                      0.0, 0.0, 1.0, 0.2026,
                                      1.0, 0.0, 0.0, 0.3922,
                                      0.0, 0.0, 0.0, 1.0;

        right_base_to_Link_arm_r_01 << 0.0, -1.0, 0.0, 0.004,
                                       0.0, 0.0, -1.0, -0.2026,
                                       1.0, 0.0, 0.0, 0.3922,
                                       0.0, 0.0, 0.0, 1.0;

        Link_arm_l_01_to_left_tool << 0.0, 0.0, -1.0, 0.0,
                                      -1.0, 0.0, 0.0, 0.0,
                                      0.0, 1.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 1.0;

        Link_arm_r_01_to_right_tool << 0.0, 0.0, -1.0, 0.0,
                                       1.0, 0.0, 0.0, 0.0,
                                       0.0, -1.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 1.0;

        Link_arm_l_01_to_left_tool_only  << -1.0000000, -0.0000000,  0.0000000, 0.0,
                                            0.0000000, -1.0000000,  0.0000000, 0.0,
                                            0.0000000,  0.0000000,  1.0000000, 0.0,
                                            0.0, 0.0, 0.0, 1.0;

        Link_arm_r_01_to_right_tool_only << -1.0000000, -0.0000000,  0.0000000, 0.0,
                                            0.0000000, -1.0000000,  0.0000000, 0.0,
                                            0.0000000,  0.0000000,  1.0000000, 0.0,
                                            0.0, 0.0, 0.0, 1.0;

        world_to_Link_waist_yaw_only << 1.0000000, -0.0000000,  0.0000000, 0.0,
                                            0.0000000, 1.0000000,  0.0000000, 0.0,
                                            0.0000000,  0.0000000,  1.0000000, 1.1,
                                            0.0, 0.0, 0.0, 1.0;

        chain_l = {"Link_arm_l_01", "Link_arm_l_02", "Link_arm_l_03", "Link_arm_l_04", "Link_arm_l_05", "Link_arm_l_06", "Link_arm_l_07"};
        chain_r = {"Link_arm_r_01", "Link_arm_r_02", "Link_arm_r_03", "Link_arm_r_04", "Link_arm_r_05", "Link_arm_r_06", "Link_arm_r_07"};
        chain_waist = {"Link_waist_yaw", "Link_waist_roll", "Link_waist_pitch", "base_link"};
    }

    Eigen::Matrix4d quaternionToMatrix(const Eigen::Vector4d& quat) {
        double w = quat(0), x = quat(1), y = quat(2), z = quat(3);
        Eigen::Matrix4d mat;
        mat << 1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y, 0,
               2 * x * y + 2 * w * z, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * w * x, 0,
               2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x * x - 2 * y * y, 0,
               0, 0, 0, 1;
        return mat;
    }

    Eigen::Matrix4d posToMatrix(const Eigen::Vector3d& pos) {
        Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
        mat(0, 3) = pos(0);
        mat(1, 3) = pos(1);
        mat(2, 3) = pos(2);
        return mat;
    }

    void parseJointTransformationsAndAxes(const std::string& xml_path) {
        XMLDocument doc;
        if (doc.LoadFile(xml_path.c_str()) != XML_SUCCESS) {
            std::cerr << "Error loading XML file: " << xml_path << std::endl;
            return;
        }

        XMLElement* root = doc.RootElement();
        if (!root) {
            std::cerr << "Root element not found in XML file" << std::endl;
            return;
        }

        XMLElement* worldbody = root->FirstChildElement("worldbody");
        if (!worldbody) {
            std::cerr << "<worldbody> element not found" << std::endl;
            return;
        }

        // Process each <body> element under <worldbody>
        for (XMLElement* base_body = worldbody->FirstChildElement("body"); base_body != nullptr; base_body = base_body->NextSiblingElement("body")) {
            ProcessBodyElements(base_body);
            
        }
    }

    void ProcessBodyElements(XMLElement* body) {
        const char* body_name = body->Attribute("name");
        std::string body_name_str(body_name);

        // Handle body position and orientation
        Eigen::Vector3d body_position(0.0, 0.0, 0.0);
        Eigen::Vector4d body_quaternion(1.0, 0.0, 0.0, 0.0);

        const char* pos_str = body->Attribute("pos");
        const char* quat_str = body->Attribute("quat");
        if (pos_str) {
            sscanf(pos_str, "%lf %lf %lf", &body_position(0), &body_position(1), &body_position(2));
        }
        if (quat_str) {
            sscanf(quat_str, "%lf %lf %lf %lf", &body_quaternion(0), &body_quaternion(1), &body_quaternion(2), &body_quaternion(3));
        }
        Eigen::Matrix4d T_body = posToMatrix(body_position) * quaternionToMatrix(body_quaternion);
        // Process joint elements within this body
        for (XMLElement* joint = body->FirstChildElement("joint"); joint != nullptr; joint = joint->NextSiblingElement("joint")) {
            const char* joint_name = joint->Attribute("name");
            const char* joint_axis = joint->Attribute("axis");
            const char* joint_range = joint->Attribute("range");
            const char* joint_pos_str = joint->Attribute("pos");
            const char* joint_quat_str = joint->Attribute("quat");

            Eigen::Vector3d joint_position(0.0, 0.0, 0.0);
            Eigen::Vector4d joint_quaternion(1.0, 0.0, 0.0, 0.0);

            if (joint_pos_str) {
                sscanf(joint_pos_str, "%lf %lf %lf", &joint_position(0), &joint_position(1), &joint_position(2));
            }
            if (joint_quat_str) {
                sscanf(joint_quat_str, "%lf %lf %lf %lf", &joint_quaternion(0), &joint_quaternion(1), &joint_quaternion(2), &joint_quaternion(3));
            }

            Eigen::Matrix4d T_joint = T_body * posToMatrix(joint_position) * quaternionToMatrix(joint_quaternion);


        if (!(std::find(chain_l.begin(), chain_l.end(), body_name_str) == chain_l.end() &&
            std::find(chain_r.begin(), chain_r.end(), body_name_str) == chain_r.end() &&
            std::find(chain_waist.begin(), chain_waist.end(), body_name_str) == chain_waist.end()))
        // if (!(std::find(chain_l.begin(), chain_l.end(), body_name_str) == chain_l.end() &&
        //     std::find(chain_r.begin(), chain_r.end(), body_name_str) == chain_r.end()))
        {
            if (joint_name) {
                transformations[body_name] = T_joint;
            }

            Eigen::Vector3d axis(0.0, 0.0, 0.0);
            if (joint_axis) {
                sscanf(joint_axis, "%lf %lf %lf", &axis(0), &axis(1), &axis(2));
                axes[body_name] = axis;
            } else {
                axes[body_name] = Eigen::Vector3d::Zero();
            }
        }

        }

        // Recursively process nested body elements
        for (XMLElement* child_body = body->FirstChildElement("body"); child_body != nullptr; child_body = child_body->NextSiblingElement("body")) {
            ProcessBodyElements(child_body);

        }
    }

    Eigen::Matrix4d calculateEndEffectorPose(const std::vector<std::string>& chain, const std::vector<double>& joint_angles) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        for (size_t i = 0; i < chain.size(); ++i) {
            const std::string& joint = chain[i];
            
            if (transformations.find(joint) != transformations.end()) {
                T *= transformations[joint];
                if (axes.find(joint) != axes.end() && axes[joint] != Eigen::Vector3d::Zero()) {
                    T *= getHomogeneousTransform(axes[joint], joint_angles[i]);
                }
            }
        }
        return T;
    }

    Eigen::Matrix4d getHomogeneousTransform(const Eigen::Vector3d& axis, double angle) {
        Eigen::Matrix3d R = axisAngleToRotationMatrix(axis, angle);
        return rotationMatrixToHomogeneous(R);
    }

    Eigen::Matrix3d axisAngleToRotationMatrix(const Eigen::Vector3d& axis, double angle) {
        Eigen::Vector3d normalized_axis = axis.normalized();
        double x = normalized_axis(0), y = normalized_axis(1), z = normalized_axis(2);
        double c = cos(angle), s = sin(angle);
        double t = 1 - c;

        Eigen::Matrix3d R;
        R << t * x * x + c, t * x * y - s * z, t * x * z + s * y,
             t * x * y + s * z, t * y * y + c, t * y * z - s * x,
             t * x * z - s * y, t * y * z + s * x, t * z * z + c;

        return R;
    }

    Eigen::Matrix4d rotationMatrixToHomogeneous(const Eigen::Matrix3d& R) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = R;
        return T;
    }

Eigen::Vector3d matrix3dToEulerAnglesZYX(Eigen::Matrix3d m3dr) {
	double beta_y=atan2(m3dr(0,2),sqrt(m3dr(0,0)*m3dr(0,0)+m3dr(0,1)*m3dr(0,1)));
	double alpha_z=atan2(-m3dr(0,1)/cos(beta_y),m3dr(0,0)/cos(beta_y));
	double gamma_x=atan2(-m3dr(1,2)/cos(beta_y),m3dr(2,2)/cos(beta_y));

	if(abs(beta_y - M_PI/2) < 10e-4)
	{
		gamma_x = 0;
		alpha_z = atan2(m3dr(1,0),m3dr(1,1));
	}
 if(abs(beta_y + M_PI/2) < 10e-4)
	{
		gamma_x = 0;
		alpha_z = atan2(m3dr(1,0),m3dr(1,1));
	}
	if (gamma_x>M_PI)
	{
		gamma_x=gamma_x-2*M_PI;
	}
	if (gamma_x<-M_PI)
	{
		gamma_x=gamma_x+2*M_PI;
	}
	if (beta_y>M_PI)
	{
		beta_y=beta_y-2*M_PI;
	}
	if (beta_y<-M_PI)
	{
		beta_y=beta_y+2*M_PI;
	}
	if (alpha_z>M_PI)
	{
		alpha_z=alpha_z-2*M_PI;
	}
	if (alpha_z<-M_PI)
	{
		alpha_z=alpha_z+2*M_PI;
	}

return Eigen::Vector3d(gamma_x, beta_y, alpha_z);

}

Eigen::Matrix3d eulerZYXToRotationMatrix_nei(double alpha, double beta, double gamma) {
    // 构造旋转矩阵
    Eigen::Matrix3d rotationMatrix;
    
    // 计算旋转矩阵
    rotationMatrix = Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX());

    return rotationMatrix;
}

Eigen::Matrix3d eulerZYXToRotationMatrix_wai(double alpha, double beta, double gamma) {
    // 构造旋转矩阵
    Eigen::Matrix3d rotationMatrix;
    
    // 计算旋转矩阵
    rotationMatrix = Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()) *
                     Eigen::AngleAxisd(beta, Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ());

    return rotationMatrix;
}


    std::vector<double> calculateLeftPose(const std::vector<double>& left_joint_angles) {
        Eigen::Matrix4d end_effector_pose_l = calculateEndEffectorPose(chain_l, left_joint_angles);
        // Eigen::Matrix4d left_Tt = left_base_to_Link_arm_l_01.inverse() * end_effector_pose_l * Link_arm_l_01_to_left_tool;
        Eigen::Matrix4d left_Tt = end_effector_pose_l * Link_arm_l_01_to_left_tool * Link_arm_l_01_to_left_tool_only;
        Eigen::Vector3d ros_left = matrix3dToEulerAnglesZYX(left_Tt.block<3, 3>(0, 0));
        Eigen::Vector3d pos_left = left_Tt.block<3, 1>(0, 3);

        return {pos_left(0), pos_left(1), pos_left(2), ros_left(0), ros_left(1), ros_left(2)};
    }

    std::vector<double> calculateRightPose(const std::vector<double>& right_joint_angles) {
        Eigen::Matrix4d end_effector_pose_r = calculateEndEffectorPose(chain_r, right_joint_angles);
        // Eigen::Matrix4d right_Tt = right_base_to_Link_arm_r_01.inverse() * end_effector_pose_r * Link_arm_r_01_to_right_tool;
        Eigen::Matrix4d right_Tt = end_effector_pose_r * Link_arm_r_01_to_right_tool * Link_arm_r_01_to_right_tool_only;
        Eigen::Vector3d ros_right = matrix3dToEulerAnglesZYX(right_Tt.block<3, 3>(0, 0));
        Eigen::Vector3d pos_right = right_Tt.block<3, 1>(0, 3);
        // std::cout<<right_Tt<<std::endl;
        return {pos_right(0), pos_right(1), pos_right(2), ros_right(0), ros_right(1), ros_right(2)};
    }

    std::vector<double> calculateWaistPose(const std::vector<double>& waist_joint_angles) {
        Eigen::Matrix4d end_effector_pose_waist = calculateEndEffectorPose(chain_waist, waist_joint_angles);
        // Eigen::Matrix4d right_Tt = right_base_to_Link_arm_r_01.inverse() * end_effector_pose_r * Link_arm_r_01_to_right_tool;
        Eigen::Matrix4d waist_Tt =  end_effector_pose_waist ;
        Eigen::Vector3d ros_waist = matrix3dToEulerAnglesZYX(waist_Tt.block<3, 3>(0, 0));
        Eigen::Vector3d pos_waist = waist_Tt.block<3, 1>(0, 3);
        // std::cout<<ros_waist<<std::endl;
        return {pos_waist(0), pos_waist(1), pos_waist(2), ros_waist(0), ros_waist(1), ros_waist(2)};
    }
    
};
