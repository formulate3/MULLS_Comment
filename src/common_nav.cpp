// By Yue Pan
#include "common_nav.h"

namespace lo
{
    //对tz赋值为0
    bool Navigation::zupt_simple(Eigen::Matrix4d &Trans, float stop_hor_dis_thre, float fix_hor_dis_thre)
    {
        // 为x和y方向上距离进行开平方，最后将z方向上的距离置为0
        float hor_displacement = std::sqrt(Trans(0, 3) * Trans(0, 3) + Trans(1, 3) * Trans(1, 3));
        // if (hor_displacement < stop_hor_dis_thre)
        // {
        //     LOG(INFO) << "Apply ZUPT";
        //     Trans(2, 3) = 0.0;
        //     if (hor_displacement < fix_hor_dis_thre)
        //         Trans.setIdentity();

        //     return true;
        // }

        Trans(2, 3) = 0.0;

        return true;
    }

    //使用队列的后几帧计算平均速度，后几帧的数量由mean_frame_num决定
    //队列中存储的是相邻两个frame的相对位姿
    double Navigation::cal_velocity(Matrix4ds &Trans, int mean_frame_num, int frame_per_second)
    {
        int mean_frame_num_used;

        mean_frame_num_used = min_(mean_frame_num, Trans.size());

        double accumulate_tx = 0, accumulate_ty = 0, accumulate_tz = 0;

        int count_sum = 0;

        for (auto iter = Trans.end() - 1; iter >= Trans.end() - mean_frame_num_used; iter--)
        {
            //LOG(INFO) << *iter;
            Eigen::Matrix4d tempMat = *iter;
            //simple implement
            accumulate_tx += tempMat(0, 3);
            accumulate_ty += tempMat(1, 3);
            accumulate_tz += tempMat(2, 3);
            count_sum++;
        }

        double vx = accumulate_tx / mean_frame_num_used * frame_per_second;
        double vy = accumulate_ty / mean_frame_num_used * frame_per_second;
        double vz = accumulate_tz / mean_frame_num_used * frame_per_second;

        double mean_linear_velocity = std::sqrt(vx * vx + vy * vy + vz * vz);

        LOG(INFO) << "current approximate velocity: " << mean_linear_velocity * 3.6 << " (km/h)\n";

        return mean_linear_velocity;
    }

    //提取4×4位姿矩阵中的translation的norm
    double Navigation::cal_translation_from_tranmat(Eigen::Matrix4d &tran_mat)
    {
        Eigen::Vector3d translation_vec;
        translation_vec = tran_mat.block<3, 1>(0, 3);
        return (translation_vec.norm());
    }

    //提取4×4位姿矩阵中的yaw角度
    double Navigation::cal_heading_deg_from_tranmat(Eigen::Matrix4d &tran_mat)
    {
        Eigen::Vector3d euler_angle = (tran_mat.block<3, 3>(0, 0)).eulerAngles(0, 1, 2); //rotation axis : z,y',x''

        double roll_deg = std::abs(euler_angle(0) / M_PI * 180.0);
        double pitch_deg = std::abs(euler_angle(1) / M_PI * 180.0);
        double yaw_deg = std::abs(euler_angle(2) / M_PI * 180.0);

        if (roll_deg > 90)
            roll_deg = 180 - roll_deg;
        if (pitch_deg > 90)
            pitch_deg = 180 - pitch_deg;
        if (yaw_deg > 90)
            yaw_deg = 180 - yaw_deg;

        return yaw_deg;
    }

    //提取4×4位姿矩阵中的旋转矩阵中的轴角中的角度
    double Navigation::cal_rotation_deg_from_tranmat(Eigen::Matrix4d &tran_mat)
    {
        Eigen::AngleAxisd rs(tran_mat.block<3, 3>(0, 0));

        double rotation_deg = std::abs(rs.angle()) * 180.0 / M_PI;
        return rotation_deg;
    }

    //TODO add the ground based extended kalman filter (ekf)

    //TODO add the scene flow (tracking of the active objects)

} // namespace lo