#ifndef TF2_LOOKUP_SERVER_GEOMETRY_H
#define TF2_LOOKUP_SERVER_GEOMETRY_H

#include <Eigen/Eigen>
#include <geometry_msgs/msg/transform_stamped.hpp>

using TransformStamped = geometry_msgs::msg::TransformStamped;


namespace tf2_lookup_server
{
    // https://github.com/fizyr/dr_eigen/blob/main/include/dr_eigen/average.hpp (accessed: 27/03/22)
    template<typename DataType, typename ForwardIterator>
    Eigen::Quaternion<DataType> averageQuaternions(ForwardIterator const & begin, ForwardIterator const & end) {
        if (begin == end) {
            throw std::logic_error("Cannot average orientations over an empty range.");
        }

        Eigen::Matrix<DataType, 4, 4> A = Eigen::Matrix<DataType, 4, 4>::Zero();
        uint sum(0);
        for (ForwardIterator it = begin; it != end; ++it) {
            Eigen::Matrix<DataType, 1, 4> q(1,4);
            q(0) = it->w();
            q(1) = it->x();
            q(2) = it->y();
            q(3) = it->z();
            A += q.transpose()*q;
            sum++;
        }
        A /= sum;

        Eigen::EigenSolver<Eigen::Matrix<DataType, 4, 4>> es(A);

        Eigen::Matrix<std::complex<DataType>, 4, 1> mat(es.eigenvalues());
        int index;
        mat.real().maxCoeff(&index);
        Eigen::Matrix<DataType, 4, 1> largest_ev(es.eigenvectors().real().block(0, index, 4, 1));

        return Eigen::Quaternion<DataType>(largest_ev(0), largest_ev(1), largest_ev(2), largest_ev(3));
    }


    TransformStamped mean(const std::vector<TransformStamped>& stfs)
    {
        TransformStamped avg;

        if(stfs.size() == 1) {
            avg = stfs.front();
        } else if(stfs.size() > 1) {
            // header
            auto first_stamp = rclcpp::Time(stfs.front().header.stamp);
            auto last_stamp = rclcpp::Time(stfs.back().header.stamp);
            auto dur_ns = (last_stamp - first_stamp).nanoseconds();
            auto dur_half = rclcpp::Duration::from_nanoseconds(dur_ns / 2);
            avg.header.stamp = first_stamp + dur_half;
            avg.header.frame_id = stfs.front().header.frame_id;
            avg.child_frame_id = stfs.front().child_frame_id;

            // gather translations, quaternions
            Eigen::Vector3d translation; translation.setZero();
            std::vector<Eigen::Quaterniond> quaternions;
            for(const auto& stf : stfs) {
                translation += Eigen::Vector3d(
                    stf.transform.translation.x,
                    stf.transform.translation.y,
                    stf.transform.translation.z
                );

                quaternions.push_back(
                    Eigen::Quaterniond(
                        stf.transform.rotation.w,
                        stf.transform.rotation.x,
                        stf.transform.rotation.y,
                        stf.transform.rotation.z
                    )
                );
            }

            // average translation
            translation /= double(stfs.size());
            avg.transform.translation.x = translation.x();
            avg.transform.translation.y = translation.y();
            avg.transform.translation.z = translation.z();

            // average quaternion
            auto avg_quat = averageQuaternions<double>(
                quaternions.begin(),
                quaternions.end()
            );
            avg.transform.rotation.x = avg_quat.x();
            avg.transform.rotation.y = avg_quat.y();
            avg.transform.rotation.z = avg_quat.z();
            avg.transform.rotation.w = avg_quat.w();
        }

        return avg;
    }
}


#endif