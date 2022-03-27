#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_msgs/action/lookup_transform.hpp>

#include "tf2_lookup_server/geometry.hpp"

using namespace std::placeholders;
using LookupTransform = tf2_msgs::action::LookupTransform;
using GoalHandleLookupTransform = rclcpp_action::ServerGoalHandle<LookupTransform>;


class TF2LookupServer : public rclcpp::Node
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TF2LookupServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) :
            Node("tf2_lookup_server", options)
        {
            double buffer_duration_s;
            this->declare_parameter<double>("buffer_duration", 10.0);
            this->get_parameter<double>("buffer_duration", buffer_duration_s);
            tf2::Duration buffer_duration(uint64_t(buffer_duration_s * 1e9));

            int hz;
            this->declare_parameter<int>("hz", 50);
            this->get_parameter<int>("hz", hz);
            loop_sleep_duration_ = new tf2::Duration(uint64_t(1e9 / hz));

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), buffer_duration);
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            this->action_server_ = rclcpp_action::create_server<LookupTransform>(
                this,
                "tf_lookup",
                std::bind(&TF2LookupServer::handle_goal, this, _1, _2),
                std::bind(&TF2LookupServer::handle_cancel, this, _1),
                std::bind(&TF2LookupServer::handle_accepted, this, _1)
            );
        }

    private:
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const LookupTransform::Goal> goal)
        {
            (void)uuid;
            (void)goal;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleLookupTransform> goal_handle)
        {
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleLookupTransform> goal_handle)
        {
            std::thread{std::bind(&TF2LookupServer::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleLookupTransform> goal_handle)
        {
            const auto goal = goal_handle->get_goal();
            auto result = std::make_shared<LookupTransform::Result>();

            // temporal window
            auto current_time = this->get_clock()->now();
            auto start_time = rclcpp::Time(goal->source_time, current_time.get_clock_type());
            auto lookup_duration = rclcpp::Duration(goal->timeout);
            rclcpp::Time timeout_time;
            if(start_time.nanoseconds() == 0) {
                timeout_time = current_time + lookup_duration;
                if(goal->advanced) { start_time = current_time; }
            }
            else { timeout_time = start_time + lookup_duration; }

            if(timeout_time < start_time) {
                auto st_copy = rclcpp::Time(start_time);
                start_time = rclcpp::Time(timeout_time);
                timeout_time = st_copy;
            }

            // buffer for averaging
            std::vector<TransformStamped> stfs; stfs.clear();
            auto try_aggregate_stfs = [&](const rclcpp::Time& time) {
                TransformStamped stf;
                if(this->lookup(goal->target_frame, goal->source_frame, time, stf)) {
                    stfs.push_back(stf);
                }
            };

            if(goal->advanced && (start_time < current_time)) {
                // aggregate from past
                auto past_lookup_time = rclcpp::Time(start_time);
                while(past_lookup_time < current_time) {
                    try_aggregate_stfs(past_lookup_time);
                    past_lookup_time += *loop_sleep_duration_;
                }
            }

            // lookup loop
            while(rclcpp::ok()) {
                current_time = this->get_clock()->now();

                if(goal_handle->is_canceling()) {
                    // cancel triggered
                    result->error.error = result->error.LOOKUP_ERROR;
                    result->error.error_string = "Canceled";
                    goal_handle->canceled(result);
                    return;
                } else if(current_time > timeout_time) {
                    // timeout exceeded
                    if(goal->advanced) {
                        if(!stfs.empty()) {
                            // averaging: success
                            result->transform = tf2_lookup_server::mean(stfs);
                            result->error.error = result->error.NO_ERROR;
                            result->error.error_string.clear();
                            goal_handle->succeed(result);
                        } else {
                            // averaging: fail
                            result->error.error = result->error.LOOKUP_ERROR;
                            result->error.error_string = "No TF data within temporal window";
                            goal_handle->abort(result);
                        }
                    } else {
                        // one-shot: fail
                        result->error.error = result->error.TIMEOUT_ERROR;
                        result->error.error_string = "Timeout";
                        goal_handle->abort(result);
                    }
                    return;
                } else {
                    if(goal->advanced) {
                        // averaging: attempt aggregation
                        try_aggregate_stfs(current_time);
                    } else {
                        // one-shot: attempt lookup
                        if(this->lookup(goal->target_frame, goal->source_frame, start_time, result->transform)) {
                            result->error.error = result->error.NO_ERROR;
                            result->error.error_string.clear();
                            goal_handle->succeed(result);
                            return;
                        }
                    }
                }

                rclcpp::sleep_for(*loop_sleep_duration_);
            }
        }

        bool lookup(
            const std::string& target_frame,
            const std::string& source_frame,
            const rclcpp::Time& time,
            TransformStamped& stf)
        {
            try {
                auto stf_res = tf_buffer_->lookupTransform(target_frame, source_frame, time);
                stf.header = stf_res.header;
                stf.child_frame_id = stf_res.child_frame_id;
                stf.transform = stf_res.transform;
                return true;
            } catch (tf2::TransformException & ex) {
                (void)ex;
                return false;
            }
        }

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp_action::Server<LookupTransform>::SharedPtr action_server_;
        tf2::Duration *loop_sleep_duration_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TF2LookupServer>());
    rclcpp::shutdown();
    return 0;
}