#include <template.h>

#include <clocale>
#include <cstdlib>
#include <iostream>

using std::cout;
using std::endl;

// 全局变量定义
int mission_num = 0;
float if_debug = 0;
float err_max = 0.2;
void print_param()
{
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
  std::cout << "if_debug: " << if_debug << std::endl;
  if(if_debug == 1) cout << "自动offboard" << std::endl;
  else cout << "遥控器offboard" << std::endl;
}


int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  // 初始化ROS节点
  ros::init(argc, argv, "template");
  ros::NodeHandle nh;

  // 订阅mavros相关话题
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  // 发布无人机多维控制话题
  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

  // 创建服务客户端
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
  ros::Rate rate(20);

  // 参数读取

  nh.param<float>("err_max", err_max, 0.15f);
  nh.param<float>("if_debug", if_debug, 0);
  double circle_radius;
  int circle_points;
  nh.param("circle_radius", circle_radius, 1.0);
  nh.param("circle_points", circle_points, 12);
  if (!std::isfinite(circle_radius) || !std::isfinite(err_max) || err_max <= 0 ||
      circle_radius <= 0 || circle_points < 3 ||
      circle_radius * std::sin(M_PI / circle_points) <= std::sqrt(2.0) * err_max) {
    ROS_ERROR("Invalid circle geometry or overlapping waypoint tolerances");
    return 1;
  }
  print_param();

  
  int choice = 0;
  std::cout << "1 to go on , else to quit" << std::endl;
  std::cin >> choice;
  if (choice != 1) return 0;
  ros::spinOnce();
  rate.sleep();
  
  // 在发布目标之前，从第一个可用的里程计信息中获取起始位置。
  while (ros::ok() && (!current_state.connected || !flag_init_position)) {
    ros::spinOnce();
    rate.sleep();
  }

  enum class Phase { Takeoff, Circle, Hover, Land };
  Phase phase = Phase::Takeoff;
  int waypoint = 0;
  ros::Time last_request = ros::Time::now();
  ros::Time hover_start;
  mavros_msgs::SetMode mode;
  mavros_msgs::CommandBool arm;
  arm.request.value = true;

  while (ros::ok()) {
    ros::spinOnce();
    const auto now = ros::Time::now();
    if (phase == Phase::Land) {
      if (!current_state.armed && std::abs(local_pos.pose.pose.position.z - init_position_z_take_off) < 0.25) {
        ROS_INFO("LANDED disarmed, relative height %.3f m", local_pos.pose.pose.position.z - init_position_z_take_off);
        return 0;
      }
      if (current_state.mode != "AUTO.LAND" && (now - last_request).toSec() > 1.0) {
        mode.request.custom_mode = "AUTO.LAND";
        set_mode_client.call(mode);
        last_request = now;
      }
    } else if (phase == Phase::Takeoff) {
      const bool arrived = mission_pos_cruise(0, 0, ALTITUDE, 0, err_max);
      if (current_state.mode == "OFFBOARD" && current_state.armed && arrived) {
        phase = Phase::Circle;
        ROS_INFO("TAKEOFF complete, entering circle");
      } else if ((now - last_request).toSec() > 3.0) {
        if (current_state.mode != "OFFBOARD") {
          if (if_debug == 1) {
            mode.request.custom_mode = "OFFBOARD";
            set_mode_client.call(mode);
          }
        } else if (!current_state.armed) {
          arming_client.call(arm);
        }
        last_request = now;
      }
    } else if (phase == Phase::Circle) {
      const double angle = 2.0 * M_PI * waypoint / circle_points;
      if (mission_pos_cruise(circle_radius * std::cos(angle), circle_radius * std::sin(angle),
                             ALTITUDE, 0, err_max)) {
        const auto& p = local_pos.pose.pose.position;
        ROS_INFO("CIRCLE %d/%d actual %.3f %.3f %.3f", waypoint, circle_points,
                 p.x - init_position_x_take_off, p.y - init_position_y_take_off,
                 p.z - init_position_z_take_off);
        if (waypoint == circle_points) {
          phase = Phase::Hover;
          hover_start = now;
        } else {
          ++waypoint;
        }
      }
    } else {
      mission_pos_cruise(circle_radius, 0, ALTITUDE, 0, err_max);
      if ((now - hover_start).toSec() >= 3.0) {
        phase = Phase::Land;
        ROS_INFO("HOVER complete, requesting AUTO.LAND");
      }
    }
    setpoint_raw.header.stamp = now;
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    rate.sleep();
  }
  return 0;
}
