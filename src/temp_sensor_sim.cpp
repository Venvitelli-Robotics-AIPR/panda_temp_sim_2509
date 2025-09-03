/*
    temp_sensor_sim node

    Copyright 2022-2025 Università degli studi della Campania "Luigi Vanvitelli"
    Author: Marco Costanzo <marco.costanzo@unicampania.it>
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <sensor_msgs/msg/temperature.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("temp_sensor_sim");

  auto pub =
      nh->create_publisher<sensor_msgs::msg::Temperature>("/temperature", 1);

  rclcpp::Rate loop_rate(1000);
  rclcpp::Time t0 = nh->now();

  double N[] = {2, 0.5, 0.05};
  double w[] = {2 * M_PI / 0.02, 2 * M_PI / 0.005, 2 * M_PI / 0.002};

  double T[] = {30, 100, 150};
  double TEMP_INTERVAL = 10;

  while (rclcpp::ok()) {

    loop_rate.sleep();
    // ros::spinOnce();

    auto t = (nh->now() - t0).seconds();

    auto cycle_t = std::fmod(t, 3 * TEMP_INTERVAL);

    double temp =
        N[0] * cos(w[0] * t) + N[1] * cos(w[1] * t) + N[2] * cos(w[2] * t);

    if (cycle_t < TEMP_INTERVAL) {
      temp += T[0];
    } else if (cycle_t < 2 * TEMP_INTERVAL) {
      temp += T[1];
    } else {
      temp += T[2];
    }

    sensor_msgs::msg::Temperature temp_msg;

    temp_msg.header.stamp = nh->now();
    temp_msg.header.frame_id = "sensor";

    temp_msg.temperature = temp;

    pub->publish(temp_msg);
  }

  return 0;
};
