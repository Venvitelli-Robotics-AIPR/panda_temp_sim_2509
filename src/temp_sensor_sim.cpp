/*
    temp_sensor_sim node

    Copyright 2022 Università degli studi della Campania "Luigi Vanvitelli"
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

#include <ros/ros.h>

#include <cmath>
#include <sensor_msgs/Temperature.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "temp_sensor_sim");

  ros::NodeHandle nh;

  ros::Publisher pub =
      nh.advertise<sensor_msgs::Temperature>("/temperature", 1);

  ros::Rate loop_rate(1000);
  ros::Time t0 = ros::Time::now();

  double N[] = {2, 0.5, 0.05};
  double w[] = {2 * M_PI / 0.02, 2 * M_PI / 0.005, 2 * M_PI / 0.002};

  double T[] = {30, 100, 150};
  double TEMP_INTERVAL = 10;

  while (ros::ok()) {

    loop_rate.sleep();
    ros::spinOnce();

    auto t = (ros::Time::now() - t0).toSec();

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

    sensor_msgs::Temperature temp_msg;

    temp_msg.header.stamp = ros::Time::now();
    temp_msg.header.frame_id = "sensor";

    temp_msg.temperature = temp;

    pub.publish(temp_msg);
  }

  return 0;
};
