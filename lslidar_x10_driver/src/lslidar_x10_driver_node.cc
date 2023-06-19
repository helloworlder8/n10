/*
 * This file is part of lslidar_x10 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <lslidar_x10_driver/lslidar_x10_driver.h>

volatile sig_atomic_t flag = 1;

static void my_handler(int sig)
{
  flag = 0;
}


int main(int argc, char** argv)
{
  // 初始化节点名称
  // 初始化ros系统与ros主节点进行通讯获取当前机器上的信息，节点，话题服务
  // 初始化ros配置环境变量，命名空间。机器人模型，日志
  // 解析读取ros参数服务器上的参数
  // 初始化ros文件系统
  // 创建ros节点句柄？
    ros::init(argc, argv, "lslidar_x10_driver_node");
    // 创建两个大管家
    ros::NodeHandle node;
    // 此句柄可以读取参数
    ros::NodeHandle private_nh("~");

    // start the driver
    // 命名空间 类 实例化 传入两个大管家
    lslidar_x10_driver::LslidarX10Driver driver(node, private_nh);
    // 驱动初始化根据返回结果进行具体操作
  if (!driver.initialize()) {
    ROS_ERROR("Cannot initialize lslidar driver...");
    // 终于初始化成功了
    return 0;
  }
    // loop until shut down or end of file
    while(ros::ok() && driver.polling()) {
        ros::spinOnce();

    }

    return 0;
}
