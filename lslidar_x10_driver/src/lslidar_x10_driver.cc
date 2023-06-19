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

#include <string>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <lslidar_x10_driver/lslidar_x10_driver.h>

namespace lslidar_x10_driver {

LslidarX10Driver::LslidarX10Driver(
        ros::NodeHandle& n, ros::NodeHandle& pn):
    nh(n),
    pnh(pn){
    return;
}

LslidarX10Driver::~LslidarX10Driver() {
    return;
}

void LslidarX10Driver::Interface_selection() {
    // 如果launch文件没有设置就用默认值net
    pnh.param("interface_selection", interface_selection, std::string("net"));
	return;
} 
// 读取参数订阅启停发布到ros
void LslidarX10Driver::initParam() {
    // 创建一个线程
    // inline 是 C++ 中的关键字，表示将函数声明为内联函数。内联函数是一种编译期间的优化技术，可以在函数被调用的地方进行代码替换，而不是实际调用函数。这种替换可以避免函数调用的开销，从而提高程序的效率。通常情况下，内联函数的代码较短，执行时间较短，且会在程序多次调用该函数时使用，否则可能会适得其反，导致程序效率降低。
    // 成员函数引用传进来
    // &LslidarX10Driver::pubScanThread创建一个线程pubscan_thread_新线程入口
    pubscan_thread_ = new boost::thread(boost::bind(&LslidarX10Driver::pubScanThread, this));
    // 订阅lslidar_order话题队列长度为1 使用回调函数当有数据时候就进行处理
	difop_switch = nh.subscribe<std_msgs::Int8>("lslidar_order", 1 ,&LslidarX10Driver::lidar_order,this);          //转速输入
    std::string frame_id = "laser";
    std::string scan_topic = "/scan";
    std::string lidar_name_ = "M10";
    is_start = true;
    // 参数读取
    pnh.param("lidar_name", lidar_name, lidar_name_);
    pnh.param("frame_id", frame_id_, frame_id);
    pnh.param("scan_topic", scan_topic_, scan_topic);
    pnh.param<double>("min_distance",min_distance,0);
    pnh.param<double>("max_distance",max_distance,30);
    pnh.param<bool>("use_gps_ts", use_gps_ts, false);
    pnh.param("truncated_mode", truncated_mode_, 0);
    pnh.param<std::vector<int>>("disable_min", disable_angle_min_range, {0});
    pnh.param<std::vector<int>>("disable_max", disable_angle_max_range, {0});
    // 读取参数的句柄进行读取没有就用默认值

    angle_able_min = 0;
    angle_able_max = 360;
    count_num = 0;
    // 扫描点数据信息的容器每个容器内存放的是一个结构体
//     typedef struct {
//     double degree;
//     double range;
//     double intensity;
// } ScanPoint;
    scan_points_.resize(4000);
        
    if (lidar_name == "M10"){
        use_gps_ts == false;
        PACKET_SIZE = 92;
        package_points = 42;
        data_bits_start = 6;
        degree_bits_start = 2;
        rpm_bits_start = 4;
        baud_rate_= 460800;
        points_size_ = 1008;
        printf("Lidar is M10 \n");
    }
    else if (lidar_name == "M10_P"){
        PACKET_SIZE = 160;
        package_points = 70;
        data_bits_start = 8;
        degree_bits_start = 4;
        rpm_bits_start = 6;
        baud_rate_=500000;
        points_size_ = 1680;
        printf("Lidar is M10_P \n");
    }
    else if (lidar_name == "M10_PLUS")
    {
        PACKET_SIZE = 104;
        package_points = 41;
        data_bits_start = 8;
        degree_bits_start = 4;
        rpm_bits_start = 6;
        points_size_ = 4000;
        baud_rate_=921600;
        printf("Lidar is M10_PLUS ! \n");
    }
    else if (lidar_name == "M10_TEST")
    {
        PACKET_SIZE = 176;
        package_points = 84;
        data_bits_start = 6;
        degree_bits_start = 2;
        rpm_bits_start = 4;
        baud_rate_= 460800;
        points_size_ = 2000; 
        use_gps_ts = false;
        printf("Lidar is M10 10K 5HZ \n");
    }
    else if (lidar_name == "N10")
    {
        // 激光雷达数据包的大小
        PACKET_SIZE = 58;
        // 成员属性每个数据包包含的点数
        package_points = 16;
        // 数据位的起始位置
        data_bits_start = 7;
        // 角度数据位的起始位置
        degree_bits_start = 5;
        // 角度数据位的结束位置
        end_degree_bits_start = 55;
        // 串口通信的波特率
        baud_rate_= 230400;
        // 缓冲区中点的数量
        points_size_ = 500; 
        // 是否使用 GPS 时间戳
        use_gps_ts = false;
        printf("Lidar is N10 ! \n");
    }
    if (lidar_name == "M10_GPS"){
        PACKET_SIZE = 102;
        package_points = 42;
        data_bits_start = 6;
        degree_bits_start = 2;
        rpm_bits_start = 4;
        baud_rate_= 460800;
        points_size_ = 1008;
        printf("Lidar is M10_GPS \n");
    }
	return;
}


// 主要作用是接收启停激光雷达的命令，并向雷达发送相应的控制指令。
void LslidarX10Driver::lidar_order(const std_msgs::Int8 msg) {
    int i = msg.data;
    if( i == 0) is_start = false;
    else        is_start = true;
    if(interface_selection == "net") msop_input_->UDP_order(msg);
    else{
        int i = msg.data;
    for(int k = 0 ; k <10 ; k++)
    {   
		int rtn;
        char data[188]= {0x00};
        data[0] = 0xA5;
        data[1] = 0x5A;
        data[2] = 0x55;
        data[186] = 0xFA;
        data[187] = 0xFB;

		if(lidar_name == "M10" || lidar_name == "M10_TEST" || lidar_name == "M10_GPS"){
            if (i <= 1){				    //雷达启停
				data[184] = 0x01;
				data[185] = char(i);
			}
			else if (i == 2){			    //雷达点云不滤波
				data[181] = 0x0A;
				data[184] = 0x06;
				if(is_start) data[185] = 0x01;
			}
			else if (i == 3){				//雷达点云正常滤波
				data[181] = 0x0B;
				data[184] = 0x06;
				if(is_start) data[185] = 0x01;
			}
			else if (i == 4){				//雷达近距离滤波
				data[181] = 0x0C;
				data[184] = 0x06;
				if(is_start) data[185] = 0x01;
			}       
            else return; 
		}
		else if(lidar_name == "M10_P"){
            if(i <= 1){
            data[185] = char(i);
            data[184] = 0x01;
            }
            else return;
		}
		else if (lidar_name == "M10_PLUS"){
            data[184] = 0x0A;
            data[185] = 0x01;
            if(i == 5)       {
            data[141] = 0x01;
            data[142] = 0x2c;
            }
            else if(i == 6)  {
            data[141] = 0x01;
            data[142] = 0x68;
            }
            else if(i == 8)  {
            data[141] = 0x01;
            data[142] = 0xe0;
            }
            else if(i == 10) {
            data[141] = 0x02;
            data[142] = 0x58;
            }
            else if(i == 12) {
            data[141] = 0x02;
            data[142] = 0xd0;
            }
            else if(i == 15) {
            data[141] = 0x03;
            data[142] = 0x84;
            }
            else if(i == 20) {
            data[141] = 0x04;
            data[142] = 0xb0;
            }
            else if(i <= 1)  {
            data[184] = 0x01;
            data[185] = char(i);
            }			
            else return; 
		}
		else if(lidar_name == "N10"){
            if(i <= 1){
            data[185] = char(i);
            data[184] = 0x01;
            }
            else if(i>=6 && i<=12){
            data[172] = char(i);
            data[184] = 0x0a;
            data[185] = 0X01;
            }
            else return; 
		}
        rtn= serial_->send((const char*)data, 188);
		if (rtn < 0)
			printf("start scan error !\n");
        else{
            if(i == 1)  usleep(3000000);
            if(i == 0)  is_start = false;
            if(i == 1)  is_start = true;
            return ;
        }   
    }
    return ;
    }
}

void LslidarX10Driver::open_serial()
{
    int code = 0;
    std::string port = "/dev/ttyUSB0";
    // 读取ros端口号如果没有设置默认值为/dev/ttyUSB0
    pnh.param("serial_port", serial_port_, port);
    // lsiosr类的单例模式，保证该类在程序运行的时候只有一个实例对象，返回本类
    serial_ = LSIOSR::instance(serial_port_, baud_rate_);
    // 初始化函数返回错误代码
    code = serial_->init();
    // 说明返回的代码错误串口打开失败
    if(code != 0)
    {
        printf("open_port %s ERROR !\n", serial_port_.c_str());
        ros::shutdown();
        exit(0);
    }
    // 成功了的话打印一句串口打开成功的话
    printf("open_port %s  OK !\n", serial_port_.c_str());
    //recv_thread_ = new boost::thread(boost::bind(&LSX10::recvThread, this));
}

bool LslidarX10Driver::createRosIO() {
  pnh.param<int>("device_port", UDP_PORT_NUMBER, 2368);
  ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
  // ROS diagnostics
  diagnostics.setHardwareID("Lslidar_X10");

  const double diag_freq = 12*24;
  diag_max_freq = diag_freq;
  diag_min_freq = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
	  diag_topic.reset(new TopicDiagnostic(
						 "lslidar_packets", diagnostics,
						 FrequencyStatusParam(&diag_min_freq, &diag_max_freq, 0.1, 10),
						 TimeStampStatusParam()));

	int hz = 10;
    if (lidar_name == "M10_P")  hz = 12;
    else if (lidar_name == "M10_PLUS")  hz = 20;
    
	double packet_rate = hz*24;
    pnh.param("pcap",dump_file,std::string(""));
    if(dump_file !="")
    {
        msop_input_.reset(new lslidar_x10_driver::InputPCAP(pnh,UDP_PORT_NUMBER,packet_rate,dump_file));
    }else{
        msop_input_.reset(new lslidar_x10_driver::InputSocket(pnh,UDP_PORT_NUMBER));
    }

    // Output
	return true;
}

int LslidarX10Driver::getScan(std::vector<ScanPoint> &points, ros::Time &scan_time, float &scan_duration)
{
  boost::unique_lock<boost::mutex> lock(mutex_);
//   scan_points_bak_赋值给points
  points.assign(scan_points_bak_.begin(), scan_points_bak_.end());
//   扫描时间赋值
  scan_time = pre_time_;
//   扫描周期赋值
  scan_duration = (time_ - pre_time_).toSec();
  return 1;
}
// 主要作用是将一个 tm 结构体表示的时间转化为 GPS 时间戳。在 GPS 时间戳中，以 GPS 时间周（week）为单位计数
uint64_t LslidarX10Driver::get_gps_stamp(struct tm t){

   uint64_t ptime =static_cast<uint64_t>(timegm(&t));
   return ptime;
}
// 驱动初始化
bool LslidarX10Driver::initialize() {
    // 读取串口型号（launch）文件中
    Interface_selection();
    // 初始化参数返回值为空
    initParam();
    // 先不看吧
    if(interface_selection == "net"){
        if (!createRosIO()) {
            ROS_ERROR("Cannot create all ROS IO...");
            return false;
        }
    }
    else
    {
        open_serial();
    }
    // 发布雷达消息话题名称为/scan 队列长度为3
    pub_ = nh.advertise<sensor_msgs::LaserScan>(scan_topic_, 3);
    // 打印初始成功
    ROS_INFO("Initialised lslidar x10 without error");
    return true;
}
// 主要作用是在每次接收到数据时对数据进行 CRC 校验，并对连接状态进行检测和维护
// 传入读取的字节数目 链接时长 30
void LslidarX10Driver::recvThread_crc(int &count,int &link_time)
{
    // 前面如果成功了那么就是1
	if(count <= 0) 
		link_time++;
	else
		link_time = 0;
	
	if(link_time > 150)
	{
        // 一直没有数据的话 链接时长一直在增加 增加到一定程度后先关闭串口再初始化如果初始化失败就打印串口初始化失败并停机0.2秒重置链接时长
		serial_->close();
		int ret = serial_->init();
		if(ret < 0)
		{
            ROS_WARN("serial open fail");
			usleep(200000);
		}
		link_time = 0;
	}
}

int LslidarX10Driver::receive_data(unsigned char *packet_bytes){
    // 对数据指针进行操作
    int link_time = 0;
    int q = 0;
    int len_H = 0;
    int len_L = 0;
    int len = 0;
    int count_2 = 0;
    int count = 0;
    // 此代码分两次读取，第一次读取首字节后面全部读完
    while(count <= 0)
    {
        // 返回读取的状态
        // serial_雷达串口类
        // 1表示想要在这个函数读取的字符串数组数目返回实际读取的字符串数目
        count = serial_->read(packet_bytes, 1);
        LslidarX10Driver::recvThread_crc(count,link_time);
    }
    // 首字节不对的话退出系统
    // 第一个字符串应该是校验为
    if(packet_bytes[0] != 0xA5)	return 0;

    while(count_2 <= 0)
    {
        // 读取新的字节 返回这次读取的字节数
        count_2 = serial_->read(packet_bytes+count, 1);
        // 两次读取的总字节数
        if(count_2 >= 0) count += count_2;
        // 校验一下
        LslidarX10Driver::recvThread_crc(count_2,link_time);
    }
    // 重新置0
    count_2 = 0;
    // 第二个字符串应该是数长度
    if(packet_bytes[1] != 0x5A)	return 0;
    while(count_2 <= 0)
    {
        count_2 = serial_->read(packet_bytes+count, 2);
        if(count_2 >= 0) count += count_2;
        LslidarX10Driver::recvThread_crc(count_2,link_time);
    }
    // 重置为0
    count_2 = 0;
    // 命名空间,类,成员属性
    if(lidar_name == "M10")                     len = 92;
    else if(lidar_name == "M10_TEST")           len = 176;
    else if(lidar_name == "M10_GPS")            len = 102;
    // n10第一个数据是没用的第二个数据是表示长度
    else if(lidar_name == "N10")                len = packet_bytes[2];
    else
    {
        len_H = packet_bytes[2];
        len_L = packet_bytes[3];
        len = len_H*256 + len_L;
    }

    while(count < len)
    {
        // 全读完
        count_2 = serial_->read(packet_bytes+count, len-count);
        if(count_2 >= 0) count += count_2;
        LslidarX10Driver::recvThread_crc(count_2,link_time);
    }
    q = len;
    if(lidar_name == "N10") 
    {
        // 校验失败返回0
        if(packet_bytes[PACKET_SIZE-1] != N10_CalCRC8(packet_bytes, PACKET_SIZE-1))						return 0;
    }
    return q;
}

uint8_t LslidarX10Driver::N10_CalCRC8(unsigned char * p, int len)
{
  uint8_t crc = 0;
  int sum = 0;

  for (int i = 0; i < len; i++)
  {
    sum += uint8_t(p[i]);
  }
  crc = sum & 0xff;
  return crc;
}
// 处理一个数据包中的字节容器指针，传入字节容器和对应的字节数
// n10一个数据包有16个点信息 78 1011 1314这样子
// 01帧头 2长度 34转速 56起始角度 7-54点云数据 5556结束角度 57 校验值
void LslidarX10Driver::data_processing(unsigned char *packet_bytes,int len)                                 //处理每一包的数据
{
    double degree;
    double end_degree;
    double degree_interval = 15.0;
    boost::posix_time::ptime t1,t2;
    // 时间
    t1 = boost::posix_time::microsec_clock::universal_time();

        // // 激光雷达数据包的大小
        // PACKET_SIZE = 58;
        // // 成员属性每个数据包包含的点数有点就有数据
        // package_points = 16;
        // // 数据位的起始位置
        // data_bits_start = 7;
        // // 角度数据位的起始位置
        // degree_bits_start = 5;
        // // 角度数据位的结束位置
        // end_degree_bits_start = 55;
        // // 串口通信的波特率
        // baud_rate_= 230400;
        // // 缓冲区中点的数量
        // points_size_ = 500; 
        // // 是否使用 GPS 时间戳
        // use_gps_ts = false;
        // printf("Lidar is N10 ! \n");
        // 初始化中针对n10雷达进行的设置

    int s = packet_bytes[degree_bits_start];
	int z = packet_bytes[degree_bits_start + 1];
    // 角度存储通过两个字节进行存储 分为高8位和低8位除100变成实际值
    // 32处理发过来的数据包，遵循一些格式
	degree = (s * 256 + z) / 100.f;
    // 转过了减回来
	degree = (degree > 360) ? degree-360 : degree;
    if(lidar_name == "N10") 
    {
		int s_e = packet_bytes[end_degree_bits_start];
		int z_e = packet_bytes[end_degree_bits_start+1];
		end_degree = (s_e * 256 + z_e) / 100.f;    
        end_degree = (end_degree > 360) ? end_degree-360 : end_degree;    

		if(degree > end_degree)
        // 起始角度和结束角度的间隔
			degree_interval = end_degree + 360 - degree;
		else
			degree_interval = end_degree - degree;	
    }

    //boost::unique_lock<boost::mutex> lock(mutex_);
    if (lidar_name == "M10_PLUS"||lidar_name == "M10_P")
    {
        PACKET_SIZE = len;
        package_points = (PACKET_SIZE - 20)/2;
	}
    int invalidValue = 0;
    int point_len = 2;
    if(lidar_name == "N10")     point_len = 3;

    if(lidar_name == "M10_GPS"||lidar_name == "M10")
    {
		int err_data_84 = packet_bytes[84];
		int err_data_85 = packet_bytes[85];
        if((err_data_84 * 256 + err_data_85) == 0xFFFF || packet_bytes[86] >= 0xF5)
        {
            packet_bytes[86] = 0xFF;
            packet_bytes[87] = 0xFF;
        }
    }
    // 3乘16
	for (int num = 0; num < point_len*package_points; num+=point_len)
	{
        // 默认都是1有数据过来的时候就会改变数据位
		int s = packet_bytes[num + data_bits_start];
		int z = packet_bytes[num + data_bits_start + 1];
		if ((s * 256 + z) == 0xFFFF)
			invalidValue++;
	}
    // 不执行
	if(use_gps_ts && lidar_name != "N10")
	{
		pTime.tm_year 	= packet_bytes[PACKET_SIZE - 12]+2000-1900;	//x+2000
		pTime.tm_mon 	= packet_bytes[PACKET_SIZE - 11]-1;			//1-12
		pTime.tm_mday 	= packet_bytes[PACKET_SIZE - 10];			//1-31
		pTime.tm_hour 	= packet_bytes[PACKET_SIZE - 9];			//0-23
		pTime.tm_min 	= packet_bytes[PACKET_SIZE - 8];			//0-59
		pTime.tm_sec 	= packet_bytes[PACKET_SIZE - 7];			//0-59
		sub_second		= (packet_bytes[PACKET_SIZE - 6]*256+packet_bytes[PACKET_SIZE - 5])*1000000 + (packet_bytes[PACKET_SIZE - 4]*256+packet_bytes[PACKET_SIZE - 3])*1000;		
		sweep_end_time_gps = get_gps_stamp(pTime);
		sweep_end_time_hardware = sub_second%1000000000;
	}
    invalidValue = package_points - invalidValue;
	for (int num = 0; num < package_points; num++)
	{
        // 对这个数据包的点云数据进行处理
        // 读取各个点的信息
		int s = packet_bytes[num*point_len + data_bits_start];
		int z = packet_bytes[num*point_len + data_bits_start + 1];
        int y  = 0;
        // 这个数据没有到
		if(lidar_name == "N10")     y = packet_bytes[num*point_len + data_bits_start+2];
        // 最后7位为深度信息  点云数据包括深度和强度信息
		int dist_temp = s & 0x7F;
        // 第一位是强度信息
		int inten_temp = s & 0x80;
        // 如果有数据
		if ((s * 256 + z) != 0xFFFF)
		{	
            if(lidar_name != "N10")
            {
                // 拿到深度信息
                scan_points_[idx].range = double(dist_temp * 256 + (z)) / 1000.f;
                // 要么是255要么是0
                if (inten_temp)	scan_points_[idx].intensity = 255;
                else 	        scan_points_[idx].intensity = 0;
            }
            else
            {
				scan_points_[idx].range = double(s * 256 + (z)) / 1000.f;
				scan_points_[idx].intensity = int(y);
            }
            if ((degree + (degree_interval / invalidValue * num)) > 360)
            // invalidValue这个是指有效数据只不过懒得重命名了
                scan_points_[idx].degree = degree + (degree_interval / invalidValue * num) - 360;
            else
                scan_points_[idx].degree = degree + (degree_interval / invalidValue * num);
            
		}
        // 都是异常情况要进行处理
        if ((scan_points_[idx].degree < last_degree && scan_points_[idx].degree < 5 && last_degree > 355)|| idx>=points_size_) 	
        {
            last_degree = scan_points_[idx].degree;
            count_num = idx;
            idx = 0;
            for(int k=0;k<scan_points_.size();k++)
            {	
                // 深度数据不合理
                if(scan_points_[k].range < min_distance || scan_points_[k].range > max_distance)
                    scan_points_[k].range = 0;
            }
            boost::unique_lock<boost::mutex> lock(mutex_);
            scan_points_bak_.resize(scan_points_.size());
            scan_points_bak_.assign(scan_points_.begin(), scan_points_.end());
            for(int k=0; k<scan_points_.size(); k++)
            {
                scan_points_[k].range = 0;
                scan_points_[k].degree = 0;
            }
            pre_time_ = time_;
            lock.unlock();
            pubscan_cond_.notify_one();
            time_ = ros::Time::now();
        }
        else
        {
            last_degree = scan_points_[idx].degree;;
            idx++;
        }
        
	}
	//lock.unlock();
    packet_bytes = {0x00};
	
    if (packet_bytes)
    {
        packet_bytes = NULL;
        delete packet_bytes;
    }
}
// 主要作用是将获取到的激光雷达的扫描数据发布到一个 ROS 话题上
// class lslidar_x10_driver::LslidarX10Driver命名空间 类 成员函数
void LslidarX10Driver::pubScanThread()
{
    // 标志位
  bool wait_for_wake = true;
//   锁住
  boost::unique_lock<boost::mutex> lock(pubscan_mutex_);
//   两个类
  ros::Time new_time;
  ros::Time last_time;
  while (ros::ok())
  {
    
    while (wait_for_wake)
    {
        // 锁执行wait方法时候，阻塞等待直到被其他线程唤醒
      pubscan_cond_.wait(lock);
    //   唤醒了标志位置位false
      wait_for_wake = false;
    }
    // 0先过

	if(count_num <= 42 )
		continue;
        //printf("count_num = %d\n",count_num);
    // 扫描点信息结构体是一个容器
    std::vector<ScanPoint> points;
    // ros时间类
    ros::Time start_time;
    float scan_time;
    this->getScan(points, start_time, scan_time);
    count_num=points_size_;
	int scan_num = ceil((angle_able_max-angle_able_min)/360*count_num)+1;
    sensor_msgs::LaserScan msg;
    msg.header.frame_id = frame_id_;
	if (use_gps_ts){
		msg.header.stamp = ros::Time(sweep_end_time_gps, sweep_end_time_hardware);
	}
    else{
		msg.header.stamp = start_time;
	}
    //msg.header.stamp = start_time;
	
	msg.angle_min = - M_PI ;
    msg.angle_max =  M_PI ;

	msg.angle_increment = 2 * M_PI / count_num;
    msg.range_min = min_distance;
    msg.range_max = max_distance;
    msg.ranges.resize(scan_num);
    msg.intensities.resize(scan_num);
    msg.scan_time = scan_time;
    msg.time_increment = scan_time / (double)(count_num - 1);
	
	for(int k=0; k<scan_num; k++){
		msg.ranges[k] = std::numeric_limits<float>::infinity();
        msg.intensities[k] = 0;
	}
    //printf("scan_num = %d\n",scan_num);
	int start_num = floor(0 * count_num / 360);
	int end_num = floor(360 * count_num / 360);

	for (int i = 0; i < count_num; i++) {
		int point_idx = round((360 - points[i].degree) * count_num / 360);
		if(point_idx<(end_num-count_num))
			point_idx += count_num;
		point_idx =  point_idx - start_num;
		if(point_idx < 0 || point_idx >= scan_num) 
			continue;
		if (points[i].range == 0.0) {
			msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
		}
		else {
			double dist = points[i].range;
			msg.ranges[point_idx] = (float) dist;
		}
        if(truncated_mode_==1)
        {
            for (int j = 0; j < disable_angle_max_range.size(); ++j) {
            if ((points[i].degree >= (disable_angle_min_range[j]) ) && (points[i].degree <= (disable_angle_max_range[j]))) {
                msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
                msg.intensities[point_idx] = 0;
              }
            }
        }
	msg.intensities[point_idx] = points[i].intensity;
        if(msg.intensities[point_idx] <=5 && msg.intensities[point_idx] >0)
        {
            msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
            msg.intensities[point_idx] = 0;
        }
        if(lidar_name == "M10" || lidar_name == "M10_P" || lidar_name == "M10_PLUS")
        {
            if(msg.intensities[point_idx] != 0)
            {
                msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
                msg.intensities[point_idx] = 0;
            }
        }
    }

	count_num = 0;
    pub_.publish(msg);
    wait_for_wake = true;
  }
}
// 主要作用是从激光雷达的输入流中获取数据，进行处理并返回结果
// 命名空间类成员函数
bool LslidarX10Driver::polling()
{
    // 状态标志位初始话的时候设置成了true
    if(!is_start) return true;
    // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    // 初始化一个字符串数组指针
     unsigned char * packet_bytes = new unsigned char[400];
     int len = 0;

    if(interface_selection == "net")
    {
        // 本质就是一个自定义消息的1构造函数
    lslidar_x10_msgs::LslidarX10PacketPtr packet(
                new lslidar_x10_msgs::LslidarX10Packet());
	struct timeval tv;
	int last_usec,now_usec;
   
        while (true)
        {
            len = 0;
            // keep reading until full packet received
            len = msop_input_->getPacket(packet);
            if(packet->data[0] == 0x5a && packet->data[1] == 0x00)  
            {
                int len_H = packet->data[1];
                int len_L = packet->data[2];
                len = len_H*256 + len_L;
                for (int i = len-1; i >0; i--)
                {
                    packet->data[i] = packet->data[i-1];
                }
                packet->data[0] = 0xa5;
            }
            
            if(dump_file !="")
            {
                if(lidar_name == "N10")             len = 58;
                else if(lidar_name == "M10")        len = 92;
                else if(lidar_name == "M10_GPS")    len = 102;
                else
                {
                    int len_H = packet->data[2];
                    int len_L = packet->data[3];
                    len = len_H*256 + len_L;
                }
            }
            if(len <= 0||len>500)    continue;
            if(packet->data[0] != 0xa5 && packet->data[1] != 0x5a)                  continue; 
            for (int i = 0; i < len; i++)
            {
                packet_bytes[i] = packet->data[i];
            }
            if(lidar_name == "N10" && packet_bytes[len-1] != N10_CalCRC8(packet_bytes, len-1))                      continue;   
            break;
        }
    }
    else{
        while (true)
        {
            len = 0;
            // 接受字节返回一个整型变量
            len = LslidarX10Driver::receive_data(packet_bytes);
            if(len == 0)    continue;
            break;
        }
        
	}
    // 处理数据 一次接受到的字符串 长度
    LslidarX10Driver::data_processing(packet_bytes,len);
    

    return true;
}

} // namespace lslidar_driver
