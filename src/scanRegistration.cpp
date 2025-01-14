// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <cmath>
#include <vector>
#include <string>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using std::atan2;
using std::cos;
using std::sin;

const double scanPeriod = 0.1;//一帧点云的扫描周期

const int systemDelay = 0; //跳过多少帧雷达数据
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 0;//扫描线数 16 KITTI为64
float cloudCurvature[400000];//点云曲率：相邻十个点分别计算三个方向的位移和，然后平方和作为曲率
int cloudSortInd[400000];//特征提取时排序用的中间变量
int cloudNeighborPicked[400000];//点是否筛选过标志：0-未筛选过，1-筛选过
// Label 2: corner_sharp
// Label 1: corner_less_sharp, 包含Label 2
// Label -1: surf_flat
// Label 0: surf_less_flat， 包含Label -1，因为点太多，最后会降采样
int cloudLabel[400000];
///利用曲率进行排序，很细节啊！（曲率小的排在前面）
bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

//一系列话题发布
ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubRemovePoints;
std::vector<ros::Publisher> pubEachScan;

bool PUB_EACH_LINE = false;

double MINIMUM_RANGE = 0.1; //去除太近的点云的距离阈值，launch文件给出的是0.3米，KITTI设置为5点（防止雷达检测到车辆本体）

///以下为LHH定义的变量
int iPointCloudIndex = -1;
int iPointIndex = -1;

/**
 * @brief 去除雷达坐标系下点云附近距离小于thres的点，并瘦身cloud.points
 * 
 * @tparam PointT 
 * @param cloud_in 
 * @param cloud_out 
 * @param thres 
 */
template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                                  pcl::PointCloud<PointT> &cloud_out, float thres)
{
    //如果发现入参和出参不是一个东西，就为出参声明一下空间，提升效率
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z 
            < thres * thres)
            continue;//跳过距离过近的点
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}
/**
 * @brief 激光雷达回调函数
 * 
 * @param laserCloudMsg 原始输入点云
 */
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    iPointCloudIndex++;//点云ID++
    //跳过开头systemDelay帧雷达数据
    if (!systemInited)
    { 
        systemInitCount++;
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        else
            return;
    }

    TicToc t_whole;
    TicToc t_prepare;
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;//PCL格式原始点云
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    std::vector<int> indices;

    // Step 首先对点云滤波，去除NaN值的无效点云
    // std::cout << "is dense = " << laserCloudIn.is_dense << std::endl;///输出显示，点云本身是稠密的，所有没有NaN点的
    // std::cout << "laserCloudIn num is " << laserCloudIn.size() << std::endl;
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);//该函数并不会减小size
    // std::cout << "new laserCloudIn num is " << laserCloudIn.size() << std::endl;
    // Step 去除在Lidar坐标系原点MINIMUM_RANGE距离以内的点
    // std::cout << "laserCloudIn num is " << laserCloudIn.size() << std::endl;
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);///实验发现，距离小于0.3米的点相当多，接近一半的样子（上万个）
    // std::cout << "new laserCloudIn num is " << laserCloudIn.size() << std::endl;

    //计算起始点和结束点的角度，由于激光雷达是顺时针旋转，这里取反就相当于逆时针了
    int cloudSize = laserCloudIn.points.size();//当前帧点云点数
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    //atan2范围是[-PI, PI]，这里加上了2PI是为了保证起始到结束相差2PI符合实际
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    //printf("end Ori %f\n", endOri);
    // if(iPointCloudIndex == 5) {
    //     std::cout << "startOri = " << startOri << std::endl;
    //     std::cout << "endOri = " << endOri << std::endl;
    // }

    bool halfPassed = false;
    int count = cloudSize;
    PointType point;//正在处理的点云中的某点
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    iPointIndex = -1;
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;//此时雷达坐标系是前左上
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;//该点的仰角///通过俯仰角确定是第几根线
        int scanID = 0;

        if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);//角度应该约为-15° -13°……，所以这里先加15转为正数，然后除以二就得到了0-15共16个索引
            //但这样有个问题是原本连续的线序，不再连续了
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 64)
        {   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies 
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);
        iPointIndex++;//某点ID++
        float ori = -atan2(point.y, point.x);//当前点水平面角度
        // if(iPointCloudIndex == 5 && iPointIndex % 15 == 0) {
        //     std::cout << iPointIndex << "ori = " << ori << std::endl;
        // }
        if (!halfPassed)
        { 
            //确保-PI / 2 < ori - startOri < 3 / 2 * PI
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            //这种case不会发生
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }
            //如果超过180度，就说明过了一半了
            if (ori - startOri > M_PI)
            {
                halfPassed = true;
                std::cout << "halfPassed = true" << std::endl;
            }
        }
        else
        {
            //确保-PI * 3 / 2 < ori - endOri < PI / 2
            ori += 2 * M_PI; //先补偿2PI
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }
        // if(iPointCloudIndex == 5 && iPointIndex % 15 == 0) {
        //     std::cout << iPointIndex << "ori = " << ori << std::endl;
        // }
        //角度的计算是为了计算相对的起始时刻
        float relTime = (ori - startOri) / (endOri - startOri);
        point.intensity = scanID + scanPeriod * relTime;//赋值intensity为线号+局部时间戳
        //根据索引值送入数据到各自的数组中
        laserCloudScans[scanID].push_back(point); 
    }
    
    cloudSize = count;
    printf("points size %d \n", cloudSize);

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());//一维索引存储的当前帧点云
    // Step 记录各线扫描的起始和终止索引
    for (int i = 0; i < N_SCANS; i++)
    { 
        scanStartInd[i] = laserCloud->size() + 5;// 记录每个scan的开始index，忽略前5个点
        *laserCloud += laserCloudScans[i];//重载了运算符，更新PointCloud类型
        scanEndInd[i] = laserCloud->size() - 6;// 记录每个scan的结束index，忽略后5个点，开始和结束处的点云scan容易产生不闭合的“接缝”，对提取edge feature不利
    }

    // printf("prepare time %f \n", t_prepare.toc());

    for (int i = 5; i < cloudSize - 5; i++)
    { 
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;// 点有没有被选选择为feature点
        cloudLabel[i] = 0;// Label 2: corner_sharp
                          // Label 1: corner_less_sharp, 包含Label 2
                          // Label -1: surf_flat
                          // Label 0: surf_less_flat， 包含Label -1，因为点太多，最后会降采样
    }


    TicToc t_pts;

    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> cornerPointsLessSharp;
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> surfPointsLessFlat;//降采样后的较多平面点

    float t_q_sort = 0;
    for (int i = 0; i < N_SCANS; i++)// 按照scan的顺序提取4种特征点
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)// 如果该scan的点数少于6个点，就跳过///因为后面要六等分均匀化
            continue;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);//降采样之前的较多平面点
        for (int j = 0; j < 6; j++)// 将该scan分成6小段执行特征检测（论文中是分成4份）
        {
            ///取出各扫描线的首尾，防止在交界处提取特征
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;// subscan的起始index
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;// subscan的结束index

            TicToc t_tmp;
            std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, comp);// 根据曲率有小到大对subscan的点进行sort（注意排序对象是个数组）
            t_q_sort += t_tmp.toc();

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)// 从后往前，即从曲率大的点开始提取corner feature
            {
                int ind = cloudSortInd[k]; //正在遍历的点的索引

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1)// 如果该点没有被选择过，并且曲率大于0.1//?这个曲率的阈值是根据什么取的
                {
                    largestPickedNum++;
                    if (largestPickedNum <= 2)// 该subscan中曲率最大的前2个点认为是corner_sharp特征点
                    {                        
                        cloudLabel[ind] = 2;
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else if (largestPickedNum <= 20)// 该subscan中曲率最大的前20个点认为是corner_less_sharp特征点
                    {                        
                        cloudLabel[ind] = 1; 
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;// 标记该点被选择过了

                    // 与当前点距离的平方 <= 0.05的点标记为选择过，避免特征点密集分布
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        //索引上邻近的点如果距离较远，则放过，也就是不进行标记，防止滥杀无辜
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }///至此完成边缘点和较多边缘点的提取

            // 提取surf平面feature，与上述类似，选取该subscan曲率最小的前4个点为surf_flat
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {
                    //这里仅提取平面点
                    cloudLabel[ind] = -1; 
                    surfPointsFlat.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    //这里不区分平面点和较多平面点，因为其余标志位默认为0，即较多平面点（除了边缘点和平面点，剩下的都标记为较多平面点：0）
                    if (smallestPickedNum >= 4)
                    { 
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    //同样标记附近点被选择过，防止特征聚集
                    for (int l = 1; l <= 5; l++)
                    { 
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }//至此完成平面点的提取

            
            // 其他的非corner特征点与surf_flat特征点一起组成surf_less_flat特征点
            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)//!注意，这里并没有考虑选取的点是否经过筛选，而是直接把所有非边缘点全部放进来了
                {///这样可能出现把部分曲率大的点也认为是较多平面点了
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }//至此完成分成6段的特征提取

        // 最后对该scan点云中提取的所有surf_less_flat特征点进行降采样，因为点太多了
        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
    printf("sort q time %f \n", t_q_sort);
    printf("seperate points time %f \n", t_pts.toc());

    //四种特征点云发布出去
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "/camera_init";
    pubLaserCloud.publish(laserCloudOutMsg);

    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "/camera_init";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);

    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "/camera_init";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "/camera_init";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "/camera_init";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

    // pub each scam
    if(PUB_EACH_LINE)
    {
        for(int i = 0; i< N_SCANS; i++)
        {
            sensor_msgs::PointCloud2 scanMsg;
            pcl::toROSMsg(laserCloudScans[i], scanMsg);
            scanMsg.header.stamp = laserCloudMsg->header.stamp;
            scanMsg.header.frame_id = "/camera_init";
            pubEachScan[i].publish(scanMsg);
        }
    }

    printf("scan registration time %f ms *************\n", t_whole.toc());
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanRegistration");//初始化ros节点，并指定名称
    ros::NodeHandle nh;//创建ros句柄

    nh.param<int>("scan_line", N_SCANS, 16);//将参数服务器中的scan_line赋值给N_SCANS，若参数服务器没有定义该变量，使用后边的默认值

    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);

    printf("scan line number %d \n", N_SCANS);

    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

    if(PUB_EACH_LINE)
    {
        for(int i = 0; i < N_SCANS; i++)
        {
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
    }
    ros::spin();

    return 0;
}
