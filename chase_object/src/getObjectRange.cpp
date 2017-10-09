/*
Modified by: Kehinde Aina Sept. 9, 2017
*/
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath> 

// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>

#include "std_msgs/String.h"

using namespace std;
//using namespace message_filters;

int j=0;
int k=0;
int m =0;
volatile bool distanceSet = false;
volatile bool angleSet = false;

geometry_msgs::Point32 ballPoint;

//get the distance between two points
void pubMeasurement(ros::Publisher _ballDirection_pub)
{
    geometry_msgs::Twist base_cmd;

    if (1) //distanceSet && angleSet
    {
        double kp = 0.005;
        double _z = ballPoint.z;

        //turn before you move forward
        if(_z <-3 || _z > 3)
        {
            if(_z>=20)
            {
                base_cmd.angular.z = 20*kp; 
            }
            else if(_z<=-20) 
            {
                base_cmd.angular.z = -20*kp;
            }
            else
            {
                base_cmd.angular.z = _z*kp;
            }
            
           // base_cmd.linear.y = base_cmd.linear.x  = 0;
            _ballDirection_pub.publish(base_cmd);
          
          distanceSet = false;
          angleSet = false;
          ROS_INFO("sequence:%d, angular vel:%f", ++m, base_cmd.angular.z);          
         // return;
        }
        else
        {
            base_cmd.angular.z =base_cmd.linear.y = base_cmd.linear.x  = 0;
            _ballDirection_pub.publish(base_cmd);            
            distanceSet = false;
            angleSet = false;
            ROS_INFO("sequence:%d, angular vel:%f", ++m, base_cmd.angular.z); 
        }

        kp= -0.09;
        double _x = ballPoint.x;
        if(_x>=0)
        {
            if(_x<=0.4 && _x >0.1) 
            {
                base_cmd.linear.x  = (0.4-_x)*kp;
            }
            else if(_x>=0.45)
            {
                //if(_x>1.0) _x = 1.0;
                base_cmd.linear.x = (0.45-_x)*kp;
            }
            else    base_cmd.linear.x = 0;

            //base_cmd.linear.y = base_cmd.angular.z = 0;

            distanceSet = false;
            angleSet = false;

            _ballDirection_pub.publish(base_cmd);
            ROS_INFO("sequence:%d, linear vel:%f", ++m, base_cmd.linear.x);

            //return;
        }      
        //ros::Duration(0.25).sleep();   //sampling time
    }
    else
    {
        distanceSet = false;
        angleSet = false;

        base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
        _ballDirection_pub.publish(base_cmd);  
    }    
}


void distanceCallback(const sensor_msgs::LaserScanConstPtr& _laserData)
{
    if(1) //angleSet && !distanceSe
    {
        double arrIndex = ballPoint.z;
        int _index;
        double muarr[3];
        int _size = _laserData->ranges.size();        

        if(arrIndex<-1)
        {            
            _index = 359 - (int)arrIndex;
            //muarr[0] = _laserData->ranges[_index-2];
            muarr[0] = _laserData->ranges[_index-1];
            muarr[1] = _laserData->ranges[_index];
            muarr[2] = _laserData->ranges[_index+1];
            //muarr[4] = _laserData->ranges[_index+2];
        }
        else if(arrIndex>1)
        {
            _index = (int)arrIndex;
            //muarr[0] = _laserData->ranges[_index-2];
            muarr[0] = _laserData->ranges[_index-1];
            muarr[1] = _laserData->ranges[_index];
            muarr[2] = _laserData->ranges[_index+1];
            //muarr[4] = _laserData->ranges[_index+2];
        }
        else
        {
            _index =0;
            //muarr[0] = _laserData->ranges[358];
            muarr[0] = _laserData->ranges[359];
            muarr[1] = _laserData->ranges[0];
            muarr[2] = _laserData->ranges[1];
            //muarr[4] = _laserData->ranges[2];
        }

        double min_val = muarr[1]; 

        if(min_val >1.0 || min_val == 0)
        {
            min_val = 0;
            for(int t=0;t<3;t++)
            {
                if(muarr[t] > min_val && muarr[t] <0.8)
                {
                    min_val = muarr[t];
                    _index = t;
                }
            }
        }


        // if(min_val <= 0 || min_val > 1)
        // {
        //     min_val = 0;
        //     for(int l=lowerThresh;l<upperThresh;l++)
        //     {
        //         if(_laserData->ranges[l] > min_val && _laserData->ranges[l] <1)
        //         {
        //             min_val = _laserData->ranges[l];
        //             _index = l;
        //         }
        //     }
        // }

        ballPoint.x = min_val;
        distanceSet = true;
        ROS_INFO("Sequence:[%d], min:%f, index:%d",++j,min_val,_index); 
        //return;
    }
    // double min_val = std::min_element(_laserData->ranges[0],_laserData->ranges[_size-1]);
    // int _index = std::distance(_laserData->ranges, min_val);
//    ROS_INFO("Sequence:[%d], length:%d, min:%f, xval:%d, angle:%f",++j,_size,min_val,_x, _xAngle);
}

void angularCallback(const geometry_msgs::Point32ConstPtr& _pointData)
{
    if (1) //angleSet
    {       
        ballPoint.z = _pointData->z;

        int _x = _pointData->x;
        double _xAngle = _pointData->z; 
        angleSet = true;
       // ROS_INFO("Sequence:[%d], x:%d, angle:%f",++k,_x,_xAngle);        
    }
    // double min_val = std::min_element(_laserData->ranges[0],_laserData->ranges[_size-1]);
    // int _index = std::distance(_laserData->ranges, min_val);
//    ROS_INFO("Sequence:[%d], length:%d, min:%f, xva:%d, angle:%f",++j,_size,min_val,_x, _xAngle);
    // ROS_INFO("Sequence:[%d], x:%d, angle:%f",++k,_x,_xAngle);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "getObjectRange");

  ros::NodeHandle nh;
  
//   message_filters::Subscriber<geometry_msgs::Point> distance_sub(nh, "ball_direction", 1);
//   message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, "scan", 1);  

//   TimeSynchronizer<Point32, LaserScan> sync(distance_sub, laser_sub, 10);
//   sync.registerCallback(boost::bind(&callback, _1, _2));

//   typedef sync_policies::ApproximateTime<geometry_msgs::Point, sensor_msgs::LaserScan> MySyncPolicy;
//   // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
//   Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), distance_sub, laser_sub);
//   sync.registerCallback(boost::bind(&publishErrorCallback, _1, _2));

  ros::Subscriber sub2 = nh.subscribe("ball_direction", 1, angularCallback);  
  //ros::Duration(0.25).sleep();   //sampling time
  ros::Subscriber sub = nh.subscribe("scan", 1, distanceCallback);

  ros::Publisher distAngle_publish = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Rate loop_rate(1);

//   ros::Publisher distAngle_publish = nh.advertise<geometry_msgs::Point32>("/distance_angle", 10);  
//   ros::Rate loop_rate(1);
  //ros::Subscriber sub = nh.subscribe("/raspicam_node/image/compressed", 10, getImageCallback);
  //ros::Publisher ballDirection_pub = nh.advertise<geometry_msgs::Point32>("ball_direction", 10);
  
  while (ros::ok())
  {
    pubMeasurement(distAngle_publish);
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}


