#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/os/Property.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <string>
#include <iostream>

class testModule : public yarp::os::RFModule
{
public:
  yarp::dev::PolyDriver transClient;
  yarp::dev::IFrameTransform *iframetrans;
    
  yarp::os::Property prop;
  
  bool transformsExist = false;
    
  double getPeriod()
  {
    return 0;
  }
  
  bool transformTest(yarp::sig::Matrix& T, yarp::sig::Vector &in, const std::string& source_frame,  yarp::sig::Vector &out, const std::string& target_frame, bool transformExists)
  {
    bool ok;
    yInfo() << " \n \n-----------------------------------------------------------------------------------";
    if (!transformExists)
    {
      ok = iframetrans->setTransformStatic(target_frame, source_frame, T);
      if (!ok)
      {
        yError() << "Cannot set transform";
        return false;
      }
    }
    yarp::os::Time::delay(1);
    
    ok = iframetrans->frameExists(target_frame);
    if (!ok)
    {
      yError() << target_frame << " frame does not exist";
      return false;
    }
    
    ok = iframetrans->frameExists(source_frame);
    if (!ok)
    {
      yError() << source_frame << " frame does not exist";
      return false;
    }
    
//     yarp::sig::Matrix Tout(4, 4);
//     ok = iframetrans->getTransform(target_frame, source_frame, Tout);
//     if (!ok)
//     {
//       yError() << "oops! no transform ?";
//       return false;
//     }
//     yInfo() << "Transform from " << source_frame << " to " << target_frame << " set to: \n" << Tout.toString();
    yInfo() << "Point in " << source_frame << " :" << in.toString();
    
    ok = iframetrans->transformPoint(target_frame, source_frame, in, out);
    if (!ok)
    {
      yError() << "transform point failed";
      return false;
    }
    
    yInfo() << "Transformed point in " << target_frame << " :" << out.toString();
    
    yInfo() << "----------------------------------------------------------------------------------- \n \n";
    return true;
  }
  
  bool updateModule()
  { 
    yarp::sig::Matrix T(4,4);
    yarp::sig::Vector in(3);
    yarp::sig::Vector out(3);
    
    
    // Test 1
    // target_T_source = [0 -1 0 0]
    //                   [1  0 0 0]
    //                   [0  0 1 0]
    //                   [0  0 0 1]
    T.zero();
    T(0, 1) = -1;
    T(1, 0) = 1;
    T(2, 2) = 1;
    T(3,3) = 1;


    // Transform input point source_p [7 3 2] in source frame to get target_p [-3 7 2] as output in target frame   
    in.zero();
    in(0) = 7; in(1) = 3; in(2) = 2;   
    //in(0) = 0; in(1) = 0; in(2) = 0; 
    out.zero();
    this->transformTest(T, in, "source", out, "target", transformsExist);

    // Test 2
    // far_target_T_target = [ 0 0 1 0]
    //                       [ 0 1 0 0]
    //                       [-1 0 0 0]
    //                       [ 0 0 0 1]
    T.zero();
    T(0, 2) = 1;
    T(1, 1) = 1;
    T(2, 0) = -1;
    T(3,3) = 1;

    
    // Transform input point target_p [-3 7 2] in target frame to get far_target_p [2 7 3] as output in far_target frame   
    in.zero();
    in(0) = -3; in(1) = 7; in(2) = 2;   
    //in(0) = 0; in(1) = 0; in(2) = 0; 
    out.zero();
    this->transformTest(T, in, "target", out, "far_target", transformsExist);
    
    if (!transformsExist) { transformsExist = true; }
  
    // Test 3
    // Transform input point source_p [7 3 2] in source frame to get far_target_p [2 7 3] as output in far_target frame
    in.zero();
    in(0) = 7; in(1) = 3; in(2) = 2;
    //in(0) = 0; in(1) = 0; in(2) = 0; 
    out.zero();
    this->transformTest(T, in, "source", out, "far_target", transformsExist);
    return false;
  }
  
  bool configure(yarp::os::ResourceFinder &rf)
  {
    prop.put("device", "transformClient");
    prop.put("remote", "/transformServer");
    prop.put("local", "/local/transformServer");
    
    bool ok = transClient.open(prop);
    if (!ok)
    {
      yError() << "Fail";
      return false;
    }
    
    if (!transClient.view(iframetrans))
    {
      yError() << "View fail";
      return false;
    }
    return true;
  }
  
  bool interruptModule()
  {
    return true;
  }
  
  bool close()
  {
    return true;
  }
};

// tf::StampedTransform getTF(tf::Matrix3x3& R, tf::Vector3& p, const std::string &dst, const std::string &src)
// {
//   tf::Transform dst_T_src;
//   tf::Matrix3x3 dst_R_src(R);
//   dst_T_src.setOrigin(p);
//   tf::Quaternion q;
//   dst_R_src.getRotation(q);
//   dst_T_src.setRotation(q);
//   
//   return tf::StampedTransform(dst_T_src, ros::Time(0), dst, src);
// }

geometry_msgs::TransformStamped getTF(tf::Matrix3x3& R, tf::Vector3& p, const std::string &dst, const std::string &src)
{
  geometry_msgs::TransformStamped dst_T_src;
  tf::Matrix3x3 dst_R_src(R);
  dst_T_src.transform.translation.x = p.getX();
  dst_T_src.transform.translation.y = p.getY();
  dst_T_src.transform.translation.z = p.getZ();
  tf::Quaternion q;
  dst_R_src.getRotation(q);
  dst_T_src.transform.rotation.w = q.getW();
  dst_T_src.transform.rotation.x = q.getX();
  dst_T_src.transform.rotation.y = q.getY();
  dst_T_src.transform.rotation.z = q.getZ();
  dst_T_src.header.frame_id = dst;
  dst_T_src.child_frame_id = src;
  
  return dst_T_src;
}

void timerBrCallback(const ros::TimerEvent& t)
{
  //static tf::TransformBroadcaster br;
  static tf2_ros::StaticTransformBroadcaster br;
  
    // target_T_source = [0 -1 0 0]
    //                   [1  0 0 0]
    //                   [0  0 1 0]
    //                   [0  0 0 1]
  tf::Vector3 target_p_source(0.0, 0.0, 0.0);
  tf::Matrix3x3 target_R_source(0, -1, 0,
                               1,  0, 0,
                               0, 0, 1);

  br.sendTransform(getTF(target_R_source, target_p_source, "target", "source"));

    // far_target_T_target = [ 0 0 1 0]
    //                       [ 0 1 0 0]
    //                       [-1 0 0 0]
    //                       [ 0 0 0 1]  
  tf::Vector3 far_target_p_target(0.0, 0.0, 0.0);
  tf::Matrix3x3 far_target_R_target(0, 0, 1, 
                                    0, 1, 0, 
                                   -1, 0, 0);

  br.sendTransform(getTF(far_target_R_target, far_target_p_target, "far_target", "target")); 
}

void timerLisCallback(const ros::TimerEvent& t1)
{
  tf::TransformListener lis;

  lis.waitForTransform("far_target", "source", ros::Time(0.05), ros::Duration(5));
  
  
  tf::Stamped<tf::Point> in;
  in.frame_id_ = "source";
  in.stamp_ = ros::Time(0);
  in.setData(tf::Point(7, 3, 2));
  //in.setData(tf::Point(0, 0, 0));
  std::cout << "ROS input point in soure frame: " << in.getX() << " , "
                                                  << in.getY() << " , "
                                                  << in.getZ() << std::endl;
  
  tf::Stamped<tf::Point> out;
  
  tf::StampedTransform transform;
  lis.lookupTransform("far_target", "source", ros::Time(0.0), transform);

  
  lis.transformPoint("far_target", ros::Time(0.05), in, "source", out);
  //lis.transformPoint("far_target", in, out);
  std::cout << "ROS Transformed poin in far_target frame: " << out.getX() << " , "
                                                            << out.getY() << " , "
                                                            << out.getZ() << std::endl;
}

int main(int argc, char* argv[])
{
  yarp::os::Network yarp;
  
  testModule module;
  yarp::os::ResourceFinder rf;
  rf.configure(argc, argv);
  rf.setVerbose(true);
  module.runModule(rf);
  
  ros::init(argc, argv, "tf_Test");
  ros::NodeHandle nh;
  
  ros::Timer timerBr = nh.createTimer(ros::Duration(0.1), timerBrCallback);
  ros::Timer timerLis = nh.createTimer(ros::Duration(0.2), timerLisCallback);
  ros::spin();
  return 0;
}