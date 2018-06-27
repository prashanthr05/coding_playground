#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/os/Property.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>

#include <string>
#include <iostream>

class testModule : public yarp::os::RFModule
{
public:
  yarp::dev::PolyDriver transClient;
  yarp::dev::IFrameTransform *iframetrans;
    
  yarp::os::Property prop;
    
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
    // T = [0 -1 0 0]
    //     [1  0 0 0]
    //     [0  0 1 0]
    //     [0  0 0 1]
    T.zero();
    T(0, 1) = -1;
    T(1, 0) = 1;
    T(2, 2) = 1;
    T(3,3) = 1;

    // Transform input point [7 3 2] in source frame to get [-3 7 2] as output in target frame   
    in.zero();
    in(0) = 7; in(1) = 3; in(2) = 2;   
    out.zero();
    this->transformTest(T, in, "source", out, "target", false);

    // Test 2
    // T = [ 0 0 1 0]
    //     [ 0 1 0 0]
    //     [-1 0 0 0]
    //     [ 0 0 0 1]
    T.zero();
    T(0, 2) = 1;
    T(1, 1) = 1;
    T(2, 0) = -1;
    T(3,3) = 1;

    // Transform input point [-3 7 2] in target frame to get [2 7 3] as output in far_target frame   
    in.zero();
    in(0) = -3; in(1) = 7; in(2) = 2;   
    out.zero();
    this->transformTest(T, in, "target", out, "far_target", false);
    
  
    // Test 3
    // Transform input point [7 3 2] in source frame to get [2 7 3] as output in far_target frame
    in.zero();
    in(0) = 7; in(1) = 3; in(2) = 2;
    out.zero();
    this->transformTest(T, in, "source", out, "far_target", true);
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

int main(int argc, char* argv[])
{
  yarp::os::Network yarp;
  
  testModule module;
  yarp::os::ResourceFinder rf;
  rf.configure(argc, argv);
  rf.setVerbose(true);
  
  module.runModule(rf);
  
  
  return 0;
}