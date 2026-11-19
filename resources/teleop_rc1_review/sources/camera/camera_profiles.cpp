#include <librealsense2/rs.hpp>
#include <iostream>
int main(){
 try{
  rs2::context context;
  for(auto device:context.query_devices()){
   std::cout<<"DEVICE "<<device.get_info(RS2_CAMERA_INFO_NAME)<<" "<<device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
            <<" USB "<<device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)<<"\n";
   for(auto sensor:device.query_sensors())for(auto profile:sensor.get_stream_profiles()){
    if(profile.stream_type()!=RS2_STREAM_COLOR)continue;
    auto p=profile.as<rs2::video_stream_profile>();
    if(p.width()==640 && p.height()==480)
     std::cout<<"COLOR "<<p.width()<<"x"<<p.height()<<" "<<profile.fps()<<" "<<rs2_format_to_string(profile.format())<<"\n";
   }
  }
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
