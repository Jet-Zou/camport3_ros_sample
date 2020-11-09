#include <ros/ros.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud.h>
#include "../include/common.hpp"
#include "../include/TYImageProc.h"

static cv::Mat depth_image, color_image;
static TY_INTERFACE_HANDLE hIface = NULL;
static TY_ISP_HANDLE hColorIspHandle = NULL;
static TY_DEV_HANDLE hDevice = NULL;

static bool b_process_exit = false;
static pthread_t display_id;
static int32_t m_depth_width;
static int32_t m_depth_height;

static int32_t m_color_width;
static int32_t m_color_height;

static pthread_mutex_t mutex_x=PTHREAD_MUTEX_INITIALIZER;

static TY_CAMERA_CALIB_INFO color_calib; 
static sensor_msgs::ImagePtr depth_msg, bmp_msg;

static TY_CAMERA_CALIB_INFO depth_calib; 
static sensor_msgs::PointCloud cloud_msg;

static bool b_depth_registration = false;
static bool b_rgb_undistortion = false;
static bool b_map_depth2rgb = false;

sensor_msgs::ImageConstPtr rawToFloatingPointConversion(sensor_msgs::ImageConstPtr raw_image)
{
  static const float bad_point = std::numeric_limits<float>::quiet_NaN ();

  sensor_msgs::ImagePtr new_image = boost::make_shared<sensor_msgs::Image>();

  new_image->header = raw_image->header;
  new_image->width = raw_image->width;
  new_image->height = raw_image->height;
  new_image->is_bigendian = 0;
  new_image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  new_image->step = sizeof(float)*raw_image->width;

  std::size_t data_size = new_image->width*new_image->height;
  new_image->data.resize(data_size*sizeof(float));

  const unsigned short* in_ptr = reinterpret_cast<const unsigned short*>(&raw_image->data[0]);
  float* out_ptr = reinterpret_cast<float*>(&new_image->data[0]);

  for (std::size_t i = 0; i<data_size; ++i, ++in_ptr, ++out_ptr)
  {
    if (*in_ptr==0 || *in_ptr==0x7FF)
    {
      *out_ptr = bad_point;
    } else
    {
      *out_ptr = static_cast<float>(*in_ptr)/1000.0f;
    }
  }

  return new_image;
}

static int count = 0;
void *thread_display(void *arg)
{
    int m_frame_idx = 0;
    TY_FRAME_DATA frame;
    while(!b_process_exit){
        int err = TYFetchFrame(hDevice, &frame, 5000);
        if( err == TY_STATUS_OK ) {
            //int fps = get_fps();
            //if (fps > 0){
            //    LOGI("fps: %d", fps);
            //}

            cv::Mat depth, irl, irr, color;
            parseFrame(frame, &depth, &irl, &irr, &color, hColorIspHandle);
            
            pthread_mutex_lock(&mutex_x);
            if(!depth.empty()){
                memcpy(depth_image.data, depth.data, m_depth_width * m_depth_height * 2);
                depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_image).toImageMsg();
                depth_msg->header.frame_id = "map";
                
                std::vector<TY_VECT_3F> p3d;
                p3d.resize(m_depth_width * m_depth_height);
                TYMapDepthImageToPoint3d(&depth_calib, m_depth_width, m_depth_height, (uint16_t*)depth.data, &p3d[0]);
                
                cloud_msg.header.stamp = ros::Time::now();
                cloud_msg.header.frame_id = "map";
                cloud_msg.points.resize(m_depth_width * m_depth_height);
                
                cloud_msg.channels.resize(1);
                cloud_msg.channels[0].name = "rgb";
                cloud_msg.channels[0].values.resize(m_depth_width * m_depth_height);
                
                for(unsigned int ii = 0; ii < m_depth_width * m_depth_height; ii++) {
                    cloud_msg.points[ii].x = p3d[ii].x / 1000.0;
                    cloud_msg.points[ii].y = p3d[ii].y / 1000.0;
                    cloud_msg.points[ii].z = p3d[ii].z / 1000.0;
                    cloud_msg.channels[0].values[ii] = 250;
                }
            }
            if(!color.empty()){
                memcpy(color_image.data, color.data, m_color_width * m_color_height * 3);
                bmp_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_image).toImageMsg();
                bmp_msg->header.frame_id = "map";
            }
            //LOGI("count = %d", count);
            count++;
            pthread_mutex_unlock(&mutex_x);
            
            m_frame_idx++;
            
            TYISPUpdateDevice(hColorIspHandle);
            ASSERT_OK( TYEnqueueBuffer(hDevice, frame.userBuffer, frame.bufferSize) );
        }
    }
    
    LOGD("thread_display exit...");
}

void Stop(int signo) 
{
    b_process_exit = true;
    printf("my_publisher stop!!!\n");
    _exit(0);
}

int main(int argc, char** argv)
{
    signal(SIGINT, Stop); 
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    nh.getParam("rgb_do_undistortion", b_rgb_undistortion);
    nh.getParam("rgbd_do_registration", b_depth_registration);
    nh.getParam("map_depth2rgb", b_map_depth2rgb);
  
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/depth", 1);
    image_transport::Publisher pub_rgb = it.advertise("camera/rgb", 1);
    ros::Publisher cloud_pub       = nh.advertise<sensor_msgs::PointCloud>("cloud", 10000);
 
    int32_t color, ir, depth;
    color = 1;
    depth = 1;
    ir = 0;
  
    LOGD("Init lib");
    ASSERT_OK( TYInitLib() );
    TY_VERSION_INFO ver;
    ASSERT_OK( TYLibVersion(&ver) );
    LOGD("     - lib version: %d.%d.%d", ver.major, ver.minor, ver.patch);

    std::vector<TY_DEVICE_BASE_INFO> selected;
    ASSERT_OK( selectDevice(TY_INTERFACE_ALL, "", "", 1, selected) );
    ASSERT(selected.size() > 0);
    TY_DEVICE_BASE_INFO& selectedDev = selected[0];

    ASSERT_OK( TYOpenInterface(selectedDev.iface.id, &hIface) );
    ASSERT_OK( TYOpenDevice(hIface, selectedDev.id, &hDevice) );

    int32_t allComps;
    ASSERT_OK( TYGetComponentIDs(hDevice, &allComps) );

    ///try to enable color camera
    if(allComps & TY_COMPONENT_RGB_CAM  && color) {
        int32_t image_mode;
        ASSERT_OK(get_default_image_mode(hDevice, TY_COMPONENT_RGB_CAM, image_mode));
        LOGD("Select color Image Mode: %dx%d", TYImageWidth(image_mode), TYImageHeight(image_mode));
        ASSERT_OK(TYSetEnum(hDevice, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, image_mode));
        
        m_color_width = TYImageWidth(image_mode);
        m_color_height = TYImageHeight(image_mode);
        
        color_image = cv::Mat::zeros(m_color_height, m_color_width, CV_8UC3);
        
        LOGD("Has RGB camera, open RGB cam");
        ASSERT_OK( TYEnableComponents(hDevice, TY_COMPONENT_RGB_CAM) );
        
        TYGetStruct(hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_CALIB_DATA, &color_calib, sizeof(color_calib));
        
        //create a isp handle to convert raw image(color bayer format) to rgb image
        ASSERT_OK(TYISPCreate(&hColorIspHandle));
        //Init code can be modified in common.hpp
        //NOTE: Should set RGB image format & size before init ISP
        ASSERT_OK(ColorIspInitSetting(hColorIspHandle, hDevice));
        //You can  call follow function to show  color isp supported features
    }

    if (allComps & TY_COMPONENT_IR_CAM_LEFT && ir) {
        LOGD("Has IR left camera, open IR left cam");
        ASSERT_OK(TYEnableComponents(hDevice, TY_COMPONENT_IR_CAM_LEFT));
    }

    if (allComps & TY_COMPONENT_IR_CAM_RIGHT && ir) {
        LOGD("Has IR right camera, open IR right cam");
        ASSERT_OK(TYEnableComponents(hDevice, TY_COMPONENT_IR_CAM_RIGHT));
    }
    
    if (allComps & TY_COMPONENT_DEPTH_CAM && depth) {
        int32_t image_mode;
        ASSERT_OK(get_default_image_mode(hDevice, TY_COMPONENT_DEPTH_CAM, image_mode));
        LOGD("Select Depth Image Mode: %dx%d", TYImageWidth(image_mode), TYImageHeight(image_mode));
        ASSERT_OK(TYSetEnum(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, image_mode));
        ASSERT_OK(TYEnableComponents(hDevice, TY_COMPONENT_DEPTH_CAM));
        
        m_depth_width = TYImageWidth(image_mode);
        m_depth_height = TYImageHeight(image_mode);
        
        depth_image = cv::Mat::zeros(m_depth_height, m_depth_width, CV_16U);
        
        //depth map pixel format is uint16_t ,which default unit is  1 mm
        //the acutal depth (mm)= PixelValue * ScaleUnit 
        float scale_unit = 1.;
        TYGetFloat(hDevice, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &scale_unit);
        
        TYGetStruct(hDevice, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_CALIB_DATA, &depth_calib, sizeof(depth_calib));
    }
  
    LOGD("Prepare image buffer");
    uint32_t frameSize;
    ASSERT_OK( TYGetFrameBufferSize(hDevice, &frameSize) );
    LOGD("     - Get size of framebuffer, %d", frameSize);

    LOGD("     - Allocate & enqueue buffers");
    char* frameBuffer[2];
    frameBuffer[0] = new char[frameSize];
    frameBuffer[1] = new char[frameSize];
    LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[0], frameSize);
    ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[0], frameSize) );
    LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[1], frameSize);
    ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[1], frameSize) );
    
    bool hasTrigger;
    ASSERT_OK(TYHasFeature(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &hasTrigger));
    if (hasTrigger) {
        LOGD("Disable trigger mode");
        TY_TRIGGER_PARAM trigger;
        trigger.mode = TY_TRIGGER_MODE_OFF;
        ASSERT_OK(TYSetStruct(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &trigger, sizeof(trigger)));
    }

    LOGD("Start capture");
    ASSERT_OK( TYStartCapture(hDevice) );
    
    //TODO
    pthread_create(&display_id, NULL, thread_display, NULL);
    
    depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_image).toImageMsg();
    bmp_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_image).toImageMsg();
 
    ros::Rate loop_rate(30);
    while (nh.ok()) {
        pthread_mutex_lock(&mutex_x);
        sensor_msgs::ImageConstPtr float_msg = rawToFloatingPointConversion(depth_msg);
        pub.publish(float_msg);
        cloud_pub.publish(cloud_msg);
        pub_rgb.publish(bmp_msg);
        pthread_mutex_unlock(&mutex_x);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    b_process_exit = true;
    LOGD("Main process info:thread_display exit");
    
    pthread_join(display_id,NULL);
    
    ASSERT_OK( TYStopCapture(hDevice) );
    ASSERT_OK( TYCloseDevice(hDevice) );
    ASSERT_OK( TYCloseInterface(hIface) );
    ASSERT_OK(TYISPRelease(&hColorIspHandle));
    ASSERT_OK( TYDeinitLib() );
    delete frameBuffer[0];
    delete frameBuffer[1];
    LOGD("Main done!");
    
}
