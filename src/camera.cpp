// Copyright (c) 2021 Yipeng Li @ BICE (yipengli.bice@gmail.com)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
#include <camera.h>
#include <ConfigurationEventPrinter.h>
#include<opencv2/calib3d.hpp>

namespace Robot{



// methods for ImageBuffer
const int ImageBuffer::buffer_size = 10;
ImageBuffer::ImageBuffer()
{
    buffer.resize(buffer_size);
}
void ImageBuffer::push(cv::Mat& img_)
{
    int new_pos = (write_pos+1)%buffer_size;
    buffer[new_pos] = img_.clone();
    write_pos = new_pos;
}
cv::Mat ImageBuffer::read() 
{
    last_read_pos = write_pos;
    cv::Mat img = buffer[write_pos].clone();
    return img;
}
cv::Mat ImageBuffer::read_last_unread() 
{
    while (last_read_pos == write_pos) {};
    return read();
}

// methods for Camera
BaslerCamera::BaslerCamera()
{
    height = 2048;
    width = 2048;
	camera.MaxNumBuffer = 15;

	trigger_method == TRIGGER_METHOD::SOFT_TRIGGER;
	line = 1;
	exposure_time = 20000; // 20ms
}

BaslerCamera::~BaslerCamera()
{
    stop();
}

void BaslerCamera::setup()
{
    PylonInitialize();
    CEventHandler eventHandler;

    // device info
    CDeviceInfo info;
    info.SetDeviceClass(Camera_t::DeviceClass());

    // attach device
    camera.Attach(CTlFactory::GetInstance().CreateFirstDevice(info));
    cout << "using device " << camera.GetDeviceInfo().GetModelName() << endl;

    camera.MaxNumBuffer = 15;

	if (trigger_method == TRIGGER_METHOD::SOFT_TRIGGER)
	{
		camera.RegisterConfiguration(new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
	}

	camera.Open();

    /*
    camera.RegisterConfiguration( new CConfigurationEventPrinter, RegistrationMode_Append, Cleanup_Delete); // Camera use.

        // Register the event handler.
    camera.RegisterImageEventHandler( &eventHandler, RegistrationMode_Append, Cleanup_None);
    camera.RegisterCameraEventHandler( &eventHandler, "ExposureEndEventData", eMyExposureEndEvent, RegistrationMode_ReplaceAll, Cleanup_None);
    camera.RegisterCameraEventHandler( &eventHandler, "FrameStartOvertriggerEventData", eMyFrameStartOvertrigger, RegistrationMode_Append, Cleanup_None);
    camera.RegisterCameraEventHandler( &eventHandler, "EventOverrunEventData", eMyEventOverrunEvent, RegistrationMode_Append, Cleanup_None);

        // Camera event processing must be activated first, the default is off.
    camera.GrabCameraEvents = true;
   */

    
    // setups
    UserSetDefaultSelectorEnums oldDefaultUserSet = camera.UserSetDefaultSelector.GetValue();
    //cout << "Loading default settings" << endl;
    camera.UserSetSelector.SetValue(UserSetSelector_Default);
    camera.UserSetLoad.Execute();
    camera.GainAuto.SetValue(GainAuto_Off);
    camera.GainRaw.SetValue(camera.GainRaw.GetMin());
    camera.ExposureAuto.SetValue(ExposureAuto_Off);
    camera.ExposureTimeRaw.SetValue(exposure_time);
    
    /*
     // Check if the device supports events.
        if ( !IsAvailable( camera.EventSelector))
        {
            throw RUNTIME_EXCEPTION( "The device doesn't support events.");
        }

        // Enable the sending of Exposure End events.
        // Select the event to be received.
        camera.EventSelector.SetValue(EventSelector_ExposureEnd);
        // Enable it.
        camera.EventNotification.SetValue(EventNotification_GenICamEvent);

        // Enable the sending of Event Overrun events.
        camera.EventSelector.SetValue(EventSelector_EventOverrun);
        camera.EventNotification.SetValue(EventNotification_GenICamEvent);

        // Enable the sending of Frame Start Overtrigger events.
        if ( IsAvailable( camera.EventSelector.GetEntry(EventSelector_FrameStartOvertrigger)))
        {
            camera.EventSelector.SetValue(EventSelector_FrameStartOvertrigger);
            camera.EventNotification.SetValue(EventNotification_GenICamEvent);
        }

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        */
	
    
	camera.StartGrabbing(GrabStrategy_OneByOne);
}

void BaslerCamera::run()
{
    //cout << "start grabing images..." << endl;
    CGrabResultPtr ptrGrabResult; 
    int counter = 0; 
    while (camera.IsGrabbing() && running)
    {
        camera.RetrieveResult(5000,ptrGrabResult,TimeoutHandling_ThrowException);
        //cout << "get " << counter++ << " image..." << endl;
        if (ptrGrabResult->GrabSucceeded())
        {
            // push to buffer
            height = ptrGrabResult->GetHeight();
            width = ptrGrabResult->GetWidth();
            
            // mono image
            cv::Mat img(height, width, CV_8UC1, (uint8_t*)ptrGrabResult->GetBuffer());
            //cout << "image: " << height << "x" << width << endl;
            
            // lock the buffer and push image data
            boost::mutex::scoped_lock lk(imageMutex);
            //imageMutex.lock();
            image_buffer.push(img);
            //imageMutex.unlock();
            //WaitObject::Sleep(40);
        }
        
    }
}

void BaslerCamera::stop()
{
    running = false;
	grabThread.join();
	camera.Close();
	//PylonTerminate();
}

bool BaslerCamera::is_empty()
{
    //imageMutex.lock();
    bool flag = image_buffer.empty();
    //imageMutex.unlock();
    return flag;
}

void BaslerCamera::get_latest_image(cv::Mat& img_)
{
    if (is_empty())
        return;
    // lock 
    //boost::shared_lock<boost::shared_mutex> read_only(imageMutex);
    boost::mutex::scoped_lock lk(imageMutex);
    //imageMutex.lock();
    img_ = image_buffer.read();
    //imageMutex.unlock();
}
void BaslerCamera::get_last_unread(cv::Mat& img_)
{
    // lock
    //boost::shared_lock<boost::shared_mutex> read_only(imageMutex);
    boost::mutex::scoped_lock lk(imageMutex);
    //imageMutex.lock();
    img_ = image_buffer.read_last_unread();
    //imageMutex.unlock();
}
void BaslerCamera::continuous_grabing()
{
    // create image grab thread
    
    running = true;
    boost::function0<void> f = boost::bind(&BaslerCamera::run, this);
    grabThread = boost::thread(f);

    cout << "start grabing images..." << endl;
       
}
void BaslerCamera::set_trigger(TRIGGER_METHOD method_, int line_)
{
	trigger_method = method_;
	line = line_;
}
void BaslerCamera::set_exposure(double time_)
{
	exposure_time = time_;
}
cv::Mat BaslerCamera::shot()
{
    CGrabResultPtr ptrGrabResult; 
    if (camera.IsGrabbing())
    {
        camera.RetrieveResult(5000,ptrGrabResult,TimeoutHandling_ThrowException);
        if (ptrGrabResult->GrabSucceeded())
        {
            // push to buffer
            height = ptrGrabResult->GetHeight();
            width = ptrGrabResult->GetWidth();
            // mono image
            cv::Mat img(height, width, CV_8UC1, (uint8_t*)ptrGrabResult->GetBuffer());
            return img;
        }
        else
        {
            exit(1);
        }
    }
    else
    {
        exit(1);
    }
}

BaslerStereoCamera::BaslerStereoCamera()
{
	left_cam_attached = false;
	right_cam_attached = false;

	exposure_time_left = 20000;
	exposure_time_right = 20000;

	is_hardware_trigger = false;

	R_ext.setIdentity();
	t_ext.setZero();
}
BaslerStereoCamera::~BaslerStereoCamera()
{

}
void BaslerStereoCamera::setup(const std::string& left_sn, const std::string& right_sn)
{
	String_t left_sn_(left_sn.c_str());
	String_t right_sn_(right_sn.c_str());

	PylonInitialize();
	CEventHandler eventHandler;

	// device info
	CDeviceInfo info;
	info.SetDeviceClass(Camera_t::DeviceClass());
	CTlFactory& tlFactory = CTlFactory::GetInstance();
	DeviceInfoList_t devices;
	if (tlFactory.EnumerateDevices(devices) == 0)
	{
		throw RUNTIME_EXCEPTION("No camera found!");
	}
	CInstantCameraArray cameras(devices.size());

	for (size_t i = 0; i < cameras.GetSize(); ++i)
	{
		//std::cout << "sn: " << devices[i].GetSerialNumber() << std::endl;
		// attach the left camera
		if (devices[i].GetSerialNumber() == left_sn_)
		{
			left_camera.camera.Attach(tlFactory.CreateDevice(devices[i]));
			left_cam_attached = true;
		}
		if (devices[i].GetSerialNumber() == right_sn_)
		{
			right_camera.camera.Attach(tlFactory.CreateDevice(devices[i]));
			right_cam_attached = true;
		}
	}
	if (left_cam_attached)
		printf("Left camera is already connected.\n");
	else
		printf("Left camera is not found!\n");
	if (right_cam_attached)
		printf("Right camera is already connected.\n");
	else
		printf("Right camera is not found!\n");

	//left_camera.camera.RegisterConfiguration(new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
	//right_camera.camera.RegisterConfiguration(new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
	
	left_camera.camera.MaxNumBuffer = 15;
	left_camera.camera.Open();
	right_camera.camera.MaxNumBuffer = 15;
	right_camera.camera.Open();
	// Now only software trigger is supported due to the lack of the squencer generator.\
	// We may use the hardware trigger if possible.
	if (is_hardware_trigger)
	{
		/* to be finised in the future */
		
	}
	else
	{
		left_camera.camera.TriggerMode.SetValue(TriggerModeEnums::TriggerMode_On);
		left_camera.camera.TriggerSelector.SetValue(TriggerSelector_FrameStart);
		left_camera.camera.TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Software);
		
		right_camera.camera.TriggerMode.SetValue(TriggerModeEnums::TriggerMode_On);
		right_camera.camera.TriggerSelector.SetValue(TriggerSelector_FrameStart);
		right_camera.camera.TriggerSource.SetValue(TriggerSourceEnums::TriggerSource_Software);
		
		left_camera.camera.TriggerSoftware.Execute();
		right_camera.camera.TriggerSoftware.Execute();
	}

	// left camera
	UserSetDefaultSelectorEnums oldDefaultUserSet1 = left_camera.camera.UserSetDefaultSelector.GetValue();
	//cout << "Loading default settings" << endl;
	left_camera.camera.UserSetSelector.SetValue(UserSetSelector_Default);
	left_camera.camera.UserSetLoad.Execute();
	left_camera.camera.GainAuto.SetValue(GainAuto_Off);
	left_camera.camera.GainRaw.SetValue(left_camera.camera.GainRaw.GetMin());
	left_camera.camera.ExposureAuto.SetValue(ExposureAuto_Off);
	left_camera.camera.ExposureTimeRaw.SetValue(exposure_time_left);

	// right camera
	UserSetDefaultSelectorEnums oldDefaultUserSet2 = right_camera.camera.UserSetDefaultSelector.GetValue();
	//cout << "Loading default settings" << endl;
	right_camera.camera.UserSetSelector.SetValue(UserSetSelector_Default);
	right_camera.camera.UserSetLoad.Execute();
	right_camera.camera.GainAuto.SetValue(GainAuto_Off);
	right_camera.camera.GainRaw.SetValue(left_camera.camera.GainRaw.GetMin());
	right_camera.camera.ExposureAuto.SetValue(ExposureAuto_Off);
	right_camera.camera.ExposureTimeRaw.SetValue(exposure_time_right);
	
	left_camera.camera.StartGrabbing(GrabStrategy_OneByOne);
	right_camera.camera.StartGrabbing(GrabStrategy_OneByOne);
}

void BaslerStereoCamera::start()
{
	printf("Left camera: ");
	left_camera.continuous_grabing();
	printf("Right camera: ");
	right_camera.continuous_grabing();
}
void BaslerStereoCamera::stop()
{
	left_camera.stop();
	right_camera.stop();
	PylonTerminate();
}
cv::Mat BaslerStereoCamera::get_left_image()
{
	cv::Mat img;
	left_camera.get_latest_image(img);
	return img;
}
cv::Mat BaslerStereoCamera::get_rectified_left_image()
{
	cv::Mat img, rimg;
	left_camera.get_latest_image(img);
	cv::remap(img, rimg, left_rmap_1, left_rmap_2, cv::INTER_LINEAR);
	return rimg;
}
cv::Mat BaslerStereoCamera::get_rectified_right_image()
{
	cv::Mat img, rimg;
	right_camera.get_latest_image(img);
	cv::remap(img, rimg, right_rmap_1, right_rmap_2, cv::INTER_LINEAR);
	return rimg;
}
cv::Mat BaslerStereoCamera::get_right_image()
{
	cv::Mat img;
	right_camera.get_latest_image(img);
	return img;
}
void BaslerStereoCamera::set_exposure(double left_, double right_)
{
	exposure_time_left = left_;
	exposure_time_right = right_;
}
void BaslerStereoCamera::set_hardware_trigger(bool v_)
{
	is_hardware_trigger = v_;
}
void BaslerStereoCamera::read_param(const std::string& filename_intrinsic_, const std::string& filename_extrinsic_)
{
	cv::FileStorage fs;
	fs.open(filename_intrinsic_.c_str(), cv::FileStorage::READ);
	cv::Mat K1, K2, dist1, dist2;
	if (fs.isOpened())
	{
		K1 = fs["K_left"].mat();
		K2 = fs["K_right"].mat();
		dist1 = fs["dist_left"].mat();
		dist2 = fs["dist_right"].mat();

		left_camera.param.fx = K1.at<float>(0,0);
		left_camera.param.fy = K1.at<float>(1,1);
		left_camera.param.cx = K1.at<float>(0,2);
		left_camera.param.cy = K1.at<float>(1,2);

		right_camera.param.fx = K2.at<float>(0, 0);
		right_camera.param.fy = K2.at<float>(1, 1);
		right_camera.param.cx = K2.at<float>(0, 2);
		right_camera.param.cy = K2.at<float>(1, 2);

		fs.release();
	}
	else
	{
		printf("Cannot read the instrinsic parameters!\n");
	}
	fs.open(filename_extrinsic_.c_str(), cv::FileStorage::READ);
	cv::Mat R1, R2, P1, P2, R, t;
	if (fs.isOpened())
	{
		R1 = fs["R1"].mat();
		R2 = fs["R2"].mat();
		P1 = fs["P1"].mat();
		P2 = fs["P2"].mat();
		R = fs["R"].mat();
		t = fs["t"].mat();

		cv::initUndistortRectifyMap(K1, dist1, R1, P1, cv::Size(height, width), CV_16SC2, left_rmap_1, left_rmap_2);
		cv::initUndistortRectifyMap(K2, dist2, R2, P2, cv::Size(height, width), CV_16SC2, right_rmap_1, right_rmap_2);
	}
	else
	{
		printf("Cannot read the extrinsic parameters!\n");
	}
}

} // end of Robot