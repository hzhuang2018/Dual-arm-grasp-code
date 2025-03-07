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
#ifndef CAMERA_H
#define CAMERA_H

#include <pylon/PylonIncludes.h>
// head file for GiGe camera
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <boost/thread.hpp>
#include <vector>
#include <atomic>
#include <memory>
#include "dll_export.h"

namespace Robot{
    using namespace Pylon;
    using namespace std;
    typedef Pylon::CBaslerGigEInstantCamera Camera_t;
    using namespace Basler_GigECameraParams;

    // Enumeration used for distinguishing different events.
enum ROBOTIC_GRASP_DLL MyEvents
{
    eMyExposureEndEvent,      // Triggered by a camera event.
    eMyFrameStartOvertrigger, // Triggered by a camera event.
    eMyEventOverrunEvent,     // Triggered by a camera event.
    eMyImageReceivedEvent,    // Triggered by the receipt of an image.
    eMyMoveEvent,             // Triggered when the imaged item or the sensor head can be moved.
    eMyNoEvent                // Used as default setting.
};

// Names of possible events for a printed output.
/*
const char* MyEventNames[] =
{
    "ExposureEndEvent     ",
    "FrameStartOvertrigger",
    "EventOverrunEvent    ",
    "ImageReceived        ",
    "Move                 ",
    "NoEvent              "
};
*/
// Used for logging received events without outputting the information on the screen
// because outputting will change the timing.
// This class is used for demonstration purposes only.
struct ROBOTIC_GRASP_DLL LogItem
{
    LogItem()
        : eventType( eMyNoEvent)
        , frameNumber(0)
    {
    }

    LogItem( MyEvents event, uint32_t frameNr)
        : eventType(event)
        , frameNumber(frameNr)
    {
        //Warning, measured values can be wrong on older PC hardware.
#if defined(PYLON_WIN_BUILD)
        QueryPerformanceCounter(&time);
#elif defined(PYLON_UNIX_BUILD)
        struct timeval tv;

        gettimeofday(&tv, NULL);
        time = static_cast<unsigned long long>(tv.tv_sec) * 1000L + static_cast<unsigned long long>(tv.tv_usec) / 1000LL;
#endif
    }


#if defined(PYLON_WIN_BUILD)
    LARGE_INTEGER time; // Recorded time stamps.
#elif defined(PYLON_UNIX_BUILD)
    unsigned long long time; // Recorded time stamps.
#endif
    MyEvents eventType; // Type of the received event.
    uint16_t frameNumber; // Frame number of the received event. A frame number may not be available for some transport layers.
                          // Frame numbers are not supported by all transport layers.
};


// Helper function for printing a log.
// This function is used for demonstration purposes only.
/*
void PrintLog( const std::vector<LogItem>& aLog)
{
#if defined(PYLON_WIN_BUILD)
    // Get the PC timer frequency.
    LARGE_INTEGER timerFrequency;
    QueryPerformanceFrequency(&timerFrequency);
#endif

    cout << std::endl << "Warning, the printed time values can be wrong on older PC hardware." << std::endl << std::endl;
    // Print the event information header.
    cout << "Time [ms]    " << "Event                 " << "FrameNumber" << std::endl;
    cout << "------------ " << "--------------------- " << "-----------" << std::endl;

    // Print the logged information.
    size_t logSize = aLog.size();
    for ( size_t i = 0; i < logSize; ++i)
    {
        // Calculate the elapsed time between the events.
        double time_ms = 0;
        if ( i)
        {
#if defined(PYLON_WIN_BUILD)
            __int64 oldTicks = ((__int64)aLog[i-1].time.HighPart << 32) + (__int64)aLog[i-1].time.LowPart;
            __int64 newTicks = ((__int64)aLog[i].time.HighPart << 32) + (__int64)aLog[i].time.LowPart;
            long double timeDifference = (long double) (newTicks - oldTicks);
            long double ticksPerSecond = (long double) (((__int64)timerFrequency.HighPart << 32) + (__int64)timerFrequency.LowPart);
            time_ms = (timeDifference / ticksPerSecond) * 1000;
#elif defined(PYLON_UNIX_BUILD)
            time_ms = aLog[i].time - aLog[i-1].time;
#endif
        }

        // Print the event information.
        cout << setw(12) << fixed << setprecision(4) << time_ms <<" "<< MyEventNames[ aLog[i].eventType ] <<" "<< aLog[i].frameNumber << std::endl;
    }
}
*/
// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 50;

// Example handler for GigE camera events.
// Additional handling is required for GigE camera events because the event network packets can be lost, doubled or delayed on the network.
class ROBOTIC_GRASP_DLL CEventHandler : public CBaslerGigECameraEventHandler, public CBaslerGigEImageEventHandler
{
public:
    CEventHandler()
        : m_nextExpectedFrameNumberImage(1)
        , m_nextExpectedFrameNumberExposureEnd(1)
        , m_nextFrameNumberForMove(1)
    {
        // Reserve space to log camera events and image events.
        m_log.reserve( c_countOfImagesToGrab * 2);
    }

    // This method is called when a camera event has been received.
    virtual void OnCameraEvent( CBaslerGigEInstantCamera& camera, intptr_t userProvidedId, GenApi::INode* /* pNode */)
    {
        if ( userProvidedId == eMyExposureEndEvent)
        {
            // An Exposure End event has been received.
            uint16_t frameNumber = (uint16_t)camera.ExposureEndEventFrameID.GetValue();
            m_log.push_back( LogItem( eMyExposureEndEvent, frameNumber));

            // If Exposure End event is not doubled.
            if ( GetIncrementedFrameNumber( frameNumber) != m_nextExpectedFrameNumberExposureEnd)
            {
                // Check whether the imaged item or the sensor head can be moved.
                if ( frameNumber == m_nextFrameNumberForMove)
                {
                    MoveImagedItemOrSensorHead();
                }

                // Check for missing Exposure End events.
                if ( frameNumber != m_nextExpectedFrameNumberExposureEnd)
                {
                    throw RUNTIME_EXCEPTION( "An Exposure End event has been lost. Expected frame number is %d but got frame number %d.", m_nextExpectedFrameNumberExposureEnd, frameNumber);
                }
                IncrementFrameNumber( m_nextExpectedFrameNumberExposureEnd);
            }
        }
        else if ( userProvidedId == eMyFrameStartOvertrigger)
        {
            // The camera has been overtriggered.
            m_log.push_back( LogItem( eMyFrameStartOvertrigger, 0));

            // Handle this error...
        }
        else if ( userProvidedId == eMyEventOverrunEvent)
        {
            // The camera was unable to send all its events to the PC.
            // Events have been dropped by the camera.
            m_log.push_back( LogItem( eMyEventOverrunEvent, 0));

            // Handle this error...
        }
        else
        {
            PYLON_ASSERT2(false, "The sample has been modified and a new event has been registered. Add handler code above.");
        }
    }

    // This method is called when an image has been grabbed.
    virtual void OnImageGrabbed( CBaslerGigEInstantCamera& camera, const CBaslerGigEGrabResultPtr& ptrGrabResult)
    {
        // An image has been received. Block ID is equal to frame number for GigE camera devices.
        uint16_t frameNumber = (uint16_t)ptrGrabResult->GetBlockID();
        m_log.push_back( LogItem( eMyImageReceivedEvent, frameNumber));

        // Check whether the imaged item or the sensor head can be moved.
        // This will be the case if the Exposure End has been lost or if the Exposure End is received later than the image.
        if ( frameNumber == m_nextFrameNumberForMove)
        {
            MoveImagedItemOrSensorHead();
        }

        // Check for missing images.
        if ( frameNumber != m_nextExpectedFrameNumberImage)
        {
            throw RUNTIME_EXCEPTION( "An image has been lost. Expected frame number is %d but got frame number %d.", m_nextExpectedFrameNumberExposureEnd, frameNumber);
        }
        IncrementFrameNumber( m_nextExpectedFrameNumberImage);
    }

    void MoveImagedItemOrSensorHead()
    {
        // The imaged item or the sensor head can be moved now...
        // The camera may not be ready for a trigger at this point yet because the sensor is still being read out.
        // See the documentation of the CInstantCamera::WaitForFrameTriggerReady() method for more information.
        m_log.push_back( LogItem( eMyMoveEvent, m_nextFrameNumberForMove));
        IncrementFrameNumber( m_nextFrameNumberForMove);
    }

    void PrintLog()
    {
        //PrintLog( m_log);
    }

private:
    void IncrementFrameNumber( uint16_t& frameNumber)
    {
        frameNumber = GetIncrementedFrameNumber( frameNumber);
    }

    uint16_t GetIncrementedFrameNumber( uint16_t frameNumber)
    {
        ++frameNumber;
        if ( frameNumber == 0)
        {
            // Zero is not a valid frame number.
            ++frameNumber;
        }
        return frameNumber;
    }

    uint16_t m_nextExpectedFrameNumberImage;
    uint16_t m_nextExpectedFrameNumberExposureEnd;
    uint16_t m_nextFrameNumberForMove;

    std::vector<LogItem> m_log;
};
    // define image buffer
    struct ROBOTIC_GRASP_DLL ImageBuffer
    {
        vector<cv::Mat> buffer;
        atomic_int write_pos{-1};
        mutable int last_read_pos{-1};

        void push(cv::Mat& img_);
        cv::Mat read(); // 读取最新的图像
        cv::Mat read_last_unread(); // 读取最后未被读取的图像

        bool empty() const {return write_pos < 0;};
        static const int buffer_size;

        ImageBuffer();
    };
    // define param for camera
    struct ROBOTIC_GRASP_DLL CameraParameters
    {
        double k1, k2, k3, p1, p2; // 相机畸变参数
        double fx, fy; 
        double cx, cy;        
    };

    // defien a camera
    class ROBOTIC_GRASP_DLL BaslerCamera
    {
	public:
		enum TRIGGER_METHOD {
			SOFT_TRIGGER,
			HADR_TRIGGER
		};
    public:
        //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		BaslerCamera();
        ~BaslerCamera();

        void run();
        void stop();
        void get_latest_image(cv::Mat& img_);// {return image_buffer.read();}
        void get_last_unread(cv::Mat& img_);// {return image_buffer.read_last_unread();}

        void continuous_grabing(); // grab image continuously
        cv::Mat shot();            // grab only one image
		void setup();
		void set_trigger(TRIGGER_METHOD method_, int line_ = 1);
		void set_exposure(double time_);

		int height;
		int width;

		CameraParameters param;
		Camera_t camera;
    private:
		
        // 
        boost::thread grabThread;   // 抓取图像线程
        boost::mutex imageMutex;    // mutex for image buffer read and write
        bool running;

        ImageBuffer image_buffer; // 图像数据
        
		TRIGGER_METHOD trigger_method;
		int line;
		double exposure_time;
        
    private:
        
        bool is_empty();
    };

	// define the stereo camera
	class ROBOTIC_GRASP_DLL BaslerStereoCamera {
	public:
		BaslerStereoCamera();
		~BaslerStereoCamera();

		// attach the camera with the specified serial numbers of the left and right cameras
		void setup(const std::string& left_sn, const std::string& right_sn);
		// start the loop thread to grab the images
		void start();
		void stop();
		// get image data from left camera
		cv::Mat get_left_image();
		cv::Mat get_rectified_left_image();
		// get image data from right camera
		cv::Mat get_right_image();
		cv::Mat get_rectified_right_image();

		// set exposure time
		void set_exposure(double left_, double right_);
		// trigger
		void set_hardware_trigger(bool v_);

		// read intrinsic and extrinsic parameters
		void read_param(const std::string& filename_intrinsic_, const std::string& filename_extrinsic_);

		// status of the cameras
		bool left_cam_attached;
		bool right_cam_attached;
	private:
		BaslerCamera left_camera;
		BaslerCamera right_camera;

		// extrinsic parameters
		// roation matrix and translation vector from right camera to left camera
		Eigen::Matrix3d R_ext;
		Eigen::Vector3d t_ext;
		
		// the maps for rectifying stereo images
		cv::Mat left_rmap_1, left_rmap_2;
		cv::Mat right_rmap_1, right_rmap_2;

		int height;
		int width;

		double exposure_time_left;
		double exposure_time_right;

		bool is_hardware_trigger;
	};

} // end of Robot

#endif