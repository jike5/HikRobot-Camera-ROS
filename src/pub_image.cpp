
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#define NIL (0)
#define MAX_IMAGE_DATA_SIZE (4 * 2048 * 2448)

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);


int main(int argc, char** argv){
	ros::init(argc, argv, "pub_image");
	ros::NodeHandle nh;
	
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("HikImage", 10);
	
	int nRet = MV_OK;
	void* handle = NULL;
	unsigned char * pData = NULL;        
    unsigned char *pDataForBGR = NULL;
	
	MV_CC_DEVICE_INFO_LIST stDeviceList;
	memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
	
	nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
	if (MV_OK != nRet)
	{
		printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
		return -1;
	}

	if (stDeviceList.nDeviceNum > 0)
	{
		for (int i = 0; i < stDeviceList.nDeviceNum; i++)
		{
			printf("[device %d]:\n", i);
			MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
			if (NULL == pDeviceInfo)
			{
				break;
			} 
			PrintDeviceInfo(pDeviceInfo);            
		}  
	} 
	else
	{
		printf("Find No Devices!\n");
		return -1;
	}
	
	unsigned int nIndex = 0;
	// 选择设备并创建句柄
	// select device and create handle
	nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
	if (MV_OK != nRet)
	{
		printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
		return -1;
	}
	// 打开设备
	// open device
	nRet = MV_CC_OpenDevice(handle);
	if (MV_OK != nRet)
	{
		printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
		return -1;
	}
	// ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
	if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
	{
		int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
		if (nPacketSize > 0)
		{
			nRet = MV_CC_SetIntValue(handle,"GevSCPSPacketSize",nPacketSize);
			if(nRet != MV_OK)
			{
				printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
			}
		}
		else
		{
			printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
		}
	}
	// set ExposureAuto 2
	nRet = MV_CC_SetEnumValue(handle, "ExposureAuto", 2);
	if(MV_OK != nRet){
		printf("MV_CC_SetEnumValue ExposureAuto fail! nRet [%x]\n", nRet);
		return -1;
	}
	else 
		printf("MV_CC_SetEnumValue ExposureAuto success\n");
	
	// ch:获取数据包大小 | en:Get payload size
	MVCC_INTVALUE stParam;
	memset(&stParam, 0, sizeof(MVCC_INTVALUE));
	nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
	if (MV_OK != nRet)
	{
		printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
		return -1;
	}
	// 开始取流
	// start grab image
	nRet = MV_CC_StartGrabbing(handle);
	if (MV_OK != nRet)
	{
		printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
		return -1;
	}
	MV_FRAME_OUT_INFO_EX stImageInfo = {0};
	memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
	pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
	if (NULL == pData)
	{
		return -1;
	}
	unsigned int nDataSize = stParam.nCurValue;
	
	ros::Rate loop_rate(10);
	while(ros::ok()){
		nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 1000);
		if(MV_OK == nRet)
		{
			//printf("Now you GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n\n", 
              //  stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
            //printf("Image Type: %d", stImageInfo.enPixelType);
            pDataForBGR = (unsigned char *)malloc(MAX_IMAGE_DATA_SIZE);;
			if (NULL == pDataForBGR)
			{
				return -1;
			}
			// 像素格式转换
			// convert pixel format 
			MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
			// 从上到下依次是：图像宽，图像高，输入数据缓存，输入数据大小，源像素格式，
			// 目标像素格式，输出数据缓存，提供的输出缓冲区大小
			// Top to bottom are：image width, image height, input data buffer, input data size, source pixel format, 
			// destination pixel format, output data buffer, provided output buffer size
			stConvertParam.nWidth = stImageInfo.nWidth;
			stConvertParam.nHeight = stImageInfo.nHeight;
			stConvertParam.pSrcData = pData;
			stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
			stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
			stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
			stConvertParam.pDstBuffer = pDataForBGR;
			stConvertParam.nDstBufferSize = MAX_IMAGE_DATA_SIZE;
			nRet = MV_CC_ConvertPixelType(handle, &stConvertParam);
			if (MV_OK != nRet)
			{
				printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
				return -1;
			}
			cv::Mat frame = cv::Mat(stImageInfo.nHeight, 
			stImageInfo.nWidth, 
			CV_8UC3, 
			pDataForBGR).clone();
			
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
			pub.publish(msg);
			loop_rate.sleep();
		}
	}
	
	// 停止取流
	// stop grab image
	nRet = MV_CC_StopGrabbing(handle);
	if (MV_OK != nRet)
	{
		printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
		return -1;
	}
	// 关闭设备
	// close device
	nRet = MV_CC_CloseDevice(handle);
	if (MV_OK != nRet)
	{
		printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
		return -1;
	}
	// 销毁句柄
	// destroy handle
	nRet = MV_CC_DestroyHandle(handle);
	if (MV_OK != nRet)
	{
		printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
		return -1;
	}
       
	ROS_INFO("TEST");
	return 0;
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}
