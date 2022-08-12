//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: August 2017
// 
// SICK AG, Waldkirch
// email: TechSupport0905@sick.de

#include <cstdio>
#include <iostream>
#include <memory>
#include <sstream>

#include "VisionaryControl.h"
#include "CoLaParameterReader.h"
#include "CoLaParameterWriter.h"
#include "VisionarySData.h"    // Header specific for the Stereo data
#include "VisionaryDataStream.h"
#include "PointXYZ.h"
#include "PointCloudPlyWriter.h"

#include <chrono>
#include <thread>

bool runStreamingDemo(const char ipAddress[], unsigned short dataPort, uint32_t numberOfFrames)
{
  using namespace visionary;
  
  // Generate Visionary instance
  auto pDataHandler = std::make_shared<VisionarySData>();
  VisionaryDataStream dataStream(pDataHandler);
  VisionaryControl visionaryControl;

  //-----------------------------------------------
  // Connect to devices data stream 
  if (!dataStream.open(ipAddress, htons(dataPort)))
  {
    std::printf("Failed to open data stream connection to device.\n");
    return false;   // connection failed
  }

  //-----------------------------------------------
  // Connect to devices control channel
  if (!visionaryControl.open(VisionaryControl::ProtocolType::COLA_B, ipAddress, 5000/*ms*/))
  {
    std::printf("Failed to open control connection to device.\n");
    return false;   // connection failed
  }
  
  //-----------------------------------------------
  // read Device Ident
  std::printf("DeviceIdent: '%s'\n", visionaryControl.getDeviceIdent().c_str());

  //-----------------------------------------------
  // Login as authorized client
  if (visionaryControl.login(IAuthentication::UserLevel::AUTHORIZED_CLIENT, "CLIENT"))
  {
    //-----------------------------------------------
    // An example of reading an writing device parameters is shown here.
    // Use the "SOPAS Communication Interface Description" PDF to determine data types for other variables

    //-----------------------------------------------
    // Set framePeriod parameter to 150000
    std::printf("Setting framePeriodTime to 150000\n");
    CoLaCommand setFramePeriodCommand = CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "framePeriodTime").parameterUDInt(150000).build();
    CoLaCommand setFramePeriodResponse = visionaryControl.sendCommand(setFramePeriodCommand);
    if (setFramePeriodResponse.getError() == CoLaError::OK)
    {
      std::printf("Successfully set framePeriodTime to 150000\n");
    }

    //-----------------------------------------------
    // Read framePeriod parameter
    CoLaCommand getFramePeriodCommand = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "framePeriodTime").build();
    CoLaCommand framePeriodResponse = visionaryControl.sendCommand(getFramePeriodCommand);
    uint32_t framePeriodTime = CoLaParameterReader(framePeriodResponse).readUDInt();
    std::printf("Read framePeriodTime = %d\n", framePeriodTime);

    //-----------------------------------------------
    // Auto Exposure functions

    /* This section demonstrates how to use the auto exposure functions by invoking the method 'TriggerAutoExposureParameterized'.
       It's also shown how the region of interest (ROI) can be set. The sample is based on the AcquisitionModeStereo = NORMAL. */

    uint32_t acquisitionModeStereo = 0; 
    CoLaCommand setAcquisitionModeStereoCommand = CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "acquisitionModeStereo").parameterUSInt(acquisitionModeStereo).build();
    CoLaCommand setAcquisitionModeStereoResponse = visionaryControl.sendCommand(setAcquisitionModeStereoCommand);

    // Set region of interest (ROI)
    uint32_t left = 160;
    uint32_t right = 480;
    uint32_t top = 128;
    uint32_t bottom = 384;

    // Set ROI for Auto Exposure 3D
    CoLaCommand setAutoExposureROICommand = CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "autoExposureROI").parameterUDInt(left).parameterUDInt(right).parameterUDInt(top).parameterUDInt(bottom).build();
    CoLaCommand setAutoExposureROIResponse = visionaryControl.sendCommand(setAutoExposureROICommand);

    // Set ROI for Auto Exposure RGB
    CoLaCommand setAutoExposureColorROICommand = CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "autoExposureColorROI").parameterUDInt(left).parameterUDInt(right).parameterUDInt(top).parameterUDInt(bottom).build();
    CoLaCommand setAutoExposureColorROIResponse = visionaryControl.sendCommand(setAutoExposureColorROICommand);

    // Set ROI for Auto White Balance
    // NOTE: The user is responisble to make sure that the region he sets the ROI to, is actually white.
    CoLaCommand setAutoWhiteBalanceROICommand = CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "autoWhiteBalanceROI").parameterUDInt(left).parameterUDInt(right).parameterUDInt(top).parameterUDInt(bottom).build();
    CoLaCommand setAutoWhiteBalanceROIResponse = visionaryControl.sendCommand(setAutoWhiteBalanceROICommand);

    // Read out actual integration time values (before auto exposure was triggered)
    // ATTENTION: This sample is based on the NORMAL acquisition mode; other modes may refer to other integration time variables
    CoLaCommand getIntegrationTimeUsCommand = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "integrationTimeUs").build();
    CoLaCommand getIntegrationTimeUsResponse = visionaryControl.sendCommand(getIntegrationTimeUsCommand);
    uint32_t integrationTimeUs = CoLaParameterReader(getIntegrationTimeUsResponse).readUDInt();
    std::printf("Read integrationTimeUs = %d\n", integrationTimeUs);

    CoLaCommand getIntegrationTimeUsColorCommand = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "integrationTimeUsColor").build();
    CoLaCommand getIntegrationTimeUsColorResponse = visionaryControl.sendCommand(getIntegrationTimeUsColorCommand);
    uint32_t integrationTimeUsColor = CoLaParameterReader(getIntegrationTimeUsColorResponse).readUDInt();
    std::printf("Read integrationTimeUsColor = %d\n", integrationTimeUsColor);
   
    /* Info: For White Balance exists no SOPAS variable; the changes are done internally in the device and applied to the image.
             If you open SOPAS and you are running this sample in parallel you can see how the image changes. */

    // Invoke auto exposure method
    if (visionaryControl.login(IAuthentication::UserLevel::SERVICE, "CUST_SERV"))
    {
      for (uint8_t autoType = 0; autoType < 3; autoType++) // 0 = Auto Exposure 3D, 1 = Auto Exposure RGB, 2 = Auto White Balance
      {
        std::printf("Invoke method 'TriggerAutoExposureParameterized' (Param: %d) ...\n", autoType);
        
        CoLaCommand invokeAutoExposureCommand = CoLaParameterWriter(CoLaCommandType::METHOD_INVOCATION, "TriggerAutoExposureParameterized").parameterUInt(1).parameterUSInt(autoType).build();
        CoLaCommand autoExposureResponse = visionaryControl.sendCommand(invokeAutoExposureCommand);

        if (autoExposureResponse.getError() != CoLaError::OK)
        {
          std::printf("ERROR: Invoking 'TriggerAutoExposureParameterized' fails! (autoExposureResponse: %d)\n", CoLaParameterReader(autoExposureResponse).readBool());
        }

        // Wait until auto exposure method is finished 
        bool autoExpParamRunning = true;
        long long startTime = std::chrono::system_clock::now().time_since_epoch().count();
        long long timeNow = startTime;
        while (autoExpParamRunning)
        {
          CoLaCommand getAutoExpParamRunningCommand = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "autoExposureParameterizedRunning").build();
          CoLaCommand autoExpParamRunningResponse = visionaryControl.sendCommand(getAutoExpParamRunningCommand);
          autoExpParamRunning = CoLaParameterReader(autoExpParamRunningResponse).readBool();

          timeNow = std::chrono::system_clock::now().time_since_epoch().count();
          if ((timeNow - startTime) <= 10000000000) // 10 sec = 10 000 000 000 ns (time after auto exposure method should be finished)
          {
            std::this_thread::sleep_for(std::chrono::seconds(1));
          }
          else 
          {
            std::printf("TIMEOUT: auto exposure function (Param: %d) needs longer than expected!\n", autoType);
          }
        }
      }      
    }

    // Read out new integration time values (after auto exposure was triggered)
    getIntegrationTimeUsCommand = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "integrationTimeUs").build();
    getIntegrationTimeUsResponse = visionaryControl.sendCommand(getIntegrationTimeUsCommand);
    integrationTimeUs = CoLaParameterReader(getIntegrationTimeUsResponse).readUDInt();
    std::printf("Read integrationTimeUs = %d\n", integrationTimeUs);

    getIntegrationTimeUsColorCommand = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "integrationTimeUsColor").build();
    getIntegrationTimeUsColorResponse = visionaryControl.sendCommand(getIntegrationTimeUsColorCommand);
    integrationTimeUsColor = CoLaParameterReader(getIntegrationTimeUsColorResponse).readUDInt();
    std::printf("Read integrationTimeUsColor = %d\n", integrationTimeUsColor);

    //-----------------------------------------------

    // Read info messages variable
    CoLaCommand getMessagesCommand = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "MSinfo").build();
    CoLaCommand messagesResponse = visionaryControl.sendCommand(getMessagesCommand);

    //-----------------------------------------------
    // Read message array, length of array is always 25 items (see MSinfo in PDF).
    CoLaParameterReader reader(messagesResponse);
    for (int i = 0; i < 25; i++) // Read 25 items
    {
      uint32_t errorId = reader.readUDInt();
      uint32_t errorState = reader.readUDInt();

      // Read ErrTimeType struct members for FirstTime
      uint16_t firstTime_PwrOnCount = reader.readUInt();
      uint32_t firstTime_OpSecs = reader.readUDInt();
      uint32_t firstTime_TimeOccur = reader.readUDInt();

      // Read ErrTimeType struct members for LastTime
      uint16_t lastTime_PwrOnCount = reader.readUInt();
      uint32_t lastTime_OpSecs = reader.readUDInt();
      uint32_t lastTime_TimeOccur = reader.readUDInt();

      uint16_t numberOccurance = reader.readUInt();
      uint16_t errReserved = reader.readUInt();
      std::string extInfo = reader.readFlexString();

      // Write all non-empty info messages to the console
      if (errorId != 0)
      {
        std::printf("Info message [0x%032x], extInfo: %s, numberOccurance: %d\n", errorId, extInfo.c_str(), numberOccurance);
      }
    }
  }

  //-----------------------------------------------
  // Logout from device after reading variables.
  if (!visionaryControl.logout())
  {
    std::printf("Failed to logout\n");
  }

  //-----------------------------------------------
  // Stop image acquisition (works always, also when already stopped)
  visionaryControl.stopAcquisition();

  //-----------------------------------------------
  // Capture a single frame
  visionaryControl.stepAcquisition();
  if (dataStream.getNextFrame())
  {
    std::printf("Frame received through step called, frame #%d, timestamp: %lu \n", pDataHandler->getFrameNum(), pDataHandler->getTimestampMS());

    //-----------------------------------------------
    // Convert data to a point cloud
    std::vector<PointXYZ> pointCloud;
    pDataHandler->generatePointCloud(pointCloud);
    pDataHandler->transformPointCloud(pointCloud);

    //-----------------------------------------------
    // Write point cloud to PLY
    const char plyFilePath[] = "VisionaryS.ply";
    std::printf("Writing frame to %s\n", plyFilePath);
    PointCloudPlyWriter::WriteFormatPLY(plyFilePath, pointCloud, pDataHandler->getRGBAMap(), true);
    std::printf("Finished writing frame to %s\n", plyFilePath);
  }

  //-----------------------------------------------
  // Start image acquisiton and continously receive frames
  visionaryControl.startAcquisition();
  for (int i = 0; i < numberOfFrames; i++)
  {
    if (!dataStream.getNextFrame())
    {
      continue;     // No valid frame received
    }
    std::printf("Frame received in continuous mode, frame #%d \n", pDataHandler->getFrameNum());
  }

  //-----------------------------------------------
  // Stop acqusition
  visionaryControl.stopAcquisition();

  visionaryControl.close();
  dataStream.close();
  return true;
}

int main(int argc, char* argv[])
{
  // Insert IP and the API port of your camera, aswell as the number of images you want to receive via cmd/terminal
  /// Default values:
  /// IP:        "192.168.1.10"
  /// API-port:  2114

  std::string deviceIpAddr("192.168.1.10");
  unsigned short deviceBlobCtrlPort = 2114u;
  unsigned cnt = 100u;

  bool showHelpAndExit = false;

  int exitCode = 0;

  for (int i = 1; i < argc; ++i)
  {
    std::istringstream argstream(argv[i]);

    if (argstream.get() != '-')
    {
      showHelpAndExit = true;
      exitCode = 1;
      break;
    }
    switch (argstream.get())
    {
    case 'h':
      showHelpAndExit = true;
      break;
    case 'c':
      argstream >> deviceBlobCtrlPort;
      break;
    case 'i':
      argstream >> deviceIpAddr;
      break;
    case 'n':
      argstream >> cnt;
      break;
    default:
      showHelpAndExit = true;
      exitCode = 1;
      break;
    }
  }

  if (showHelpAndExit)
  {
    std::cout << argv[0] << " [option]*" << std::endl;
    std::cout << "where option is one of" << std::endl;
    std::cout << "-h          show this help and exit" << std::endl;
    std::cout << "-i<IP>      connect to the device with IP address <IP>; default is 192.168.1.10" << std::endl;
    std::cout << "-c<port>    assume the BLOB control port of the device was configured to <port>; default is 2114" << std::endl;
    std::cout << "-n<cnt>     acquire <cnt> frames and stop; default is 100" << std::endl;

    return exitCode;
  }

  runStreamingDemo(deviceIpAddr.c_str(), deviceBlobCtrlPort, cnt);
}
