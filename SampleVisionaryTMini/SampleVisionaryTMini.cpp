//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: July 2020
// 
// @auther:  Patrick Ebner
// SICK AG, Waldkirch
// email: TechSupport0905@sick.de

#include <cstdio>
#include <iostream>
#include <memory>
#include <sstream>
#include <fstream>
#include <iterator>

#include "VisionaryControl.h"
#include "CoLaParameterReader.h"
#include "CoLaParameterWriter.h"
#include "VisionaryTMiniData.h"    // Header specific for the Time of Flight data
#include "VisionaryDataStream.h"
#include "PointXYZ.h"
#include "PointCloudPlyWriter.h"

bool runStreamingDemo(const char ipAddress[], unsigned short dataPort, uint32_t numberOfFrames)
{
  using namespace visionary;
  
  // Generate Visionary instance
  auto pDataHandler = std::make_shared<VisionaryTMiniData>();
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
  if (!visionaryControl.open(VisionaryControl::ProtocolType::COLA_2, ipAddress, 5000/*ms*/))
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
    // Set enDepthMask parameter to false
    {
      std::printf("Setting enDepthMask to false\n");
      CoLaCommand setEnDepthMaskCommand = CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "enDepthMask").parameterBool(false).build();
      CoLaCommand setEnDepthMaskResponse = visionaryControl.sendCommand(setEnDepthMaskCommand);
      if (setEnDepthMaskResponse.getError() == CoLaError::OK)
      {
        std::printf("Successfully set enDepthMask to false\n");
      }
    }

    //-----------------------------------------------
    // Read humidity parameter
    CoLaCommand getHumidity = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "humidity").build();
    CoLaCommand humidityResponse = visionaryControl.sendCommand(getHumidity);
    const double humidity = CoLaParameterReader(humidityResponse).readLReal();
    std::printf("Read humidity = %f\n", humidity);

    //-----------------------------------------------
    // Read info messages variable
    CoLaCommand getMessagesCommand = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "MSinfo").build();
    CoLaCommand messagesResponse = visionaryControl.sendCommand(getMessagesCommand);

    //-----------------------------------------------
  }

  {
    CoLaCommand setEnDepthMaskCommand = CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "enDepthMask").parameterBool(true).build();
    CoLaCommand setEnDepthMaskResponse = visionaryControl.sendCommand(setEnDepthMaskCommand);
    if (setEnDepthMaskResponse.getError() != CoLaError::OK)
    {
      std::printf("Failed to set enDepthMask to true\n");
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
    std::printf("Frame received through step called, frame #%d, timestamp: %u \n", pDataHandler->getFrameNum(), pDataHandler->getTimestampMS());

    //-----------------------------------------------
    // Convert data to a point cloud
    std::vector<PointXYZ> pointCloud;
    pDataHandler->generatePointCloud(pointCloud);
    pDataHandler->transformPointCloud(pointCloud);

    //-----------------------------------------------
    // Write point cloud to PLY
    const char plyFilePath[] = "VisionaryT.ply";
    std::printf("Writing frame to %s\n", plyFilePath);
    PointCloudPlyWriter::WriteFormatPLY(plyFilePath, pointCloud, pDataHandler->getIntensityMap(), true);
    std::printf("Finished writing frame to %s\n", plyFilePath);
  }

  //-----------------------------------------------
  // Start image acquisiton and continously receive frames
  visionaryControl.startAcquisition();
  for (uint32_t i = 0; i < numberOfFrames; i++)
  {
    if (!dataStream.getNextFrame())
    {
      continue;     // No valid frame received
    }
    std::printf("Frame received in continuous mode, frame #%d \n", pDataHandler->getFrameNum());
    std::vector<uint16_t> intensityMap = pDataHandler->getIntensityMap();
  }

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
