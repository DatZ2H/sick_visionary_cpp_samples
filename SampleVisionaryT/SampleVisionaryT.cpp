//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: November 2016
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
#include "VisionaryTData.h"    // Header specific for the Time of Flight data
#include "VisionaryDataStream.h"
#include "PointXYZ.h"
#include "PointCloudPlyWriter.h"

bool runStreamingDemo(const char ipAddress[], unsigned short dataPort, uint32_t numberOfFrames)
{
  using namespace visionary;
  
  // Generate Visionary instance
  auto pDataHandler = std::make_shared<VisionaryTData>();
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
    // Set integrationTimeUs parameter to 3800
    std::printf("Setting integrationTimeUs to 3800\n");
    CoLaCommand setIngrationTimeCommand = CoLaParameterWriter(CoLaCommandType::WRITE_VARIABLE, "integrationTimeUs").parameterUDInt(3800).build();
    CoLaCommand setIngrationTimeResponse = visionaryControl.sendCommand(setIngrationTimeCommand);
    if (setIngrationTimeResponse.getError() == CoLaError::OK)
    {
      std::printf("Successfully set integrationTimeUs to 3800\n");
    }

    //-----------------------------------------------
    // Read integrationTimeUs parameter
    CoLaCommand getIntegrationTimeCommand = CoLaParameterWriter(CoLaCommandType::READ_VARIABLE, "integrationTimeUs").build();
    CoLaCommand integrationTimeResponse = visionaryControl.sendCommand(getIntegrationTimeCommand);
    uint32_t integrationTimeUs = CoLaParameterReader(integrationTimeResponse).readUDInt();
    std::printf("Read integrationTimeUs = %d\n", integrationTimeUs);

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
    const char plyFilePath[] = "VisionaryT.ply";
    std::printf("Writing frame to %s\n", plyFilePath);
    PointCloudPlyWriter::WriteFormatPLY(plyFilePath, pointCloud, pDataHandler->getIntensityMap(), true);
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

    // Get data cartesian/ polar data if available, otherwise pointers are NULL and size is zero. If the camera should send this data, please check the option in SOPAS ET. (Configuration->API data channels)
    //-----------------------------------------------
    // Cartesian data, also used for the Detection grid
    std::vector<PointXYZC> cartesian = pDataHandler->getCartesianData();
    for (std::vector<PointXYZC>::iterator it = cartesian.begin(); it != cartesian.end(); ++it)
    {
      std::printf("X: %g, Y: %g, Z: %g, C: %g \n", it->x, it->y, it->z, it->c);
    }

    // Polar data
    std::vector<float> scanPoints = pDataHandler->getPolarDistanceData();
    for (std::vector<float>::iterator it = scanPoints.begin(); it != scanPoints.end(); ++it)
    {
        std::printf("Scan Point: %g \n", *it);
    }

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
