//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: May 2019
// 
// SICK AG, Waldkirch
// email: TechSupport0905@sick.de

#include <vector>

#include "VisionaryAutoIPScan.h"

int main()
{
  unsigned int timeout = 5000;  // The time how long to wait for a response from the devices. 
  VisionaryAutoIPScan ipScan;

  std::vector<VisionaryAutoIPScan::DeviceInfo> deviceList = ipScan.doScan(timeout);  // scan for devices
  printf("Number of found devices: %u \n", deviceList.size());

  // print device info for every found device
  for (auto it : deviceList)
  {
    printf("Device name: %s \n", it.DeviceName.c_str());
    printf("MAC Address: %s \n", it.MacAddress.c_str());
    printf("IP Address: %s \n", it.IpAddress.c_str());
    printf("Subnet: %s \n", it.SubNet.c_str());
    printf("Port %s \n", it.Port.c_str());
  }

  // Wait for user before closing console
  printf("Press Enter to continue . . .");
  getchar();
  return 0;
}