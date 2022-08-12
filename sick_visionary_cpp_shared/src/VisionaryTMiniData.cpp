//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: August 2020
// 
// SICK AG, Waldkirch
// email: TechSupport0905@sick.de

#include <cstdio>

#include "VisionaryTMiniData.h"
#include "VisionaryEndian.h"

// Boost library used for parseXML function
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

namespace visionary 
{

static const boost::property_tree::ptree& empty_ptree() {
  static boost::property_tree::ptree t;
  return t;
}

const float VisionaryTMiniData::DISTANCE_MAP_UNIT = 0.25f;

VisionaryTMiniData::VisionaryTMiniData() : VisionaryData()
{
}

VisionaryTMiniData::~VisionaryTMiniData()
{
}

bool VisionaryTMiniData::parseXML(const std::string & xmlString, uint32_t changeCounter)
{
  //-----------------------------------------------
  // Check if the segment data changed since last receive
  if (m_changeCounter == changeCounter)
  {
    return true;  //Same XML content as on last received blob
  }
  m_changeCounter = changeCounter;
  m_preCalcCamInfoType = VisionaryData::UNKNOWN;

  //-----------------------------------------------
  // Build boost::property_tree for easy XML handling
  boost::property_tree::ptree xmlTree;
  std::istringstream ss(xmlString);
  try {
    boost::property_tree::xml_parser::read_xml(ss, xmlTree);
  }
  catch (...) {
    std::printf("Reading XML tree in BLOB failed.");
    return false;
  }

  //-----------------------------------------------
  // Extract information stored in XML with boost::property_tree
  const boost::property_tree::ptree dataSetsTree = xmlTree.get_child("SickRecord.DataSets", empty_ptree());
  m_dataSetsActive.hasDataSetDepthMap = static_cast<bool>(dataSetsTree.get_child_optional("DataSetDepthMap"));

  // DataSetDepthMap specific data 
  {
    boost::property_tree::ptree dataStreamTree = dataSetsTree.get_child("DataSetDepthMap.FormatDescriptionDepthMap.DataStream", empty_ptree());

    m_cameraParams.width = dataStreamTree.get<int>("Width", 0);
    m_cameraParams.height = dataStreamTree.get<int>("Height", 0);

    if (m_dataSetsActive.hasDataSetDepthMap)
    {
      int i = 0;
      BOOST_FOREACH(const boost::property_tree::ptree::value_type &item, dataStreamTree.get_child("CameraToWorldTransform"))
      {
        m_cameraParams.cam2worldMatrix[i] = item.second.get_value<double>(0.);
        ++i;
      }
    }
    else
    {
      std::fill(m_cameraParams.cam2worldMatrix, m_cameraParams.cam2worldMatrix + 16, 0.0);
    }

    m_cameraParams.fx = dataStreamTree.get<double>("CameraMatrix.FX", 0.0);
    m_cameraParams.fy = dataStreamTree.get<double>("CameraMatrix.FY", 0.0);
    m_cameraParams.cx = dataStreamTree.get<double>("CameraMatrix.CX", 0.0);
    m_cameraParams.cy = dataStreamTree.get<double>("CameraMatrix.CY", 0.0);

    m_cameraParams.k1 = dataStreamTree.get<double>("CameraDistortionParams.K1", 0.0);
    m_cameraParams.k2 = dataStreamTree.get<double>("CameraDistortionParams.K2", 0.0);
    m_cameraParams.p1 = dataStreamTree.get<double>("CameraDistortionParams.P1", 0.0);
    m_cameraParams.p2 = dataStreamTree.get<double>("CameraDistortionParams.P2", 0.0);
    m_cameraParams.k3 = dataStreamTree.get<double>("CameraDistortionParams.K3", 0.0);

    m_cameraParams.f2rc = dataStreamTree.get<double>("FocalToRayCross", 0.0);

    m_distanceByteDepth = getItemLength(dataStreamTree.get<std::string>("Distance", ""));
    m_intensityByteDepth = getItemLength(dataStreamTree.get<std::string>("Intensity", ""));

    //const auto distanceDecimalExponent = dataStreamTree.get<int>("Distance.<xmlattr>.decimalexponent", 0);
    // Scaling is fixed to 0.25mm on ToF Mini
    m_scaleZ = DISTANCE_MAP_UNIT;
  }
  
  return true;
}

bool VisionaryTMiniData::parseBinaryData(std::vector<uint8_t>::iterator itBuf, size_t size)
{
  size_t dataSetslength = 0;

  if (m_dataSetsActive.hasDataSetDepthMap)
  {
    const size_t numPixel = m_cameraParams.width * m_cameraParams.height;
    const size_t numBytesDistance = numPixel * m_distanceByteDepth;
    const size_t numBytesIntensity = numPixel * m_intensityByteDepth;

    //-----------------------------------------------
    // The binary part starts with entries for length, a timestamp
    // and a version identifier
    const uint32_t length = readUnalignLittleEndian<uint32_t>(&*itBuf);
    dataSetslength += length;
    if (dataSetslength > size)
    {
      wprintf(L"Malformed data, length in depth map header does not match package size.");
      return false;
    }
    itBuf += sizeof(uint32_t);

    m_blobTimestamp = readUnalignLittleEndian<uint64_t>(&*itBuf);
    itBuf += sizeof(uint64_t);

    const uint16_t version = readUnalignLittleEndian<uint16_t>(&*itBuf);
    itBuf += sizeof(uint16_t);

    //-----------------------------------------------
    // The content of the Data part inside a data set has changed since the first released version.
    if (version > 1) {
      // more frame information follows in this case: frame number, data quality, device status
      m_frameNum = readUnalignLittleEndian<uint32_t>(&*itBuf);
      itBuf += sizeof(uint32_t);

      //const uint8_t dataQuality = readUnalignLittleEndian<uint8_t>(&*itBuf);
      itBuf += sizeof(uint8_t);

      //const uint8_t deviceStatus = readUnalignLittleEndian<uint8_t>(&*itBuf);
      itBuf += sizeof(uint8_t);
    }
    else {
      ++m_frameNum;
    }

    //-----------------------------------------------
    // Extract the Images depending on the informations extracted from the XML part
    m_distanceMap.resize(numPixel);
    memcpy(&m_distanceMap[0], &*itBuf, numBytesDistance);
    itBuf += numBytesDistance;

    m_intensityMap.resize(numPixel);
    memcpy(&m_intensityMap[0], &*itBuf, numBytesIntensity);
    itBuf += numBytesIntensity;

    //-----------------------------------------------
    // Data ends with a (unused) 4 Byte CRC field and a copy of the length byte
    //const uint32_t unusedCrc = readUnalignLittleEndian<uint32_t>(&*itBuf);
    itBuf += sizeof(uint32_t);

    const uint32_t lengthCopy = readUnalignLittleEndian<uint32_t>(&*itBuf);
    itBuf += sizeof(uint32_t);

    if (length != lengthCopy)
    {
      wprintf(L"Malformed data, length in header does not match package size.");
      return false;
    }
  }
  else
  {
    m_distanceMap.clear();
    m_intensityMap.clear();
  }

  return true;
}

void VisionaryTMiniData::generatePointCloud(std::vector<PointXYZ> &pointCloud)
{
  return VisionaryData::generatePointCloud(m_distanceMap, VisionaryData::RADIAL, pointCloud);
}

const std::vector<uint16_t>& VisionaryTMiniData::getDistanceMap() const
{
  return m_distanceMap;
}

const std::vector<uint16_t>& VisionaryTMiniData::getIntensityMap() const
{
  return m_intensityMap;
}

}
