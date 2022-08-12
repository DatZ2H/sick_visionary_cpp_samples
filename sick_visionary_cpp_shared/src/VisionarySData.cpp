//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: November 2019
// 
// SICK AG, Waldkirch
// email: TechSupport0905@sick.de

#include <cstdio>
#include <cmath>

#include "VisionarySData.h"
#include "VisionaryEndian.h"

// Boost library used for parseXML function
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

namespace visionary 
{

VisionarySData::VisionarySData() : VisionaryData()
{
}

VisionarySData::~VisionarySData()
{
}

bool VisionarySData::parseXML(const std::string & xmlString, uint32_t changeCounter)
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

  boost::property_tree::ptree dataStreamTree = xmlTree.get_child("SickRecord.DataSets.DataSetStereo.FormatDescriptionDepthMap.DataStream");

  //-----------------------------------------------
  // Extract information stored in XML with boost::property_tree
  m_cameraParams.width = dataStreamTree.get<int>("Width", 0);
  m_cameraParams.height = dataStreamTree.get<int>("Height", 0);

  int i = 0;
  BOOST_FOREACH(const boost::property_tree::ptree::value_type &item, dataStreamTree.get_child("CameraToWorldTransform")) {
    m_cameraParams.cam2worldMatrix[i] = item.second.get_value<double>(0.);
    ++i;
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

  m_zByteDepth = getItemLength(dataStreamTree.get<std::string>("Z", ""));
  m_rgbaByteDepth = getItemLength(dataStreamTree.get<std::string>("Intensity", ""));
  m_confidenceByteDepth = getItemLength(dataStreamTree.get<std::string>("Confidence", ""));

  const auto distanceDecimalExponent = dataStreamTree.get<int>("Z.<xmlattr>.decimalexponent", 0);
  m_scaleZ = powf(10.0f, static_cast<float>(distanceDecimalExponent));

  return true;
}

bool VisionarySData::parseBinaryData(std::vector<uint8_t>::iterator itBuf, size_t size)
{
  const size_t numPixel = m_cameraParams.width * m_cameraParams.height;
  const size_t numBytesZ = numPixel * m_zByteDepth;
  const size_t numBytesRGBA = numPixel * m_rgbaByteDepth;
  const size_t numBytesConfidence = numPixel * m_confidenceByteDepth;


  //-----------------------------------------------
  // The binary part starts with entries for length, a timestamp
  // and a version identifier
    const uint32_t length = readUnalignLittleEndian<uint32_t>(&*itBuf);
    if (length > size)
    {
      std::printf("Malformed data, length in depth map header does not match package size.");
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
    m_zMap.resize(numPixel);
    memcpy(&m_zMap[0], &*itBuf, numBytesZ);
    itBuf += numBytesZ;

    m_rgbaMap.resize(numPixel);
    memcpy(&m_rgbaMap[0], &*itBuf, numBytesRGBA);
    itBuf += numBytesRGBA;

    m_confidenceMap.resize(numPixel);
    memcpy(&m_confidenceMap[0], &*itBuf, numBytesConfidence);
    itBuf += numBytesConfidence;

    //-----------------------------------------------
    // Data ends with a (unused) 4 Byte CRC field and a copy of the length byte
    //const uint32_t unusedCrc = readUnalignLittleEndian<uint32_t>(&*itBuf);
    itBuf += sizeof(uint32_t);

    const uint32_t lengthCopy = readUnalignLittleEndian<uint32_t>(&*itBuf);
    itBuf += sizeof(uint32_t);

    if (length != lengthCopy)
    {
      std::printf("Malformed data, length in header does not match package size.");
      return false;
    }

  return true;
}

void VisionarySData::generatePointCloud(std::vector<PointXYZ> &pointCloud)
{
  return VisionaryData::generatePointCloud(m_zMap, VisionaryData::PLANAR, pointCloud);
}

const std::vector<uint16_t>& VisionarySData::getZMap() const
{
  return m_zMap;
}

const std::vector<uint32_t>& VisionarySData::getRGBAMap() const
{
  return m_rgbaMap;
}

const std::vector<uint16_t>& VisionarySData::getConfidenceMap() const
{
  return m_confidenceMap;
}

}
