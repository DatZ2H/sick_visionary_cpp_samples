//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: November 2019
// 
// SICK AG, Waldkirch
// email: TechSupport0905@sick.de

#include <cstdio>
#include <cmath>

#include "VisionaryTData.h"
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

VisionaryTData::VisionaryTData() : VisionaryData()
{
}

VisionaryTData::~VisionaryTData()
{
}

bool VisionaryTData::parseXML(const std::string & xmlString, uint32_t changeCounter)
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
  m_dataSetsActive.hasDataSetPolar2D = static_cast<bool>(dataSetsTree.get_child_optional("DataSetPolar2D"));
  m_dataSetsActive.hasDataSetCartesian = static_cast<bool>(dataSetsTree.get_child_optional("DataSetCartesian"));

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
    m_confidenceByteDepth = getItemLength(dataStreamTree.get<std::string>("Confidence", ""));

    const auto distanceDecimalExponent = dataStreamTree.get<int>("Distance.<xmlattr>.decimalexponent", 0);
    m_scaleZ = powf(10.0f, static_cast<float>(distanceDecimalExponent));
  }
  // DataSetPolar2D specific data
  m_numPolarValues = dataSetsTree.get_child("DataSetPolar2D.FormatDescription.DataStream.<xmlattr>.datalength", empty_ptree()).get_value<uint8_t>(0);

  // DataSetCartesian specific data
  if (m_dataSetsActive.hasDataSetCartesian)
  {
    boost::property_tree::ptree dataStreamTree = dataSetsTree.get_child("DataSetCartesian.FormatDescriptionCartesian.DataStream", empty_ptree());
    if ("uint32" != dataStreamTree.get<std::string>("Length", "") ||
      "float32" != dataStreamTree.get<std::string>("X", "") ||
      "float32" != dataStreamTree.get<std::string>("Y", "") ||
      "float32" != dataStreamTree.get<std::string>("Z", "") ||
      "float32" != dataStreamTree.get<std::string>("Intensity", ""))
    {
      std::printf("DataSet Cartesian does not contain the expected format. Won't be used");
      m_dataSetsActive.hasDataSetCartesian = false;
    }
    // To be sure float is 32 bit on this machine, otherwise the parsing of the binary part won't work
    assert(sizeof(float) == 4);
  }

  return true;
}

bool VisionaryTData::parseBinaryData(std::vector<uint8_t>::iterator itBuf, size_t size)
{
  size_t dataSetslength = 0;

  if (m_dataSetsActive.hasDataSetDepthMap)
  {
    const size_t numPixel = m_cameraParams.width * m_cameraParams.height;
    const size_t numBytesDistance = numPixel * m_distanceByteDepth;
    const size_t numBytesIntensity = numPixel * m_intensityByteDepth;
    const size_t numBytesConfidence = numPixel * m_confidenceByteDepth;

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
      wprintf(L"Malformed data, length in header does not match package size.");
      return false;
    }
  }
  else
  {
    m_distanceMap.clear();
    m_intensityMap.clear();
    m_confidenceMap.clear();
  }
  
  if (m_dataSetsActive.hasDataSetPolar2D)
  {
    const uint32_t length = readUnalignLittleEndian<uint32_t>(&*itBuf);
    dataSetslength += length;
    if (dataSetslength > size)
    {
      std::printf("Malformed data, length in polar scan header does not match package size.");
      return false;
    }
    itBuf += sizeof(uint32_t);
    m_blobTimestamp = readUnalignLittleEndian<uint64_t>(&*itBuf);
    itBuf += sizeof(uint64_t);

    //const uint16_t deviceID = readUnalignLittleEndian<uint16_t>(&*itBuf);
    itBuf += sizeof(uint16_t);
    //const uint32_t scanCounter = readUnalignLittleEndian<uint32_t>(&*itBuf);
    itBuf += sizeof(uint32_t);
    //const uint32_t systemCounterScan = readUnalignLittleEndian<uint32_t>(&*itBuf);
    itBuf += sizeof(uint32_t);
    //const float scanFrequency = readUnalignLittleEndian<float>(&*itBuf);
    itBuf += sizeof(float);
    //const float measurementFrequency = readUnalignLittleEndian<float>(&*itBuf);
    itBuf += sizeof(float);

    m_angleFirstScanPoint = readUnalignLittleEndian<float>(&*itBuf);
    itBuf += sizeof(float);
    m_angularResolution = readUnalignLittleEndian<float>(&*itBuf);
    itBuf += sizeof(float);
    //const float polarScale = readUnalignLittleEndian<float>(&*itBuf);
    itBuf += sizeof(float);
    //const float polarOffset = readUnalignLittleEndian<float>(&*itBuf);
    itBuf += sizeof(float);

    m_polarDistanceData.resize(m_numPolarValues);

    memcpy(&m_polarDistanceData[0], &*itBuf, (m_numPolarValues * sizeof(float)));
    itBuf += (m_numPolarValues * sizeof(float));

    //const float rssiAngleFirstScanPoint = readUnalignLittleEndian<float>(&*itBuf);
    itBuf += sizeof(float);
    //const float rssiAngularResolution = readUnalignLittleEndian<float>(&*itBuf);
    itBuf += sizeof(float);
    //const float rssiPolarScale = readUnalignLittleEndian<float>(&*itBuf);
    itBuf += sizeof(float);
    //const float rssiPolarOffset = readUnalignLittleEndian<float>(&*itBuf);
    itBuf += sizeof(float);

    m_polarConfidenceData.resize(m_numPolarValues);
    memcpy(&m_polarConfidenceData[0], &*itBuf, (m_numPolarValues * sizeof(float)));
    itBuf += (m_numPolarValues * sizeof(float));

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
  }
  else
  {
    m_polarDistanceData.clear();
    m_polarConfidenceData.clear();
  }

  m_numCartesianValues = 0;
  if (m_dataSetsActive.hasDataSetCartesian)
  {
    const uint32_t length = readUnalignLittleEndian<uint32_t>(&*itBuf);
    dataSetslength += length;
    if (dataSetslength > size)
    {
      std::printf("Malformed data, length in cartesian header does not match package size.");
      return false;
    }
    itBuf += sizeof(uint32_t);
    m_blobTimestamp = readUnalignLittleEndian<uint64_t>(&*itBuf);
    itBuf += sizeof(uint64_t);
    //const uint16_t version = readUnalignLittleEndian<uint16_t>(&*itBuf);
    itBuf += sizeof(uint16_t);

    m_numCartesianValues = readUnalignLittleEndian<uint32_t>(&*itBuf);
    itBuf += sizeof(uint32_t);
    
    m_cartesianData.resize(m_numCartesianValues);
    memcpy(&m_cartesianData[0], &*itBuf, (m_numCartesianValues * sizeof(PointXYZC)));
    itBuf += (m_numCartesianValues * sizeof(PointXYZC));

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
  }
  else
  {
    m_cartesianData.clear();
  }

  return true;
}

void VisionaryTData::generatePointCloud(std::vector<PointXYZ> &pointCloud)
{
  return VisionaryData::generatePointCloud(m_distanceMap, VisionaryData::RADIAL, pointCloud);
}

const std::vector<uint16_t>& VisionaryTData::getDistanceMap() const
{
  return m_distanceMap;
}

const std::vector<uint16_t>& VisionaryTData::getIntensityMap() const
{
  return m_intensityMap;
}

const std::vector<uint16_t>& VisionaryTData::getConfidenceMap() const
{
  return m_confidenceMap;
}

uint8_t VisionaryTData::getPolarSize() const
{
  return m_numPolarValues;
}

float VisionaryTData::getPolarStartAngle() const
{
    return m_angleFirstScanPoint;
}

float VisionaryTData::getPolarAngularResolution() const
{
    return m_angularResolution;
}

const std::vector<float>& VisionaryTData::getPolarDistanceData() const
{
  return m_polarDistanceData;
}

const std::vector<float>& VisionaryTData::getPolarConfidenceData() const
{
  return m_polarConfidenceData;
}

uint32_t VisionaryTData::getCartesianSize() const
{
  return m_numCartesianValues;
}

const std::vector<PointXYZC>& VisionaryTData::getCartesianData()const
{
  return m_cartesianData;
}

}
