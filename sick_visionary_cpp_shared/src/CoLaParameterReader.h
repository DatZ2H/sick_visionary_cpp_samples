//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: October 2018
// 
// SICK AG, Waldkirch
// email: TechSupport0905@sick.de

#pragma once

#include <cstdint>
#include <string>
#include "CoLaCommand.h"

namespace visionary 
{

/// <summary>
/// Class for reading data from a <see cref="CoLaCommand" />.
/// </summary>
class CoLaParameterReader
{
private:
  CoLaCommand m_command;
  size_t m_currentPosition;

public:
  CoLaParameterReader(CoLaCommand command);
  ~CoLaParameterReader();

  /// <summary>
  /// Rewind the position to the first parameter.
  /// </summary>
  void rewind();

  /// <summary>
  /// Read a signed short int (8 bit, range [-128, 127]) and advances position by 1 byte.
  /// </summary>
  const int8_t readSInt();

  /// <summary>
  /// Read a unsigned short int (8 bit, range [0, 255]) and advances position by 1 byte.
  /// </summary>
  const uint8_t readUSInt();

  /// <summary>
  /// Read a signed int (16 bit, range [-32768, 32767]) and advances position by 2 bytes.
  /// </summary>
  const int16_t readInt();

  /// <summary>
  /// Read a unsigned int (16 bit, range [0, 65535]) and advances position by 2 bytes.
  /// </summary>
  const uint16_t readUInt();

  /// <summary>
  /// Read a signed double int (32 bit) and advances position by 4 bytes.
  /// </summary>
  const int32_t readDInt();

  /// <summary>
  /// Read a unsigned int (32 bit) and advances position by 4 bytes.
  /// </summary>
  const uint32_t readUDInt();

  /// <summary>
  /// Read a IEEE-754 single precision (32 bit) and advances position by 4 bytes.
  /// </summary>
  const float readReal();

  /// <summary>
  /// Read a IEEE-754 double precision (64 bit) and advances position by 8 bytes.
  /// </summary>
  const double readLReal();

  /// <summary>
  /// Read a boolean and advance the position by 1 byte.
  /// </summary>
  const bool readBool();

  /// <summary>
  /// Read a flex string, and advance position according to string size.
  /// </summary>
  std::string readFlexString();
};

}
