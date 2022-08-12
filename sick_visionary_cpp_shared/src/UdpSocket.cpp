//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: January 2020
// 
// SICK AG, Waldkirch
// email: TechSupport0905@sick.de

#include <cstring>

#include "UdpSocket.h"

namespace visionary 
{

UdpSocket::UdpSocket()
{
  memset(&m_udpAddr, 0, sizeof(m_udpAddr));
}

int UdpSocket::connect(const std::string& hostname, uint16_t port)
{
  int iResult = 0;
  int trueVal = 1;
  long timeoutSeconds = 5L;
#ifdef _WIN32
  //-----------------------------------------------
  // Initialize Winsock
  WSADATA wsaData;
  iResult = ::WSAStartup(MAKEWORD(2, 2), &wsaData);
  if (iResult != NO_ERROR)
  {
    return iResult;
  }
#endif

  //-----------------------------------------------
  // Create a receiver socket to receive datagrams
  m_socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (m_socket == INVALID_SOCKET) {
    return INVALID_SOCKET;
  }

  //-----------------------------------------------
  // Bind the socket to any address and the specified port.
  m_udpAddr.sin_family = AF_INET;
  m_udpAddr.sin_port = port;
  m_udpAddr.sin_addr.s_addr = inet_addr(hostname.c_str());

  // Set the timeout for the socket to 5 seconds
#ifdef _WIN32
  // On Windows timeout is a DWORD in milliseconds (https://docs.microsoft.com/en-us/windows/desktop/api/winsock/nf-winsock-setsockopt)
  long timeoutMs = timeoutSeconds * 1000L;
  iResult = setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeoutMs, sizeof(timeoutMs));
#else
  struct timeval tv;
  tv.tv_sec = timeoutSeconds;  /* 5 seconds Timeout */
  tv.tv_usec = 0L;  // Not init'ing this can cause strange errors
  iResult = setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(struct timeval));
#endif

  if (iResult >= 0)
  {
    iResult = setsockopt(m_socket, SOL_SOCKET, SO_BROADCAST, (const char*)&trueVal, sizeof(trueVal));
  }

  return iResult;
}

int UdpSocket::shutdown()
{
  // Close the socket when finished receiving datagrams
#ifdef _WIN32
  closesocket(m_socket);
  WSACleanup();
#else
  close(m_socket);
#endif
  m_socket = INVALID_SOCKET;
  return 0;
}

int UdpSocket::send(const std::vector<std::uint8_t>& buffer)
{
  // send buffer via UDP socket
  return sendto(m_socket, reinterpret_cast<const char*>(buffer.data()), (int)buffer.size(), 0, (struct sockaddr*) &m_udpAddr, sizeof(m_udpAddr));
}

int UdpSocket::recv(std::vector<std::uint8_t>& buffer, std::size_t maxBytesToReceive)
{
  // receive from TCP Socket
  buffer.resize(maxBytesToReceive);
  char* pBuffer = reinterpret_cast<char*>(buffer.data());

  return ::recv(m_socket, pBuffer, maxBytesToReceive, 0);
}

int UdpSocket::read(std::vector<std::uint8_t>& buffer, std::size_t nBytesToReceive)
{
  // receive from TCP Socket
  buffer.resize(nBytesToReceive);
  char* pBuffer = reinterpret_cast<char*>(buffer.data());

  int bytesReceived = 0;
  while (nBytesToReceive > 0)
  {
    bytesReceived = ::recv(m_socket, pBuffer, nBytesToReceive, 0);

    if (bytesReceived == SOCKET_ERROR || bytesReceived == 0)
    {
      return false;
    }
    pBuffer += bytesReceived;
    nBytesToReceive -= bytesReceived;
  }
  pBuffer = NULL;
  return bytesReceived;
}

}
