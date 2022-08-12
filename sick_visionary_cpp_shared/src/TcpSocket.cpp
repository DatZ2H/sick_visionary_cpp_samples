//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: November 2019
// 
// SICK AG, Waldkirch
// email: TechSupport0905@sick.de

#include "TcpSocket.h"

namespace visionary 
{

int TcpSocket::connect(const std::string& hostname, uint16_t port)
{
  int iResult = 0;
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
  m_socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (m_socket == INVALID_SOCKET) {
    return INVALID_SOCKET;
  }

  //-----------------------------------------------
  // Bind the socket to any address and the specified port.
  sockaddr_in recvAddr;
  recvAddr.sin_family = AF_INET;
  recvAddr.sin_port = port;
  recvAddr.sin_addr.s_addr = inet_addr(hostname.c_str());

  iResult = ::connect(m_socket, (sockaddr*)&recvAddr, sizeof(recvAddr));
  if (iResult != 0)
  {
    return iResult;
  }

  // Set the timeout for the socket to 5 seconds
  long timeoutSeconds = 5L;
#ifdef _WIN32
  // On Windows timeout is a DWORD in milliseconds (https://docs.microsoft.com/en-us/windows/desktop/api/winsock/nf-winsock-setsockopt)
  long timeoutMs = timeoutSeconds * 1000L;
  iResult = setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeoutMs, sizeof(DWORD));
#else
  struct timeval tv;
  tv.tv_sec = timeoutSeconds;  /* 5 seconds Timeout */
  tv.tv_usec = 0L;  // Not init'ing this can cause strange errors
  iResult = setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(struct timeval));
#endif

  return iResult;
}

int TcpSocket::shutdown()
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

int TcpSocket::send(const std::vector<std::uint8_t>& buffer)
{
  // send buffer via TCP socket
  return ::send(m_socket, (char*)buffer.data(), (int)buffer.size(), 0);
}

int TcpSocket::recv(std::vector<std::uint8_t>& buffer, std::size_t maxBytesToReceive)
{
  // receive from TCP Socket
  buffer.resize(maxBytesToReceive);
  char* pBuffer = reinterpret_cast<char*>(buffer.data());

  return ::recv(m_socket, pBuffer, maxBytesToReceive, 0);
}

int TcpSocket::read(std::vector<std::uint8_t>& buffer, std::size_t nBytesToReceive)
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
