/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, EAIBOT, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** @page SDMLidarDriver
 * SDMLidarDriver API
    <table>
        <tr><th>Library     <td>SDMLidarDriver
        <tr><th>File        <td>SDMLidarDriver.h
        <tr><th>Author      <td>ZhanYi [code at ydlidar com]
        <tr><th>Source      <td>https://github.com/ydlidar/YDLidar-SDK
        <tr><th>Version     <td>1.0.0
    </table>
    This SDMLidarDriver support [TYPE_SDM](\ref LidarTypeID::TYPE_SDM) LiDAR

* @copyright    Copyright (c) @2015-2023 EAIBOT
     Jump to the @link ::ydlidar::SDMLidarDriver @endlink interface documentation.
*/
#ifndef SDM_YDLIDAR_DRIVER_H
#define SDM_YDLIDAR_DRIVER_H

#include <stdlib.h>
#include <atomic>
#include <map>
#include "core/serial/serial.h"
#include "core/base/locker.h"
#include "core/base/thread.h"
#include "core/common/ydlidar_protocol.h"
#include "core/common/ydlidar_help.h"

#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The YDLIDAR SDK requires a C++ compiler to be built"
#endif
#endif

#define SDK_SDM_POINT_COUNT 1
#define SDK_CMD_HEADFLAG0 0xAA // Protocol header identifier 1
#define SDK_CMD_HEADFLAG1 0x55 // Protocol header identifier 2
#define SDK_CMD_STARTSCAN 0x60 // Start scanning
#define SDK_CMD_STOPSCAN 0x61 // Stop scanning
#define SDK_CMD_GETVERSION 0x62 // Get version information
#define SDK_CMD_SELFCHECK 0x63 // Self-check
#define SDK_CMD_SETFREQ 0x64 // Set output frequency
#define SDK_CMD_SETFILTER 0x65 // Set filter
#define SDK_CMD_SETBAUDRATE 0x66 // Set serial port baud rate
#define SDK_CMD_SETOUTPUT 0x67 // Set output data format
#define SDK_CMD_RESET 0x68 // Restore factory settings
#define SDK_BUFFER_MAXLEN 100 // Cache length

// Set 1-byte alignment
#pragma pack(1)

//SDM Radar Protocol Header
struct SdkSdmHead {
    uint8_t head0 = 0;
    uint8_t head1 = 0;
    uint8_t cmd = 0;
    uint8_t size = 0;
};
#define SDKSDMHEADSIZE sizeof(SdkSdmHead)
// SDM radar single-point data
struct SdkSdmPc {
    uint16_t dist = 0; // distance
    uint8_t intensity = 0; // intensity
    uint8_t env = 0; // environment interference data
};
// SDM radar point cloud data
struct SdkSdmPcs {
    SdkSdmHead head;
    SdkSdmPc point; // A single data point
    uint8_t cs = 0;
};
#define SDKSDMPCSSIZE sizeof(SdkSdmPcs)
// SDM Radar Equipment Information
struct SdkSdmDeviceInfo {
    uint8_t model = 0; // Radar model number
    uint8_t hv = 0; // Hardware version
    uint8_t fvm = 0; // Firmware major version
    uint8_t fvs = 0; // Firmware minor version
    uint8_t sn[SDK_SNLEN] = {0}; // Serial number
};
#define SDKSDMDEVICEINFOSIZE sizeof(SdkSdmDeviceInfo)

#pragma pack()


using namespace std;

namespace ydlidar 
{

using namespace core;
using namespace core::serial;
using namespace core::base;

/*!
* SDM Control Class
*/
class SDMLidarDriver : public DriverInterface 
{
public:
  // Constructor
  SDMLidarDriver();
  // Destructor
  virtual ~SDMLidarDriver();
  /*!
  * @brief Connecting radar \n
  * After a successful connection, the ::disconnect function must be used to close the connection.
  * @param[in] port serial number
  * @param[in] baudrate baud rate, YDLIDAR-SDM radar baud rate: 961200
  * @return returns connection status
  * @retval 0     success
  * @retval < 0   failure
  * @note After a successful connection, the ::disconnect function must be used to close the connection.
  * @see Function::GSLidarDriver::disconnect ("::" indicates the presence of a connection function, you can see the disconnect in the documentation turn green, click it to jump to disconnect.)
  */
  result_t connect(const char *port, uint32_t baudrate);

  /*!
  * @brief Disconnect radar connection
  */
  void disconnect();

  /*!
  * @brief Get the current SDK version number \n
  * Static function
  * @return Returns the current SDK version number
  */
  virtual std::string getSDKVersion();

  /*!
  * @brief lidarPortList Obtain radar port
  * @return Online Radar List
  */
  static std::map<std::string, std::string> lidarPortList();

  /*!
  * @brief Scanning status \n
  * @return returns the current radar scanning status
  * @retval true     scanning
  * @retval false    not scanning
  */
  bool isscanning() const;

  /*!
  * @brief Connection status \n
  * @return returns the connection status
  * @retval true     successful
  * @retval false    failed
  */
  bool isconnected() const;

  /*!
  * @brief Set automatic reconnection for radar anomalies \n
  * @param[in] enable    whether to enable automatic reconnection:
  *     true	enable
  *	  false closure
  */
  void setAutoReconnect(const bool &enable);

  /*!
 * @brief Configure radar address \n
 * @param[in] timeout  Timeout
 * @return returns execution result
 * @retval RESULT_OK       Configuration successful
 * @retval RESULT_FAILE or RESULT_TIMEOUT   Configuration timeout
 */
  result_t setDeviceAddress(uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief Start Scan \n
  * @param[in] force    Scanning mode
  * @param[in] timeout  Timeout
  * @return returns execution result
  * @retval RESULT_OK       Start successful
  * @retval RESULT_FAILE    Start failed
  * @note Only need to start once successfully
  */
  result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief Stop Scan \n
  * @return returns execution result
  * @retval RESULT_OK       Stop successful
  * @retval RESULT_FAILE    Stop failed
  */
  result_t stop();

  /*!
  * @brief Get Laser Data \n
  * @param[in] nodebuffer Laser point information
  * @param[in] count      Number of laser points in a circle
  * @param[in] timeout    Timeout
  * @return returns execution result
  * @retval RESULT_OK       Successfully obtained
  * @retval RESULT_FAILE    Failed to obtain
  * @note Before obtaining, you must use the ::startScan function to start the scan
  */
  result_t grabScanData(node_info *nodebuffer, size_t &count,
                        uint32_t timeout = DEFAULT_TIMEOUT) ;


  /*!
  * @brief Compensation laser angle \n
  * Limit the angle to between 0 and 360 degrees
  * @param[in] nodebuffer Laser point information
  * @param[in] count      Number of laser points in a circle
  * @return returns execution result
  * @retval RESULT_OK       Success
  * @retval RESULT_FAILE    Failure
  * @note Before compensating, you must use the ::grabScanData function to successfully obtain laser data
  */
  result_t ascendScanData(node_info *nodebuffer, size_t count);

  /*!
  * @brief Reset the LiDAR (restore factory settings) \n
  * @param[in] timeout      Timeout
  * @return returns execution result
  * @retval RESULT_OK       Success
  * @retval RESULT_FAILE    Failure
  * @note Stop scanning before executing the current operation, if scanning is in progress, call the stop function to stop scanning
  */
  result_t reset(uint8_t addr, uint32_t timeout = DEFAULT_TIMEOUT);

  // Enable or disable the filtering function.
  result_t enableFilter(bool yes=true);

  // Set scan frequency
  result_t setScanFreq(float sf, uint32_t timeout);

 protected:

  /*!
  * @brief Create a thread to parse radar data \n
  * @note Before creating the radar data parsing thread, the ::startScan function must be used to successfully start the scan.
  */
  result_t createThread();


  /*!
  * @brief Reconnect and start scanning \n
  * @param[in] force    Scanning mode
  * @param[in] timeout  Timeout
  * @return returns execution result
  * @retval RESULT_OK       Start successful
  * @retval RESULT_FAILE    Start failed
  * @note sdk Automatic reconnection call
  */
  result_t startAutoScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT) ;

  /*!
  * @brief stopScan
  * @param timeout
  * @return
  */
  result_t stopScan(uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
   * @brief waitDevicePackage
   * @param timeout
   * @return
   */
  result_t waitDevicePackage(uint32_t timeout = DEFAULT_TIMEOUT);
  /*!
  * @brief Unpacking laser data \n
  * @param[in] node Laser dot information after unpacking
  * @param[in] timeout     Timeout
  */
  result_t waitPackage(node_info *node, uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief Send data to radar \n
  * @param[in] nodebuffer Laser information pointer
  * @param[in] count      Number of laser points
  * @param[in] timeout      Timeout
  * @return returns execution result
  * @retval RESULT_OK       Success
  * @retval RESULT_TIMEOUT  Timeout
  * @retval RESULT_FAILE    Failure
  */
  result_t waitScanData(node_info *nodebuffer, size_t &count,
                        uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief Laser data parsing thread \n
  */
  int cacheScanData();

  /*!
  * @brief Send data to radar \n
  * @param[in] cmd 	 Command code
  * @param[in] payload      Payload
  * @param[in] payloadsize      Payload size
  * @return returns execution result
  * @retval RESULT_OK       Success
  * @retval RESULT_FAILE    Failure
  */
  result_t sendCmd(uint8_t cmd,
                       const uint8_t *data = NULL,
                       size_t size = 0);

  /*!
  * @brief Wait for response data \n
  * @param[in] cmd Command code
  * @param[out] data Response data
  * @param[in] timeout Timeout
  * @return returns execution result
  * @retval RESULT_OK       Successfully obtained
  * @retval RESULT_TIMEOUT  Timeout
  * @retval RESULT_FAILE    Failed to obtain
  * @note When timeout = -1, it will wait indefinitely
  */
  result_t waitResp(uint8_t cmd,
                   uint32_t timeout = DEFAULT_TIMEOUT);
  result_t waitResp(uint8_t cmd,
                   std::vector<uint8_t> &data,
                   uint32_t timeout = DEFAULT_TIMEOUT);
  /*!
  * @brief Wait for laser data packet header \n
  * @param[out] head Packet header
  * @param[in] cmd Command code
  * @param[out] data Response data
  * @param[in] timeout Timeout
  * @return returns execution result
  * @retval RESULT_OK       Successfully obtained
  * @retval RESULT_TIMEOUT  Timeout
  * @retval RESULT_FAILE    Failed to obtain
  * @note When timeout = -1, it will wait indefinitely
  */
  // result_t waitResHeader(SdkSdmHead *head,
  //                        uint8_t cmd,
  //                        uint32_t timeout = DEFAULT_TIMEOUT);
  // result_t waitResHeader(SdkSdmHead *head,
  //                        uint8_t cmd,
  //                        std::vector<uint8_t> &data,
  //                        uint32_t timeout = DEFAULT_TIMEOUT);

  /*!
  * @brief Wait for fixed number of serial data \n
  * @param[in] data_count 	 Data size to wait for
  * @param[in] timeout    	 Wait time
  * @param[in] returned_size   Actual data size
  * @return returns execution result
  * @retval RESULT_OK       Successfully obtained
  * @retval RESULT_TIMEOUT  Timeout
  * @retval RESULT_FAILE    Failed to obtain
  * @note When timeout = -1, it will wait indefinitely
  */
  result_t waitForData(size_t data_count, uint32_t timeout = DEFAULT_TIMEOUT,
                       size_t *returned_size = NULL);

  /*!
  * @brief Get serial data \n
  * @param[in] data 	 Data pointer
  * @param[in] size    Data size
  * @return returns execution result
  * @retval RESULT_OK       Success
  * @retval RESULT_FAILE    Failure
  */
  result_t getData(uint8_t *data, size_t size);

  /*!
  * @brief Send serial data \n
  * @param[in] data 	 Data pointer
  * @param[in] size    Data size
  * @return returns execution result
  * @retval RESULT_OK       Success
  * @retval RESULT_FAILE    Failure
  */
  result_t sendData(const uint8_t *data, size_t size);
  /*!
  * @brief Close data grabbing channel \n
  */
  void disableDataGrabbing();
  /*!
  * @brief Set serial port DTR \n
  */
  void setDTR();
  /*!
  * @brief Clear serial port DTR \n
  */
  void clearDTR();
  /*!
   * @brief flushSerial
   */
  void flushSerial();
  /*!
   * @brief checkAutoConnecting
   */
  result_t checkAutoConnecting();
  /**
   * @brief Get error information
   * @param isTCP TCP or UDP
   * @return error information
   */
  virtual const char *DescribeError(bool isTCP = false);
  /**
   * @brief GS2 radar has no health information\n
   * @return result status
   * @retval RESULT_OK success
   * @retval RESULT_FAILE or RESULT_TIMEOUT failed
   */
  virtual result_t getHealth(device_health &health, uint32_t timeout = DEFAULT_TIMEOUT);
    /**
   * @brief get Device information \n
   * @param[in] info     Device information
   * @param[in] timeout  timeout
   * @return result status
   * @retval RESULT_OK       success
   * @retval RESULT_FAILE or RESULT_TIMEOUT   failed
   */
  virtual result_t getDeviceInfo(device_info &info, uint32_t timeout = DEFAULT_TIMEOUT);

private:
  serial::Serial *_serial = nullptr; // serial port
  std::vector<uint8_t> recvBuff; // A data cache
  device_health health_;
};

} // namespace ydlidar

#endif // SDM_YDLIDAR_DRIVER_H
