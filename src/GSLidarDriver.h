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

/** @page GSLidarDriver
 * GSLidarDriver API
    <table>
        <tr><th>Library     <td>GSLidarDriver
        <tr><th>File        <td>GSLidarDriver.h
        <tr><th>Author      <td>Tony [code at ydlidar com]
        <tr><th>Source      <td>https://github.com/ydlidar/YDLidar-SDK
        <tr><th>Version     <td>1.0.0
    </table>
    This GSLidarDriver support [TYPE_TRIANGLE](\ref LidarTypeID::TYPE_TRIANGLE) and [TYPE_TOF](\ref LidarTypeID::TYPE_TOF) LiDAR

* @copyright    Copyright (c) 2018-2020  EAIBOT
     Jump to the @link ::ydlidar::GSLidarDriver @endlink interface documentation.
*/
#ifndef GS2_YDLIDAR_DRIVER_H
#define GS2_YDLIDAR_DRIVER_H

#include <stdlib.h>
#include <atomic>
#include <map>
#include <list>
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

using namespace std;

namespace ydlidar
{

  using namespace core;
  using namespace core::serial;
  using namespace core::base;

  /*!
   * GS Radar Control
   */
  class GSLidarDriver : public DriverInterface
  {
  public:
    /*!
     * A constructor.
     * A more elaborate description of the constructor.
     */
    GSLidarDriver(uint8_t type = YDLIDAR_TYPE_SERIAL);
    /*!
     * A destructor.
     * A more elaborate description of the destructor.
     */
    virtual ~GSLidarDriver();

    /*!
     * @brief Connecting radar \n
     * After a successful connection, the ::disconnect function must be used to close the connection.
     * @param[in] port_path serial number
     * @param[in] baudrate Baud rate, YDLIDAR-GS2 radar baud rate: 961200
     * @return Returns connection status
     * @retval 0     Success
     * @retval < 0   Failure
     * @note After a successful connection, the ::disconnect function must be used to close the connection.
     * @see Function::GSLidarDriver::disconnect (“::” indicates the presence of a connection function, you can see the disconnect function turn green in the documentation, clicking it will take you to the disconnect function.)
     */
    result_t connect(const char *port_path, uint32_t baudrate);

    /*!
     * @brief Disconnecting radar \n
     */
    void disconnect();

    /*!
     * @brief Getting current SDK version \n
     * Static function
     * @return Returns current SDK version
     */
    virtual std::string getSDKVersion();

    /*!
     * @brief lidarPortList Obtain radar port
     * @return Online lidar list
     */
    static std::map<std::string, std::string> lidarPortList();

    /*!
     * @brief Scanning status \n
     * @return Returns current lidar scanning status
     * @retval true     Scanning
     * @retval false    Scanning stopped
     */
    bool isscanning() const;

    /*!
     * @brief Connection status \n
     * @return Returns connection status
     * @retval true     Success
     * @retval false    Failure
     */
    bool isconnected() const;

    /*!
     * @brief Set whether the radar has signal quality. \n
     * After a successful connection, the ::disconnect function must be used to close the connection.
     * @param[in] isintensities    Whether to include signal quality:
     *     true	Include signal quality
     *	  false	Exclude signal quality
     * @note Only S4B (baud rate is 153600) radar supports signal quality, other models do not support it.
     */
    void setIntensities(const bool &isintensities);

    /*!
     * @brief Set whether the radar automatically reconnects on failure. \n
     * @param[in] enable    Whether to enable automatic reconnection:
     *     true	Enable
     *	  false Disable
     */
    void setAutoReconnect(const bool &enable);

    /*!
     * @brief Obtain radar equipment information \n
     * @param[in] parameters     Device information
     * @param[in] timeout  Timeout duration
     * @return Returns execution result
     * @retval RESULT_OK       Success
     * @retval RESULT_FAILE or RESULT_TIMEOUT   Failure
     */
    result_t getDevicePara(gs_device_para &info, uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief Configure radar address \n
     * @param[in] timeout  Timeout duration
     * @return Returns execution result
     * @retval RESULT_OK       Configuration successful
     * @retval RESULT_FAILE or RESULT_TIMEOUT   Configuration timeout
     */
    result_t setDeviceAddress(uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief Start scanning \n
     * @param[in] force    Scan mode
     * @param[in] timeout  Timeout duration
     * @return Returns execution result
     * @retval RESULT_OK       Start successful
     * @retval RESULT_FAILE    Start failed
     * @note Only need to start once successfully
     */
    result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief Stop scanning \n
     * @return Returns execution result
     * @retval RESULT_OK       Stop successful
     * @retval RESULT_FAILE    Stop failed
     */
    result_t stop();

    /*!
     * @brief Get laser data \n
     * @param[in] nodebuffer Laser point information
     * @param[in] count      Number of laser points per revolution
     * @param[in] timeout    Timeout duration
     * @return Returns execution result
     * @retval RESULT_OK       Get successful
     * @retval RESULT_FAILE    Get failed
     * @note Before getting the data, you must use the ::startScan function to start the scan
     */
    result_t grabScanData(node_info *nodebuffer, size_t &count,
                          uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief Compensation laser angle \n
     * Limit the angle to between 0 and 360 degrees
     * @param[in] nodebuffer Laser point information
     * @param[in] count      Number of laser points per revolution
     * @return Returns execution result
     * @retval RESULT_OK       Success
     * @retval RESULT_FAILE    Failure
     * @note Before compensating, you must use the ::grabScanData function to successfully obtain laser data
     */
    result_t ascendScanData(node_info *nodebuffer, size_t count);

    /*!
     * @brief Reset laser radar \n
     * @param[in] timeout      Timeout duration
     * @return Returns execution result
     * @retval RESULT_OK       Success
     * @retval RESULT_FAILE    Failure
     * @note Stop scanning before executing the current operation, if scanning is in progress, call the ::stop function to stop scanning
     */
    result_t reset(uint8_t addr, uint32_t timeout = DEFAULT_TIMEOUT);

  protected:
    /*!
     * @brief Create a thread to parse radar data \n
     * @note Before creating the thread to parse radar data, you must use the ::startScan function to start the scan successfully
     */
    result_t createThread();

    /*!
     * @brief Reconnect and start scanning \n
     * @param[in] force    Scan mode
     * @param[in] timeout  Timeout duration
     * @return Returns execution result
     * @retval RESULT_OK       Start successful
     * @retval RESULT_FAILE    Start failed
     * @note sdk automatically reconnects and starts scanning
     */
    result_t startAutoScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT);

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
     * @param[in] timeout     Timeout duration
     */
    result_t waitPackage(node_info *node, uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief Send data to radar \n
     * @param[in] nodebuffer Laser information pointer
     * @param[in] count      Number of laser points
     * @param[in] timeout      Timeout duration
     * @return Returns execution result
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
     * @return Returns execution result
     * @retval RESULT_OK       Success
     * @retval RESULT_FAILE    Failure
     */
    result_t sendCommand(uint8_t cmd,
                         const void *payload = NULL,
                         size_t payloadsize = 0);

    /*!
     * @brief Send data to radar \n
     * @param[in] addr Module address
     * @param[in] cmd 	 Command code
     * @param[in] payload      Payload
     * @param[in] payloadsize      Payload size
     * @return Returns execution result
     * @retval RESULT_OK       Success
     * @retval RESULT_FAILE    Failure
     */
    result_t sendCommand(uint8_t addr,
                         uint8_t cmd,
                         const void *payload = NULL,
                         size_t payloadsize = 0);

    /*!
     * @brief Wait for laser data header \n
     * @param[in] header 	 Header
     * @param[in] timeout      Timeout duration
     * @return Returns execution result
     * @retval RESULT_OK       Success
     * @retval RESULT_TIMEOUT  Timeout
     * @retval RESULT_FAILE    Failure
     * @note When timeout = -1, it will keep waiting
     */
    result_t waitResponseHeader(gs_package_head *header,
                                uint32_t timeout = DEFAULT_TIMEOUT);
    result_t waitResponseHeaderEx(gs_package_head *header,
                                  uint8_t cmd,
                                  uint32_t timeout = DEFAULT_TIMEOUT);

    /*!
     * @brief Wait for a fixed number of serial data \n
     * @param[in] data_count 	 Data size to wait for
     * @param[in] timeout    	 Timeout duration
     * @param[in] returned_size   Actual data size
     * @return Returns execution result
     * @retval RESULT_OK       Success
     * @retval RESULT_TIMEOUT  Timeout
     * @retval RESULT_FAILE    Failure
     * @note When timeout = -1, it will keep waiting
     */
    result_t waitForData(size_t data_count, uint32_t timeout = DEFAULT_TIMEOUT,
                         size_t *returned_size = NULL);

    /*!
     * @brief Get serial port data \n
     * @param[in] data 	 Data pointer
     * @param[in] size    Data size
     * @return Returns execution result
     * @retval RESULT_OK       Success
     * @retval RESULT_FAILE    Failure
     */
    result_t getData(uint8_t *data, size_t size);

    /*!
     * @brief Send data through serial port \n
     * @param[in] data 	 Data pointer
     * @param[in] size    Data size
     * @return Returns execution result
     * @retval RESULT_OK       Success
     * @retval RESULT_FAILE    Failure
     */
    result_t sendData(const uint8_t *data, size_t size);

    /*!
     * @brief checkTransDelay
     */
    void checkTransDelay();

    /*!
     * @brief Close data acquisition channel \n
     */
    void disableDataGrabbing();

    /*!
     * @brief Configure serial port DTR \n
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

    /*!
     * @brief  The distance and angle of the point can be calculated.
     */
    void angTransform(uint16_t dist, int n, double *dstTheta, uint16_t *dstDist);
    void angTransform2(uint16_t dist, int n, double *dstTheta, uint16_t *dstDist);

    /**
     * @brief Serial port error message
     * @param isTCP   TCP or UDP
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

    // Obtain device information
    virtual result_t getDeviceInfo(
      device_info &di, 
      uint32_t timeout = DEFAULT_TIMEOUT/4);
    // Obtain cascaded radar device information
    virtual result_t getDeviceInfo(
      std::vector<device_info_ex> &dis,
      uint32_t timeout = DEFAULT_TIMEOUT);
    virtual result_t getDeviceInfo1(
      device_info &di, 
      uint32_t timeout = DEFAULT_TIMEOUT);
    virtual result_t getDeviceInfo2(
      device_info &di, 
      uint32_t timeout = DEFAULT_TIMEOUT);

    /**
     * @brief Set lidar work mode (currently only for GS2 lidar)
     * @param[in] mode Lidar work mode, 0 for obstacle avoidance mode; 1 for edge following mode
     * @param[in] addr Lidar address, first lidar address is 0x01; second lidar address is 0x02; third lidar address is 0x04;
     * @return Returns RESULT_OK on success, otherwise returns non-RESULT_OK
     */
    virtual result_t setWorkMode(int mode = 0, uint8_t addr = 0x00);

    // Start OTA upgrade
    virtual bool ota();
    // Start OTA
    bool startOta(uint8_t addr);
    // OTA upgrade in progress
    bool execOta(uint8_t addr, const std::vector<uint8_t>& data);
    // Stop OTA
    bool stopOta(uint8_t addr);
    // Determine if the response is normal
    bool isOtaRespOk(uint8_t addr,
                     uint8_t cmd,
                     uint16_t offset,
                     const std::vector<uint8_t>& data);
    bool sendData(uint8_t addr,
                  uint8_t cmd,
                  const std::vector<uint8_t> &data,
                  uint8_t cmdRecv,
                  std::vector<uint8_t> &dataRecv,
                  int timeout = 500);

    // Unimplemented virtual functions
    virtual result_t getScanFrequency(scan_frequency &frequency, uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setScanFrequencyDis(scan_frequency &frequency,
                                         uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setScanFrequencyAdd(scan_frequency &frequency,
                                         uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setScanFrequencyAddMic(scan_frequency &frequency,
                                            uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setScanFrequencyDisMic(scan_frequency &frequency,
                                            uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t getSamplingRate(sampling_rate &rate,
                                     uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setSamplingRate(sampling_rate &rate,
                                     uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t getZeroOffsetAngle(offset_angle &angle,
                                        uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }
    virtual result_t setScanHeartbeat(scan_heart_beat &beat,
                                      uint32_t timeout = DEFAULT_TIMEOUT) { return RESULT_OK; }

  public:
    enum
    {
      DEFAULT_TIMEOUT = 2000,    /**< Default timeout. */
      DEFAULT_HEART_BEAT = 1000, /**< Default heartbeat time. */
      MAX_SCAN_NODES = 160 * 3,     /**< Maximum scan nodes. */
      DEFAULT_TIMEOUT_COUNT = 3, // Error count
    };

  private:
    int PackageSampleBytes; //Number of laser points contained in a package
    ChannelDevice *_comm = nullptr; //Communication object
    uint32_t trans_delay; //Time to transmit one byte via serial port
    int sample_rate; //Sampling rate

    gs_node_package package; //Package with signal quality
    uint8_t CheckSum; //Checksum
    uint8_t CheckSumCal;
    bool CheckSumResult;

    uint8_t *globalRecvBuffer = nullptr;

    double k0[LIDAR_MAXCOUNT];
    double k1[LIDAR_MAXCOUNT];
    double b0[LIDAR_MAXCOUNT];
    double b1[LIDAR_MAXCOUNT];
    double bias[LIDAR_MAXCOUNT];
    int m_models[LIDAR_MAXCOUNT] = {0};
    int model = YDLIDAR_GS2; //Radar model
    uint8_t moduleNum = 0; // Module number
    uint8_t moduleCount = 1; // Current module count
    int nodeCount = 0; // Current package point count
    uint64_t stamp = 0; // Timestamp
    std::list<gs_module_nodes> datas; // Module data
    double m_pitchAngle = Angle_PAngle;
    uint32_t lastStamp = 0; // Last time
  };

} // namespace ydlidar

#endif // GS2_YDLIDAR_DRIVER_H
