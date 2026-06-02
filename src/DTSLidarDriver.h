#ifndef DTSLIDARDRIVER_H
#define DTSLIDARDRIVER_H
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

#define SDK_DTS_POINT_COUNT 1
#define SDK_CMD_HEADFLAG 0xA5 // Protocol header identifier 1
#define SDK_CMD_STARTSCAN 0x01 // Enable distance measurement
#define SDK_CMD_STOPSCAN 0x02 // Stop distance measurement
#define SDK_CMD_CALIBPARAM 0x06 // Get calibration parameters
#define SDK_DTS_DEVNUM 0x03 // Device number
#define SDK_DTS_DEVTYPE 0x20 // Device type
#define SDK_DTS_RESERVED 0x00 // Reserved bits
#define SDK_DTS_BUFFLEN 100

// Set 1-byte alignment
#pragma pack(1)

// DTS Radar Protocol Header
struct SdkDTSHead
{
    uint8_t head = 0; // Package header
    uint8_t devNum = 0; // Device number
    uint8_t devType = 0; // Device type
    uint8_t cmd = 0; // Command function code
    uint8_t reserved = 0; // Reserved bits
    uint16_t size = 0; // Data size
};
#define SDKDTSHEADSIZE sizeof(SdkDTSHead)
// DTS Radar Single Point Data
struct SdkDTSPc
{

    uint16_t subPeakQuality = 0; //Sub-peak quality
    uint16_t tempCode = 0; //Temperature code
    uint16_t subPeakIntensity = 0; //Sub-peak intensity
    uint16_t mainPeakQuality = 0; //Main peak quality
    uint16_t mainPeakCalib = 0; //Main peak calibration
    uint16_t mainPeakIntensity = 0; //Main peak intensity
    uint16_t sunlitBase = 0; //Sunlit base
};
// DTS Radar One Package Point Cloud Data
struct SdkDTSPcs
{
    SdkDTSHead head;
    SdkDTSPc point; // One package data only has a single point
    uint16_t cs = 0;// Checksum
};

#define SDKDTSPCSSIZE sizeof(SdkDTSPcs)
// Cancel 1-byte alignment
#pragma pack()

// Calibration parameter structure
struct CalibParamInfo
{
    float k = 0.0;
    float b = 0.0;
    char SN[9] = {0};
};

using namespace std;

namespace ydlidar
{
using namespace core;
using namespace core::serial;
using namespace core::base;

/*********************DTS radar ****************/
class DTSLidarDriver : public DriverInterface
{
public:
    DTSLidarDriver();
    virtual ~DTSLidarDriver();

    result_t connect(const char *port, uint32_t baudrate);
    void disconnect();
    result_t stopScan(uint32_t timeout = DEFAULT_TIMEOUT / 2);
    result_t stop();
    /*
     * @brief Acquiring laser data
     * @param nodebuffer out: Laser dot information
     * @param count      in: Number of laser dots in a circle
     * @param timeout    in: Timeout
     * @return
     */
    result_t grabScanData(node_info *nodebuffer, size_t &count,
                          uint32_t timeout = DEFAULT_TIMEOUT);

    /*
     * @brief Waiting for scan data
     * @param nodes   out: Array to store node information
     * @param count   out: Size of the node information array, input represents the expected number of nodes to receive, output represents the actual number of nodes received
     * @param timeout in: Timeout (milliseconds)
     * @return result_t Operation result, success returns RESULT_OK, failure returns RESULT_FAIL
     */
    result_t waitScanData(node_info *nodes,
                          size_t &count,
                          uint32_t timeout = DEFAULT_TIMEOUT);

    // Laser data parsing thread
    int cacheScanData();

    result_t createThread();

    result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT);

    virtual std::string getSDKVersion();

    virtual const char *DescribeError(bool isTCP = false);
    // Laser data parsing thread
    bool isscanning() const;
    // Laser data parsing thread
    bool isconnected() const;

    /*
     * @brief Should I configure automatic reconnection in case of radar malfunction?
     * @param enable in:whether to enable automatic reconnection
     */
    void setAutoReconnect(const bool &enable);

    /*
     * @brief Acquiring laser data \n
     * @param[in] node Acquired laser point information
     * @param[in] timeout     Timeout
     */
    result_t waitPackage(node_info *node, uint32_t timeout = DEFAULT_TIMEOUT);

    result_t sendCmd(uint8_t cmd,
                     const uint8_t *data = NULL,
                     size_t size = 0);
    // Serial port sending data
    result_t sendData(const uint8_t *data, size_t size);

    // Waiting for a response
    result_t waitResp(uint8_t cmd,
                     uint32_t timeout = DEFAULT_TIMEOUT);

    /*
     * @brief Waiting for data(Retrieve only the data from the response data area)
     * @param cmd      in: command word
     * @param data     out: Received data
     * @param timeout  in：Timeout
     * @return
     */
    result_t waitResp(uint8_t cmd,
                     std::vector<uint8_t> &data,
                     uint32_t timeout = DEFAULT_TIMEOUT);
    /*
     * Calculate the size of the received data
     * @param srcSize  in:Expected size of the data to receive
     * @param timeout  in:Timeout
     * @param dstSize  out:Actual size of the received data
     * @return Returns the result, indicating the status of waiting for data
     */
    result_t waitForData(size_t srcSize, uint32_t timeout = DEFAULT_TIMEOUT,
                         size_t *dstSize = NULL);
    /*
     * @brief Reading data from the serial port
     * @param data   out: Data read from the serial port
     * @param size   in: Specified size of data to read
     * @return
     */
    result_t getData(uint8_t *data, size_t size);

    // Received data(none)
    result_t setScanFreq(float sf, uint32_t timeout);
    // Obtain calibration parameters
    result_t getCalibParam(uint32_t timeout);
    // Automatic connection
    result_t checkAutoConnecting();
    // Reconnect and start scanning
    result_t startAutoScan(bool force = false,
                           uint32_t timeout = DEFAULT_TIMEOUT);
    // Obtain device information
    virtual result_t getDeviceInfo(device_info &info,
                                   uint32_t timeout = DEFAULT_TIMEOUT);
    // Get health status
    virtual result_t getHealth(device_health &health,
                               uint32_t timeout = DEFAULT_TIMEOUT);
    // error message
    virtual const char *getErrorDesc(bool isTCP = false);
    // Close data acquisition channel
    void disableDataGrabbing();
    void flushSerial();
    // CRC checksum (CRC-16/MODBUS)
    uint16_t calculateCrc(const vector<uint8_t>& data);

private:
    serial::Serial *_serial = nullptr; // serial port
    std::vector<uint8_t> recvBuff; // A data cache
    float k = 0; // Calibration parameter k
    float b = 0; // Calibration parameter b
};
}
#endif // DTSLIDARDRIVER_H
