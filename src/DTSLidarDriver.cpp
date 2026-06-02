#include <math.h>
#include "DTSLidarDriver.h"
#include "core/serial/common.h"
#include "ydlidar_config.h"


using namespace impl;
namespace ydlidar
{

DTSLidarDriver::DTSLidarDriver()
    : _serial(NULL)
{
    // Serial port configuration parameters
    isAutoReconnect = true;
    isAutoconnting = false;
    m_baudrate = 921600;
    m_PointTime = 1e9 / 5000;
    retryCount = 0;
    m_SingleChannel = false;
    m_LidarType = TYPE_SDM18;

    nodeIndex = 0;
    recvBuff = std::vector<uint8_t>(SDKDTSPCSSIZE, 0);
    scan_node_count = 0;
    scan_node_buf = new node_info[SDK_DTS_POINT_COUNT * 5];
}

DTSLidarDriver::~DTSLidarDriver()
{
    m_isScanning = false;
    isAutoReconnect = false;
    _thread.join();

    {
        ScopedLocker l(_cmd_lock);
        if (_serial)
        {
            if (_serial->isOpen())
            {
                _serial->flush();
                _serial->closePort();
            }
            delete _serial;
            _serial = NULL;
        }
    }

    {
        ScopedLocker l(_lock);
        if (scan_node_buf)
        {
            delete[] scan_node_buf;
            scan_node_buf = NULL;
        }
        scan_node_count = 0;
    }
}

result_t DTSLidarDriver::connect(const char *port, uint32_t baudrate)
{
    m_baudrate = baudrate;
    m_port = string(port);
    {
        ScopedLocker l(_cmd_lock);
        if (!_serial)
        {
            _serial = new serial::Serial(
                m_port,
                m_baudrate,
                serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT));
        }
        if (!_serial->open())
        {
            return RESULT_FAIL;
        }

        m_isConnected = true;
    }

    stopScan();
    // clearDTR();

    return RESULT_OK;
}

void DTSLidarDriver::disconnect()
{
    isAutoReconnect = false;

    if (!m_isConnected)
        return;

    stop();
    delay(10);

    ScopedLocker l(_cmd_lock);
    if (_serial)
    {
        if (_serial->isOpen())
        {
            _serial->closePort();
        }
    }

    m_isConnected = false;
}

result_t DTSLidarDriver::stopScan(uint32_t timeout)
{
    UNUSED(timeout);
    result_t ans;

    if (!m_isConnected)
        return RESULT_FAIL;

    ScopedLocker l(_cmd_lock);
    if ((ans = sendCmd(SDK_CMD_STOPSCAN)) != RESULT_OK)
    {
        return ans;
    }
    if ((ans = waitResp(SDK_CMD_STOPSCAN, timeout)) != RESULT_OK)
    {
        return ans;
    }

    delay(10);

    return RESULT_OK;
}

result_t DTSLidarDriver::stop()
{
    if (isAutoconnting)
        isAutoReconnect = false;

    disableDataGrabbing();
    stopScan();
    flushSerial();

    return RESULT_OK;
}

/*
 * @brief Acquiring laser data
 * @param nodebuffer out: Laser dot information
 * @param count      in: Number of laser dots in a circle
 * @param timeout    in: Timeout
 * @return
 */
result_t DTSLidarDriver::grabScanData(
    node_info *nodebuffer, 
    size_t &count, 
    uint32_t timeout)
{
    result_t ret = RESULT_FAIL;
    switch (_dataEvent.wait(timeout))
    {
    case Event::EVENT_TIMEOUT:
        count = 0;
        ret = RESULT_TIMEOUT;
        break;
    case Event::EVENT_OK:
    {
        ScopedLocker l(_lock);
        count = min(count, scan_node_count);
        memcpy(nodebuffer, scan_node_buf, count * SDKNODESIZE);
        scan_node_count = 0;
        ret = RESULT_OK;
        _dataEvent.set(false); // Reset status
        break;
    }
    default:
        count = 0;
        ret = RESULT_FAIL;
        break;
    }

    return ret;
}

/*
 * @brief Waiting for scan data
 * @param nodes   out: Array to store node information
 * @param count   out: Size of the node information array, input indicates the expected number of nodes to receive, output indicates the actual number of nodes received
 * @param timeout in: Timeout duration (milliseconds)
 * @return result_t Operation result, success returns RESULT_OK, failure returns RESULT_FAIL
 */
result_t DTSLidarDriver::waitScanData(
    node_info *nodes,
    size_t &count, 
    uint32_t timeout)
{
    result_t ret = RESULT_FAIL;
    // Check if connected
    if (!m_isConnected)
    {
        count = 0;
        return RESULT_FAIL;
    }

    size_t recvCount = 0;
    uint32_t st = getms();
    uint32_t wt = 0;

    // Looping and waiting to receive data from the node
    while ((wt = getms() - st) < timeout && 
        recvCount < count)
    {
        node_info node;
        memset(&node, 0, SDKNODESIZE);
        // Unpacking laser data
        ret = waitPackage(&node, timeout - wt);
        if (!IS_OK(ret))
        {
            count = recvCount;
            return ret;
        }
        // single point
        nodes[recvCount++] = node;
        if (recvCount == count)
            return RESULT_OK;
    }

    count = recvCount;
    return RESULT_FAIL;
}

// Laser data parsing thread
int DTSLidarDriver::cacheScanData()
{
    node_info local_buf[SDK_DTS_POINT_COUNT];
    size_t count = SDK_DTS_POINT_COUNT;
    result_t ret = RESULT_FAIL;
    int timeout_count = 0;
    retryCount = 0;
    m_isScanning = true;
    while (m_isScanning)
    {
        count = SDK_DTS_POINT_COUNT;
        ret = waitScanData(local_buf, count);
        //If point cloud parsing fails
        if (!IS_OK(ret))
        {
            if (timeout_count > DEFAULT_TIMEOUT_COUNT)
            {
                if (!isAutoReconnect)
                {
                    fprintf(stderr, "[YDLIDAR] Exit scanning thread!\n");
                    fflush(stderr);
                    m_isScanning = false;
                    return RESULT_FAIL;
                }
                else
                {
                    ret = checkAutoConnecting();
                    if (IS_OK(ret))
                    {
                        timeout_count = 0;
                    }
                    else
                    {
                        m_isScanning = false;
                        return RESULT_FAIL;
                    }
                }
            }
            else
            {
                timeout_count ++;
                fprintf(stderr, "[YDLIDAR] Timeout count %d\n", timeout_count);
                fflush(stderr);
            }
        }
        else
        {
            timeout_count = 0;
            retryCount = 0;

            ScopedLocker l(_lock);
            memcpy(scan_node_buf, local_buf, sizeof(node_info) * count);
            scan_node_count = count;
            _dataEvent.set();
        }
    }
    m_isScanning = false;
    return RESULT_OK;
}

/*
 *@brief Create a thread to parse radar data
 *@note Before creating the radar data parsing thread, the startScan function must be used to successfully start the scan.
 */
result_t DTSLidarDriver::createThread()
{
    _thread = CLASS_THREAD(DTSLidarDriver, cacheScanData);
    if (!_thread.getHandle())
    {
        m_isScanning = false;
        printf("[YDLIDAR] Fail to create DTS thread\n");
        return RESULT_FAIL;
    }
    printf("[YDLIDAR] Create DTS thread [0x%X]\n", _thread.getHandle());
    fflush(stdout);
    m_isScanning = true;

    return RESULT_OK;
}

result_t DTSLidarDriver::startScan(bool force, uint32_t timeout)
{
    result_t ret = RESULT_FAIL;
    if (!m_isConnected)
        return RESULT_FAIL;
    if (m_isScanning)
        return RESULT_OK;
    // Stop before starting
    stopScan();
    // Set the default scan frequency(none)
    ret = setScanFreq(10.0, timeout);
    if (!IS_OK(ret))
    {
        printf("[YDLIDAR] Fail to setting scan frequency\n");
        return ret;
    }
    // Obtain calibration parameters
//    ret = getCalibParam(timeout);
//    if (!IS_OK(ret))
//    {
//        return ret;
//    }

    // Send radar start command
    ScopedLocker l(_cmd_lock);
    if ((ret = sendCmd(SDK_CMD_STARTSCAN)) != RESULT_OK)
        return ret;
    // The dual-channel radar is waiting for the start-up response command.
    if (!m_SingleChannel)
    {
        ret = waitResp(SDK_CMD_STARTSCAN, timeout);
        if (ret != RESULT_OK)
        {
            printf("[YDLIDAR] Response to start scan error!\n");
            return ret;
        }
    }

    ret = createThread(); // Create thread
    return ret;
}

std::string DTSLidarDriver::getSDKVersion()
{
    return YDLIDAR_SDK_VERSION_STR;
}

const char *DTSLidarDriver::DescribeError(bool isTCP)
{
    if (_serial)
    {
        return _serial->DescribeError();
    }
    return nullptr;
}

bool DTSLidarDriver::isscanning() const
{
    return m_isScanning;
}

bool DTSLidarDriver::isconnected() const
{
    return m_isConnected;
}

void DTSLidarDriver::setAutoReconnect(const bool &enable)
{
    isAutoReconnect = enable;
}

result_t DTSLidarDriver::waitPackage(node_info *node, uint32_t timeout)
{
    int pos = 0;
    uint32_t st = getms();
    uint32_t wt = 0;
    uint16_t dataSize = 0; // The length of the data in the data area
    uint16_t cs = 0; // CRC
    result_t ret = RESULT_FAIL;
    vector<uint8_t> crCdata;
    memset(node, 0, SDKNODESIZE);
    while ((wt = getms() - st) < timeout)
    {
        size_t srcSize = SDKDTSHEADSIZE - pos;
        size_t dstSize = 0;

        // dstSize is the actual size of the received data.
        result_t ans = waitForData(srcSize, timeout - wt, &dstSize);
        if (!IS_OK(ans))
            return ans;

        // Read a specified amount of data from the serial port.
        getData(recvBuff.data(), dstSize);

        for (size_t i = 0; i < dstSize; ++i)
        {
            uint8_t c = recvBuff[i];
            switch (pos)
            {
            case 0:
                if (c != SDK_CMD_HEADFLAG)
                {
                    pos = 0;
                    continue;
                }
                crCdata.clear();
                break;
            case 1:
                if (c != SDK_DTS_DEVNUM)
                {
                    pos = 0;
                    continue;
                }
                break;
            case 2:
                if (c != SDK_DTS_DEVTYPE)
                {
                    pos = 0;
                    continue;
                }
                break;
            case 3:
                if (c != SDK_CMD_STARTSCAN) // Determine whether the parsed command word matches the specified command word.
                {
                    pos = 0;
                    continue;
                }
                break;
            case 4:
                if (c != SDK_DTS_RESERVED) // Determine whether the parsed command word matches the specified command word.
                {
                    pos = 0;
                    continue;
                }
                break;
            case 5:
                dataSize = uint16_t(c) << 8; // Length of data retrieved from the data area
                break;
            case 6:
                dataSize += uint16_t(c);
                break;
            default:
                break;
            }
            pos ++;
            crCdata.push_back(c);
        }

        // If the protocol header is found
        if (pos == SDKDTSHEADSIZE)
        {
            pos = 0;
            // Retrieve the remaining data and calculate the checksum.(Data length in the data area + CRC)
            size_t srcSize = dataSize + 2;
            size_t dstSize = 0;
            // dstSize is the actual size of the received data.
            ret = waitForData(srcSize, timeout - wt, &dstSize);
            if (!IS_OK(ret))
                return ret;
            // Read a specified amount of data from the serial port.
            getData(recvBuff.data(), dstSize);
            // Data in the storage area(Used to calculate CRC)
            for (size_t i = 0; i < dataSize; ++i)
            {
                crCdata.push_back(recvBuff[i]);
            }
            // Calculate the check code
            cs = calculateCrc(crCdata);
            // CRC returned by serial port
            uint16_t csRaw = (recvBuff[dataSize] << 8) | recvBuff[dataSize+1];
            if (cs != csRaw)
            {
                printf("[YDLIDAR] CRC error calc[0x%04X] != src[0x%04X]\n",
                       cs, csRaw);
                fflush(stdout);
                return RESULT_FAIL;
            }
            break;
        }
    }

    if (IS_OK(ret))
    {
        (*node).sync = Node_Sync;
        (*node).stamp = getTime();
        (*node).index = 0;
        (*node).scanFreq = uint8_t(0);
        (*node).qual = 0;

        // Extract the centroid of the main peak
        uint16_t mainPeakQuality = (recvBuff[7] << 8) | recvBuff[6];
        uint16_t qual = (recvBuff[11] << 8) | recvBuff[10];
        (*node).qual = qual;
        // Convert the main peak centroid data to decimal and then subtract... b value, then divide by k value
        (*node).dist = mainPeakQuality;
        // Is there only one point like this?
        (*node).angle = 0;
        //(*node).qual = 0;
        (*node).is = 0;
        return RESULT_OK;
    }
    else
    {
        return RESULT_FAIL;
    }
}

result_t DTSLidarDriver::sendCmd(uint8_t cmd, const uint8_t *data, size_t dataSize)
{
    if (!m_isConnected)
        return RESULT_FAIL;

    size_t size = SDKDTSHEADSIZE + dataSize;
    vector<uint8_t> buff(size + 2, 0);

    SdkDTSHead head;
    head.head = SDK_CMD_HEADFLAG;
    head.devNum = SDK_DTS_DEVNUM;
    head.devType = SDK_DTS_DEVTYPE;
    head.cmd = cmd;
    head.reserved  = SDK_DTS_RESERVED;
    head.size = uint16_t(dataSize);
    memcpy(&buff[0], &head, SDKDTSHEADSIZE);

    if (data && dataSize)
        memcpy(&buff[SDKDTSHEADSIZE], data, dataSize);

    // CRC check
    uint16_t cs = calculateCrc(buff);
//    buff[0] = static_cast<uint8_t>(cs >> 8);  // Assign the high byte to the element at index 0.
//    buff[1] = static_cast<uint8_t>(cs);       // Assign the low byte to the element at index 1.
    buff[size] = static_cast<uint8_t>(cs >> 8);  //Assign the high byte to the element at index 0;
    buff[size+1] = static_cast<uint8_t>(cs);// Assign the low byte to the element at index 1.
    return sendData(buff.data(), buff.size());
}

result_t DTSLidarDriver::sendData(const uint8_t *data, size_t size)
{
    if (!_serial || !_serial->isOpen())
        return RESULT_FAIL;

    if (!data || !size)
        return RESULT_FAIL;
    size_t r = 0;
    while (size)
    {
        r = _serial->writeData(data, size);
        if (!r)
            return RESULT_FAIL;

        if (m_Debug)
        {
            printf("send: ");
            printHex(data, r);
        }

        size -= r;
        data += r;
    }

    return RESULT_OK;
}

result_t DTSLidarDriver::waitResp(
        uint8_t cmd, uint32_t timeout)
{
    std::vector<uint8_t> data;
    return waitResp(cmd, data, timeout);
}

/*
 * @brief DTSLidarDriver::waitResp
 * @param cmd      in: command word
 * @param data     out: Received data
 * @param timeout  in：Timeout
 * @return
 */
result_t DTSLidarDriver::waitResp(
        uint8_t cmd,
        std::vector<uint8_t> &data,
        uint32_t timeout)
{
    int pos = 0;
    uint32_t st = getms();
    uint32_t wt = 0;
    vector<uint8_t> recvBuff(SDK_DTS_BUFFLEN, 0);
    uint16_t cs = 0;
    uint8_t dataSize = 0;
    vector<uint8_t> crCdata;

    while ((wt = getms() - st) < timeout)
    {
        size_t srcSize = SDKDTSHEADSIZE - pos;
        size_t dstSize = 0;
        // dstSize is the actual size of the received data.
        result_t ans = waitForData(srcSize, timeout - wt, &dstSize);
        if (!IS_OK(ans))
            return ans;
        // Read a specified amount of data from the serial port.
        getData(recvBuff.data(), dstSize);

        for (size_t i = 0; i < dstSize; ++i)
        {
            uint8_t c = recvBuff[i];
            switch (pos)
            {
            case 0:
                if (c != SDK_CMD_HEADFLAG)
                {
                    pos = 0;
                    continue;
                }
                crCdata.clear();
                break;
            case 1:
                if (c != SDK_DTS_DEVNUM)
                {
                    pos = 0;
                    continue;
                }
                break;
            case 2:
                if (c != SDK_DTS_DEVTYPE)
                {
                    pos = 0;
                    continue;
                }
                break;
            case 3:
                if (c != cmd) // Determine whether the parsed command word matches the specified command word.
                {
                    pos = 0;
                    continue;
                }
                break;
            case 4:
                if (c != SDK_DTS_RESERVED) // Determine whether the parsed command word matches the specified command word.
                {
                    pos = 0;
                    continue;
                }
                break;
            case 5:
                dataSize = uint16_t(c) << 8; // Length of data retrieved from the data area
                break;
            case 6:
                dataSize += uint16_t(c);
                break;
            default:
                break;
            }

            pos ++;
            crCdata.push_back(c);
        }

        // If the protocol header is found
        if (pos == SDKDTSHEADSIZE)
        {
            pos = 0;
            // Retrieve the remaining data and calculate the checksum.(Data length in the data area + CRC)
            size_t srcSize = dataSize + 2;
            size_t dstSize = 0;
            // dstSize is the actual size of the received data.
            result_t ans = waitForData(srcSize, timeout - wt, &dstSize);
            if (!IS_OK(ans))
                return ans;
            // Read a specified amount of data from the serial port.
            getData(recvBuff.data(), dstSize);

            for (size_t i = 0; i < dataSize; ++i)
            {
                crCdata.push_back(recvBuff[i]);
                data.push_back(recvBuff[i]); // Data stored in output parameter
            }
            cs = calculateCrc(crCdata);
            // CRC returned by serial port
            uint16_t csRaw = (recvBuff[dataSize] << 8) | recvBuff[dataSize+1];
            // Check if CRC is consistent
            if (cs != csRaw)
            {
                printf("[YDLIDAR] CRC error calc[0x%04X] != src[0x%04X]\n",
                    cs, csRaw);
                return RESULT_FAIL;
            }
            return RESULT_OK;
        }
    }
    return RESULT_FAIL;
}

/*
 * Waiting for data function
 * @param srcSize  in: Expected data size
 * @param timeout  in: Timeout
 * @param dstSize  out:Actual received data size
 * @return The returned result indicates the status of waiting for data.
 */
result_t DTSLidarDriver::waitForData(size_t srcSize, uint32_t timeout, size_t *dstSize)
{
    // Used to store the actual size of the received data.
    size_t size = 0;

    // If dstSize is a null pointer, then it will point to the size variable.
    if (!dstSize)
        dstSize = &size;

    // Waiting for data
    result_t ret = _serial->waitfordata(srcSize, timeout, dstSize);

    // If the actual received data size is larger than the expected size, it will be truncated to the expected size.
    if (IS_OK(ret))
    {
        if (*dstSize > srcSize)
            *dstSize = srcSize;
    }
    return ret;
}

/*
 * @brief Read a specified amount of data from the serial port.
 * @param data   out: Store data read from the serial port
 * @param size   in: Specify the size of the data to be read
 * @return
 */
result_t DTSLidarDriver::getData(uint8_t *data, size_t size)
{
    // Check if the serial port is open.
    if (!_serial || !_serial->isOpen())
    {
        return RESULT_FAIL;
    }

    size_t r;

    // Read data in a loop until the specified size has been read.
    while (size)
    {
        // Reading data from the serial port
        r = _serial->readData(data, size);

        // If the read fails, return a failure result.
        if (!r)
        {
            return RESULT_FAIL;
        }

        //If debug mode is enabled, print the read data.
        if (m_Debug)
        {
            printf("recv: ");
            printHex(data, r);
        }

        // Update remaining data size and data pointers
        size -= r;
        data += r;
    }

    return RESULT_OK;
}

// Set scan frequency(none)
result_t DTSLidarDriver::setScanFreq(float sf, uint32_t timeout)
{
    m_ScanFreq = 0;
    return RESULT_OK;
}

result_t DTSLidarDriver::getCalibParam(uint32_t timeout)
{
    ScopedLocker l(_cmd_lock);
    if (sendCmd(SDK_CMD_CALIBPARAM) != RESULT_OK)
    {
        printf("[YDLIDAR] Fail to send CalibParam cmd\n");
        return RESULT_FAIL;
    }
    vector<uint8_t> data;
    if (waitResp(SDK_CMD_CALIBPARAM, data, timeout) != RESULT_OK)
    {
        printf("[YDLIDAR] Fail to get CalibParam\n");
        return RESULT_FAIL;
    }

    // Extract the values ​​of calibration parameters k and b.
    memcpy(&k, &data[7], sizeof(float));
    memcpy(&b, &data[11], sizeof(float));

    printf("[YDLIDAR] CalibParam k[%f] b[%f]\n", k, b);

//    CalibParamInfo calibParamInfo;
//    memcpy(&calibParamInfo, data.data(), sizeof(CalibParamInfo));
//    k = calibParamInfo.k;
//    b = calibParamInfo.b;
    return RESULT_OK;
}

result_t DTSLidarDriver::checkAutoConnecting()
{
    result_t ans = RESULT_FAIL;
    isAutoconnting = true;
    while (isAutoReconnect && isAutoconnting)
    {
        {
            ScopedLocker l(_cmd_lock);
            if (_serial)
            {
                if (_serial->isOpen() || m_isConnected)
                {
                    m_isConnected = false;
                    _serial->closePort();
                    delete _serial;
                    _serial = nullptr;
                }
            }
        }
        retryCount ++;
        if (retryCount > 10)
            retryCount = 10;

        int retryConnect = 0;
        while (isAutoReconnect &&
            connect(m_port.c_str(), m_baudrate) != RESULT_OK)
        {
            retryConnect ++;
            if (retryConnect > 5)
            {
                retryConnect = 5;
            }
            setDriverError(NotOpenError);

            delay(50 * retryConnect);
        }

        if (!isAutoReconnect)
        {
            m_isScanning = false;
            return RESULT_FAIL;
        }

        if (isconnected())
        {
            delay(10);
            {
                ans = startAutoScan();
                if (!IS_OK(ans))
                {
                    ans = startAutoScan();
                }
            }

            if (IS_OK(ans))
            {
                isAutoconnting = false;
                return ans;
            }
        }
    }
    return RESULT_FAIL;
}

result_t DTSLidarDriver::startAutoScan(bool force, uint32_t timeout)
{
    result_t ans;

    if (!m_isConnected)
        return RESULT_FAIL;

    flushSerial();
    delay(10);
    {
        ScopedLocker l(_cmd_lock);
        if (sendCmd(SDK_CMD_STARTSCAN) != RESULT_OK)
        {
            return RESULT_FAIL;
        }
        if (!m_SingleChannel)
        {
            if ( waitResp(SDK_CMD_STARTSCAN, timeout) != RESULT_OK)
            {
                return RESULT_FAIL;
            }
        }
    }
    return RESULT_OK;
}

result_t DTSLidarDriver::getDeviceInfo(device_info &info, uint32_t timeout)
{
    if (!m_isConnected)
        return RESULT_FAIL;

    ScopedLocker l(_cmd_lock);
    if (sendCmd(SDK_CMD_CALIBPARAM) != RESULT_OK)
    {
        printf("[YDLIDAR] Fail to send CalibParam cmd\n");
        return RESULT_FAIL;
    }
    vector<uint8_t> data;
    if (waitResp(SDK_CMD_CALIBPARAM, data, timeout) != RESULT_OK)
    {
        printf("[YDLIDAR] Fail to get CalibParam\n");
        return RESULT_FAIL;
    }
    // printHex(data.data(), data.size());

    // Extract the values ​​of calibration parameters k and b.
    // memcpy(&k, &data[0], sizeof(float));
    // memcpy(&b, &data[4], sizeof(float));

    // printf("[YDLIDAR] CalibParam k[%f] b[%f]\n", k, b);

    memset(&info, 0, DEVICEINFOSIZE);
    info.model = YDLIDAR_SDM18;

    return RESULT_OK;
}

result_t DTSLidarDriver::getHealth(device_health &health, uint32_t timeout)
{
    health.status = 0;
    health.error_code = 0;
    UNUSED(timeout);
    return RESULT_OK;
}

const char *DTSLidarDriver::getErrorDesc(bool isTCP)
{
    UNUSED(isTCP);
    if (_serial)
    {
        return _serial->DescribeError();
    }
    return nullptr;
}

void DTSLidarDriver::disableDataGrabbing()
{
    if (m_isScanning)
    {
        m_isScanning = false;
        _dataEvent.set();
    }
    _thread.join();
}

void DTSLidarDriver::flushSerial()
{
    if (!m_isConnected)
        return;

    ScopedLocker l(_cmd_lock);
    size_t len = _serial->available();
    if (len)
    {
        _serial->readSize(len);
    }

    delay(20);
}

uint16_t DTSLidarDriver::calculateCrc(const vector<uint8_t> &data)
{
    uint16_t crc = 0xFFFF;  // Initial value is 0xFFFF

    for (const auto& byte : data)
    {
        crc ^= byte;
        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

}

