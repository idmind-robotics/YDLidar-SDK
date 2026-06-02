#ifndef NOISEFILTER_H
#define NOISEFILTER_H
#include "FilterInterface.h"

#define MAX_INCLUDE_ANGLE 12.0f // Maximum angle
#define MAX_INCLINE_ANGLE 7.0f // Maximum tilt angle
#define MIN_NOISEPOINT_COUNT 2 //Minimum number of noise points


struct FilterBlock
{
    int start_index;
    int end_index;
};

class YDLIDAR_API NoiseFilter : public FilterInterface
{
public:
    enum FilterStrategy
    {
        FS_Normal, // Noise filtering
        FS_Tail, // Old trailing filter
        FS_TailStrong, // Trailing filter
        FS_TailWeek, // Trailing filter
        FS_TailStrong2, //Trailing filter
    };
public:
    NoiseFilter();
    ~NoiseFilter() override;
    void filter(const LaserScan &in,
                 int lidarType,
                 int version,
                 LaserScan &out) override;

    std::string version() const;
    void setStrategy(int value) override;

protected:
    void filter_noise(const LaserScan &in,
                      int lidarType,
                      int version,
                      LaserScan &out);
    // Filter trailing method 1
    void filter_tail(const LaserScan &in,
                     int lidarType,
                     int version,
                     LaserScan &out);
    // Filter trailing method 2
    void filter_tail2(const LaserScan &in,
                      int lidarType,
                      int version,
                      LaserScan &out);

    double calcInclineAngle(double reading1, double reading2,
                            double angleBetweenReadings) const;
    /**
   * @brief getTargtAngle
   * @param reading1
   * @param angle1
   * @param reading2
   * @param angle2
   * @return
   */
    double calcTargetAngle(double reading1, double angle1,
                           double reading2, double angle2);

    /**
   * @brief calculateTargetOffset
   * @param reading1
   * @param angle1
   * @param reading2
   * @param angle2
   * @return
   */
    double calcTargetOffset(double reading1, double angle1,
                            double reading2, double angle2);

    /**
   * @brief isRangeValid
   * @param reading
   * @return
   */
    bool isRangeValid(const LaserConfig &config, double reading) const;

    /**
   * @brief isIncreasing
   * @param value
   * @return
   */
    bool isIncreasing(double value) const;

    /**
   * Defines how many readings next to an invalid reading get marked as invalid
   * */

protected:
    double minIncline, maxIncline;
    int nonMaskedNeighbours;
    int maskedNeighbours;
    bool m_Monotonous;
    bool maskedFilter;

    float maxIncludeAngle = MAX_INCLUDE_ANGLE;
    float maxInclineAngle = MAX_INCLINE_ANGLE;
};

#endif // NOISEFILTER_H
