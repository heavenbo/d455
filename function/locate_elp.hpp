#include <opencv2/opencv.hpp>
#include <math.h>
namespace locate
{
    const double depth_scale = 0.001;
    const double cx = 320.207;
    const double cy = 236.529;
    const double fx = 388.677;
    const double fy = 388.677;
    typedef struct
    {
        double real_x;
        double real_y;
        double real_z;
    } axis;
    axis Cal_axis(cv::Mat &img, cv::Point pixel)
    //计算深度 pixel.y为row,pixel.x为col
    {
        axis A;
        uint16_t depth_value = img.at<uint16_t>(pixel.y, pixel.x);
        A.real_z = depth_value * depth_scale;
        A.real_y = A.real_z / fy * (pixel.y - cy);
        A.real_x = A.real_z / fx * (pixel.x - cx);
        return A;
    }

    double filter_axis(uint16_t b[], int n, double error)
    //n为数组长度，error为容许的误差
    {
        double sum = 0;
        for (int i = 0; i < n; i++)
        {
            sum = sum + b[i];
        }
        int n_real=n;
        double sum_real=sum;
        for (int i = 0; i < n; i++)
        {
            if(abs(b[i]-sum/n)>error)
            {
                n_real--;
                b[i]=0;
                sum_real=sum_real-sum;
            }
        }
        return(sum_real/n_real);
    }
}