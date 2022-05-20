#include <emc/io.h>

#include "opencv2/imgproc.hpp"
#include <opencv2/highgui.hpp>
#include <ros/rate.h>

#include <string>

double resolution = 0.01;
cv::Point2d canvas_center;

const bool draw_robot = false;
double robotRadius = 0.21; // HERO radius [m]

// ----------------------------------------------------------------------------------------------------

cv::Point2d worldToCanvas(double x, double y)
{
    return cv::Point2d(-y / resolution, -x / resolution) + canvas_center;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    std::string robot_name = "hero";
    if (argc > 1)
        robot_name=argv[1];

    emc::IO io(robot_name);

    ros::Rate r(30);
    while(io.ok())
    {
        cv::Mat canvas(500, 500, CV_8UC3, cv::Scalar(50, 50, 50));
        canvas_center = cv::Point2d(canvas.rows / 2, canvas.cols / 2);

        cv::Scalar robot_color(0, 0, 255);

        // scaling of the head and LRF (which should be drawn inside the circumference of the robot)
        const double scaling = 1.0;

        if (draw_robot) {
            std::vector<std::pair<double, double> > robot_points;
            std::vector<std::pair<double, double> > robot_head_points;
            std::vector<std::pair<double, double> > eye_points;
            std::vector<std::pair<double, double> > eye_pupil_points;

            robot_points.push_back(std::pair<double, double>(0.0,0.0));
            robot_head_points.push_back(std::pair<double, double>( 0.08*scaling, -0.16*scaling));
            robot_head_points.push_back(std::pair<double, double>( 0.08*scaling,  0.16*scaling));
            robot_head_points.push_back(std::pair<double, double>(-0.08*scaling,  0.16*scaling));
            robot_head_points.push_back(std::pair<double, double>(-0.08*scaling, -0.16*scaling));
            eye_points.push_back(std::pair<double, double>( 0.0*scaling,  0.07*scaling));
            eye_points.push_back(std::pair<double, double>( 0.0*scaling, -0.07*scaling));
            eye_pupil_points.push_back(std::pair<double, double>( 0.025*scaling,  0.07*scaling));
            eye_pupil_points.push_back(std::pair<double, double>( 0.025*scaling, -0.07*scaling));

            cv::Point2d pRobot = worldToCanvas(robot_points[0].first, robot_points[0].second);
            cv::circle(canvas, pRobot, robotRadius/resolution, robot_color, 2);

            for(unsigned int i = 0; i < robot_head_points.size(); ++i)
            {
                unsigned int j = (i + 1) % robot_head_points.size();
                cv::Point2d p1 = worldToCanvas(robot_head_points[i].first, robot_head_points[i].second);
                cv::Point2d p2 = worldToCanvas(robot_head_points[j].first, robot_head_points[j].second);
                cv::line(canvas, p1, p2, robot_color, 2);
            }
            for(unsigned int i = 0; i < eye_points.size(); ++i)
            {
                cv::Point2d pEye = worldToCanvas(eye_points[i].first, eye_points[i].second);
                cv::circle(canvas, pEye, scaling*4, robot_color, 2);
            }
            for(unsigned int i = 0; i < eye_pupil_points.size(); ++i)
            {
                cv::Point2d pEyePupil = worldToCanvas(eye_pupil_points[i].first, eye_pupil_points[i].second);
                cv::circle(canvas, pEyePupil, scaling*1.5, robot_color, 2);
            }
        }

        // Dot to indicate LRF position
        cv::Point2d pLRF = worldToCanvas(0.15*scaling, 0.0*scaling);
        if (!draw_robot) pLRF = worldToCanvas(0.0*scaling, 0.0*scaling);
        cv::circle(canvas, pLRF, scaling*2, robot_color, 2);

        emc::LaserData scan;
        if (!io.readLaserData(scan))
            continue;

        double a = scan.angle_min;
        for(unsigned int i = 0; i < scan.ranges.size(); ++i)
        {
            double x = cos(a) * scan.ranges[i];
            double y = sin(a) * scan.ranges[i];

            cv::Point2d p = worldToCanvas(x, y);
            if (p.x >= 0 && p.y >= 0 && p.x < canvas.cols && p.y < canvas.rows)
                canvas.at<cv::Vec3b>(p) = cv::Vec3b(0, 255, 0);

            a += scan.angle_increment;
        }

        cv::imshow("Laser Vizualization", canvas);
        cv::waitKey(3);

        r.sleep();
    }

    return 0;
}
