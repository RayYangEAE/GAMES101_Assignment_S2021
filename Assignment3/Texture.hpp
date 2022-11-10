//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        u = u + 1 - int(u + 1);
        v = v + 1 - int(v + 1);
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        u = u + 1 - int(u + 1);
        v = v + 1 - int(v + 1);
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        int u_img0 = u_img;
        int v_img0 = v_img;
        int u_img1 = MIN(u_img0 + 1, width - 1);
        int v_img1 = MIN(v_img0 + 1, height - 1);

        auto c00= image_data.at<cv::Vec3b>(v_img0, u_img0);
        auto c01 = image_data.at<cv::Vec3b>(v_img0, u_img1);
        auto c10 = image_data.at<cv::Vec3b>(v_img1, u_img0);
        auto c11 = image_data.at<cv::Vec3b>(v_img1, u_img1);

        float tu = u_img - u_img0;
        float tv = v_img - v_img0;
        auto c0 = c00 * (1 - tu) + c01 * tu;
        auto c1 = c10 * (1 - tu) + c11 * tu;
        auto color = c0 * (1 - tv) + c1 * tv;
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H
