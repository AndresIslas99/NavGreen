#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

// Depth normalization: 0.3-10m range → 0-255
TEST(ImagePipeline, DepthNormalization) {
  cv::Mat depth(4, 4, CV_32FC1);
  depth.setTo(5.0f);  // 5m uniform
  // Expected: (5.0 - 0.3) / 9.7 * 255 ≈ 123.5
  cv::Mat norm;
  depth.copyTo(norm);
  norm = (norm - 0.3) / 9.7 * 255.0;
  norm.convertTo(norm, CV_8U);
  EXPECT_NEAR(norm.at<uint8_t>(0, 0), 123, 2);
}

// NaN depth → mapped to max (10m)
TEST(ImagePipeline, DepthNanHandling) {
  cv::Mat depth(4, 4, CV_32FC1, cv::Scalar(std::nanf("")));
  depth.setTo(10.0f, ~(depth == depth));  // Replace NaN with 10.0
  EXPECT_FLOAT_EQ(depth.at<float>(0, 0), 10.0f);
}

// JPEG encoding produces non-empty output
TEST(ImagePipeline, JpegEncoding) {
  cv::Mat img(64, 64, CV_8UC3, cv::Scalar(128, 64, 32));
  std::vector<uchar> buf;
  cv::imencode(".jpg", img, buf, {cv::IMWRITE_JPEG_QUALITY, 70});
  EXPECT_GT(buf.size(), 100u);
}

// Depth colormap inversion (JET: red=near, blue=far)
TEST(ImagePipeline, DepthColormapInversion) {
  cv::Mat gray(1, 2, CV_8UC1);
  gray.at<uint8_t>(0, 0) = 0;    // near → should be red after inversion
  gray.at<uint8_t>(0, 1) = 255;  // far → should be blue after inversion
  cv::Mat inverted = 255 - gray;
  cv::Mat color;
  cv::applyColorMap(inverted, color, cv::COLORMAP_JET);
  // Near pixel (inverted=255) in JET → red-ish (high R, low B)
  EXPECT_GT(color.at<cv::Vec3b>(0, 0)[2], 100);  // R channel
  // Far pixel (inverted=0) in JET → blue-ish (high B, low R)
  EXPECT_GT(color.at<cv::Vec3b>(0, 1)[0], 100);  // B channel
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
