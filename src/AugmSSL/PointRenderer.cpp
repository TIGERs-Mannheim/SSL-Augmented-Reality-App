#include "PointRenderer.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

namespace tigers {

PointRenderer::PointRenderer() {
	vSupportPoints.push_back(Point2f(1, 0));
	vSupportPoints.push_back(Point2f(-1, 0));
	vSupportPoints.push_back(Point2f(0, 1));
	vSupportPoints.push_back(Point2f(0, -1));
	vSupportPoints.push_back(Point2f(0.70710678118, 0.70710678118)); // sqrt(2)/2 -> length=1
}

PointRenderer::~PointRenderer() {
}

void PointRenderer::render(ShapeCollection* shapeCollection, cv::Mat& drawing,
		Transformer& transformer) {
	std::vector<Point2f> vCirclePosReal;
	std::vector<cv::Scalar> vColors;
	std::vector<float> vThickness;

	for (int i = 0; i < shapeCollection->points_size(); i++) {
		tigers::Point c = shapeCollection->points(i);

		Point2f circlePosField(c.vpoint().x(), c.vpoint().y());

		float radius = 25;

		for (int j = 0; j < 5; j++) {
			vCirclePosReal.push_back(
					(vSupportPoints.at(j) * radius) + circlePosField);
		}

		cv::Scalar color(c.color().b(), c.color().g(), c.color().r());
		vColors.push_back(color);
		vThickness.push_back(c.thickness());
	}
	if (vCirclePosReal.size() > 3) {
		std::vector<Point2f> vCirclePosImage;
		perspectiveTransform(vCirclePosReal, vCirclePosImage, transformer.H);

		for (int i = 0; i < vCirclePosImage.size(); i += 5) {
			std::vector<Point2f> vSP(vCirclePosImage.begin() + i,
					vCirclePosImage.begin() + i + 5);
			cv::RotatedRect box = cv::fitEllipse(vSP);
			int idx = floor(i / 5);
			ellipse(drawing, box, vColors.at(idx), -vThickness.at(idx));
		}
	}
}

} // namespace tigers

