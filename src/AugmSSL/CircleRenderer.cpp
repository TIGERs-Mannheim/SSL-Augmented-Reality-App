#include "CircleRenderer.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

namespace tigers {

CircleRenderer::CircleRenderer() {
	vSupportPoints.push_back(Point2f(1, 0));
	vSupportPoints.push_back(Point2f(-1, 0));
	vSupportPoints.push_back(Point2f(0, 1));
	vSupportPoints.push_back(Point2f(0, -1));
	vSupportPoints.push_back(Point2f(0.70710678118, 0.70710678118)); // sqrt(2)/2 -> length=1
}

CircleRenderer::~CircleRenderer() {
}

void CircleRenderer::render(ShapeCollection* shapeCollection, cv::Mat& drawing,
		Transformer& transformer) {
	std::vector<Point2f> vCirclePosReal;
	std::vector<float> vRadius;
	std::vector<float> vThickness;
	std::vector<cv::Scalar> vColors;

	for (int i = 0; i < shapeCollection->circles_size(); i++) {
		Point2f circlePosField(shapeCollection->circles(i).vcenter().x(),
				shapeCollection->circles(i).vcenter().y());

		float radius = shapeCollection->circles(i).radius();
		radius = radius > .05 ? radius : .05;

		float thickness = shapeCollection->circles(i).linewidth();

		for (int j = 0; j < 5; j++) {
			vCirclePosReal.push_back(
					(vSupportPoints.at(j) * radius) + circlePosField);
		}

		cv::Scalar color(shapeCollection->circles(i).color().b(),
				shapeCollection->circles(i).color().g(),
				shapeCollection->circles(i).color().r());
		vRadius.push_back(radius);
		vThickness.push_back(thickness);
		vColors.push_back(color);
	}
	if (vCirclePosReal.size() > 3) {
		std::vector<Point2f> vCirclePosImage;
		perspectiveTransform(vCirclePosReal, vCirclePosImage, transformer.H);

		for (int i = 0; i < vCirclePosImage.size(); i += 5) {

			std::vector<Point2f> vSP(vCirclePosImage.begin() + i,
					vCirclePosImage.begin() + i + 5);
			cv::RotatedRect box = cv::fitEllipse(vSP);
			// circle(drawing, vCirclePosImage.at(i), vRadius.at(i), vColors.at(i), 4);
			int idx = floor(i / 5);
			ellipse(drawing, box, vColors.at(idx), vThickness.at(idx));
		}
	}
}

} // namespace tigers

