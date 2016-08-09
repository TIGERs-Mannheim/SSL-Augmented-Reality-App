#include "LineRenderer.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

namespace tigers {

//static void arrowedLine(cv::Mat& img, Point2f pt1, Point2f pt2,
//						const Scalar& color, int thickness = 1, int lineType = 8, int shift = 0,
//						double normalizationFactor = 10)
//{
//	const double tipSize = norm(pt1 - pt2) / normalizationFactor; // Factor to normalize the size of the tip depending on the length of the arrow
//	line(img, pt1, pt2, color, thickness, lineType, shift);
//	const double angle = atan2((double) pt1.y - pt2.y, (double) pt1.x - pt2.x);
//	Point2f p((pt2.x + tipSize * cos(angle + CV_PI / 4)),
//			  (pt2.y + tipSize * sin(angle + CV_PI / 4)));
//	line(img, p, pt2, color, thickness, lineType, shift);
//
//	p.x = (pt2.x + tipSize * cos(angle - CV_PI / 4));
//	p.y = (pt2.y + tipSize * sin(angle - CV_PI / 4));
//	line(img, p, pt2, color, thickness, lineType, shift);
//}

static void wideLine(std::vector<Point3f>& outpts, Point3f pt1, Point3f pt2,
		float width = 20) {
	const double angle = atan2((double) pt1.y - pt2.y, (double) pt1.x - pt2.x);

	Point3f p1((pt1.x + (width * 0.5 * sin(angle))),
			pt1.y + (width * 0.5 * cos(angle)), pt1.z);

	Point3f p2((pt1.x - (width * 0.5 * sin(angle))),
			pt1.y - (width * 0.5 * cos(angle)), pt1.z);

	Point3f p3((pt2.x + (width * 0.5 * sin(angle))),
			pt2.y + (width * 0.5 * cos(angle)), pt1.z);

	Point3f p4((pt2.x - (width * 0.5 * sin(angle))),
			pt2.y - (width * 0.5 * cos(angle)), pt1.z);

	outpts.push_back(p1);
	outpts.push_back(p2);
	outpts.push_back(p4);
	outpts.push_back(p3);
}

LineRenderer::LineRenderer() {
}

LineRenderer::~LineRenderer() {
}

void LineRenderer::render(ShapeCollection* shapeCollection, cv::Mat& drawing,
		Transformer& transformer) {
	std::vector<Point3f> vVectorTips; // index%2==0: start; index%2==1: end
	std::vector<cv::Scalar> vColors;
	for (int i = 0; i < shapeCollection->lines_size(); i++) {
		tigers::Line l = shapeCollection->lines(i);
		Point3f pointFieldStart(l.vstart().x(), l.vstart().y(), l.vstart().z());
		Point3f pointFieldEnd(l.vend().x(), l.vend().y(), l.vend().z());
		cv::Scalar color(l.color().b(), l.color().g(), l.color().r());
		vColors.push_back(color);
		wideLine(vVectorTips, pointFieldStart, pointFieldEnd, l.linewidth());
	}
	if (vColors.size() > 3) {
		std::vector<Point2f> vPointsImage;
		transformer.transform(vVectorTips, vPointsImage);
		cv::Point polyPoints[4];
		for (int i = 0; i < vPointsImage.size(); i += 4) {
			std::copy(vPointsImage.begin() + i, vPointsImage.begin() + i + 4,
					polyPoints);
			cv::fillConvexPoly(drawing, polyPoints, 4,
					vColors.at(floor(i / 4)));

//			arrowedLine(drawing, vPointsImage.at(i), vPointsImage.at(i + 1),
//					vColors.at(floor(i / 2)));
		}
	}
}

} // namespace tigers

