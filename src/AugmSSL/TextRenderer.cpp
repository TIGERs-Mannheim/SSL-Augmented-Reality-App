#include "TextRenderer.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
namespace tigers {
TextRenderer::TextRenderer() {
}

void TextRenderer::render(ShapeCollection* shapeCollection, cv::Mat& drawing,
		Transformer& transformer) {
	std::vector < Point3f > vRatingPosReal;
	std::vector < cv::Scalar > vColors;
	std::vector < std::string > vStrings;

	for (int i = 0; i < shapeCollection->texts_size(); i++) {
		tigers::Text txt = shapeCollection->texts(i);

		Point3f ratingPosField(txt.vpoint().x(), txt.vpoint().y(), 0);

		cv::Scalar color(txt.color().b(), txt.color().g(), txt.color().r());
		vColors.push_back(color);

		vStrings.push_back(txt.text());
	}

	if (vRatingPosReal.size() > 0) {
		std::vector < Point2f > vRatingPosImage;
		transformer.transform(vRatingPosReal, vRatingPosImage);

		for (int i = 0; i < vRatingPosImage.size(); i += 1) {
			putText(drawing, vStrings.at(i), vRatingPosImage[i], FONT_HERSHEY_PLAIN, 1,
							vColors.at(i));
		}
	}
}

} // namespace tigers

