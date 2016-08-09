#ifndef GOALPOINTSRENDERER_H_
#define GOALPOINTSRENDERER_H_

#include "IRenderer.h"

namespace tigers {

class TextRenderer: public IRenderer {

private:
	std::vector<cv::Point3f> vSupportPoints;
public:
	TextRenderer();
	virtual ~TextRenderer() {
	}
	virtual void render(tigers::ShapeCollection* shapeCollection, cv::Mat& drawing,
			Transformer& transformer);
};

} /* namespace tigers */

#endif /* GOALPOINTSRENDERER_H_ */
