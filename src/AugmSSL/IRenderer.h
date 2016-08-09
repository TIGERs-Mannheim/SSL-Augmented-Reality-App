#ifndef IRENDERER_H_
#define IRENDERER_H_
#include "set"
#include "augm_wrapper.pb.h"
#include <opencv2/highgui/highgui.hpp>
#include "Transformer.h"

namespace tigers {

class IRenderer {

public:
	virtual ~IRenderer() {
	}
	virtual void render(tigers::ShapeCollection* shapeCollection, cv::Mat& drawing,
			Transformer& transformer) = 0;
};

} /* namespace tigers */

#endif /* IRENDERER_H_ */
