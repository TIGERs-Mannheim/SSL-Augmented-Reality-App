#ifndef LINERENDERER_H_
#define LINERENDERER_H_

#include "IRenderer.h"

namespace tigers
{

class LineRenderer: public IRenderer
{

public:
	LineRenderer();
	virtual ~LineRenderer();
	virtual void render(tigers::ShapeCollection* shapeCollection, cv::Mat& drawing,
			Transformer& transformer);
};

} /* namespace tigers */

#endif /* LINERENDERER_H_ */
