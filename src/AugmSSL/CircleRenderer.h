#ifndef CIRCLERENDERER_H_
#define CIRCLERENDERER_H_

#include "IRenderer.h"


namespace tigers
{

class CircleRenderer: public IRenderer
{

private:
    std::vector<cv::Point2f> vSupportPoints;
public:
    CircleRenderer();
    virtual ~CircleRenderer();
       virtual void render(tigers::ShapeCollection* shapeCollection, cv::Mat& drawing, Transformer& transformer);
};

} /* namespace tigers */

#endif /* CIRCLERENDERER_H_ */
