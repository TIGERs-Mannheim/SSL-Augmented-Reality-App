#ifndef POINTRENDERER_H_
#define POINTRENDERER_H_

#include "IRenderer.h"


namespace tigers
{

class PointRenderer: public IRenderer
{

private:
    std::vector<cv::Point2f> vSupportPoints;
public:
    PointRenderer();
    virtual ~PointRenderer();
       virtual void render(tigers::ShapeCollection* shapeCollection, cv::Mat& drawing, Transformer& transformer);
};

} /* namespace tigers */

#endif /* POINTRENDERER_H_ */
