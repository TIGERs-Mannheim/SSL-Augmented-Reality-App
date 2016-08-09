#ifndef FIELDINIT_H_
#define FIELDINIT_H_

#include "FieldModel.h"
#include <vector>

namespace tigers{

    class FieldInit
    {
    public:
        FieldInit(FieldModel* fieldModel);
        ~FieldInit();

        bool isInitialized;
        bool active = true;
        bool initializeField(cv::Mat& H, cv::Mat imgDst);
        void initCorners(int width, int length);
        void initCornersH(cv::Mat H);
        void initFieldLines();

        void drawRawField();

        int maxDist;
        int selectedCorner;
        std::vector<cv::Point2f> corners;
        std::vector<bool> vValidCorners;
    private:
        cv::Mat imgSrc;
        FieldModel* fieldModel;
        std::vector<cv::Point2f*> vFieldLines;

    };
}
    static void pullCorner(int event, int x, int y, int flags, void* fieldInit);


#endif /*FIELDINIT_H_*/
