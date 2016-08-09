/*
 * FieldCorner.h
 *
 *  Created on: Jul 2, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#ifndef FIELDCORNER_H_
#define FIELDCORNER_H_


#include <opencv2/imgproc/imgproc.hpp>

namespace tigers
{

class FieldCorner
{
public:
	FieldCorner();
	virtual ~FieldCorner();

	cv::Point2f point;

	std::vector<FieldCorner*> getNeighbors() const;
	int getNumNeighbors() const;
	FieldCorner* getNeighbor(int i) const;
	void pop_Neighbor();
	void insert_Neighbor(FieldCorner* nb);
	void remove_Neighbor(FieldCorner* nb);
	friend std::ostream& operator<<(std::ostream& out, const FieldCorner& corner);

private:
	std::vector<FieldCorner*> neighbors;


};

} /* namespace tigers */

#endif /* FIELDCORNER_H_ */
