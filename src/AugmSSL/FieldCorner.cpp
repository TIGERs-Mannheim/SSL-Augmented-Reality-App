/*
 * FieldCorner.cpp
 *
 *  Created on: Jul 2, 2014
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "FieldCorner.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

namespace tigers
{

FieldCorner::FieldCorner()
{
}

FieldCorner::~FieldCorner()
{
}

std::ostream& operator<<(std::ostream& out, const FieldCorner& corner) // output
{
	out << corner.point << " ->";
	for (int i = 0; i < corner.getNumNeighbors(); i++)
	{
		out << " " << corner.getNeighbor(i)->point;
	}
	return out;
}

std::vector<FieldCorner*> FieldCorner::getNeighbors() const
{
	// return by value (copy of neighbors)
	return neighbors;
}

int FieldCorner::getNumNeighbors() const
{
	return neighbors.size();
}

FieldCorner* FieldCorner::getNeighbor(int i) const
{
	return neighbors.at(i);
}

void FieldCorner::pop_Neighbor()
{
	if (neighbors.size() <= 1)
	{
		neighbors = std::vector<FieldCorner*>();
	}
	else
	{
		neighbors.pop_back();
	}
}

void FieldCorner::insert_Neighbor(FieldCorner* nb)
{
	if (nb == this)
	{
		// can not be neighbor of myself
		return;
	}
	for (int i = 0; i < neighbors.size(); i++)
	{
		if (neighbors.at(i) == nb)
		{
			// already inserted
			return;
		}
	}
	neighbors.push_back(nb);
}

void FieldCorner::remove_Neighbor(FieldCorner* nb)
{
	std::vector<FieldCorner*> oldNbs(neighbors);
	neighbors.clear();
	for (int i = 0; i < oldNbs.size(); i++)
	{
		if (oldNbs.at(i) != nb)
		{
			neighbors.push_back(oldNbs.at(i));
		}
	}
}

} /* namespace tigers */
