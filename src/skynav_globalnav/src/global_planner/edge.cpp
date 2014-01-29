/*
 * edge.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: jurriaan
 */

#include "graph.h"

Edge::Edge(Node* p_A, Node* p_B, float fLength) {
	this->p_mA = p_A;
	this->p_mB = p_B;
	this->mLenght = fLength;
}

Edge::Edge(Node* p_A, Node* p_B) {
	this->p_mA = p_A;
	this->p_mB = p_B;
	this->mLenght = -1; //edge length can never be negative, CHECK if lenght has not been assigned
}

Edge::~Edge() {
	// TODO Auto-generated destructor stub
}

float Edge::getLength() {
	if (!mLenght == -1) {
		return mLenght;
	} else {
		return calcLenght();
	}
}

float Edge::calcLenght() {
	int xd, yd;
	float d;
	xd = p_mB->getXpos() - p_mA->getXpos();
	yd = p_mB->getYpos() - p_mA->getYpos();
	// Euclidian Distance
	d = (sqrt((xd * xd) + (yd * yd))); // Manhattan distance: d=abs(xd)+abs(yd) // Chebyshev distance: d=max(abs(xd), abs(yd));
	return d;
}
bool Edge::compare(Edge* p_edge) {
	if (p_mA == p_edge->p_mA || p_mA == p_edge->p_mB) {
		if (p_mB == p_edge->p_mA || p_mB == p_edge->p_mB) {
			return true;
		}
	}
	return false;
}

Node* Edge::getA() {
	return p_mA;
}
Node* Edge::getB() {
	return p_mB;
}

