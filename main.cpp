/*
 * main.cpp
 *
 *  Created on: Sep 30, 2011
 *      Author: yasir
 */

#include <Graph.hpp>
using namespace Olson;
int main()
{
	Olson::Graph G;
	G.addVertex(VERTEX_SE2(0,0.,0.,0.));
	G.addVertex(VERTEX_SE2(1,0.,1.,0.));
	G.addVertex(VERTEX_SE2(2,1.,0.,0.));
	G.addVertex(VERTEX_SE2(3,1.,1.,0.));
	G.addVertex(VERTEX_SE2(4,1.,1.,0.));

	G.addEdge(0,1,SE2(1.0,1.0,0.0),Eigen::Matrix3d::Identity());
	G.addEdge(1,2,SE2(1.0,0.0,0.0),Eigen::Matrix3d::Identity());
	G.addEdge(2,3,SE2(0.0,1.0,0.0),Eigen::Matrix3d::Identity());
	G.addEdge(3,4,SE2(-1.0,0.0,0.0),Eigen::Matrix3d::Identity()*1000000);
	G.addEdge(4,1,SE2(0.0,-1.0,0.0),Eigen::Matrix3d::Identity());


	G.print();

	G.DijkstraProjection(3);


	return 0;
}
