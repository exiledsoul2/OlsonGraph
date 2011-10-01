/*
 * Graph.hpp
 *
 *  Created on: Sep 30, 2011
 *      Author: yasir
 */

#ifndef GRAPH_HPP_
#define GRAPH_HPP_
#include <vector>
#include "se2.hpp"
#include "se2_edge.hpp"

#include <iostream>
namespace Olson
{
	using namespace Eigen;

	typedef std::vector< VERTEX_SE2, Eigen::aligned_allocator<VERTEX_SE2> > VertexList;
	typedef std::vector< EDGE_SE2 , Eigen::aligned_allocator<EDGE_SE2> > EdgeList;

	struct VERTEX_SE2
	{
		int id;
		SE2 pose;
		bool visited;
		EdgeList edges;
		VERTEX_SE2(): id(-1), visited(false){}
		VERTEX_SE2(int Id, SE2 Pose): id(Id), pose(Pose), visited(false){}
		VERTEX_SE2(int id, double x, double y, double theta):id(id),pose(x,y,theta),visited(false){}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	class Graph
	{
		VertexList Vertices;
		//std::vector< EDGE_SE2 > Edges;
		EdgeList Path;

		void setAllUnvisited()
		{
			VertexList::iterator it;
			for(it=Vertices.begin();it!=Vertices.end();it++)
				it->visited = false;
		}
		VERTEX_SE2& getVertexByID(int ID)
		{
			VertexList::iterator it;
			for(it=Vertices.begin();it!=Vertices.end();it++)
			{
				if(it->id == ID)
					return *it;
			}
			return (Vertices.back());
		}
	public:
		Graph(){}
		~Graph(){ Vertices.clear();}

		size_t VertexCount()
		{
			return Vertices.size();
		}

		bool addVertex(const VERTEX_SE2& v)
		{
			std::cout<<"Adding Vertex with ID "<<v.id<<std::endl;
			Vertices.push_back(v);
			return true;
		}
		bool addEdge(int from, int to, const SE2& Transf, const Matrix3d& information)
		{
			VERTEX_SE2& fromVertex = getVertexByID(from);
			VERTEX_SE2& toVertex   = getVertexByID(to);

			EDGE_SE2 e = EDGE_SE2(&toVertex,Transf,information);

			fromVertex.edges.push_back(e);
			toVertex.edges.push_back(EDGE_SE2(&fromVertex,e.inverse()));
			return true;
		}

		void DijkstraProjection(int Vertex_ID)
		{

			DijkstraProjection(getVertexByID(Vertex_ID));
		}

		void DijkstraProjection(VERTEX_SE2& startVertex)
		{
			Path.clear();
			setAllUnvisited();

			for(size_t i=0; i<startVertex.edges.size();i++)
			{
				Path.push_back((startVertex.edges[i]));
			}
			startVertex.visited = true;

			while(!allVisited())
			{
				EDGE_SE2 leastCostEdge = minUncertaintyPath();
				if(!getVertexByID(leastCostEdge.to()->id).visited)
				{
					getVertexByID(leastCostEdge.to()->id).visited = true;
					VERTEX_SE2 * b = leastCostEdge.to();
					std::cout<<"Shortest Path to vertex "<<b->id<<"-> "<<leastCostEdge.toVector().transpose()<<std::endl;
					std::cout<<leastCostEdge._covariance<<std::endl;
					Path.erase(Path.begin());
					for(size_t i=0;i<b->edges.size();i++)
					{
						if(!(getVertexByID(b->edges[i].to()->id).visited)){
							std::cout<<b->id<<" "<<b->edges[i].to()->id<<std::endl;
							Path.push_back((leastCostEdge)*b->edges[i]);
							std::cout<<Path.size()<<std::endl;
						}
					}
					EdgeList::iterator it;

				}
				else{
					//std::cout<<"Already Visited "<<leastCostEdge.to()->id<<std::endl;
					Path.erase(Path.begin());
				}

			}

		}

		bool allVisited()
		{
			for(size_t i=0; i<Vertices.size();i++)
			{
				if(!Vertices[i].visited)
				{
					return false;
				}
			}
			return true;
		}

		EDGE_SE2 minUncertaintyPath()
		{
			//find the Edge with the smallest det(covariance)
			std::sort(Path.begin(),Path.end());
//			std::cout<<"Now Contains : "<<std::endl;
//			for(EdgeList::iterator it = Path.begin(); it!=Path.end(); it++)
//			{
//				std::cout<<it->to()->id<<std::endl;
//			}
			//std::cout<< Path[0].to()->id<<std::endl;
			return Path[0];
		}

		void print()
		{
			VertexList::iterator vertices;
			for(vertices=Vertices.begin(); vertices!=Vertices.end();vertices++)
			{
				EdgeList::iterator e;
				std::cout<<"Vertex : ID "<<vertices->id<<" [ ";
				for(e=vertices->edges.begin(); e!=vertices->edges.end();e++)
				{
					std::cout<< e->_to->id << " ";
				}
				std::cout<<" ]"<<std::endl;
			}
		}
	};

}

#endif /* GRAPH_HPP_ */
