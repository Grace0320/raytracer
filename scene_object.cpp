/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements scene_object.h

***********************************************************/

#include <cmath>
#include <iostream>
#include "scene_object.h"

bool UnitSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
	// TODO: implement intersection code for UnitSquare, which is
	// defined on the xy-plane, with vertices (0.5, 0.5, 0), 
	// (-0.5, 0.5, 0), (-0.5, -0.5, 0), (0.5, -0.5, 0), and normal
	// (0, 0, 1).
	//
	// Your goal here is to fill ray.intersection with correct values
	// should an intersection occur.  This includes intersection.point, 
	// intersection.normal, intersection.none, intersection.t_value.   
	//
	// HINT: Remember to first transform the ray into object space  
	// to simplify the intersection test.

	//need transformed ray for affinely transformed squares
	Vector3D affDir = worldToModel*ray.dir;
	Point3D affOrigin = worldToModel*ray.origin;
	
	Point3D p0(0.5, 0.5, 0);
	Point3D p1(-0.5, 0.5, 0);
	Point3D p2(-0.5, -0.5, 0);
	Vector3D tmp1 = p2 -p0;
	Vector3D tmp2 = p1 -p0;
	Vector3D objNorm = tmp1.cross(tmp2);
	objNorm.normalize();
	objNorm = -1*objNorm;

	//find lambda
	Vector3D temp = p0 - affOrigin;
	double lambda = (temp.dot(objNorm))/(affDir.dot(objNorm));
	
	Point3D intersection = Point3D(affOrigin[0] + lambda*affDir[0], affOrigin[1] + lambda*affDir[1], affOrigin[2] + lambda*affDir[2]);
	if(intersection[0] <= 0.5 && intersection[0] >= -0.5 && intersection[1] <= 0.5 && intersection[1] >= -0.5){
	 	if(lambda < ray.intersection.t_value){
			ray.intersection.none = false;
			ray.intersection.t_value = lambda;
			objNorm = transNorm(worldToModel, objNorm);
			ray.intersection.normal = objNorm;
			ray.intersection.normal.normalize();
			ray.intersection.point = modelToWorld*intersection;
			return true;
		}
	}
	return false;
}

bool UnitSphere::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
	// TODO: implement intersection code for UnitSphere, which is centred 
	// on the origin.  
	//
	// Your goal here is to fill ray.intersection with correct values
	// should an intersection occur.  This includes intersection.point, 
	// intersection.normal, intersection.none, intersection.t_value.   
	//
	// HINT: Remember to first transform the ray into object space  
	// to simplify the intersection test.

	Point3D affOrigin = worldToModel*ray.origin;
	Vector3D affDir = worldToModel*ray.dir;
	
	Point3D objOrigin(0, 0, 0);
	Vector3D temp = affOrigin - objOrigin;
	double A = affDir.dot(affDir);
	double B = affDir.dot(temp);
	double C = temp.dot(temp) - 1;
	
	//find determinant
	double D = pow(B, 2) - A*C;
	double lambda1, lambda2;
	if (D >= 0) {
		lambda1 = -B/A + sqrt(D)/A;
		lambda2 = -B/A - sqrt(D)/A;
		Point3D intersection;
		Vector3D objNorm;
		
		double endLambda;
		
		
		//determine intersection lambda		
		if(lambda1 < 0) {
			if(lambda2 < 0) return false;
			else {
				endLambda = lambda2;
			}
		}
		else{
			if(lambda2 < 0) endLambda = lambda1;
			else{
				if(lambda1 > lambda2) endLambda = lambda2;
				else endLambda = lambda1;
			}
		}
		
		if(endLambda < ray.intersection.t_value){

				//calculate intersection point				
				intersection = Point3D(affOrigin[0] + endLambda*affDir[0], affOrigin[1] + endLambda*affDir[1], affOrigin[2] + endLambda*affDir[2]);
				//normal to a sphere n= <2x, 2y, 2z>
				objNorm[0] = 2*intersection[0];
				objNorm[1] = 2*intersection[1];
				objNorm[2] = 2*intersection[2];
				objNorm.normalize();
				ray.intersection.point = modelToWorld*intersection;
				ray.intersection.t_value = endLambda;
				objNorm = transNorm(worldToModel, objNorm);
				ray.intersection.normal = objNorm;
				ray.intersection.none = false;
				return true;
		}
		
	}
	return false;
}

bool UnitCylinder::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
	

	double lambda0, lambda1;
	const Point3D start = worldToModel*ray.origin;
	const Vector3D dir = worldToModel*ray.dir;

	
	double a = dir[0]*dir[0] + dir[2]*dir[2];
	double b = 2*start[0]*dir[0] + 2*start[2]*dir[2];
	double c = start[0]*start[0] + start[2]*start[2]
		- 1;

	double d = b*b - 4*a*c;
	if (d<0)
		return false;
	
	double sqd = sqrtf(d);
	lambda0 = (-b + sqd) / (2 * a);
	lambda1 = (-b - sqd) / (2 * a);

	//order lambdas
	if (lambda0>lambda1) {float tmp = lambda0;lambda0=lambda1;lambda1=tmp;}

	double y0 = start[1] + lambda0 * dir[1];
	double y1 = start[1] + lambda1 * dir[1];

	Point3D intersection;
	Vector3D normal;

	if (y0<-1)
	{
		if (y1<-1)
			return false;
		else
		{
			// hit the cap
			double lambdah = lambda0 + (lambda1-lambda0) * (y0+1) / (y0-y1);
			if (lambdah<=0) return false;
			if (lambdah < ray.intersection.t_value) {
				intersection = Point3D(start[0]+lambdah*dir[0],start[1]+lambdah*dir[1],start[2]+lambdah*dir[2]);
				normal = Vector3D(0, -1, 0);
				ray.intersection.point = modelToWorld*intersection;
				ray.intersection.normal = transNorm(worldToModel, normal);
				ray.intersection.t_value = lambdah;
				ray.intersection.none = false;
				return true;
			}
			else return false;
		}
	}
	else if (y0>=-1 && y0<=1)
	{
		// hit the cylinder
		if (lambda0<=0) return false;
		if(lambda0 < ray.intersection.t_value){
			intersection = Point3D(start[0]+lambda0*dir[0],start[1]+lambda0*dir[1],start[2]+lambda0*dir[2]);
			normal = Vector3D(intersection[0], 0, intersection[2]);
			ray.intersection.point = modelToWorld*intersection;
			ray.intersection.normal = transNorm(worldToModel, normal);
			ray.intersection.normal.normalize();
			ray.intersection.t_value = lambda0;
			ray.intersection.none = false;
			return true;
		}
		else return false;
	}
	else if (y0>1)
	{
		if (y1>1) return false;
		else
		{
			// hit the other cap
			float lambdah = lambda0 + (lambda1-lambda0) * (y0-1) / (y0-y1);
			if (lambdah<=0) return false;
			if(lambdah<ray.intersection.t_value){
				intersection = Point3D(start[0]+lambdah*dir[0],start[1]+lambdah*dir[1],start[2]+lambdah*dir[2]);
				normal = Vector3D(0, 1, 0);
				ray.intersection.point = modelToWorld*intersection;
				ray.intersection.normal = transNorm(worldToModel, normal);
				ray.intersection.normal.normalize();
				ray.intersection.t_value = lambdah;
				ray.intersection.none = false;
				return true;
			}
			else return false;
		}
	}

	return false;

}







