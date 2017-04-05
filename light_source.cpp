/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements light_source.h

***********************************************************/

#include <cmath>
#include "light_source.h"
#include "util.h"

void PointLight::shade( Ray3D& ray ) {
	// TODO: implement this function to fill in values for ray.col 
	// using phong shading.  Make sure your vectors are normalized, and
	// clamp colour values to 1.0.
	//
	// It is assumed at this point that the intersection information in ray 
	// is available.  So be sure that traverseScene() is called on the ray 
	// before this function.  
	
	
	double r_d, g_d, b_d, r_s, g_s, b_s, r_a, g_a, b_a;
	r_a = ray.intersection.mat->ambient[0];
	g_a = ray.intersection.mat->ambient[1];
	b_a = ray.intersection.mat->ambient[2];

	r_d = ray.intersection.mat->diffuse[0];
	g_d = ray.intersection.mat->diffuse[1];
	b_d = ray.intersection.mat->diffuse[2];

	r_s = ray.intersection.mat->specular[0];
	g_s = ray.intersection.mat->specular[1];
	b_s = ray.intersection.mat->specular[2];
	double spec_exp = ray.intersection.mat->specular_exp;
	
	Vector3D n = ray.intersection.normal;
	n.normalize();
	Vector3D s = _pos - ray.intersection.point;
	s.normalize();
	Vector3D de = -ray.dir;
	de.normalize();
	Vector3D m = 2*(n.dot(s))*n - s;
	m.normalize();

	double sdotn = s.dot(n);
	double mdotde = m.dot(de);
	
	ray.col[0]=r_a*_col_ambient[0]+r_d*_col_diffuse[0]*max(0,sdotn)+r_s*_col_specular[0]*pow((max(0, mdotde)),spec_exp);
	ray.col[1]=g_a*_col_ambient[1]+g_d*_col_diffuse[1]*max(0,sdotn)+g_s*_col_specular[1]*pow((max(0, mdotde)),spec_exp);
	ray.col[2]=b_a*_col_ambient[2]+b_d*_col_diffuse[2]*max(0,sdotn)+b_s*_col_specular[2]*pow((max(0, mdotde)),spec_exp);
	ray.col.clamp();
	
}

