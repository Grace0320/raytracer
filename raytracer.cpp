/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		Implementations of functions in raytracer.h, 
		and the main function which specifies the 
		scene to be rendered.	

***********************************************************/


#include "raytracer.h"
#include "bmp_io.h"
#include "util.h"
#include "materials.cpp"
#include <cmath>
#include <iostream>
#include <cstdlib>

Raytracer::Raytracer() : _lightSource(NULL) {
	_root = new SceneDagNode();
}

Raytracer::~Raytracer() {
	delete _root;
}

SceneDagNode* Raytracer::addObject( SceneDagNode* parent, 
		SceneObject* obj, Material* mat ) {
	SceneDagNode* node = new SceneDagNode( obj, mat );
	node->parent = parent;
	node->next = NULL;
	node->child = NULL;
	
	// Add the object to the parent's child list, this means
	// whatever transformation applied to the parent will also
	// be applied to the child.
	if (parent->child == NULL) {
		parent->child = node;
	}
	else {
		parent = parent->child;
		while (parent->next != NULL) {
			parent = parent->next;
		}
		parent->next = node;
	}
	
	return node;;
}

LightListNode* Raytracer::addLightSource( LightSource* light ) {
	LightListNode* tmp = _lightSource;
	_lightSource = new LightListNode( light, tmp );
	return _lightSource;
}

void Raytracer::rotate( SceneDagNode* node, char axis, double angle ) {
	Matrix4x4 rotation;
	double toRadian = 2*M_PI/360.0;
	int i;
	
	for (i = 0; i < 2; i++) {
		switch(axis) {
			case 'x':
				rotation[0][0] = 1;
				rotation[1][1] = cos(angle*toRadian);
				rotation[1][2] = -sin(angle*toRadian);
				rotation[2][1] = sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'y':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][2] = sin(angle*toRadian);
				rotation[1][1] = 1;
				rotation[2][0] = -sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'z':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][1] = -sin(angle*toRadian);
				rotation[1][0] = sin(angle*toRadian);
				rotation[1][1] = cos(angle*toRadian);
				rotation[2][2] = 1;
				rotation[3][3] = 1;
			break;
		}
		if (i == 0) {
		    node->trans = node->trans*rotation; 	
			angle = -angle;
		} 
		else {
			node->invtrans = rotation*node->invtrans; 
		}	
	}
}

void Raytracer::translate( SceneDagNode* node, Vector3D trans ) {
	Matrix4x4 translation;
	
	translation[0][3] = trans[0];
	translation[1][3] = trans[1];
	translation[2][3] = trans[2];
	node->trans = node->trans*translation; 	
	translation[0][3] = -trans[0];
	translation[1][3] = -trans[1];
	translation[2][3] = -trans[2];
	node->invtrans = translation*node->invtrans; 
}

void Raytracer::scale( SceneDagNode* node, Point3D origin, double factor[3] ) {
	Matrix4x4 scale;
	
	scale[0][0] = factor[0];
	scale[0][3] = origin[0] - factor[0] * origin[0];
	scale[1][1] = factor[1];
	scale[1][3] = origin[1] - factor[1] * origin[1];
	scale[2][2] = factor[2];
	scale[2][3] = origin[2] - factor[2] * origin[2];
	node->trans = node->trans*scale; 	
	scale[0][0] = 1/factor[0];
	scale[0][3] = origin[0] - 1/factor[0] * origin[0];
	scale[1][1] = 1/factor[1];
	scale[1][3] = origin[1] - 1/factor[1] * origin[1];
	scale[2][2] = 1/factor[2];
	scale[2][3] = origin[2] - 1/factor[2] * origin[2];
	node->invtrans = scale*node->invtrans; 
}

Matrix4x4 Raytracer::initInvViewMatrix( Point3D eye, Vector3D view, 
		Vector3D up ) {
	Matrix4x4 mat; 
	Vector3D w;
	view.normalize();
	up = up - up.dot(view)*view;
	up.normalize();
	w = view.cross(up);

	mat[0][0] = w[0];
	mat[1][0] = w[1];
	mat[2][0] = w[2];
	mat[0][1] = up[0];
	mat[1][1] = up[1];
	mat[2][1] = up[2];
	mat[0][2] = -view[0];
	mat[1][2] = -view[1];
	mat[2][2] = -view[2];
	mat[0][3] = eye[0];
	mat[1][3] = eye[1];
	mat[2][3] = eye[2];

	return mat; 
}

void Raytracer::traverseScene( SceneDagNode* node, Ray3D& ray ) {
	SceneDagNode *childPtr;

	// Applies transformation of the current node to the global
	// transformation matrices.
	_modelToWorld = _modelToWorld*node->trans;
	_worldToModel = node->invtrans*_worldToModel; 
	if (node->obj) {
		// Perform intersection.
		if (node->obj->intersect(ray, _worldToModel , _modelToWorld )) {
			ray.intersection.mat = node->mat;
		}
	}
	// Traverse the children.
	childPtr = node->child;
	while (childPtr != NULL) {
		traverseScene(childPtr, ray);
		childPtr = childPtr->next;
	}

	// Removes transformation of the current node from the global
	// transformation matrices.
	_worldToModel = node->trans*_worldToModel;
	_modelToWorld = _modelToWorld*node->invtrans;
}

void Raytracer::computeShading( Ray3D& ray ) {
	LightListNode* curLight = _lightSource;
	for (;;) {
		if (curLight == NULL) break;
		// Each lightSource provides its own shading function.

		// Implement shadows here if needed.

		curLight->light->shade(ray);
		curLight = curLight->next;
	}
}

void Raytracer::initPixelBuffer() {
	int numbytes = _scrWidth * _scrHeight * sizeof(unsigned char);
	_rbuffer = new unsigned char[numbytes];
	_gbuffer = new unsigned char[numbytes];
	_bbuffer = new unsigned char[numbytes];
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			_rbuffer[i*_scrWidth+j] = 0;
			_gbuffer[i*_scrWidth+j] = 0;
			_bbuffer[i*_scrWidth+j] = 0;
		}
	}
}

void Raytracer::flushPixelBuffer( char *file_name ) {
	bmp_write( file_name, _scrWidth, _scrHeight, _rbuffer, _gbuffer, _bbuffer );
	delete _rbuffer;
	delete _gbuffer;
	delete _bbuffer;
}

Colour Raytracer::shadeRay( Ray3D& ray, int mirrorcount, int transmission) {
	Colour col(0.0, 0.0, 0.0); 
	Colour mirrorCol(0.0, 0.0, 0.0);
	Colour transCol(0.0, 0.0, 0.0);
	traverseScene(_root, ray); 
	double R = 0;  //reflection coefficient

	// Don't bother shading if the ray didn't hit 
	// anything.
	if (!ray.intersection.none) {
		computeShading(ray); 
		Point3D origin = ray.intersection.point;  //in world coordinates already
		Vector3D d = ray.dir;
		d.normalize();
		Vector3D n = ray.intersection.normal;
		n.normalize();
		double n2 = ray.intersection.mat->r_index;
		double cosT, Ro;
		if(mirrorcount != 0) {
			//reflection
			
			//generate specular reflection coefficient
			LightListNode* curLight = _lightSource;
			Point3D lightPos = curLight->light->get_position();
			Vector3D incident = lightPos - origin;
			incident.normalize();
			Vector3D halfAngle = incident + -1*d;
			halfAngle.normalize();
			cosT = halfAngle.dot(incident);
			Ro = ((1-n2)/(1+n2))*((1-n2)/(1+n2));
			R = Ro + (1-Ro)*pow(1-cosT, 5);


			//glossy reflection
			//int i, a, b;
			//srand(1);
			//for(i = 0; i<49;i++){
			//	a = (double)rand()/RAND_MAX;
			//	b = (double)rand()/RAND_MAX;

				Vector3D mirror = 2*(n.dot(d))*n - d;
				mirror.normalize();

				//create new ray
				Ray3D bounceRay(origin, mirror);
				bounceRay.intersection.t_value = 100000;
				mirrorCol = shadeRay(bounceRay, mirrorcount - 1, transmission);
			//}
		}
		if(transmission!=0){
			//refraction (already have required normal n, ray direction d, & origin)
			//determine if refraction possible:
			double ratio = 1/(ray.intersection.mat->r_index);
			double cosT1 = -1*d.dot(n);
			double cos2T2 = 1 - ratio*ratio*(1-cosT1*cosT1);
			if (cos2T2 >= 0) {  //if not total internal refraction
				Vector3D refract = ratio*d + (ratio*cosT1 - sqrt(cos2T2))*n;
				Ray3D transRay(origin, refract);
				transRay.intersection.t_value = 100000;
				transCol = shadeRay(transRay, mirrorcount, transmission - 1);
			}
			
		}
		col = R*ray.col + mirrorCol + (1-R)*transCol ;
		col.clamp();
	}
	
	return col; 
}	

void Raytracer::render( int width, int height, Point3D eye, Vector3D view, 
		Vector3D up, double fov, char* fileName ) {
	Matrix4x4 viewToWorld;
	_scrWidth = width;
	_scrHeight = height;
	double factor = (double(height)/2)/tan(fov*M_PI/360.0);

	initPixelBuffer();
	viewToWorld = initInvViewMatrix(eye, view, up);

	// Construct a ray for each pixel.
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			// Sets up ray origin and direction in view space, 
			// image plane is at z = -1.
			Point3D origin(0, 0, 0);
			Point3D imagePlane;
			imagePlane[0] = (-double(width)/2 + 0.5 + j)/factor;
			imagePlane[1] = (-double(height)/2 + 0.5 + i)/factor;
			imagePlane[2] = -1;
			/*Point3D c1;
			c1[0] = (-double(width)/2 + 1 + j)/factor;
			c1[1] = (-double(height)/2 + 1 + i)/factor;
			c1[2] = -1;
			Point3D c2;
			c2[0] = (-double(width)/2 + j)/factor;
			c2[1] = (-double(height)/2  + i)/factor;
			c2[2] = -1;
			Point3D c3;
			c3[0] = (-double(width)/2 +0.5+ j)/factor;
			c3[1] = (-double(height)/2  + i)/factor;
			c3[2] = -1;
			Point3D c4;
			c4[0] = (-double(width)/2 + j)/factor;
			c4[1] = (-double(height)/2 +0.5 + i)/factor;
			c4[2] = -1;*/
			
			Point3D worldPixel = viewToWorld*imagePlane;
			Vector3D dir = worldPixel - eye;
			Ray3D ray(worldPixel, dir);
			ray.intersection.t_value = 100000;
			Colour col = shadeRay(ray, 2, 1);

			/*worldPixel = viewToWorld*c1;
			dir = worldPixel - eye;
			Ray3D ray2 (worldPixel, dir);
			col = col + shadeRay(ray2,2,1);

			worldPixel = viewToWorld*c2;
			dir = worldPixel - eye;
			Ray3D ray3(worldPixel, dir);
			col = col + shadeRay(ray3,2,1);

			worldPixel = viewToWorld*c3;
			dir = worldPixel - eye;
			Ray3D ray4(worldPixel, dir);
			col = col + shadeRay(ray4,2,1);

			worldPixel = viewToWorld*c4;
			dir = worldPixel - eye;
			Ray3D ray5(worldPixel, dir);
			col = col + shadeRay(ray5,2,1);

			double s = 0.2;
			col= s*col;	*/
			
			_rbuffer[i*width+j] = int(col[0]*255);
			_gbuffer[i*width+j] = int(col[1]*255);
			_bbuffer[i*width+j] = int(col[2]*255);
		}
	}

	flushPixelBuffer(fileName);
}

int main(int argc, char* argv[])
{	
	// Build your scene and setup your camera here, by calling 
	// functions from Raytracer.  The code here sets up an example
	// scene and renders it from two different view points, DO NOT
	// change this if you're just implementing part one of the 
	// assignment.  
	Raytracer raytracer;
	int width = 320; 
	int height = 240; 

	if (argc == 3) {
		width = atoi(argv[1]);
		height = atoi(argv[2]);
	}

	// Camera parameters.
	Point3D eye(0, 0, 1);
	Vector3D view(0, 0, -1);
	Vector3D up(0, 1, 0);
	double fov = 60;

	// Defines a point light source.
	raytracer.addLightSource( new PointLight(Point3D(2, 1, -5), 
				Colour(0.9, 0.9, 0.9) ) );

	// Add a unit square into the scene with material mat.
	SceneDagNode* sphere = raytracer.addObject( new UnitSphere(), &glass );
	//SceneDagNode* cyl = raytracer.addObject( new UnitCylinder(), &glass );
	SceneDagNode* plane = raytracer.addObject( new UnitSquare(), &gold);
	SceneDagNode* plane1 = raytracer.addObject( new UnitSquare(), &ruby);
	//SceneDagNode* plane2 = raytracer.addObject( new UnitSquare(), &ruby );
	//SceneDagNode* plane3 = raytracer.addObject( new UnitSquare(), &turquoise );

	// Apply some transformations to the unit square.
	double factor1[3] = { 1.0, 2.0, 1.0 };
	double factor2[3] = { 6.0, 6.0, 6.0 };
	double factor3[3] = { 4.0, 4.0, 4.0 };
	double factor4[3] = { 3.0, 3.0, 3.0 };
	double factor5[3] = { 2.0, 2.0, 2.0 };
	
	raytracer.translate(sphere, Vector3D(0, 0, -5));	
	raytracer.rotate(sphere, 'x', -45); 
	raytracer.rotate(sphere, 'z', 45); 
	//raytracer.scale(sphere, Point3D(0, 0, 0), factor1);

	raytracer.translate(plane, Vector3D(0, -1, -5));	
	raytracer.rotate(plane, 'x', -90); 
	raytracer.scale(plane, Point3D(0, 0, 0), factor2);

	raytracer.translate(plane1, Vector3D(-1, 0, -5));	
	raytracer.rotate(plane1, 'y', -90); 
	raytracer.scale(plane1, Point3D(0, 0, 0), factor2);

	/*raytracer.translate(plane2, Vector3D(0, 0, -9));	
	raytracer.rotate(plane2, 'y', -45); 
	raytracer.scale(plane2, Point3D(0, 0, 0), factor4);

	raytracer.translate(plane3, Vector3D(0, 0, -8.5));	
	raytracer.rotate(plane3, 'y', -45); 
	raytracer.scale(plane3, Point3D(0, 0, 0), factor5);*/

	//raytracer.translate(cyl, Vector3D(-3,0,-5));
	//raytracer.rotate(cyl, 'y', 45);
	
	// Render the scene, feel free to make the image smaller for
	// testing purposes.	
	raytracer.render(width, height, eye, view, up, fov, "view1.bmp");
	
	// Render it from a different point of view.
	Point3D eye2(4, 2, 1);
	Vector3D view2(-4, -2, -6);
	raytracer.render(width, height, eye2, view2, up, fov, "view2.bmp");
	
	return 0;
}

