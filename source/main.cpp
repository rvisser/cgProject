#ifdef WIN32
#include <windows.h>
#endif
#include <GL/glut.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "raytracing.h"
#include "mesh.h"
#include "traqueboule.h"
#include "imageWriter.h"
#include "KdTree.h"



//This is the main application
//Most of the code in here, does not need to be modified.
//It is enough to take a look at the function "drawFrame",
//in case you want to provide your own different drawing functions



Vec3Df MyCameraPosition;

//MyLightPositions stores all the light positions to use
//for the ray tracing. Please notice, the light that is 
//used for the real-time rendering is NOT one of these, 
//but following the camera instead.
std::vector<Vec3Df> MyLightPositions;

//Main mesh 
Mesh MyMesh;

unsigned int WindowSize_X = 800;  // resolution X
unsigned int WindowSize_Y = 800;  // resolution Y

unsigned int RenderSize_X = 800;
unsigned int RenderSize_Y = 800;

unsigned int selectedLight = 0;

unsigned int sampling = 4; //Supersampling factor. A value of 4 will lead to 16x supersampling (4 times x, 4 times y)
unsigned int bounces = 1;//max bounces determines reflection depth

bool kdTreeVerbose = false;

extern KD * tree;

GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };

/**
 * Main function, which is drawing an image (frame) on the screen
*/
void drawFrame( )
{
	yourDebugDraw();
}

//animation is called for every image on the screen once
void animate()
{
	MyCameraPosition=getCameraPosition();
	glutPostRedisplay();
}



void display(void);
void reshape(int w, int h);
void keyboard(unsigned char key, int x, int y);

/**
 * Main Programme
 */
int main(int argc, char** argv)
{
    glutInit(&argc, argv);

    //framebuffer setup
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );

    // positioning and size of window
    glutInitWindowPosition(200, 100);
    glutInitWindowSize(WindowSize_X,WindowSize_Y);
    glutCreateWindow(argv[0]);

    //initialize viewpoint
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0,0,-4);
    tbInitTransform();     // This is for the trackball, please ignore
    tbHelp();             // idem
	MyCameraPosition=getCameraPosition();

	//activate the light following the camera
    glEnable( GL_LIGHTING );
    glEnable(GL_COLOR_MATERIAL);

/*
    glEnable( GL_LIGHT0 );
    int LightPos[4] = {0,0,2,0};
    int MatSpec [4] = {1,1,1,1};
    glLightiv(GL_LIGHT0,GL_POSITION,LightPos);
    */


	//normals will be normalized in the graphics pipeline
	glEnable(GL_NORMALIZE);
    //clear color of the background is black.
	glClearColor (0.0, 0.0, 0.0, 0.0);


	// Activate rendering modes
    //activate depth test
	glEnable( GL_DEPTH_TEST );
    //draw front-facing triangles filled
	//and back-facing triangles as wires
    glPolygonMode(GL_FRONT,GL_FILL);
    glPolygonMode(GL_BACK,GL_FILL);
    //interpolate vertex colors over the triangles
	glShadeModel(GL_SMOOTH);

	// glut setup... to ignore
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutDisplayFunc(display);
    glutMouseFunc(tbMouseFunc);    // trackball
    glutMotionFunc(tbMotionFunc);  // uses mouse
    glutIdleFunc( animate);


	init();


	//main loop for glut... this just runs your application
    glutMainLoop();

    return 0;  // execution never reaches this point
}











/**
 * OpenGL setup - functions do not need to be changed! 
 * you can SKIP AHEAD TO THE KEYBOARD FUNCTION
 */
//what to do before drawing an image
 void display(void)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);//store GL state
    // Effacer tout
    glClear( GL_COLOR_BUFFER_BIT  | GL_DEPTH_BUFFER_BIT); // clear image

    glLoadIdentity();

    tbVisuTransform(); // init trackball

    drawFrame( );    //actually draw

    glutSwapBuffers();//glut internal switch
	glPopAttrib();//return to old GL state

	//Update the light sources in the scene so it has a shaded display.
	for (unsigned int i = 0; i < MyLightPositions.size(); ++i) {
		if(!glIsEnabled(GL_LIGHT0 + i)){
		    glEnable( GL_LIGHT0 + i);
		    glLightfv(GL_LIGHT0 + i, GL_AMBIENT, light_ambient);
		    glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, light_diffuse);
		    glLightfv(GL_LIGHT0 + i, GL_SPECULAR, light_specular);
		    std::cout << "I AM ENABLED :  " << GL_LIGHT0 + i << std::endl;
		}

		float xl = MyLightPositions[i][0];
		float yl = MyLightPositions[i][1];
		float zl = MyLightPositions[i][2];
	    float LightPos[4] = {xl, yl, zl, 1};

	    glLightfv(GL_LIGHT0 + i,GL_POSITION,LightPos);
	}
}
//Window changes size
void reshape(int w, int h)
{
    glViewport(0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //glOrtho (-1.1, 1.1, -1.1,1.1, -1000.0, 1000.0);
    gluPerspective (50, (float)w/h, 0.01, 1000);
    glMatrixMode(GL_MODELVIEW);
}


//transform the x, y position on the screen into the corresponding 3D world position
void produceRay(int x_I, int y_I, Vec3Df * origin, Vec3Df * dest)
{
		int viewport[4];
		double modelview[16];
		double projection[16];
		glGetDoublev(GL_MODELVIEW_MATRIX, modelview); //recuperer matrices
		glGetDoublev(GL_PROJECTION_MATRIX, projection); //recuperer matrices
		glGetIntegerv(GL_VIEWPORT, viewport);//viewport
		int y_new = viewport[3] - y_I;

		double x, y, z;

		gluUnProject(x_I, y_new, 0, modelview, projection, viewport, &x, &y, &z);
		origin->p[0]=float(x);
		origin->p[1]=float(y);
		origin->p[2]=float(z);
		gluUnProject(x_I, y_new, 1, modelview, projection, viewport, &x, &y, &z);
		dest->p[0]=float(x);
		dest->p[1]=float(y);
		dest->p[2]=float(z);
}










// react to keyboard input
void keyboard(unsigned char key, int x, int y)
{
    printf("key %d pressed at %d,%d\n",key,x,y);
    fflush(stdout);
    switch (key)
    {
	//add/update a light based on the camera position.
	case 'l':
		MyLightPositions.push_back(getCameraPosition());
		break;

	//Set last light to camera positions.
	case 'm':
		MyLightPositions[MyLightPositions.size()-1]=getCameraPosition();
		break;

	//Select previous light.
	case 52:		//touch left arrow
	{
			if(MyLightPositions.size() == 0){
				break;
			}
			else if(selectedLight == 0){
				selectedLight = MyLightPositions.size()-1;
				break;
			}
			else{
				selectedLight--;
				break;
			}
			break;
	}

	//Select next light.
	case 54:		//touch right arrow
	{
			if(MyLightPositions.size() == 0){
				break;
			}
			else if(selectedLight == MyLightPositions.size()-1){
				selectedLight = 0;
				break;
			}
			else{
				selectedLight++;
				break;
			}
			break;
	}

	//Move selected light to camera position.
	case 50:		//touch down arrow
		MyLightPositions[selectedLight]=getCameraPosition();
		break;

	//Remove selected light.
	case 'x':
	{
		if(MyLightPositions.size() == 1){
		    printf("You are not allowed to remove the last light in the scene.\n");
		    fflush(stdout);
		}
		else{
			MyLightPositions.erase(MyLightPositions.begin()+selectedLight);
			if(glIsEnabled(GL_LIGHT0 + selectedLight)){
					    glDisable( GL_LIGHT0 + selectedLight);
					    std::cout << "I AM DISABLED :  " << GL_LIGHT0 + selectedLight << std::endl;
			}
			if(selectedLight > 0)
				selectedLight--;
			else
				selectedLight = MyLightPositions.size()-1;
		}
		break;
	}
	case 's':{
		//Trace single ray
		kdTreeVerbose = true;
		Vec3Df testRayOrigin, testRayDestination;
		produceRay(x, y, &testRayOrigin, &testRayDestination);
		performRayTracing(testRayOrigin, testRayDestination,bounces);
		kdTreeVerbose = false;
		tree->prettyPrintHit(testRayOrigin, testRayDestination);
		break;
	}

	case 'r':
	{
		//Pressing r will launch the raytracing.
		cout<<"Raytracing"<<endl;


		//Setup an image with the size of the current image.
		Image result(RenderSize_X,RenderSize_Y);

		//produce the rays for each pixel, by first computing
		//the rays for the corners of the frustum.
		Vec3Df origin00, dest00;
		Vec3Df origin01, dest01;
		Vec3Df origin10, dest10;
		Vec3Df origin11, dest11;
		Vec3Df origin, dest;


		produceRay(0,0, &origin00, &dest00);
		produceRay(0,WindowSize_Y-1, &origin01, &dest01);
		produceRay(WindowSize_X-1,0, &origin10, &dest10);
		produceRay(WindowSize_X-1,WindowSize_Y-1, &origin11, &dest11);

		//Vec3Df colors [WindowSize_Y * WindowSize_X] = {};
		//array<Vec3Df, WindowSize_Y * WindowSize_X] colors = {};
		Vec3Df *colors;
		colors = new Vec3Df[RenderSize_Y * RenderSize_X];


		for (unsigned int y=0; y<RenderSize_Y;++y)
		{
			std::cout << "Progress: " << y << " of " << RenderSize_Y << std::endl;
			#pragma omp parallel for private(origin, dest), schedule(dynamic)
			for (unsigned int x=0; x<RenderSize_X;++x)
			{
				Vec3Df comp = Vec3Df(0, 0, 0);
				//produce the rays for each pixel, by interpolating 
				//the four rays of the frustum corners.

				//Use supersampling for a less pixelated result
				for(unsigned xs = 0; xs < sampling; ++xs){
					for(unsigned ys = 0; ys < sampling; ++ys){
						float xscale=1.0f-float(x*sampling + xs)/(RenderSize_X*sampling-1);
						float yscale=1.0f-float(y*sampling + ys)/(RenderSize_Y*sampling-1);

						origin=yscale*(xscale*origin00+(1-xscale)*origin10)+
							(1-yscale)*(xscale*origin01+(1-xscale)*origin11);
						dest=yscale*(xscale*dest00+(1-xscale)*dest10)+
							(1-yscale)*(xscale*dest01+(1-xscale)*dest11);

						//launch raytracing for the given ray.
						comp += performRayTracing(origin, dest, bounces);
					}
				}

				colors[RenderSize_X * y + x] = comp / (sampling * sampling);
				//store the result in an image 
			}
		}
		float maxintensity = 1;
		for (unsigned int y=0; y<RenderSize_Y;++y)
			for (unsigned int x=0; x<RenderSize_X;++x){
				maxintensity = std::max(maxintensity,colors[RenderSize_X * y + x][0]);
				maxintensity = std::max(maxintensity,colors[RenderSize_X * y + x][1]);
				maxintensity = std::max(maxintensity,colors[RenderSize_X * y + x][2]);
			}
		for (unsigned int y=0; y<RenderSize_Y;++y)
			for (unsigned int x=0; x<RenderSize_X;++x)
				result.setPixel(x,y, RGBAValue(colors[RenderSize_X * y + x][0]/maxintensity, colors[RenderSize_X * y + x][1]/maxintensity, colors[RenderSize_X * y + x][2]/maxintensity, 1));
		delete [] colors;
		result.writeImage("result.ppm");
		cout<<"Raytracing finished"<<endl;
		break;
	}
	case 27:     // touche ESC
        exit(0);
    }


	//produce the ray for the current mouse position
	Vec3Df testRayOrigin, testRayDestination;
	produceRay(x, y, &testRayOrigin, &testRayDestination);

	yourKeyboardFunc(key,x,y, testRayOrigin, testRayDestination);
}

//changeHighlight(old, new)
