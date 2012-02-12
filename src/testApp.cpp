#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
    // enable depth->rgb image calibration
    // esto e para sincronizar as duas camaras, para que coincidan os pixeles
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	kinect.open();
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 90;
	farThreshold = 40;
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle); // angulo da camara. Cambiar
	
	// start from the front
	bDrawPointCloud = false;

    /*
     ---------------INICIAMOS LA LETRA
     */
    
    //cargamos el tipo de letra
    font.loadFont("Futura-Bold.ttf", 700, true, true, true);//tipo, tama–o, antialiasing, todos caracteres, forma vectorial
    fontPercent.loadFont("Futura-Bold.ttf", 40);
    abecedario="ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    
    //repartimos la cadena en un vector
    for (int i=0; i<abecedario.size(); i++) {
        letras.push_back(ofToChar(abecedario.substr(i)));
        
    }
    //sincronizaci—n
    ofSetVerticalSync (true);
    ofBackground (0);
    
    //escoge una nueva
    escogeNueva();
    verLetra = false;

    // warp cosas
    warp.resize(4);
    currentPoint = 0;
    warpDone=false;
    
    contadorFotos = 0;
    proyectaFoto = false;
    retardo = 3;
    percent = 0;
}

//--------------------------------------------------------------
void testApp::update() {
	
    // borramos puntos dentro
    puntosDentro.clear();
    puntosFuera.clear();

	ofBackground(0, 0, 0);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		if(warpDone){
            grayImage.warpPerspective(warp[0], warp[1], warp[2], warp[3]);
            grayImage.mirror(false, true);

            grayImage.flagImageChanged();
        }
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImage.getPixels();
			
			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
        
        
		
		// update the cv images
        // Sube as cousas aa textura
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
        
        contadorPuntosDentro = 0;
        puntosTotales = 0;
        
        if(dibujoLetra.getOutline().size()>0){
            for(int j=0; j<contourFinder.blobs.size(); j++){
                for(int i= 0; i<contourFinder.blobs[j].pts.size(); i++){
                    puntosTotales++;
                    
                    ofPoint p = contourFinder.blobs[j].pts[i];
                    ofPoint pMapeado;
                    pMapeado.x = ofMap(p.x, 0, 640, 0, ofGetWidth());
                    pMapeado.y = ofMap(p.y, 0, 480, 0, ofGetHeight());
                    ofPoint pDesplazado = pMapeado - ofPoint(ofGetWidth()/2-350,ofGetHeight());
                    
                    bool dentroContorno;   
                    dentroContorno = ofInsidePoly(pDesplazado, dibujoLetra.getOutline()[0].getVertices());
                    
                    bool dentroFurado=false;
                    for(int h = 1; h<dibujoLetra.getOutline().size(); h++){
                        dentroFurado = ofInsidePoly(pDesplazado, dibujoLetra.getOutline()[h].getVertices());
                        if(dentroFurado){
                            break;
                        }
                    }
                    
                    if(dentroContorno && !dentroFurado){
                      //  escogeNueva();
                        contadorPuntosDentro++;
                        puntosDentro.push_back(pMapeado);
                    }else{
                        puntosFuera.push_back(pMapeado);                        
                    }
                }
            }
        
            float umbral = puntosTotales * 0.90;
            if(puntosDentro.size() >= umbral && !proyectaFoto && contourFinder.nBlobs > 0){
                fotoFinish();
            }
            
            percent = float(puntosDentro.size()) / float(puntosTotales) *100.;
            
        }
        

	}
    
    if((proyectaFoto && ofGetElapsedTimef() - tiempoInicial > retardo) || contourFinder.nBlobs==0){
        proyectaFoto = false;
        escogeNueva();
    }
	
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void testApp::draw() {
	
    if(verLetra){
        ofHideCursor();
        ofBackground (0,0,0);
        //lo pintamos en el escenario
        ofSetColor(255, 255, 255);
        ofFill();
        //dibujoLetra.setFilled(true);
        dibujoLetra.draw(ofGetWidth()/2-350,ofGetHeight());
        
        ofSetColor(0, 255, 0);
        for(int i=0; i<puntosDentro.size(); i++){
            ofCircle(puntosDentro[i], 3);            
        }
        
        ofSetColor(255, 0, 0);
        for(int i=0; i<puntosFuera.size(); i++){
            ofCircle(puntosFuera[i], 3);            
        }
        
        
        //font.drawString("A",200,100);
        if (percent>0){
        fontPercent.drawString(ofToString(percent)+"%", 20, 60);
        }
        //ofDrawBitmapString(ofToString(contadorPuntosDentro) + " " + ofToString(puntosTotales),20,50);



        if(proyectaFoto){
            ofSetColor(255, 255, 255);
            fotoMaton.draw(0,0, ofGetWidth(), ofGetHeight());
        }
        
    }else{
        ofShowCursor();
        ofSetColor(255, 255, 255);
        
        if(bDrawPointCloud) {
            easyCam.begin();
            drawPointCloud();
            easyCam.end();
        } else {
            // draw from the live kinect
            kinect.drawDepth(660, 10, 400, 300);
            kinect.draw(0, 0);
            
            grayImage.draw(660, 320, 400, 300);
            contourFinder.draw(660, 320, 400, 300);
            
            // circulos del warp
            for(int i = 0; i<warp.size(); i++){
                ofCircle(warp[i], 5);
            }
            
#ifdef USE_TWO_KINECTS
            kinect2.draw(420, 320, 400, 300);
#endif
        }
        
        // draw instructions
        ofSetColor(255, 255, 255);
        stringstream reportStream;
        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl
        << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
        << "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
        << "set near threshold " << nearThreshold << " (press: + -)" << endl
        << "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
        << ", fps: " << ofGetFrameRate() << endl
        << "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
        << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl;
        ofDrawBitmapString(reportStream.str(),20,652);
        

    }

}

void testApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
    
}


//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
            
        case 'l':
            verLetra = !verLetra;
            break;
        
        case 'f':
            ofToggleFullscreen();
            break;
	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
    if(verLetra){
        escogeNueva();
    }else{
        // Marcar puntos para warp.
        warp[currentPoint].set(x, y);
        currentPoint ++;
        if(currentPoint == 4){
            currentPoint = 0;
            warpDone = true;
        }
    }
}
//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
                             
//--------------------------------------------------------------
void testApp::escogeNueva () {
    //cogemos un numero aleatorio
    int alea=int(ofRandom(0, abecedario.size()-1));
    //lo pillamos del vector
    letraActual=letras[alea];
    dibujoLetra = font.getCharacterAsPoints(letraActual);
                                 
}

//--------------------------------------------------------------
void testApp::fotoFinish() {
    contadorFotos++;
    proyectaFoto = true;
    fotoMaton.setFromPixels(kinect.getPixels(), 640, 480, OF_IMAGE_COLOR);
    string prefijo=ofToString(letraActual);
    fotoMaton.saveImage(prefijo+"_foto_" + ofGetTimestampString() + ".jpg");
    tiempoInicial = ofGetElapsedTimef();
}



