/***************************************************************
 * Name:      FexWrite.cpp
 * Purpose:   extract and display the descriptors according to the input parameters
 * Author:    Muhammet Bastan (bastan@cs.bilkent.edu.tr)
 * Created:   16.06.2009
 * Copyright: Muhammet Bastan (http://www.cs.bilkent.edu.tr/~bastan)
 * License:
 **************************************************************/

#include <iostream>
#include <fstream>

#include "FexWrite.h"

// Color Structure
void FexWrite::computeWriteCSD( Frame* frame, int descSize )
{
	if(!frame)
		return;

    // compute the descriptor
	XM::ColorStructureDescriptor* csd = Feature::getColorStructureD(frame, descSize);

    std::cout << "\n---- Color Structure ---" << std::endl;

	// write to screen
	for(unsigned int i = 0; i < csd->GetSize(); i++)
		std::cout << (int)csd->GetElement(i) << " " ;
	std::cout << std::endl;

    std::cout << "---------------" << std::endl;

    // release descriptor
	delete csd;
}


// Scalable Color
void FexWrite::computeWriteSCD( Frame* frame, bool maskFlag, int descSize )
{
	if(!frame)
		return;

    // compute descriptor
	XM::ScalableColorDescriptor* scd = Feature::getScalableColorD( frame, maskFlag, descSize );

    std::cout << "\n---- Scalable Color ---" << std::endl;

	// write to screen
	for(unsigned int i = 0; i < scd->GetNumberOfCoefficients(); i++)
		std::cout << (int)  scd->GetCoefficient(i) << " ";
	std::cout << std::endl;

    std::cout << "---------------" << std::endl;

    // release descriptor
	delete scd;
}


// Color Layout
void FexWrite::computeWriteCLD( Frame* frame, int numYCoef, int numCCoef, char* filepath)
{
	if(!frame)
		return;
	std::ofstream file;
	file.open(filepath);

    // compute the descriptor
	XM::ColorLayoutDescriptor* cld = Feature::getColorLayoutD( frame, numYCoef, numCCoef );

//    std::cout << "\n---- Color Layout ---" << std::endl;
//    file << "\n---- Color Layout ---" << std::endl;

	// write to screen

	// number of coefficients
	int numberOfYCoeff = cld->GetNumberOfYCoeff();
	int numberOfCCoeff = cld->GetNumberOfCCoeff();


	// values
	int *y_coeff, *cb_coeff, *cr_coeff;
	y_coeff = cld->GetYCoeff();
	cb_coeff = cld->GetCbCoeff();
	cr_coeff = cld->GetCrCoeff();



	int i = 0;
	// Y coeff (DC and AC), first value is the DC value
	for ( i = 0; i < numberOfYCoeff; i++ ){
//		std::cout << y_coeff[i] << " " ;
		file << y_coeff[i] << " " ;
	}
	file << "\n";

	//Cb coeff  (DC and AC), first value is the DC value
	for ( i = 0; i < numberOfCCoeff; i++ ){
//		std::cout << cb_coeff[i] << " ";
		file << cb_coeff[i] << " ";
	}
	file << "\n";

	//Cr coeff  (DC and AC), first value is the DC value
	for ( i = 0; i < numberOfCCoeff; i++ ){
//		std::cout << cr_coeff[i] << " ";
		file << cr_coeff[i] << " ";
	}


//	std::cout << std::endl;

//	std::cout << "---------------" << std::endl;

    // release the descriptor
	delete cld;
	file.close();
}


// Dominant Color
void FexWrite::computeWriteDCD( Frame* frame,
                                bool normalize, bool variance, bool spatial,
                                int bin1, int bin2, int bin3 )
{
	if(!frame)
		return;

    // compute the descriptor
	XM::DominantColorDescriptor* dcd = Feature::getDominantColorD( frame, normalize, variance, spatial, bin1, bin2, bin3 );

    std::cout << "\n---- Dominant Color ---" << std::endl;

	// write to screen

	// number of dominant colors
	int ndc = dcd->GetDominantColorsNumber();
	//std::cout << ndc << " " ;

	// spatial coherency
	if(spatial)
        std::cout << dcd->GetSpatialCoherency();

	// dominant colors: percentage(1) centroid value (3) color variance (3)
	XM::DOMCOL* domcol = dcd->GetDominantColors();
	for( int i = 0; i < ndc; i++ )
	{
		std::cout << " " << domcol[i].m_Percentage
                  << " " << domcol[i].m_ColorValue[0]
                  << " " << domcol[i].m_ColorValue[1]
                  << " " << domcol[i].m_ColorValue[2];
        if(variance)
        std::cout << " " << domcol[i].m_ColorVariance[0]
                  << " " << domcol[i].m_ColorVariance[1]
                  << " " << domcol[i].m_ColorVariance[2];
	}

	std::cout << std::endl;

	std::cout << "---------------" << std::endl;

    // release the descriptor
	delete dcd;
}

// Face Recognition
// input: frame->gray, where gray is the grayscale, normalized image of size 46x56
void FexWrite::computeWriteFRD( Frame* frame )
{
	if(!frame)
		return;

    // compute the descriptor
	FaceRecognitionFeature  frf;
	XM::FRD* frd = frf.getFaceRecognitionD( frame );


    std::cout << "\n---- Face Recognition ---" << std::endl;

	// write to screen: a vector of size 48
	for( unsigned int i = 0; i < 48; i++)
		std::cout << frd->eigenfeature[i] << " ";
	std::cout  << std::endl;

    std::cout << "---------------" << std::endl;

    // release the descriptor
	delete frd;
}

// Homogeneous Texture
void FexWrite::computeWriteHTD( Frame* frame, int layerFlag )
{
	if(!frame)
		return;

	XM::HomogeneousTextureDescriptor* htd = Feature::getHomogeneousTextureD( frame, layerFlag );

    // get a pointer to the values
	int* values = htd->GetHomogeneousTextureFeature();

    std::cout << "\n---- Homogeneous Texture ---" << std::endl;

	// write to screen

	// values[0]: mean, values[1]: std, values[2-31] base layer (energy)
	int i;
	for(i = 0; i < 32; i++)
		std::cout << values[i] << " " ;

    // if full layer, print values[32-61] (energy deviation)
    if(layerFlag)
        for(i = 32; i < 62; i++)
            std::cout << values[i] << " " ;

	std::cout << std::endl;

    std::cout << "---------------" << std::endl;

    // release the descriptor
	delete htd;
}


// Edge Histogram
void FexWrite::computeWriteEHD( Frame* frame )
{
	if(!frame)
		return;

    // compute the descriptor
	XM::EdgeHistogramDescriptor* ehd = Feature::getEdgeHistogramD( frame );

    // get a pointer to the values
	char* de = ehd->GetEdgeHistogramElement();

    std::cout << "\n---- Edge Histogram ---" << std::endl;

	// write to screen
	for( unsigned int i = 0; i < ehd->GetSize(); i++)
		std::cout << (int)de[i] << " ";
	std::cout  << std::endl;

    std::cout << "---------------" << std::endl;

    // release the descriptor
	delete ehd;
}


// Region Shape
void FexWrite::computeWriteRSD( Frame* frame )
{
    if(!frame)
		return;

    // compute the descriptor
    XM::RegionShapeDescriptor* rsd = Feature::getRegionShapeD( frame );

    std::cout << "\n---- Region Shape ---" << std::endl;

    int i,j;
	for(i=0; i<ART_ANGULAR; i++)
		for(j=0; j<ART_RADIAL; j++)
			if(i != 0 || j != 0)
				std::cout << (int)rsd->GetElement(i, j) << " ";

	std::cout << std::endl;

    std::cout << "---------------" << std::endl;

    // release the descriptor
    delete rsd;
}

// Contour Shape
void FexWrite::computeWriteCShD( Frame* frame )
{
    if(!frame)
		return;

    // compute the descriptor
    XM::ContourShapeDescriptor* csd = Feature::getContourShapeD( frame );

    std::cout << "\n---- Contour Shape ---" << std::endl;

    unsigned long lgcv[2];
    csd->GetGlobalCurvature(lgcv[0], lgcv[1]);
    std::cout << "Global Curvature: " << (int)lgcv[0] << " " << (int)lgcv[1] << std::endl;


    unsigned int num = csd->GetNoOfPeaks();
    if (num > 0)
    {
        unsigned long lpcv[2];
        csd->GetPrototypeCurvature(lpcv[0], lpcv[1]);
        std::cout << "Prototype Curvature: " << (int)lpcv[0] << " " << (int)lpcv[1] << std::endl;
    }

    std::cout << "Highest Peak Y: " << (int)csd->GetHighestPeakY() << std::endl;

    if (num > 0)
    std::cout << "Peaks:" << std::endl;

    for (unsigned int i = 1; i < num; i++)
    {
        unsigned short xp, yp;

        csd->GetPeak(i, xp, yp);
        std::cout << "Peak X: " << xp << std::endl;
        std::cout << "Peak Y: " << yp << std::endl;
    }

	std::cout << "---------------" << std::endl;

    // release the descriptor
    delete csd;

}
