//
// Created by zhi on 9/26/17.
//

#include <math.h>
#include <iostream>
#include <vector>
#include "spline.h"
#include <mgl2/mgl.h>

using namespace std;

int main() {
    vector<double> X(5), Y(5);
    X[0]=0.1; X[1]=0.4; X[2]=1.2; X[3]=1.8; X[4]=2.0;
    Y[0]=0.1; Y[1]=0.7; Y[2]=0.6; Y[3]=1.1; Y[4]=0.9;

    tk::spline s;
    s.set_points(X,Y);    // currently it is required that X is already sorted

////    gr->WriteFrame("test.png");
    mglData a(50), b(50);
    for(int i=0;i<50;i++) {
        a[i] = 2.2*(double)i/50.0;
        b[i] = s(a[i]);
    }
    mglData A(5), B(5);
    for(int i=0;i<5;i++) {
        A[i] = X[i];
        B[i] = Y[i];
    }

    mglGraph *gr = new mglGraph;
    //gr->Plot((mglData)X,(mglData)Y,"ro ");
    gr->SetRanges(0,2.5,0,2.5);
    gr->Plot(A,B,"ro ");
    gr->Plot(a,b,"k. ");
    gr->Title("interpolation");

    gr->Axis();
    gr->Grid();
    gr->Box();
    //gr->SetOrigin(0.5,0.5)
    gr->WriteFrame("test.png");


//    mglData in(9), arg(99), e, s;
//    gr->Fill(in,"x^3/1.1"); gr->Fill(arg,"4*x+4");
//    gr->Plot(in,"ko ");     gr->Box();
//    e = in.Evaluate(arg,false); gr->Plot(e,"b.","legend 'Evaluate'");
//    s = in.SubData(arg);    gr->Plot(s,"r.","legend 'SubData'");
//    gr->Legend(2);
//    //mgls_prepare1d(&y);
//    gr->SetOrigin(0,0,0);
////    gr->SubPlot(2,2,0,"");

//    gr->Box();
///    gr->Dots(y,y,y);
//    gr->WriteFrame("test.png");
//
//    gr->SubPlot(2,2,2,"");  gr->Title("'!' style; 'rgb' palette");
//    gr->Box();  gr->Plot(y,"o!rgb");
//
//    gr->SubPlot(2,2,3,"");  gr->Title("just markers");
//    gr->Box();  gr->Plot(y," +");
//
//    gr->SubPlot(2,2,1); gr->Title("3d variant");
//    gr->Rotate(50,60);  gr->Box();
//    mglData yc(30), xc(30), z(30);  z.Modify("2*x-1");
//    yc.Modify("sin(pi*(2*x-1))"); xc.Modify("cos(pi*2*x-pi)");
//    gr->Plot(xc,yc,z,"rs");
//    gr->SubPlot(2,2,0); gr->Title("Axis origin, Grid"); gr->SetOrigin(0,0);
//    gr->Axis(); gr->Grid(); gr->FPlot("x^3");
//
//    gr->SubPlot(2,2,1); gr->Title("2 axis");
//    gr->SetRanges(-1,1,-1,1); gr->SetOrigin(-1,-1,-1);  // first axis
//    gr->Axis(); gr->Label('y',"axis 1",0);  gr->FPlot("sin(pi*x)");
//    gr->SetRanges(0,1,0,1);   gr->SetOrigin(1,1,1);   // second axis
//    gr->Axis(); gr->Label('y',"axis 2",0);  gr->FPlot("cos(pi*x)");
//
//    gr->SubPlot(2,2,3); gr->Title("More axis");
//    gr->SetOrigin(NAN,NAN); gr->SetRange('x',-1,1);
//    gr->Axis(); gr->Label('x',"x",0); gr->Label('y',"y_1",0);
//    gr->FPlot("x^2","k");
//    gr->SetRanges(-1,1,-1,1); gr->SetOrigin(-1.3,-1); // second axis
//    gr->Axis("y","r");  gr->Label('y',"#r{y_2}",0.2);
//    gr->FPlot("x^3","r");
//
//    gr->SubPlot(2,2,2); gr->Title("4 segments, inverted axis");
//    gr->SetOrigin(0,0);
//    gr->InPlot(0.5,1,0.5,1);  gr->SetRanges(0,10,0,2);  gr->Axis();
//    gr->FPlot("sqrt(x/2)");   gr->Label('x',"W",1); gr->Label('y',"U",1);
//    gr->InPlot(0,0.5,0.5,1);  gr->SetRanges(1,0,0,2); gr->Axis("x");
//    gr->FPlot("sqrt(x)+x^3"); gr->Label('x',"\\tau",-1);
//    gr->InPlot(0.5,1,0,0.5);  gr->SetRanges(0,10,4,0);  gr->Axis("y");
//    gr->FPlot("x/4"); gr->Label('y',"L",-1);
//    gr->InPlot(0,0.5,0,0.5);  gr->SetRanges(1,0,4,0); gr->FPlot("4*x^2");

    gr->WriteFrame("test.png");

    return EXIT_SUCCESS;
}