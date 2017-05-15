// @uthor Asheesh Sharma
// Feel free to edit distribute 
// Example for C++ Interface to Gnuplot

// requirements:
// * gnuplot has to be installed (http://www.gnuplot.info/download.html)
// * for Windows: set Path-Variable for Gnuplot path (e.g. C:/program files/gnuplot/bin)
//             or set Gnuplot path with: Gnuplot::set_GNUPlotPath(const std::string &path);


#include <iostream>
#include "gnuplot_i.hpp" //Gnuplot class handles POSIX-Pipe-communication with Gnuplot
#include <string>
#include <fstream>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)
 #include <conio.h>   //for getch(), needed in wait_for_key()
 #include <windows.h> //for Sleep()
 void sleep(int i) { Sleep(i*1000); }
#endif


#define SLEEP_LGTH 2  // sleep time in seconds
#define NPOINTS    50 // length of array

void wait_for_key(); // Programm halts until keypress

using namespace std;
int coords[250][3]={{1,-33,33},{2,-99,-97},{3,-59,50},{4,0,14},{5,-17,-66},{6,-69,-19},{7,31,12},{8,5,-41},{9,-12,10},{10,-64,70},{11,-12,85},{12,-18,64},{13,-77,-16},{14,-53,88},{15,83,-24},{16,41,24},{17,17,21},{18,42,96},{19,-65,0},{20,-47,-26},{21,85,36},{22,-35,-54},{23,54,-21},{24,64,89},{25,55,-17},{26,17,-25},{27,-61,66},{28,-61,26},{29,17,-72},{30,79,38},{31,-62,-2},{32,-90,-68},{33,52,66},{34,-54,-50},{35,8,-84},{36,37,-90},{37,-83,49},{38,35,-1},{39,7,59},{40,12,48},{41,57,95},{42,92,28},{43,-3,97},{44,-7,52},{45,42,-15},{46,77,-43},{47,59,-49},{48,25,91},{49,69,-14},{50,-82,-19},{51,74,-70},{52,69,59},{53,29,33},{54,-97,9},{55,-58,9},{56,28,93},{57,7,73},{58,-28,73},{59,-76,55},{60,41,42},{61,92,40},{62,-84,-29},{63,-12,42},{64,51,-45},{65,-37,46},{66,-97,35},{67,14,89},{68,60,58},{69,-63,-75},{70,-18,34},{71,-46,-82},{72,-86,-79},{73,-43,-30},{74,-44,7},{75,-3,-20},{76,36,41},{77,-30,-94},{78,79,-62},{79,51,70},{80,-61,-26},{81,6,94},{82,-19,-62},{83,-20,51},{84,-81,37},{85,7,31},{86,52,12},{87,83,-91},{88,-7,-92},{89,82,-74},{90,-70,85},{91,-83,-30},{92,71,-61},{93,85,11},{94,66,-48},{95,78,-87},{96,9,-79},{97,-36,4},{98,66,39},{99,92,-79},{100,-46,-17},{101,-30,-63},{102,-42,63},{103,20,42},{104,15,98},{105,1,-17},{106,64,20},{107,-96,85},{108,93,-29},{109,-40,-84},{110,86,35},{111,91,36},{112,62,-8},{113,-24,4},{114,11,96},{115,-53,62},{116,-28,-71},{117,7,-4},{118,95,-9},{119,-3,17},{120,53,-90},{121,58,-19},{122,-83,84},{123,-1,49},{124,-4,-3},{125,-82,17},{126,-43,47},{127,6,-6},{128,70,99},{129,68,-29},{130,-94,-30},{131,-94,-20},{132,-21,77},{133,64,37},{134,-70,-19},{135,88,65},{136,2,29},{137,33,57},{138,-70,6},{139,-38,-56},{140,-80,-95},{141,-5,-39},{142,8,-22},{143,-61,-76},{144,76,-22},{145,49,-71},{146,-30,-68},{147,1,34},{148,77,79},{149,-58,-97},{150,82,64},{151,-80,55},{152,81,-86},{153,39,-49},{154,-67,72},{155,-25,-89},{156,-44,-95},{157,32,-68},{158,-17,49},{159,93,49},{160,99,81},{161,10,-49},{162,63,-41},{163,38,39},{164,-28,39},{165,-2,-47},{166,38,8},{167,-42,-6},{168,-67,88},{169,19,93},{170,40,27},{171,-61,56},{172,43,33},{173,-18,-39},{174,-69,-18},{175,75,19},{176,31,85},{177,25,58},{178,-16,36},{179,91,15},{180,60,-39},{181,49,-47},{182,42,33},{183,16,-81},{184,-78,53},{185,53,-80},{186,-46,-26},{187,-25,-54},{188,69,-46},{189,0,-78},{190,-84,74},{191,-16,16},{192,-63,-14},{193,51,-77},{194,-39,61},{195,5,97},{196,-55,39},{197,70,-14},{198,0,95},{199,-45,-24},{200,38,7},{201,50,-37},{202,59,71},{203,-73,-96},{204,-29,72},{205,-47,12},{206,-88,-61},{207,-88,36},{208,-46,-3},{209,26,-37},{210,-39,-67},{211,92,27},{212,-80,-31},{213,93,-50},{214,-20,-5},{215,-22,73},{216,-4,-7},{217,54,-48},{218,-70,39},{219,54,-82},{220,29,41},{221,-87,51},{222,-96,-36},{223,49,8},{224,-5,43},{225,-26,54},{226,-11,60},{227,40,61},{228,82,35},{229,-92,12},{230,-93,-86},{231,-66,63},{232,-72,-87},{233,-57,-84},{234,23,52},{235,-56,-62},{236,-19,59},{237,63,-14},{238,-13,38},{239,-19,87},{240,44,-84},{241,98,-17},{242,-16,62},{243,3,66},{244,26,22},{245,-38,-81},{246,70,80},{247,17,-35},{248,96,-83},{249,-77,44},{250,-14,80}};

//int example_rout[]={1,149,232,203,140,2,230,72,32,206,222,130,1,1,212,91,62,131,50,13,134,174,6,80,192,31,19,1,1,138,125,229,54,66,207,84,249,37,1,1,221,151,184,59,218,28,55,205,1,1,97,74,208,167,100,199,20,186,73,1,1,161,8,165,141,173,187,82,5,116,1,1,210,146,101,22,139,34,235,69,143,1,1,233,71,245,109,156,77,155,88,189,35,1,1,96,183,29,157,36,240,185,219,120,1,1,145,193,51,89,99,248,87,152,95,1,1,92,78,213,46,188,94,47,217,64,1,1,181,153,209,247,26,142,105,75,216,1,1,124,127,117,45,23,25,121,237,112,49,129,162,180,201,1,1,197,144,15,108,241,118,93,179,1,1,30,228,21,110,211,42,111,61,159,1,1,160,135,150,148,246,24,128,41,18,1,1,176,56,48,169,104,114,195,43,198,81,67,57,1,1,243,39,40,103,53,182,172,170,16,1,1,60,163,76,220,234,177,137,227,33,79,202,1,1,52,68,98,133,175,106,86,223,166,200,1,1,38,7,244,17,85,191,9,113,214,1,1,4,119,136,147,224,123,44,63,238,178,70,1,1,164,65,126,196,3,171,231,1,1,107,122,190,27,10,154,90,168,14,1,1,115,102,194,225,83,158,226,242,1,1,236,12,204,58,215,132,250,11,239,1};
int main(int argc, char* argv[])
{
    // if path-variable for gnuplot is not set, do it with:
    // Gnuplot::set_GNUPlotPath("C:/program files/gnuplot/bin/");

    // set a special standard terminal for showonscreen (normally not needed),
    //   e.g. Mac users who want to use x11 instead of aqua terminal:
    // Gnuplot::set_terminal_std("x11");
    //Read File
    cout << "*** Reading solution file ***" << endl << endl;
    ifstream file(argv[1]);
    cout<<"Args: Filename:"<<argv[1]<<" Size: "<<argv[2]<<endl;
    int example_rout[atoi(argv[2])];
    if(file.is_open())
    {
        for(int i = 0; i < atoi(argv[2]); ++i)
        {
            file >> example_rout[i];
        }
        cout<<"----------------Input buffer-----------------\n";
	for(int i=0;i<atoi(argv[2]);++i) {
  		printf("%.2d ", example_rout[i]);
	}
        cout<<"\n----------------Input buffer-----------------\n";

    }else{
	cout<<"The solution file is invalid"<<endl;
    }
    //
    // Using the GnuplotException class
    //
    try
    {
        Gnuplot g1("lines");

        g1.set_pointsize(2).set_style("points");

        std::vector<double> x, y, y2, dy, z;
        int route;
//cout<<"here "<<sizeof(example_rout)<<endl;
        for (int i = 0; i < sizeof(example_rout)/sizeof(example_rout[0]); i++)  // fill double arrays x, y, z
        {
	    //cout<<"here "<<i<<endl;
            route=example_rout[i];
	    //cout<<route<<"Cords"<<coords[route][1]<<","<<coords[route][2]<<endl;
            x.push_back(coords[route-1][1]);
            y.push_back(coords[route-1][2]);
        }
        g1.savetogif("output");
        g1.set_style("points").plot_xy(x,y,"",7," lc rgb 'red',\\\n'' index 0 u 1:2 notitle with lp pt 7 lc rgb 'blue'");
	cout<<"Generating";
        int A,B,C,D;
        std::stringstream vect;
	int linstile=1;
        for (int i=0; i<sizeof(example_rout)-1; i++){
	    cout<<".";
            vect.clear();
            A=x[i];
            B=y[i];
            //cout<<"-------------------- "<<i<<" --------------------"<<endl;
            C=x[i+1];
            D=y[i+1];
	    //cout<<A<<","<<B<<","<<C<<","<<D;
	    if(x[i]==-33 && y[i]==33){
		linstile++;
	    }
            if (i==0){
            vect << " lc rgb 'red',\\\n'' index 0 u 1:2 notitle with lp pt 7 lc rgb 'blue'\n set arrow from " << A << "," << B << " to " << C << "," << D <<"ls "<<linstile<< "\nset label at "<<C<<","<<D<<" "" point pointtype 7 pointsize 4 lc rgb 'green'";
            g1.set_style("points").plot_xy(x,y,"",7,vect.str());
            }
            if (i>0){
            vect << "\n set arrow from " << A << "," << B << " to " << C << "," << D <<"ls "<<linstile<< "\nset label at "<<C<<","<<D<<" "" point pointtype 7 pointsize 4 lc rgb 'green'";
            //"\nset label at "<<C<<D<<" "" point pointtype 7 pointsize 10"
            g1.set_style("points").plot_xy(x,y,"",7,vect.str());
            }

        }
        //wait_for_key();

    }
    catch (GnuplotException ge)
    {
        cout << ge.what() << endl;
    }


    //cout << endl << "*** end of gnuplot example" << endl;

    return 0;
}



void wait_for_key ()
{
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)  // every keypress registered, also arrow keys
    cout << endl << "Press any key to continue..." << endl;

    FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));
    _getch();
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
    cout << endl << "Press ENTER to continue..." << endl;

    std::cin.clear();
    std::cin.ignore(std::cin.rdbuf()->in_avail());
    std::cin.get();
#endif
    return;
}
