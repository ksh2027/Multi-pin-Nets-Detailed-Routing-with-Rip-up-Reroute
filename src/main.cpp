// Designed by ksh2027 
#include "io.hpp"
#include "detailed_routing.hpp"

int main(int argc, char const *argv[]){
    cout<< "---code start---" << endl;
    clock_t a,b; 
    a=clock();
    Small_routing sm;
    io::readLayout(&sm, argv[1]);
    //build grid
    sm.initialize();
    //2-pin decomposition
    //sm.pin_dcp();
    //T2T maze routing
    sm.T2T_main();
    //post process
    sm.post_proc();
    io::writeLayout(&sm, argv[2]);
    b=clock();
    cout<<"total_cost "<<sm.cost_total<<endl;
    cout<<"total_via "<<sm.via_total<<endl;
    cout<<"total_WL "<<sm.WL_total<<endl;
    cout<<"total_reroute "<<sm.rr_total<<endl;
    sm.time_total=double(b-a)/CLOCKS_PER_SEC;
	cout<<"Time "<<sm.time_total<<endl;
    io::writeOutput(&sm, argv[3]);

	cout<<endl<<"---code end---"<<endl;
    return 0;
}