#include <emc/io.h>
#include <emc/rate.h>

int main(){

    emc::IO io;
    emc::Rate r(10);
    //io.sendOpendoorRequest();
    

    while(io.ok())
    {
        // Send a reference to the base controller (vx, vy, vtheta)
        io.sendBaseReference(0, 0, 0.3);

        io.speak("Hello my name is Pico");


        r.sleep();
    }

    return 0;
}
