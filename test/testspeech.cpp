#include <emc/io.h>
#include <emc/rate.h>

int main(){

    emc::IO io;
    emc::Rate r(0.3);
    //io.sendOpendoorRequest();
    

    while(io.ok())
    {
        // Send a reference to the base controller (vx, vy, vtheta)
        io.speak("Hello my name is Pico");

        r.sleep();

        io.speak("I can turn left");

        io.sendBaseReference(0, 0, 0.3);

        r.sleep();

        io.speak("I can turn right");

        io.sendBaseReference(0, 0, -0.3);

        r.sleep();

        io.sendBaseReference(0, 0, 0.0);

        io.speak("I am a robot.....");

        r.sleep();
    }

    return 0;
}
