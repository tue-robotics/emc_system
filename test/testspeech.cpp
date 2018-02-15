#include <emc/io.h>

int main(){

    emc::IO io;
    #io.sendOpendoorRequest();
    io.speak("Hello my name is Pico");
    return 0;
}
