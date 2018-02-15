#include <emc/io.h>

int main(){

    emc::IO io;
    io.sendOpendoorRequest();
    //io.speak("hallo");
    return 0;
}
