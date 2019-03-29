//
// Created by Anıl ARAS on 2019-03-29.
//

#include "softPwm.c"
#include "softPwm.h"
#include <signal.h>


void intHandler(int dummy) {
    softPwmStop   (127) ;
    exit(1);
}

int main(int argc, char **argv){
    signal(SIGINT, intHandler);

    softPwmCreate(127, 0, 200);
        softPwmWrite(127, atoi(argv[1]));

    while(1);

}