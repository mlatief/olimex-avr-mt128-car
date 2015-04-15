#include <util/delay.h>
#include <stdlib.h>

/* Implement more flexible but less acurate delay since maximum delay of provided _delay_ms is 262.14 ms / F_CPU in MHz*/
/* The provided delay is chunked into 100ms delays hopefully not hitting _delay_ms limit */
void delay_msec(int ms)
{
    int s = ms/100;
    int rem = ms - 100*s;
    for(;s>0;s--){
        _delay_ms(100);
    }
    
    _delay_ms(rem); 
}



int rrand(int max)
{
    int r = (rand() * 1.0 / RAND_MAX) * max;
    return r;
}
