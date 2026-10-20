#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "LEDSEG.h"
static uint16_t segments;
void RCC_APB2PeriphClockCmd(int c,int e) { (void)c; (void)e; }
void GPIO_Init(void *p,GPIO_InitTypeDef *c) { (void)p; (void)c; }
void GPIO_SetBits(void *p,uint16_t b) { (void)p; segments|=b; }
void GPIO_WriteBit(void *p,uint16_t b,BitAction v) {
    if(p!=GPIOA) exit(2);
    if(v==Bit_SET) segments|=b; else segments&=(uint16_t)(0xffu^b);
}
int main(void) {
    int i, failures=0;
    static const unsigned char expected[16]={
        0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90,
        0x88,0x83,0xc6,0xa1,0x86,0x8e
    };
    for(i=0;i<16;++i) {
        Digital_Display((uint8_t)i);
        if(segments!=expected[i]) ++failures;
        printf("%s: common-anode hex digit %X\n",segments==expected[i]?"PASS":"FAIL",i);
    }
    Digital_Display(16); if(segments!=0xffu) ++failures;
    printf("%s: out-of-range input blanks the display\n",segments==0xffu?"PASS":"FAIL");
    printf("17 checks, %d failures\n",failures);
    return failures?1:0;
}
