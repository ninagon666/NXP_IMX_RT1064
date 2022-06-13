#ifndef _openartart_mini_h
#define _openartart_mini_h

#include "headfile.h"


typedef struct openart_param_t {
    int8_t rx_array[5];     //�洢���ڽ�������
    int8_t openart_result;  //���

    enum {
        NONE, ANIMAL, FRUIT, TRANSPORTATION 
    } fa_type;              // ���Ʒ�����


} openart_param_t;


extern openart_param_t openart;

void openart_send(void);

void openart_mini(void);

void check_openart(void);

#endif

