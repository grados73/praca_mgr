#ifndef FUNKCJE_WLASNE_H_
#define FUNKCJE_WLASNE_H_

//--------------------------------------------------
#include "stm32l4xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
//--------------------------------------------------

//================== ZMIENNE ====================================================#


//==================== FUNKCJE OBSLUGI WYSWIETLACZA =============================#
void oled_start(uint8_t * flaga);
void oled_ekr_glowny(uint16_t czas_s, uint16_t czas_min, uint16_t puls, uint16_t ilosc_satelit);
void oled_czas(uint16_t czas_s, uint16_t czas_min);
void oled_akt_puls(uint16_t puls);
void oled_akt_il_sat(uint16_t satelity);
void oled_header(uint16_t header);
void oled_koniec(uint16_t dystans, uint16_t minuty, uint16_t sekundy, uint16_t sr_tetno);
void oled_end( uint16_t flaga_stoper);
//=================================================================================#

//==================== FUNKCJE LICZACE STATYSTYKI =================================#
uint16_t licz_sr_puls(uint16_t puls, uint16_t * wsk_max_puls);
uint16_t licz_puls(uint16_t cnt_pulse);
uint16_t licz_odleglosc(uint16_t predkosc, uint16_t * wsk_max_predkosc);
//=================================================================================#
void konwert_liczb(uint16_t liczba, uint8_t * liczba_jed, uint8_t * liczba_dzies, uint8_t *liczba_setek);
//--------------------------------------------------


#endif /* FUNKCJE_WLASNE_H_ */
//



