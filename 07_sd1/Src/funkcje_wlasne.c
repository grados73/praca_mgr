#include "funkcje_wlasne.h"
#include "SSD1331.h"
//================ ZMIENNE =============================
uint32_t suma_pulsu = 0;
uint16_t licznik_suma_pulsu = 0;
uint16_t sredni_puls_za_trip = 0;
uint16_t odlegloscui = 0;
uint16_t odlegloscdo = 0;
uint16_t temp_puls = 0;
uint16_t temp_predkosc = 0;
uint16_t flagakk = 50;// FLAGA OD CZYSZCZENIE EKRANY PO EKRANIE KONCOWYM;

#define CZAS_EKRANU_KONCOWEGO 5000




//================ FUNKCJE ==============================

//========== LICZENIE AKTUALNEGO PULSU===================
uint16_t licz_puls(uint16_t licznik) // licznik -inkrementowany z f=100Hz (t=10ms) pomiedzy przerwaniami 
{
	float zmienna_pom_counter = (float) licznik;          //powolanie zmiennej typu float, aby operacja byla 
																							            //wykonywana na zmiennej zmiennoprzecinkowej, co
																							 //zapewnia lepsza precyzje obliczen, pózniej jawne rzutowanie
  zmienna_pom_counter = ((100/zmienna_pom_counter)*60);   //obliczanie pulsu na podstawie czasu pomiedzy  
																												  //uderzeniami serca
	uint16_t bierzacy_puls = (uint16_t) zmienna_pom_counter; // ponowne jawne rzutowanie w celu zwrócenia 
																													 //wartosci calowitej
	return bierzacy_puls; 																	 //zwracana wartosc
}

//===================== LICZENIE SREDNIEGO PULSU ZA TRENING ================================== 
uint16_t licz_sr_puls(uint16_t puls,  uint16_t * wsk_max_puls)
{
	
	temp_puls = *wsk_max_puls;
	
			if(puls == 777) // RESET
			{
				suma_pulsu = 0;
				licznik_suma_pulsu = 0;
			}
			else
			{
				suma_pulsu += puls;
				licznik_suma_pulsu++;
				sredni_puls_za_trip = (uint16_t) (suma_pulsu/licznik_suma_pulsu);
			}
			
			if( (puls > temp_puls) && (puls < 250))  *wsk_max_puls = puls; // LICZENIE MAX PULSU
			
			
	return sredni_puls_za_trip;
}
// %%%%%%%%%% zmienic zwracana zmienna na typu struct, i zwracac sredni puls i max, przynajmniej z 15s.

//============================= GPS - ODLEGLOSC =====================================#
uint16_t licz_odleglosc(uint16_t predkosc, uint16_t * wsk_max_predkosc)
{
	temp_predkosc = *wsk_max_predkosc;
	odlegloscdo = (uint16_t) predkosc * 0.278; // PRZELICZENIE "predkosc" w [km/h] NA  "odlegloscdo" [m/s]
	odlegloscdo *= 5;    // MNOZYMY x5 BO PRZERWANIE JEST CO 5s, WIEC JEST TO PREDKOSC PRZEZ OSTATNIE 5s
	odlegloscui = (uint16_t) odlegloscdo; // JAWNE RZUTOWANIE, ODLEGLOSC PRZEBYTA PRZEZ OST 5s W METRACH!
	
	if(predkosc > temp_predkosc) *wsk_max_predkosc = predkosc; // LICZENIE MAX PREDKOSCI
	
	return odlegloscui;
}
//%%%%%%%%%%%%% zmienic zwracana zmienna na typu strukturalnego i zwracac przebyta droge, srednia predkosc, max i min predkosc.

//============================= GPS - ODLEGLOSC =====================================#




//=================!!!! -=OLED=- !!!==================================

//================= EKRAN STARTOWY =================================
void oled_start(uint8_t * flaga)
{
		*flaga = 0;
		ssd1331_clear_screen(BLACK);
		ssd1331_display_string(25, 30, "GRADOS", FONT_1608, YELLOW);
	// PIERWSZY DELAY!!!
		HAL_Delay(200);  // !!
			ssd1331_clear_screen(BLACK);
		ssd1331_display_string(22, 00, "WITAJ NA", FONT_1206, GOLDEN);
		ssd1331_display_string(22, 12, "TRENINGU", FONT_1206, GOLDEN);
		ssd1331_display_string(0, 20, "---------------", FONT_1206, GOLDEN);
		ssd1331_display_string(15, 32, "POCZEKAJ NA", FONT_1206, GREEN);
		ssd1331_display_string(33, 44, "GPS", FONT_1608, RED);
	// DRUGI DELAY!!!
		HAL_Delay(500); // !!
				*flaga = 1;// SYGNAL O ZAKONCZENIU EKRANU STARTOWEGO

//ssd1331_draw_line
}

//======================== EKRAN GLOWNY =========================================
void oled_ekr_glowny(uint16_t czas_s, uint16_t czas_min, uint16_t puls, uint16_t ilosc_satelit)
{
		ssd1331_clear_screen(BLACK);
	//================ START + KOLKO ZIELONE =============================
		
		ssd1331_display_string(0,0, "START?", FONT_1608, GOLDEN);
		ssd1331_draw_circle(60,9, 1, GREEN);
		ssd1331_draw_circle(60,8, 7, GREEN);
		ssd1331_draw_circle(60,8, 6, GREEN);
		ssd1331_draw_circle(60,8, 5, GREEN);
		ssd1331_fill_rect(55,3,11,11,GREEN);
	
	
	//=================== CZAS 00:00 ==========================
		ssd1331_display_num(25,15, czas_min, 2,FONT_1608, WHITE);
				if(czas_min < 10) ssd1331_display_string(25,15, "0", FONT_1608, WHITE);
		ssd1331_display_string(40,13,":",FONT_1608,WHITE);
		ssd1331_display_num(46,15, czas_s, 2 ,FONT_1608, WHITE);
				if(czas_min < 10) ssd1331_display_string(46,15,"0",FONT_1608, WHITE);
	
	//===================== SATELITY ============================
			ssd1331_display_string(0 , 33, "Ilosc satelit: ", FONT_1206, GREEN);
			ssd1331_display_num(85, 34, ilosc_satelit, 2, FONT_1206, RED);
			
	
	//===================== PULS ================================== 
	
			ssd1331_display_string(0 , 45, "Akt. puls: ", FONT_1206, GREEN);
			ssd1331_display_num(85, 46, puls, 2, FONT_1206, RED);
			HAL_Delay(100);
			
	
		
}
//===================== STOPER ================================== 
void oled_czas(uint16_t czas_s, uint16_t czas_min)
{
	if((czas_min == 0) && (czas_s == 0) && (flagakk == 0)) ssd1331_clear_screen(BLACK);
	ssd1331_fill_rect(26,16,40,18,BLACK);
	ssd1331_display_num(25,15, czas_min, 2,FONT_1608, WHITE);
				if(czas_min < 10) ssd1331_display_string(25,15, "0", FONT_1608, WHITE);
		ssd1331_display_string(40,13,":",FONT_1608,WHITE);
		ssd1331_display_num(46,15, czas_s, 2 ,FONT_1608, WHITE);
				if(czas_s < 10) ssd1331_display_string(46,15,"0",FONT_1608, WHITE);
	flagakk = 10;
}

//===================== ZMIANA HEADERA START/PAUSE ================================== 
void oled_header(uint16_t header)
{
	if(header == 1) // START + KOLKO
	{
		ssd1331_fill_rect(0,0,96,17,BLACK); // WYCZYSZCZENIE POLA
		ssd1331_display_string(0,0, "START?", FONT_1608, GOLDEN);
		ssd1331_display_string(72,0, "/", FONT_1608, RED);
		ssd1331_display_string(86,3, "X", FONT_1206, BLUE);
		ssd1331_draw_circle(88,9, 7, BLUE);
		
		ssd1331_draw_circle(60,9, 1, GREEN);
		ssd1331_draw_circle(60,8, 7, GREEN);
		ssd1331_draw_circle(60,8, 6, GREEN);
		ssd1331_draw_circle(60,8, 5, GREEN);
		ssd1331_fill_rect(55,3,11,11,GREEN);
	}
	else if(header == 2) // PAUSE
	{
		ssd1331_fill_rect(0,0,96,17,BLACK); // WYCZYSZCZENIE POLA
		ssd1331_display_string(0,0, "PAUSE?", FONT_1608, GOLDEN);
		ssd1331_draw_circle(60,9, 1, BLUE);
		ssd1331_draw_circle(60,8, 7, BLUE);
		ssd1331_draw_circle(60,8, 6, BLUE);
		ssd1331_draw_circle(60,8, 5, BLUE);
		ssd1331_fill_rect(55,3,11,11,BLUE);
	}
	else if(header == 3)
	{
		ssd1331_fill_rect(0,0,96,17,BLACK); // WYCZYSZCZENIE POLA
		ssd1331_display_string(0,0, "KONIEC?", FONT_1608, GOLDEN);
		ssd1331_display_string(65,3, "X", FONT_1206, BLUE);
		ssd1331_draw_circle(67,8, 7, BLUE);
	}
}

//===================== WYSWIETLANIE PULSU ==================================#
void oled_akt_puls(uint16_t puls)
{
	
			ssd1331_fill_rect(80,46,20,11,BLACK);
			ssd1331_display_string(0 , 45, "Akt. puls: ", FONT_1206, GREEN);
			ssd1331_display_num(85, 46, puls, 2, FONT_1206, RED);
	
}

//===================== WYSWIETLANIE ILOSCI SATELIT ==================================#
void oled_akt_il_sat(uint16_t satelity)
{
			ssd1331_fill_rect(83,35,20,11,BLACK);
			ssd1331_display_string(0 , 33, "Ilosc satelit: ", FONT_1206, GREEN);
			ssd1331_display_num(85, 34, satelity, 2, FONT_1206, RED);
}

//===================== WYSWIETLANIE EKRANU KONCOWEGO ==================================#
void oled_koniec(uint16_t dystans, uint16_t minuty, uint16_t sekundy, uint16_t sr_tetno)
{
			ssd1331_clear_screen(BLACK);
			ssd1331_display_string(18,0, "GRATULACJE!", FONT_1206, GOLDEN);
			ssd1331_display_string(0,12, "DYSTANS:", FONT_1206, RED);
			ssd1331_display_num(74,12, dystans, 2,FONT_1206, WHITE);
	
			ssd1331_display_string(0,24, "CZAS:", FONT_1206, RED);
			ssd1331_display_num(60,24, minuty, 2,FONT_1206, WHITE);
						if(minuty < 10) ssd1331_display_string(60,24, "0", FONT_1206, WHITE);
			ssd1331_display_string(74,23,":",FONT_1206,WHITE);
			ssd1331_display_num(80,24, sekundy, 2 ,FONT_1206, WHITE);
					if(sekundy < 10) ssd1331_display_string(80,24,"0",FONT_1206, WHITE);
	
			ssd1331_display_string(0,36, "SR. TETNO:", FONT_1206, RED);
			ssd1331_display_num(74,36, sr_tetno, 2,FONT_1206, WHITE);
	
			ssd1331_display_string(0,48, "Zapisac? > / ||", FONT_1206, GREEN);
	flagakk = 0;
			
}

void oled_end( uint16_t flaga_stoper)
{
	if (flaga_stoper == 1)  // " > "
	{
		ssd1331_clear_screen(BLACK);
		ssd1331_display_string(0,0, "Zapisano trening",FONT_1206, WHITE);
		ssd1331_display_string(0,12, "Nowy trening =>",FONT_1206, WHITE);
		ssd1331_display_string(0,24, "wcisnij reset,",FONT_1206, WHITE);
		ssd1331_display_string(0,36, "lub wylacz",FONT_1206, WHITE);
		ssd1331_display_string(0,48, "zasilanie!",FONT_1206, WHITE);
	}
	else if (flaga_stoper == 0)  // " || "
	{
		ssd1331_clear_screen(BLACK);  
		ssd1331_display_string(0,0,  "Odrzuc. trening,",FONT_1206, WHITE);
		ssd1331_display_string(0,12, "Nowy trening =>",FONT_1206, WHITE);
		ssd1331_display_string(0,24, "wcisnij reset,",FONT_1206, WHITE);
		ssd1331_display_string(0,36, "lub wylacz ",FONT_1206, WHITE);
		ssd1331_display_string(0,48, "zasilanie!",FONT_1206, WHITE);

	}
	else if( flaga_stoper == 100)
	{
		ssd1331_clear_screen(BLACK);  
		ssd1331_display_string(0,0,  "Zapisac? \" > \"",FONT_1206, WHITE);
		ssd1331_display_string(0,30, "Odrzucic? \" || \" ", FONT_1206, WHITE);
	}
}


//================================ KONWERSJA NA ASCII DO SD ==========================================================
void konwert_liczb(uint16_t liczba, uint8_t * liczba_jed, uint8_t * liczba_dzies, uint8_t *liczba_setek)
{
	*liczba_jed = ((liczba%10)+48); // Jednosci
	if(liczba < 10) *liczba_dzies = 48; //0
	if(((liczba%100)>9)  && ((liczba%100) < 20)) *liczba_dzies = 49; //1
	if(((liczba%100)>19) && ((liczba%100) < 30)) *liczba_dzies = 50; //2
	if(((liczba%100)>29) && ((liczba%100) < 40)) *liczba_dzies = 51; //3
	if(((liczba%100)>39) && ((liczba%100) < 50)) *liczba_dzies = 52; //4
	if(((liczba%100)>49) && ((liczba%100) < 60)) *liczba_dzies = 53; //5
	if(((liczba%100)>59) && ((liczba%100) < 70)) *liczba_dzies = 54; //6
	if(((liczba%100)>69) && ((liczba%100) < 80)) *liczba_dzies = 55; //7
	if(((liczba%100)>79) && ((liczba%100) < 90)) *liczba_dzies = 56; //8
	if(((liczba%100)>89) && ((liczba%100) < 100))*liczba_dzies = 57; //9
	
	if(liczba > 99) //setki
	{
		if(((liczba%1000)>99)  && ((liczba%1000) < 200)) *liczba_setek = 49; //1
	if(((liczba%1000)>199) && ((liczba%1000) < 300)) *liczba_setek = 50; //2
	if(((liczba%1000)>299) && ((liczba%1000) < 400)) *liczba_setek = 51; //3
	if(((liczba%1000)>399) && ((liczba%1000) < 500)) *liczba_setek = 52; //4
	if(((liczba%1000)>499) && ((liczba%1000) < 600)) *liczba_setek = 53; //5
	if(((liczba%1000)>599) && ((liczba%1000) < 700)) *liczba_setek = 54; //6
	if(((liczba%1000)>699) && ((liczba%1000) < 800)) *liczba_setek = 55; //7
	if(((liczba%1000)>799) && ((liczba%1000) < 900)) *liczba_setek = 56; //8
	if(((liczba%1000)>899) && ((liczba%1000) < 1000))*liczba_setek = 57; //9
	}
	else liczba_setek = 0;
}

//================================ KONWERSJA NA ASCII DO SD ==========================================================

