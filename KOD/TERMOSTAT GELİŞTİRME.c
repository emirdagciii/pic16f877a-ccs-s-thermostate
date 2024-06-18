#include <16f877.h>
#device ADC=10 // 0 ile 1023
#use delay(clock=4000000)
#fuses XT, NOWDT, NOPUT, NOLVP, NOCPD, NOPROTECT, NODEBUG, NOBROWNOUT, NOWRT

#use fast_io(a)  //ADC i�lem portu
#use fast_io(b)  //LCD i�lem portu
#use fast_io(d)  // Butonlarim

#define yukari pin_d0
#define asagi  pin_d1
#define use_portb_lcd TRUE
#define led pin_c0

#include <lcd.c>

unsigned long int deger; // Analog digital converter ham de�eri (0 ile 1023 arasinda bir sayi)
float gerilim, sicaklik;
signed int8 ayar=0; // 8bitlik veri genisligi vardir. -128 ve +127
int1 ayar_yukari=1;
int1 eski_yukari=1;
int1 ayar_asagi=1;
int1 eski_asagi=1;

// Timer0 kesmesi i�in kesme sayac�
int timer0_isr_counter = 0;

// Timer0 kesmesi
#int_timer0
void timer0_isr() {
    // Timer0 kesmesini yeniden ba�lat
    set_timer0(131); // 1 / (4 MHz / 16) = her clock 4 mikrosaniye bunu da 131*4=524 mikrosaniye yani yarim milisaniyeye denk gelerek delayms(0.5) e e�it oluyor
    timer0_isr_counter++; // Kesme sayac�n� art�rarak kesmelerin ne s�kl�kla ger�ekle�tigini anliyor
}

void main()
{
    setup_psp(PSP_DISABLED);
    setup_timer_1(T1_DISABLED);
    setup_CCP1(CCP_OFF);
    setup_CCP2(CCP_OFF);

    // Timer0 konfig�rasyonu
    setup_timer_0(T0_INTERNAL | T0_DIV_16); // Fosc/4/16
    set_timer0(131); 
    enable_interrupts(INT_TIMER0);
    enable_interrupts(GLOBAL);

    set_tris_a(0x20); // Port A konfig�rasyonu (RA5 giri�)
    set_tris_b(0x00); // Port B konfig�rasyonu (LCD ��k��)
    set_tris_d(0xFF); // Port D konfig�rasyonu (butonlar giri�)
    set_tris_c(0x00); // Port C konfig�rasyonu (LED ��k��)

    lcd_init();
    setup_adc(adc_clock_div_32);  //1.6 kHz ile 500 kHz aras�nda bir saat frekans� �nerilir
    setup_adc_ports(ALL_ANALOG); // T�m analog pinlerin AN0'dan AN7'ye kadar analog giri� olarak ayarlanmas�n� sa�lar.Buraya sadece AN4 de yaz�labilir.

    set_adc_channel(4); //ADC'nin AN4 RA5 pininde bulunan analog giri� kanal�ndan veri okuyaca��n� belirtir.
    delay_us(20); //Kald�rlamaz...

    printf(lcd_putc,"\fSicaklik=");

    while(true)
    {
        
        ayar_yukari = input(pin_d0); //bas�l� olup olmad���n� kontrol eder
        if(ayar_yukari != eski_yukari)
        {
            eski_yukari = ayar_yukari;
            if(ayar_yukari == 0)
            {
                ayar++;
                lcd_gotoxy(8,2);
                printf(lcd_putc,"AYAR=%d C ",ayar);
            }
        }

        ayar_asagi = input(pin_d1);
        if(ayar_asagi != eski_asagi)
        {
            eski_asagi = ayar_asagi;
            if(ayar_asagi == 0)
            { 
                ayar--;
                if(ayar<0)
                {ayar=0;}
                lcd_gotoxy(8,2);
                printf(lcd_putc,"AYAR=%d C ",ayar);
            }
        }

        // Timer0 kesmesiyle 1 ms'de bir ADC okumas� 
        if (timer0_isr_counter >= 20) // 20 kez kesme oldu�unda 20*0.5ms=1ms
        {
        //Sicakl�k sens�r� 10mV da 1 derece artar.
            timer0_isr_counter = 0; // Kesme sayac�n� s�f�rla
            deger = read_adc(); //AN4 kanal�ndaki analog sinyali dijital de�ere �evirir ve bu de�eri deger de�i�kenine atar. Bu de�er 0 ile 1023 aras�nda bir say�d�r.
            gerilim = deger * 4.88760;  // 5V/1023 = 0.0488760V = 4.88760mV      ADCnin 1023 de�eri 5V a kar��l�k gelir.her bir dijital ad�m 5V/1023 = 0.00488760V veya 4.88760mV.
            sicaklik = (gerilim / 10)+2; //0mV 2 derece 10mV 3 derece 
            int sicaklik_int = (int)sicaklik;

            lcd_gotoxy(10,1);
            printf(lcd_putc,"%d C", sicaklik_int);

            if (sicaklik_int <= ayar) 
            {
                output_high(led); 
            } 
            else 
            {
                output_low(led); 
            }
        }
    }
}

