#include <16f877.h>
#device ADC=10 // 0 ile 1023
#use delay(clock=4000000)
#fuses XT, NOWDT, NOPUT, NOLVP, NOCPD, NOPROTECT, NODEBUG, NOBROWNOUT, NOWRT

#use fast_io(a)  //ADC iþlem portu
#use fast_io(b)  //LCD iþlem portu
#use fast_io(d)  // Butonlarim

#define yukari pin_d0
#define asagi  pin_d1
#define use_portb_lcd TRUE
#define led pin_c0

#include <lcd.c>

unsigned long int deger; // Analog digital converter ham deðeri (0 ile 1023 arasinda bir sayi)
float gerilim, sicaklik;
signed int8 ayar=0; // 8bitlik veri genisligi vardir. -128 ve +127
int1 ayar_yukari=1;
int1 eski_yukari=1;
int1 ayar_asagi=1;
int1 eski_asagi=1;

// Timer0 kesmesi için kesme sayacý
int timer0_isr_counter = 0;

// Timer0 kesmesi
#int_timer0
void timer0_isr() {
    // Timer0 kesmesini yeniden baþlat
    set_timer0(131); // 1 / (4 MHz / 16) = her clock 4 mikrosaniye bunu da 131*4=524 mikrosaniye yani yarim milisaniyeye denk gelerek delayms(0.5) e eþit oluyor
    timer0_isr_counter++; // Kesme sayacýný artýrarak kesmelerin ne sýklýkla gerçekleþtigini anliyor
}

void main()
{
    setup_psp(PSP_DISABLED);
    setup_timer_1(T1_DISABLED);
    setup_CCP1(CCP_OFF);
    setup_CCP2(CCP_OFF);

    // Timer0 konfigürasyonu
    setup_timer_0(T0_INTERNAL | T0_DIV_16); // Fosc/4/16
    set_timer0(131); 
    enable_interrupts(INT_TIMER0);
    enable_interrupts(GLOBAL);

    set_tris_a(0x20); // Port A konfigürasyonu (RA5 giriþ)
    set_tris_b(0x00); // Port B konfigürasyonu (LCD çýkýþ)
    set_tris_d(0xFF); // Port D konfigürasyonu (butonlar giriþ)
    set_tris_c(0x00); // Port C konfigürasyonu (LED çýkýþ)

    lcd_init();
    setup_adc(adc_clock_div_32);  //1.6 kHz ile 500 kHz arasýnda bir saat frekansý önerilir
    setup_adc_ports(ALL_ANALOG); // Tüm analog pinlerin AN0'dan AN7'ye kadar analog giriþ olarak ayarlanmasýný saðlar.Buraya sadece AN4 de yazýlabilir.

    set_adc_channel(4); //ADC'nin AN4 RA5 pininde bulunan analog giriþ kanalýndan veri okuyacaðýný belirtir.
    delay_us(20); //Kaldýrlamaz...

    printf(lcd_putc,"\fSicaklik=");

    while(true)
    {
        
        ayar_yukari = input(pin_d0); //basýlý olup olmadýðýný kontrol eder
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

        // Timer0 kesmesiyle 1 ms'de bir ADC okumasý 
        if (timer0_isr_counter >= 20) // 20 kez kesme olduðunda 20*0.5ms=1ms
        {
        //Sicaklýk sensörü 10mV da 1 derece artar.
            timer0_isr_counter = 0; // Kesme sayacýný sýfýrla
            deger = read_adc(); //AN4 kanalýndaki analog sinyali dijital deðere çevirir ve bu deðeri deger deðiþkenine atar. Bu deðer 0 ile 1023 arasýnda bir sayýdýr.
            gerilim = deger * 4.88760;  // 5V/1023 = 0.0488760V = 4.88760mV      ADCnin 1023 deðeri 5V a karþýlýk gelir.her bir dijital adým 5V/1023 = 0.00488760V veya 4.88760mV.
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

