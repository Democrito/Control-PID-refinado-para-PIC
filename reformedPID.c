// Este programa est� basado en el Control PID de Brett Beauregard: http://brettbeauregard.com/blog/wp-content/uploads/2012/07/Gu%C3%ADa-de-uso-PID-para-Arduino.pdf

#include <18F4550.h>                                                        // Hardware compatible para cualquier microcontrolador de la familia 18Fxx5x.
#DEVICE HIGH_INTS=TRUE                                                      // Activa la prioridad de interrupciones externas.
#fuses HSPLL, NOWDT, NOPROTECT, NOLVP, NODEBUG, USBDIV, PLL5, CPUDIV1, VREGEN// Cristal de 20MHz con PLL interno a 48MHz.
#include <usb_bootloader.h>
#include <math.h>

#use delay(clock=48000000)                                                  // El PLL interno est� configurado para hacer funcionar el micro a 48MHz.
#use TIMER(TIMER=1, TICK=1ms, BITS=16, NOISR)                               // Activa un "Timer" en el que incrementa un contador interno (de 16 bits) cada 0.6ms.

#byte porta = 0xF80                                                         // Direcci�n del puerto A para la familia 18Fxx5x.
#byte portb = 0xF81                                                         // Direcci�n del puerto B para la familia 18Fxx5x.
#byte portc = 0xF82                                                         // Direcci�n del puerto C para la familia 18Fxx5x.
#byte portd = 0xF83                                                         // Direcci�n del puerto D para la familia 18Fxx5x (S�lo 40/44 pines).
#byte porte = 0xF84                                                         // Direcci�n del puerto E para la familia 18Fxx5x.

#use fast_io(a)
#use fast_io(b)
#use fast_io(c)
#use fast_io(d)
#use fast_io(e)

#define USB_HID_DEVICE     FALSE                                            // HID desactivado.
#define USB_EP1_TX_ENABLE  USB_ENABLE_BULK                                  // turn on EP1(EndPoint1) for IN bulk/interrupt transfers.
#define USB_EP1_RX_ENABLE  USB_ENABLE_BULK                                  // turn on EP1(EndPoint1) for OUT bulk/interrupt transfers.
#define USB_EP1_TX_SIZE    4                                                // Tama�o en bytes del buffer de salida.
#define USB_EP1_RX_SIZE    4                                                // Tama�o en bytes del buffer de entrada.

#include <pic18_usb.h>                                                      // Microchip PIC18Fxx5x Hardware layer for CCS's PIC USB driver.
#include "usb_desc_scope.h"                                                 // Enumerador modificado.
#include <usb.c>                                                            // handles usb setup tokens and get descriptor reports.

// Variables globales para el c�lculo del controlador PID.
int32  lastTime=0, SampleTime=0;
double Inp=0.0, Setpoint=0.0, out=0.0;
double ITerm=0.0, lastInput=0.0, dInput=0.0;
double kp=0.0, ki=0.0, kd=0.0;
double outMin=0.0, outMax=0.0;
double error=0.0, kpid=0.0;
// Resto de variables.
int8 pwm=0;                                                                 // Variable para el PWM (resoluci�n de 0 a 255).
signed int32  contador=0;                                                   // Variable contador para el encoder.
signed int32  stp[1]={0};                                                   // Variable para la recepci�n de datos por USB.

// Interrupci�n RB0/INT con prioridad alta, para el encoder.
#int_ext fast
void IntRB0(void)                                                           // Decodificador del encoder X1.
{
   if (bit_test(portb,0))
   {  
      ext_int_edge(H_TO_L); if (bit_test(portb,1)) contador++; 
   }
   else                        
   {  
      ext_int_edge(L_TO_H); if (bit_test(portb,1)) contador--;
   }
}

// C�lculo PID.
void Compute(void)
{
   int32 now = get_ticks();                                                 // Toma el n�mero de "ticks" por unidad de tiempo.
   int32 timeChange = (now - lastTime);
   
   if (timeChange>=SampleTime)                                              // Si se cumple el tiempo de muestreo, entonces se calcula el error.                                          
   {                                                                      
      Inp = (double)contador;                                               // Carga en Inp la posici�n del encoder.
      // Calcula todos los errores.
      error = (Setpoint - Inp) * kp;
      dInput = (Inp - lastInput) * kd;
      if (dInput == 0.0) ITerm += (ki * error); else ITerm -= (dInput*ki) / kpid;
      if (ITerm > outMax) ITerm = outMax; else if (ITerm < outMin) ITerm = outMin;
      
      // Calculamos la funci�n de salida del PID.
      out = error + ITerm - dInput;  
      if (out > outMax) out = outMax; else if (Out < outMin) Out = outMin;
      // Guardamos el valor de algunas variables para el pr�ximo rec�lculo.
      lastInput = Inp;
      lastTime = now;
   }
}
  
void SetTunings(double kp, double ki, double kd)                            // A las constantes PID se le incluye el tiempo, de esta manera evitamos repetir c�lculos.
{
   double SampleTimeInSec = (double)SampleTime / 1000.0;
   kp = kp;
   ki = ki * SampleTimeInSec;
   kd = kd / SampleTimeInSec;
   kpid = kp  * ki * kd;
}
 
void SetSampleTime(int16 NewSampleTime)
{
   if (NewSampleTime > 0) SampleTime = NewSampleTime;                       // Tiempo de muestreo, ha de ser mayor o igual a 1.
}
                   
void main(void)
{  
   set_tris_a(0b111111);                                                    // En B7 est� un led para avisar de que se ha llegado al punto designado.
   set_tris_b(0b01111111);                                                  // B0 y B1 son las entradas del encoder. B0 corresponde a la interrupci�n externa.
   set_tris_c(0b11111001);                                                  // C1 y C2 son las salidas hardware PWM de los 18Fxx5x.
   set_tris_d(0b11111111);
   set_tris_e(0b11111111);
   
   output_low(PIN_C1);                                                      // Mantenemos a cero los dos PWM para evitar que el motor haga cosas extra�as al iniciar el PIC.
   output_low(PIN_C2);
   
   usb_init();                                                              // Inicializa el USB.
   usb_task();                                                              // Habilita el USB y sus interrupciones.
   usb_wait_for_enumeration();                                              // Espera a que el USB sea reconocido por el PC.
   
   setup_adc_ports(NO_ANALOGS);                                             // Sin ADC, sin comparadores,
   setup_adc(ADC_OFF);                                                      // todo digital.
   setup_comparator(NC_NC_NC_NC);
   setup_vref(false);
   setup_timer_2(t2_div_by_1, 255, 1);                                      // Frecuencia de la se�al PWM.
   setup_ccp1(CCP_PWM);                                                     // Configura CCP1 como PWM y sale por la patilla C2.
   setup_ccp2(CCP_PWM);                                                     // Configura CCP2 como PWM y sale por la patilla C1.
   
   ext_int_edge(L_TO_H);                                                    // Inicialmente se activar� la interrupci�n externa por flanco de subida.
   enable_interrupts(int_ext);                                              // Activa interrupci�n externa. 
   enable_interrupts(GLOBAL);                                               // Activa interrupciones globales.
   
   outMax =  255.0;                                                         // L�mite m�ximo del controlador PID.
   outMin = -255.0;                                                         // L�mite m�nimo del controlador PID.
   
   SetSampleTime(1);                                                        // Tiempo de muestreo: 1ms.
   
   kp = 5.0;                                                                // Constantes PID �ptimas para mi encoder. (Motor de 12V con encoder de 334 ppr.)
   ki = 0.07;
   kd = 100.0;
   
   SetTunings(kp, ki, kd);                                                  // Calcula las constantes de sintonizaci�n.
   Setpoint=0.0;                                                            // Posici�n inicial.
   
   while(true)
   {
      Compute();                                                            // Realiza los c�lculos PID y el valor se guardar� en la variable global "out".
      
      if(abs(error) <= kp)                                                  // Motor parado cuando el motor est� posicionado. Utilizo KP como margen de error,
      {                                                                     // y me da buen resultado. Si fuese necesario puedes poner un valor num�rico ah�.
         set_pwm1_duty(0);
         set_pwm2_duty(0);
         output_high(PIN_B7);                                               // Enciende el led cuando el motor llega a la posici�n designada.
      }
      else
      {
         output_low(PIN_B7);                                                // Apaga el led cuando el motor se est� posicionando.
         pwm=(int8)abs(out);                                                // Convierte "out" a un valor siempre positivo y lo formatea a 8 bits de resoluci�n.
         
         if(out > 0.0)                                                      // Motor hacia delante con el PWM correspondiente.
         {
            set_pwm1_duty(0);
            set_pwm2_duty(pwm);
         }
         else                                                               // Motor hacia atr�s   con el PWM correspondiente.
         {
            set_pwm1_duty(pwm);
            set_pwm2_duty(0);
         }
      }
                                                                            // Comunicaci�n USB tipo Bulk Transfer.
      if(usb_enumerated())                                                  // Si hay comunicaci�n USB...
      {
         if(usb_kbhit(1))                                                   // Si recibe alg�n dato, entonces...
         {
            usb_get_packet(1, stp, 4);                                      // tomar el valor recibido del ordenador; 4 bytes,
            Setpoint=(double)stp[0];                                        // y lo carga en la variable "Setpoint" con el formato "double".
            //cnv[0]=contador;                                              // Esta parte est� deshabilitada, lo pongo como ejemplo para enviar datos al PC.
            //usb_put_packet(1, cnv, 4, USB_DTS_TOGGLE);                    // Enviar el contenido de "cnv" al ordenador.
         }
      }
   }
}
