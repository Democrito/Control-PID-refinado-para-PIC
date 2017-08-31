// Este programa está basado en el Control PID de Brett Beauregard: http://brettbeauregard.com/blog/wp-content/uploads/2012/07/Gu%C3%ADa-de-uso-PID-para-Arduino.pdf
// Está enfocado para el control de posición de un motor y el algoritmo original lo modifiqué para "suavizar" la respuesta de dicho motor dándole unas características únicas,
// entre ellas un posicionamiento al punto designado mucho más suave y progresivo y permite "sintonizar" las constantes PID de un modo más intuitivo y sencillo.
// Más información: https://sites.google.com/site/proyectosroboticos/control-de-motores/control-pid-con-encoder

#include <18F4550.h>                                                        // Hardware compatible para cualquier microcontrolador de la familia 18Fxx5x.
#DEVICE HIGH_INTS=TRUE                                                      // Activa la prioridad de las interrupciones externas.
#fuses HSPLL, NOWDT, NOPROTECT, NOLVP, NODEBUG, USBDIV, PLL5, CPUDIV1, VREGEN// Cristal de 20MHz con PLL interno a 48MHz.
#include <usb_bootloader.h>
#include <math.h>

#use delay(clock=48000000)                                                  // El PIC está funcionando a 48MHz gracias al PLL interno.
#use timer(TIMER=1, TICK=1ms, BITS=16, ISR)                                 // Activa un "Timer" en el que incrementa un contador interno (de 16 bits) cada 0.68239ms. Más abajo se configura el TIMER1 para habilitar esta parte. 

#byte porta = 0xF80                                                         // Dirección del puerto A para la familia 18Fxx5x.
#byte portb = 0xF81                                                         // Dirección del puerto B para la familia 18Fxx5x.
#byte portc = 0xF82                                                         // Dirección del puerto C para la familia 18Fxx5x.
#byte portd = 0xF83                                                         // Dirección del puerto D para la familia 18Fxx5x (Sólo 40/44 pines).
#byte porte = 0xF84                                                         // Dirección del puerto E para la familia 18Fxx5x.

#use fast_io(a)
#use fast_io(b)
#use fast_io(c)
#use fast_io(d)
#use fast_io(e)
                                                                            // Configuramos para una comunicación en modo Bulk Transfer.
#define USB_HID_DEVICE     FALSE                                            // HID desactivado.
#define USB_EP1_TX_ENABLE  USB_ENABLE_BULK                                  // Turn on EP1(EndPoint1) for IN bulk/interrupt transfers.
#define USB_EP1_RX_ENABLE  USB_ENABLE_BULK                                  // Turn on EP1(EndPoint1) for OUT bulk/interrupt transfers.
#define USB_EP1_TX_SIZE    4                                                // Tamaño en bytes del buffer de salida.
#define USB_EP1_RX_SIZE    4                                                // Tamaño en bytes del buffer de entrada.

#include <pic18_usb.h>                                                      // Microchip PIC18Fxx5x Hardware layer for CCS's PIC USB driver.
#include "usb_desc_scope.h"                                                 // Enumerador modificado.
#include <usb.c>                                                            // Handles USB setup tokens and get descriptor reports.

// Variables globales para el cálculo del controlador PID.
unsigned int16  lastTime=0, SampleTime=0;
double          Inp=0.0, Setpoint=0.0, out=0.0;
double          ITerm=0.0, lastInput=0.0, dInput=0.0;
double          kp=0.0, ki=0.0, kd=0.0;
double          outMin=0.0, outMax=0.0;
double          error=0.0;
// Resto de variables.
unsigned int8   pwm=0;                                                      // Variable para el PWM (resolución de 0 a 255).
signed int32    contador=0;                                                 // Variable contador para el encoder.
signed int32    stp[1]={0};                                                 // Variable para la recepción de datos por USB.

// Interrupción RB0/INT con prioridad alta, para el encoder.
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

// Cálculo PID.
void Compute(void)
{
   unsigned int16 now = get_ticks();                                        // Toma el número de "ticks" por unidad de tiempo.
   unsigned int16 timeChange = (now - lastTime);
   
   if (timeChange >= SampleTime)                                            // Si se cumple el tiempo de muestreo, entonces se calcula el error.                                          
   {                                                                      
      Inp = (double)contador;                                               // Carga en Inp la posición del encoder.
      // Calcula todos los errores.
      
      error = (Setpoint - Inp)   * kp;
      dInput = (Inp - lastInput) * kd;
      
      if ((dInput == 0.0) || (error == 0.0)) ITerm += (error * ki); else ITerm -= (error * ki); // Cuando tengas el PID ajustado prueba a cambiar esta línea por esta otra: if ((dInput == 0.0) || (error == 0.0)) ITerm += (error * ki); else ITerm -= (dInput / (kp * ki * kd) );
      if (ITerm > outMax) ITerm = outMax; else if (ITerm < outMin) ITerm = outMin;
      
      // Calculamos la función de salida del PID.
      out = error + ITerm - dInput;                                         // Salida del controlador PID. Contiene el valor PWM y el signo será para girar en un sentido o en el otro.
      if (out > outMax) out = outMax; else if (out < outMin) out = outMin;
      // Guardamos el valor de algunas variables para el próximo recálculo.
      lastInput = Inp;
      lastTime  = now;
   }
}
  
void SetTunings(double kp, double ki, double kd)                            // A las constantes KI y KD se le incluye el tiempo, de esta manera evitamos repetir cálculos en Compute().
{
   double SampleTimeInSec = (double)SampleTime / 1000.0;
   kp = kp;
   ki = ki * SampleTimeInSec;
   kd = kd / SampleTimeInSec;
}
 
void SetSampleTime(int16 NewSampleTime)
{
   if (NewSampleTime > 0) SampleTime = NewSampleTime;                       // Tiempo de muestreo, obliga de ser mayor o igual a 1.
}
                   
void main(void)
{  
   set_tris_a(0b111111);                                                    // En B7 existe un led para avisar que está en la meta designada.
   set_tris_b(0b01111111);                                                  // B0 y B1 son las entradas del encoder. B0 corresponde a la interrupción externa.
   set_tris_c(0b11111001);                                                  // C1 y C2 son las salidas hardware PWM de los 18Fxx5x.
   set_tris_d(0b11111111);
   set_tris_e(0b11111111);
   
   output_low(PIN_C1);                                                      // Mantenemos a cero los dos PWM para evitar que el motor haga cosas extrañas al iniciar el PIC.
   output_low(PIN_C2);
   
   usb_init();                                                              // Inicializa el USB.
   usb_task();                                                              // Habilita el USB y sus interrupciones.
   usb_wait_for_enumeration();                                              // Espera a que el USB sea reconocido por el PC.
   
   setup_adc_ports(NO_ANALOGS);                                             // Sin ADC, sin comparadores,
   setup_adc(ADC_OFF);                                                      // todo digital.
   setup_comparator(NC_NC_NC_NC);
   setup_vref(false);
   
   setup_timer_1(t1_internal|t1_div_by_1);                                  // Configuración del TIMER1 para el tiempo de muestreo del control PID. Esta línea es importante, de otra manera no funcionaría la línea de arriba "#use timer(TIMER=1, TICK=1ms, BITS=16, ISR)".
   setup_timer_2(t2_div_by_1, 255, 1);                                      // Frecuencia de la señal PWM.
   setup_ccp1(CCP_PWM);                                                     // Configura CCP1 como PWM y sale por la patilla C2.
   setup_ccp2(CCP_PWM);                                                     // Configura CCP2 como PWM y sale por la patilla C1.
   
   ext_int_edge(L_TO_H);                                                    // Inicialmente se activará la interrupción externa por flanco de subida.
   enable_interrupts(int_ext);                                              // Activa interrupción externa. 
   enable_interrupts(GLOBAL);                                               // Activa interrupciones globales.
   
   outMax =  255.0;                                                         // Límite máximo del controlador PID. 0...255 ---> hacia delante.
   outMin = -outMax;                                                        // Límite mínimo del controlador PID. 0..-255 ---> hacia atrás.
   
   SetSampleTime(25);                                                       // Tiempo de muestreo en milisegundos. (En realidad cada tick es de 682,39 microsegundos.)
   
   kp = 5.0;                                                                // Constantes PID óptimas para mi encoder. (Motor de 12V con encoder de 334 ppr.)
   ki = 0.05;
   kd = 25.0;
   
   SetTunings(kp, ki, kd);                                                  // Calcula las constantes de sintonización.
   Setpoint=(double)contador;                                               // Posición inicial igual a la del encoder para evitar que al ponerlo en marcha haga cosas extrañas.
   
   while(true)
   {
      Compute();                                                            // Realiza los cálculos PID y el valor se guardará en la variable global "out".
      
      if (error == 0.0)                                                     // Motor parado cuando el motor está posicionado.
      {
         set_pwm1_duty(0);
         set_pwm2_duty(0);
         output_high(PIN_B7);                                               // Enciende el led cuando el motor llega a la posición designada.
      }
      else
      {
         output_low(PIN_B7);                                                // Apaga el led cuando el motor se está posicionando.
         
         pwm=(int8)abs(out);                                                // Convierte "out" a un valor siempre positivo y lo formatea a 8 bits de resolución. Es el PWM.
         
         if (out > 0.0)                                                     // Motor hacia delante con el PWM correspondiente.
         {
            set_pwm1_duty(0);
            set_pwm2_duty(pwm);
         }
         else                                                               // Motor hacia atrás   con el PWM correspondiente.
         {
            set_pwm1_duty(pwm);
            set_pwm2_duty(0);
         }
      }
                                                                            // Comunicación USB tipo Bulk Transfer.
      if (usb_enumerated())                                                 // Si hay comunicación USB...
      {
         if (usb_kbhit(1))                                                  // Si recibe algún dato, entonces...
         {
            usb_get_packet(1, stp, 4);                                      // tomar el valor recibido del ordenador; 4 bytes, y lo carga
            Setpoint=(double)stp[0];                                        // en la variable "Setpoint" con el formato "double".
            //cnv[0]=contador;                                              // Esta parte está deshabilitada, lo pongo como ejemplo para enviar datos al PC.
            //usb_put_packet(1, cnv, 4, USB_DTS_TOGGLE);                    // Enviaría el contenido de "cnv" al ordenador.
         }
      }
   }
}
