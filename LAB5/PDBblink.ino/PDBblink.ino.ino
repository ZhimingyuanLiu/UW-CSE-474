
#include <kinetis.h>

#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE \
  | PDB_SC_CONT | PDB_SC_PRESCALER(7) | PDB_SC_MULT(1))
int ledStatus;
void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  ledStatus = 1;
  SIM_SCGC6 |= SIM_SCGC6_PDB;  


  PDB0_MOD = 48000000 / 128 / 10 / 1 ;
  // Interrupt delay
  PDB0_IDLY = 0;
  PDB0_SC = PDB0_SC | PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE | PDB_SC_CONT | PDB_SC_PRESCALER(7) | PDB_SC_MULT(1);
  // PDB0_CH0DLY0 = 0;
  PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
  // Software trigger (reset and restart counter)
  PDB0_SC |= PDB_SC_SWTRIG;
  // Enable interrupt request
  NVIC_ENABLE_IRQ(IRQ_PDB);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void pdb_isr() {
   digitalWrite(13, (ledStatus = !ledStatus));// invert the value of the LED each interrupt
   PDB0_SC = PDB_CONFIG | PDB_SC_LDOK; // (also clears interrupt flag)
}
