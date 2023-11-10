//Data structure for sending data as binary to main controller
typedef union {
  float data;
  byte bin[4];
} float_bin_union;

float_bin_union u_setpoint;
float a =0.0;

void setup() {
  //  Starting of serial communication 
  Serial.begin(115200); 

}

long time_start = millis();

void loop() {

    if (Serial.available()>=4) {
        //a = Serial.parseFloat();
        Serial.readBytes((byte*)&a, 4);
        /*u_setpoint.bin[0] = Serial.read();
        u_setpoint.bin[1] = Serial.read();
        u_setpoint.bin[2] = Serial.read();
        u_setpoint.bin[3] = Serial.read();*/
      Serial.print("DATA : ");
      Serial.println(a,5);
    } 
  


}

