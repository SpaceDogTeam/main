
/*  DESCRIPTION OF THE FUNCTIONALITIES */


/*  Variables definition and their uses */

#define APWM_PIN  5       // Speed of motor A and B in PWM units, value from 0 to 255
#define AEN_PIN   6

#define BPWM_PIN  7      // This sets the direction of motor A and B, set to HIGH for outward motion and LOW for inward motion
#define BEN_PIN   8

long initialMillis = 0; // This records the initial time, at which the program started running
long currentMillis = 0;

bool pin1 = 0;
int sped = 165;

/*  Functions used and their description */

void stop();  // This functions stop the motion of both motors

void setup() {
  // put your setup code here, to run once:
  analogWrite(APWM_PIN, sped);     
  analogWrite(BPWM_PIN, sped);

  digitalWrite(AEN_PIN, pin1);
  digitalWrite(BEN_PIN, pin1);

  Serial.begin(115200);

  initialMillis = millis(); 
}

void loop() {
  // put your main code here, to run repeatedly:
  currentMillis = -initialMillis + millis();
  if (currentMillis > 4*1000)  {
    stop();
  }
}
void stop(){
  Serial.println("motion is done");
  digitalWrite(APWM_PIN, 0);
  digitalWrite(BPWM_PIN, 0);
 }
