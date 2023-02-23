float degree = 90, b = 400, a = 0;
double target_dist1 = 0, phi1 = 0, theta1 = 0, phi_rad1 = 0, theta_rad1 = 0;

void setup() {
  Serial.begin(115200);

}
void loop () {
  if (!a) {
    if (Serial.available()) {
      degree = Serial.parseFloat() + Serial.parseFloat() ;
      a = 1;
    }
  }

  else {
    if (Serial.available()) {
      b = Serial.parseFloat() + Serial.parseFloat() ;
      a = 0;
    }
  }
  angle_calculation1((degree * 8.889), b);
  Serial.print(degree);
  Serial.print("    ");
  Serial.print(b);
  Serial.print("    ");
  Serial.print(target_dist1);
  Serial.print("    ");
  Serial.println(phi1);
}
void angle_calculation1 (float steps, float d) {
  theta1 = (steps * 0.1125);
  theta_rad1 = (theta1 * 0.0174);
  target_dist1 = sq(d) + sq(40.3) - ((d) * 80.6 * cos(theta_rad1));
  target_dist1 = sqrt(target_dist1);
  phi_rad1 = asin(sin(theta_rad1) * (d / target_dist1));
  phi1  = (phi_rad1 * 57.3248);

}
