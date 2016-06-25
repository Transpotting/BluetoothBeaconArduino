#define uECC_WORD_SIZE 1
#define uECC_SUPPORTS_secp192r1 0
#define uECC_SUPPORTS_secp224r1 0
#define uECC_SUPPORTS_secp256r1 0
#define uECC_SUPPORTS_secp256k1 0

#include <EEPROM.h>
#include <uECC.h>

#include <SoftwareSerial.h>

// Bluetooth module comm.
SoftwareSerial btModule(2,3);
#define BT_NAME "BUS-1070"

#define EEPROM_PUBKEY_ADDR 0
#define EEPROM_PUBKEY_SIG 0x08

// Disabled sensor / light
#define PIN_DISABLED 12
#define TIMEOUT_DISABLED 5000

// GPS functionality

// GPS simulation points and interpolation
#define GPS_A_LAT 46.732272
#define GPS_A_LONG 14.095801
#define GPS_B_LAT 46.765677
#define GPS_B_LONG 14.380072
#define GPS_INTERP_MAX_STEP 1000

float gps_lat = GPS_A_LAT;
float gps_long = GPS_A_LONG;
byte dir = 1; // 1 = up, 0 = down
int interp_step = 0;
uint32_t count = 0;
long time_disabled = 0;

// ECC cryptography
#define CURVE_SIG_SIZE 40
const struct uECC_Curve_t *curve = uECC_secp160r1();
uint8_t pri[32] = {0};
uint8_t pub[64] = {0};

/**
 * Linear interpolation between numbers s and e, with max_step steps, for the current this_step.
 */
float interp(float s, float e, float this_step, float max_step) {
  return s + ((e - s) / max_step) * this_step;
}

/**
 * Formats the current lat/long coordinates into a string of the form "LAT, LONG"
 */
void format_gps(char *line) {
  line[0] = 0;
  char num[10];
  dtostrf(gps_lat, 0, 6, num);
  strcat(line, num);
  strcat(line, ", ");
  dtostrf(gps_long, 0, 6, num);
  strcat(line, num);
}

/**
 * Sends current GPS coordinates.
 */
void send_gps(bool bt) {
  char line[25];
  format_gps(line);
  if (bt)
    btModule.println(line);
  else
    Serial.println(line);
}

/**
 * Increments GPS simulation interpolation for current points.
 */
void interp_gps() {
  interp_step++;
  if (interp_step >= GPS_INTERP_MAX_STEP) {
    interp_step = 0;
    dir = !dir;
  }
  if (dir == 1) {
    gps_lat = interp(GPS_A_LAT, GPS_B_LAT, interp_step, GPS_INTERP_MAX_STEP);
    gps_long = interp(GPS_A_LONG, GPS_B_LONG, interp_step, GPS_INTERP_MAX_STEP);
  } else {
    gps_lat = interp(GPS_B_LAT, GPS_A_LAT, interp_step, GPS_INTERP_MAX_STEP);
    gps_long = interp(GPS_B_LONG, GPS_A_LONG, interp_step, GPS_INTERP_MAX_STEP);   
  }
}


/**
 * Random number generator for RCC crypto.
 * (weak)
 */
static int RNG(uint8_t *p_dest, unsigned p_size) {
  while(p_size) {
    long v = random();
    unsigned l_amount = min(p_size, sizeof(long));
    memcpy(p_dest, &v, l_amount);
    p_size -= l_amount;
    p_dest += l_amount;
  }
  return 1;
}


/**
 * Either generates an ECC keypair and stores it in EEPROM,
 * or reads it from the EEPROM.
 */
bool generate_ecc() {
  if (EEPROM.read(EEPROM_PUBKEY_ADDR) != EEPROM_PUBKEY_SIG) {
    if (!uECC_make_key(pub, pri, curve)) {
      Serial.println("ECC make key fail");
      return false;
    }
    Serial.println("Made new ECC key");
    int addr = EEPROM_PUBKEY_ADDR + 1;
    for (int i = 0; i < sizeof(pub); i++)
      EEPROM.write(addr++, pub[i]);
    for (int i = 0; i < sizeof(pri); i++)
      EEPROM.write(addr++, pri[i]);
    EEPROM.write(EEPROM_PUBKEY_ADDR, EEPROM_PUBKEY_SIG);
  } else {
    int addr = EEPROM_PUBKEY_ADDR + 1;
    for (int i = 0; i < sizeof(pub); i++)
      pub[i] = EEPROM.read(addr++);
    for (int i = 0; i < sizeof(pri); i++)
      pri[i] = EEPROM.read(addr++);
  }

  if (!uECC_valid_public_key(pub, curve)) {
    Serial.println("Invalid pubkey");
    return false;
  }

  return true;
}

/**
 * Sends the public part of the ECC keypair.
 */
void send_pub(bool bt) {
  for (int i = 0; i < uECC_curve_public_key_size(curve); i++) {
    char hd[3] = {0};
    sprintf(hd, "%02x", pub[i]);
    if (bt)
      btModule.print(hd);
    else
      Serial.print(hd);
  }
  if (bt)
    btModule.println();
  else
    Serial.println();
}

/**
 * Signs a string and sends the signature.
 */
void sign_send(String s, bool bt) {
  uint8_t hash[32] = {0};
  uint8_t sig[64] = {0};

  s.getBytes(hash, s.length());
  
  if (!uECC_sign(pri, hash, sizeof(hash), sig, curve)) {
      Serial.println("uECC_sign() failed");
      return;
  }

  for (int i = 0; i < CURVE_SIG_SIZE; i++) {
    char hd[3] = {0};
    sprintf(hd, "%02x", sig[i]);
    if (bt)
      btModule.print(hd);
    else
      Serial.print(hd);
  }

  if (bt)
    btModule.println();
  else
    Serial.println();
}

/**
 * Arduino setup.
 */
void setup() {
  randomSeed(analogRead(0));
  uECC_set_rng(&RNG);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  Serial.begin(9600);
  btModule.begin(9600);

  delay(100);
  btModule.println("AT+NAME" BT_NAME);

  generate_ecc();
  
  digitalWrite(13, LOW);
  pinMode(PIN_DISABLED, OUTPUT);
  digitalWrite(PIN_DISABLED, LOW);
}

/**
 * Arduino main code.
 */
void loop() {
  count++;
  
  if (btModule.available()) {
    String line = btModule.readStringUntil('\n');
    line.trim();
    Serial.println("BT: " + line);

    if (line.startsWith("WTH")) {
      String cmd = line.substring(4);
      String param = "";
      int p = cmd.indexOf(' ');
      if (p != -1) {
        param = cmd.substring(p + 1);
        cmd = cmd.substring(0, p);
      }
      /*
      Serial.print("CMD: '");
      Serial.print(cmd);
      Serial.print("' PARAM: '");
      Serial.print(param);
      Serial.println("'");
      */
      if (cmd == "POS") {
        send_gps(true);
      } else if (cmd == "DISABLED") {
        time_disabled = millis();
        digitalWrite(PIN_DISABLED, HIGH);
      } else if (cmd == "SIGN") {
        sign_send(param, true);
      } else if (cmd == "PUB") {
        send_pub(true);
      } else if (cmd == "PING") {
        btModule.println("PONG");
      } else if (cmd == "NAME") {
        btModule.println(BT_NAME);
      }
    }
  }

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("WTH")) {
      String cmd = line.substring(4);
      String param = "";
      int p = cmd.indexOf(' ');
      if (p != -1) {
        param = cmd.substring(p + 1);
        cmd = cmd.substring(0, p);
      }
      if (cmd == "NAME") {
        Serial.println(BT_NAME);
      } else if (cmd == "PING") {
        Serial.println("PONG");
      }
    }
  }

  if (count % 1000 == 0) {
    /* General "often" maintainance */
    interp_gps();
    if (time_disabled) {
      if (millis() - time_disabled > TIMEOUT_DISABLED) {
        time_disabled = 0;
        digitalWrite(PIN_DISABLED, LOW);
      }
    }
  }

  if (count % 10000 == 0) {
    Serial.print("POS ");
    send_gps(false);
  }
  
}
