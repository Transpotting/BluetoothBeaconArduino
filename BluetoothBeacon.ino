#define uECC_WORD_SIZE 1
#define uECC_SUPPORTS_secp192r1 0
#define uECC_SUPPORTS_secp224r1 0
#define uECC_SUPPORTS_secp256r1 0
#define uECC_SUPPORTS_secp256k1 0

#include <EEPROM.h>
#include <uECC.h>

#include <SoftwareSerial.h>

#define BT_NAME "BUS-1070"

#define EEPROM_PUBKEY_ADDR 0
#define EEPROM_PUBKEY_SIG 0x08

SoftwareSerial btModule(2,3);

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


bool GenerateECC() {
  uint8_t pri[32] = {0};
  uint8_t pub[64] = {0};
  uint8_t hash[32] = {0};
  uint8_t sig[64] = {0};

  Serial.println();

  const struct uECC_Curve_t *curve = uECC_secp160r1();

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

  Serial.print("PUB: ");
  for (int i = 0; i < uECC_curve_public_key_size(curve); i++) {
    char hd[3] = {0};
    sprintf(hd, "%02x", pub[i]);
    Serial.print(hd);
  }

  Serial.println();
  
  memcpy(hash, pub, sizeof(hash));
  if (!uECC_sign(pri, hash, sizeof(hash), sig, curve)) {
      Serial.println("uECC_sign() failed");
      return false;
  }

  for (int i = 0; i < 64; i++) {
    char hd[3] = {0};
    sprintf(hd, "%02x", sig[i]);
    Serial.print(hd);
  }

  Serial.println();

  return true;
}

void setup() {
  randomSeed(analogRead(0));
  uECC_set_rng(&RNG);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  Serial.begin(9600);
  btModule.begin(9600);

  delay(100);
  btModule.print("AT+NAME" BT_NAME);

  GenerateECC();
  
  digitalWrite(13, LOW);
}

void loop() {
  if (btModule.available()) {
    String line = btModule.readStringUntil('\n');
    Serial.print(line);
  }

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    btModule.print(line);
  }
}
