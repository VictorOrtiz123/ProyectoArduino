// Sistema de control de acceso - Arduino MEGA 2560
// Maquina de estados no bloqueante con sensores, RFID, keypad, LCD y servo.

#include <Keypad.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include <EEPROM.h>
#include <SPI.h>
#include <MFRC522.h>

// Pines LED RGB (catodo comun: HIGH enciende)
#define PIN_R        28
#define PIN_G        30
#define PIN_B        32

// Buzzer y servo
#define PIN_BUZZER   7
#define PIN_SERVO    13

// Boton de configuracion
#define PIN_BOTON_CONFIG 6

// Pines analogicos de sensores
#define PIN_TEMP     A0
#define PIN_LUZ      A2
#define PIN_HALL     A4
#define PIN_SONIDO   A8

// RFID RC522 por SPI
#define SS_PIN   53
#define RST_PIN  9
MFRC522 mfrc522(SS_PIN, RST_PIN);

// LCD 16x2
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Keypad 4x4
const byte FILAS = 4;
const byte COLS  = 4;
char teclas[FILAS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte pinesFilas[FILAS] = {27, 29, 31, 33};
byte pinesCols[COLS]   = {35, 37, 39, 41};
Keypad teclado = Keypad(makeKeymap(teclas), pinesFilas, pinesCols, FILAS, COLS);

// Servo de cerradura
Servo servoCerradura;
const int SERVO_CERRADO = 0;
const int SERVO_ABIERTO = 90;

// Constantes del termistor (Steinhart-Hart)
const float TH_R1 = 10000.0;
const float TH_C1 = 0.001129148;
const float TH_C2 = 0.000234125;
const float TH_C3 = 0.0000000876741;

// Modulo de reloj: unico lugar donde se usa millis()
unsigned long relojActual() {
  return millis();
}

bool tiempoCumplido(unsigned long inicio, unsigned long duracion) {
  return (relojActual() - inicio) >= duracion;
}

unsigned long tiempoTranscurrido(unsigned long inicio) {
  return relojActual() - inicio;
}

// Estados del sistema
enum EstadoSistema {
  ST_INICIO,
  ST_ERROR_CORTO,
  ST_BLOQUEADO,
  ST_ABIERTO,
  ST_MONITOREO,
  ST_ALARMA_SENSOR,
  ST_GESTION
};
EstadoSistema estado = ST_INICIO;

// Hora manual en formato HHMM (sin RTC)
int horaActual = 1800;

// Configuracion de roles
const byte NUM_ROLES     = 4;
const byte LARGO_CLAVE   = 4;
const byte LARGO_TARJETA = 20;

// Mapa EEPROM
#define EE_MAGIC        0
#define EE_MAGIC_VAL    0x82
#define EE_CLAVES       1
#define EE_TARJETAS     17
#define EE_UMBRAL       110

const char* rolNombre[NUM_ROLES] = {"Seguridad", "Operarios", "Coordinad.", "Gerentes"};

// Claves por rol (se cargan desde EEPROM)
char claves[NUM_ROLES][LARGO_CLAVE + 1] = {"1111", "2222", "3333", "4444"};

// UIDs de tarjetas RFID (reemplazar con los reales)
char tarjetas[NUM_ROLES][LARGO_TARJETA + 1] = {
  "5634DA73",
  "C12FD60E",
  "AABBCCDD",
  "11223344"
};

// Horarios por rol en formato HHMM
const int horarioIni[NUM_ROLES] = {   0,  600,  700,  800};
const int horarioFin[NUM_ROLES] = {2359, 1800, 2000, 1700};
const char* rolHorarioTxt[NUM_ROLES] = {"00:00-23:59", "06:00-18:00", "07:00-20:00", "08:00-17:00"};

// Buffer de clave ingresada
char bufferClave[LARGO_CLAVE + 1];
byte longBuffer = 0;

byte rolActivo = 0;

// Contadores de uso de credenciales (cambio obligatorio a las 4 veces)
byte usosClave[NUM_ROLES]   = {0, 0, 0, 0};
byte usosTarjeta[NUM_ROLES] = {0, 0, 0, 0};
const byte USOS_PARA_CAMBIO = 4;
bool pendienteCambio = false;
bool cambioPorTarjeta = false;
byte rolCambio = 0;

// Intentos fallidos
byte intentosFallidos = 0;
const byte MAX_INTENTOS = 3;

// Causa del error para el LCD
enum CausaError { ERR_CLAVE, ERR_TARJETA, ERR_HORARIO };
CausaError causaError = ERR_CLAVE;

// Sensores e indices
enum Sensor { S_TEMP = 0, S_LUZ = 1, S_SONIDO = 2, S_HALL = 3 };
int  umbral[4];
int  valorSensor[4];

// Alarmas por grupo: ambiental (Temp+Luz) y puerta (Sonido+Hall)
bool grupoEnAlarmaAmbiental = false;
bool grupoEnAlarmaPuerta    = false;
int  contadorAlarmaAmbiental = 0;
int  contadorAlarmaPuerta    = 0;
int  grupoAlarmaActivo = -1;

// Duraciones en ms
const unsigned long DUR_SERVO       = 10000;
const unsigned long DUR_ERROR_CORTO = 1000;
const unsigned long DUR_BLOQUEO     = 7000;
const unsigned long DUR_HORARIO     = 10000;
const unsigned long DUR_MONITOREO   = 18000;
const unsigned long PASO_MONITOR    = 3000;

unsigned long tServoAbierto = 0;
unsigned long tError        = 0;
unsigned long tHorario      = 0;
unsigned long tMonitoreo    = 0;
bool servoEstaAbierto = false;

// Intervalos de tareas periodicas
const unsigned long INT_SENSORES = 150;
const unsigned long INT_LCD      = 200;
unsigned long tSensores = 0;
unsigned long tLCD      = 0;

// Parpadeo de LED
unsigned long tBlink = 0;
bool ledRojoEstado = false;

// Antirebote del boton
bool botonLecturaPrev = HIGH;
bool botonEstable      = HIGH;
unsigned long tBotonCambio = 0;
const unsigned long DEBOUNCE_BOTON = 30;

// Pantalla de horarios
bool mostrandoHorarios = false;
byte rolHorario = 0;

// Gestion de credenciales y umbrales
enum TipoGestion { G_CLAVE, G_TARJETA, G_UMBRAL_AMBIENTAL, G_UMBRAL_PUERTA };
TipoGestion gestionTipo = G_CLAVE;
byte pasoGestionUmbral = 0;
char bufferGestion[6];
byte longGestion = 0;
bool gestionError = false;

// Prototipos
unsigned long relojActual();
bool tiempoCumplido(unsigned long inicio, unsigned long duracion);
unsigned long tiempoTranscurrido(unsigned long inicio);
void actualizarMaquinaEstados();
void tareaTeclado();
void tareaRFID();
void tareaSensores();
void tareaServo();
void tareaAlarma();
void tareaLCD();
void tareaHorarios();
void tareaBotonConfiguracion();
void tareaGestion();
void tareaMonitoreo();
void cargarEEPROM();
void guardarClaveEEPROM(byte rol);
void guardarTarjetaEEPROM(byte rol);
void guardarUmbralEEPROM(byte s);
void ledColor(bool r, bool g, bool b);
bool enHorario(byte rol);
void verificarClave();
void procesarTarjeta(char* card);
void capturarNuevaTarjeta(char* card);
void accesoConcedido(byte rol, bool esTarjeta);
void accesoFallido(CausaError causa);
void avanzarHorario();
void iniciarMonitoreo();
void iniciarGestionClave(byte rol);
void iniciarGestionTarjeta(byte rol);
void iniciarGestionUmbralGrupo(byte grupo);
void manejarTeclaGestion(char k);
bool claveYaExiste(const char* nuevaClave);
void confirmarGestion();
void resetBuffer();

void setup() {
  Serial.begin(9600);

  SPI.begin();
  mfrc522.PCD_Init();

  pinMode(PIN_R, OUTPUT);
  pinMode(PIN_G, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  ledColor(false, false, false);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  pinMode(PIN_BOTON_CONFIG, INPUT_PULLUP);

  servoCerradura.attach(PIN_SERVO);
  servoCerradura.write(SERVO_CERRADO);

  lcd.begin(16, 2);
  lcd.clear();

  cargarEEPROM();

  resetBuffer();
  estado = ST_INICIO;

  lcd.setCursor(0, 0);
  lcd.print("Sistema listo");
  lcd.setCursor(0, 1);
  lcd.print("Ingrese clave");
}

void loop() {
  actualizarMaquinaEstados();
  tareaTeclado();
  tareaRFID();
  tareaBotonConfiguracion();
  tareaSensores();
  tareaServo();
  tareaHorarios();
  tareaMonitoreo();
  tareaGestion();
  tareaAlarma();
  tareaLCD();
}

// Transiciones automaticas por tiempo
void actualizarMaquinaEstados() {
  if (estado == ST_ERROR_CORTO) {
    if (tiempoCumplido(tError, DUR_ERROR_CORTO)) {
      estado = ST_INICIO;
      resetBuffer();
    }
  }
  else if (estado == ST_BLOQUEADO) {
    if (tiempoCumplido(tError, DUR_BLOQUEO)) {
      estado = ST_INICIO;
      intentosFallidos = 0;
      resetBuffer();
    }
  }
}

// Carga EEPROM o escribe valores por defecto si es la primera vez
void cargarEEPROM() {
  if (EEPROM.read(EE_MAGIC) != EE_MAGIC_VAL) {
    const char* defClaves[NUM_ROLES]   = {"1111", "2222", "3333", "4444"};
    const char* defTarjetas[NUM_ROLES] = {"5634DA73", "C12FD60E", "AABBCCDD", "11223344"};

    for (byte r = 0; r < NUM_ROLES; r++)
      for (byte i = 0; i < LARGO_CLAVE; i++)
        EEPROM.update(EE_CLAVES + r * LARGO_CLAVE + i, defClaves[r][i]);

    for (byte r = 0; r < NUM_ROLES; r++) {
      byte base = EE_TARJETAS + r * (LARGO_TARJETA + 1);
      for (byte i = 0; i <= LARGO_TARJETA; i++) {
        char c = (i < strlen(defTarjetas[r])) ? defTarjetas[r][i] : '\0';
        EEPROM.update(base + i, c);
      }
    }

    int defT = 30, defL = 950, defS = 150, defH = 500;
    EEPROM.put(EE_UMBRAL + S_TEMP   * 2, defT);
    EEPROM.put(EE_UMBRAL + S_LUZ    * 2, defL);
    EEPROM.put(EE_UMBRAL + S_SONIDO * 2, defS);
    EEPROM.put(EE_UMBRAL + S_HALL   * 2, defH);

    EEPROM.update(EE_MAGIC, EE_MAGIC_VAL);
  }

  for (byte r = 0; r < NUM_ROLES; r++) {
    for (byte i = 0; i < LARGO_CLAVE; i++)
      claves[r][i] = (char)EEPROM.read(EE_CLAVES + r * LARGO_CLAVE + i);
    claves[r][LARGO_CLAVE] = '\0';
  }

  for (byte r = 0; r < NUM_ROLES; r++) {
    byte base = EE_TARJETAS + r * (LARGO_TARJETA + 1);
    for (byte i = 0; i < LARGO_TARJETA; i++)
      tarjetas[r][i] = (char)EEPROM.read(base + i);
    tarjetas[r][LARGO_TARJETA] = '\0';
  }

  for (byte s = 0; s < 4; s++) EEPROM.get(EE_UMBRAL + s * 2, umbral[s]);
}

void guardarClaveEEPROM(byte rol) {
  for (byte i = 0; i < LARGO_CLAVE; i++)
    EEPROM.update(EE_CLAVES + rol * LARGO_CLAVE + i, claves[rol][i]);
}

void guardarTarjetaEEPROM(byte rol) {
  byte base = EE_TARJETAS + rol * (LARGO_TARJETA + 1);
  for (byte i = 0; i <= LARGO_TARJETA; i++)
    EEPROM.update(base + i, tarjetas[rol][i]);
}

void guardarUmbralEEPROM(byte s) {
  EEPROM.put(EE_UMBRAL + s * 2, umbral[s]);
}

// Verifica si la hora actual esta dentro del horario del rol
bool enHorario(byte rol) {
  return (horaActual >= horarioIni[rol] && horaActual <= horarioFin[rol]);
}

// Atiende el teclado segun el estado actual
void tareaTeclado() {
  char k = teclado.getKey();
  if (!k) return;

  if (estado == ST_GESTION) {
    manejarTeclaGestion(k);
    return;
  }

  if (estado == ST_ERROR_CORTO || estado == ST_BLOQUEADO || estado == ST_ALARMA_SENSOR) return;

  if (estado == ST_MONITOREO) {
    estado = ST_INICIO;
    mostrandoHorarios = false;
    resetBuffer();
  }

  if (k == '#') {
    avanzarHorario();
    return;
  }

  if (k == '*') {
    if (mostrandoHorarios) mostrandoHorarios = false;
    verificarClave();
    return;
  }

  if (k >= '0' && k <= '9') {
    if (mostrandoHorarios) {
      mostrandoHorarios = false;
      resetBuffer();
    }
    if (longBuffer < LARGO_CLAVE) {
      bufferClave[longBuffer++] = k;
      bufferClave[longBuffer] = '\0';
    }
  }
}

// Antirebote del boton; si se presiona durante horarios, vuelve al inicio
void tareaBotonConfiguracion() {
  bool lectura = digitalRead(PIN_BOTON_CONFIG);

  if (lectura != botonLecturaPrev) {
    tBotonCambio = relojActual();
    botonLecturaPrev = lectura;
  }

  if (tiempoTranscurrido(tBotonCambio) > DEBOUNCE_BOTON) {
    if (lectura != botonEstable) {
      botonEstable = lectura;
      if (botonEstable == LOW) {
        if (mostrandoHorarios) {
          mostrandoHorarios = false;
          estado = ST_INICIO;
          resetBuffer();
        }
      }
    }
  }
}

// Avanza entre horarios de roles al presionar '#'
void avanzarHorario() {
  if (!mostrandoHorarios) {
    mostrandoHorarios = true;
    rolHorario = 0;
  } else {
    rolHorario = (rolHorario + 1) % NUM_ROLES;
  }
  tHorario = relojActual();
}

// Timeout de la pantalla de horarios
void tareaHorarios() {
  if (estado == ST_INICIO && mostrandoHorarios) {
    if (tiempoCumplido(tHorario, DUR_HORARIO)) {
      mostrandoHorarios = false;
      resetBuffer();
    }
  }
}

// Compara la clave ingresada con las almacenadas y verifica horario
void verificarClave() {
  if (longBuffer == 0) return;

  int rol = -1;
  if (longBuffer == LARGO_CLAVE) {
    for (byte r = 0; r < NUM_ROLES; r++) {
      if (strcmp(bufferClave, claves[r]) == 0) { rol = r; break; }
    }
  }

  if (rol < 0) {
    accesoFallido(ERR_CLAVE);
  } else if (!enHorario((byte)rol)) {
    accesoFallido(ERR_HORARIO);
  } else {
    usosClave[rol]++;
    accesoConcedido((byte)rol, false);
  }
  resetBuffer();
}

// Lee tarjetas RFID y entrega el UID en texto hex a procesarTarjeta()
void tareaRFID() {
  if (!mfrc522.PICC_IsNewCardPresent()) return;
  if (!mfrc522.PICC_ReadCardSerial()) return;

  const char HEXCH[] = "0123456789ABCDEF";
  char uid[LARGO_TARJETA + 1];
  byte n = 0;
  for (byte i = 0; i < mfrc522.uid.size && n < LARGO_TARJETA - 1; i++) {
    byte b = mfrc522.uid.uidByte[i];
    uid[n++] = HEXCH[(b >> 4) & 0x0F];
    uid[n++] = HEXCH[b & 0x0F];
  }
  uid[n] = '\0';

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();

  Serial.print("UID leido: ");
  Serial.println(uid);

  procesarTarjeta(uid);
}

// Procesa el UID leido: acceso o captura de nueva tarjeta en gestion
void procesarTarjeta(char* card) {
  if (estado == ST_GESTION) {
    if (gestionTipo == G_TARJETA) capturarNuevaTarjeta(card);
    return;
  }

  if (mostrandoHorarios) mostrandoHorarios = false;

  if (estado == ST_ERROR_CORTO || estado == ST_BLOQUEADO || estado == ST_ALARMA_SENSOR) return;

  if (estado == ST_MONITOREO) estado = ST_INICIO;

  for (byte r = 0; r < NUM_ROLES; r++) {
    if (strcmp(card, tarjetas[r]) == 0) {
      if (!enHorario(r)) {
        accesoFallido(ERR_HORARIO);
      } else {
        usosTarjeta[r]++;
        accesoConcedido(r, true);
      }
      return;
    }
  }
  accesoFallido(ERR_TARJETA);
}

// Valida y guarda la nueva tarjeta durante gestion
void capturarNuevaTarjeta(char* card) {
  byte len = strlen(card);

  if (len == 0 || len > LARGO_TARJETA) { gestionError = true; return; }

  if (strcmp(card, tarjetas[rolCambio]) == 0) { gestionError = true; return; }

  for (byte r = 0; r < NUM_ROLES; r++) {
    if (r != rolCambio && strcmp(card, tarjetas[r]) == 0) { gestionError = true; return; }
  }

  strcpy(tarjetas[rolCambio], card);
  guardarTarjetaEEPROM(rolCambio);
  usosTarjeta[rolCambio] = 0;
  pendienteCambio = false;
  estado = ST_INICIO;
  resetBuffer();
}

// Abre el servo y marca cambio pendiente si se alcanzo el limite de usos
void accesoConcedido(byte rol, bool esTarjeta) {
  rolActivo = rol;
  intentosFallidos = 0;

  servoCerradura.write(SERVO_ABIERTO);
  servoEstaAbierto = true;
  tServoAbierto = relojActual();
  estado = ST_ABIERTO;

  pendienteCambio = false;
  cambioPorTarjeta = false;
  rolCambio = rol;
  if (esTarjeta) {
    if (usosTarjeta[rol] >= USOS_PARA_CAMBIO) { pendienteCambio = true; cambioPorTarjeta = true; }
  } else {
    if (usosClave[rol] >= USOS_PARA_CAMBIO)   { pendienteCambio = true; cambioPorTarjeta = false; }
  }
}

// Incrementa intentos fallidos y transiciona a error corto o bloqueo
void accesoFallido(CausaError causa) {
  causaError = causa;
  intentosFallidos++;

  if (intentosFallidos >= MAX_INTENTOS) {
    estado = ST_BLOQUEADO;
  } else {
    estado = ST_ERROR_CORTO;
  }
  tError = relojActual();
  resetBuffer();
}

// Cierra el servo tras el tiempo configurado y decide el siguiente estado
void tareaServo() {
  if (servoEstaAbierto && tiempoCumplido(tServoAbierto, DUR_SERVO)) {
    servoCerradura.write(SERVO_CERRADO);
    servoEstaAbierto = false;

    if (estado == ST_ABIERTO) {
      if (pendienteCambio) {
        if (cambioPorTarjeta) iniciarGestionTarjeta(rolCambio);
        else                  iniciarGestionClave(rolCambio);
      } else {
        iniciarMonitoreo();
      }
    }
  }
}

// Inicia el monitoreo post-acceso
void iniciarMonitoreo() {
  estado = ST_MONITOREO;
  tMonitoreo = relojActual();
}

// Finaliza el monitoreo tras el tiempo configurado
void tareaMonitoreo() {
  if (estado == ST_MONITOREO) {
    if (tiempoCumplido(tMonitoreo, DUR_MONITOREO)) {
      estado = ST_INICIO;
      resetBuffer();
    }
  }
}

// Placeholder de tarea de gestion (las transiciones son por eventos)
void tareaGestion() {
}

// Lee sensores y evalua alarmas por grupo (ambiental y puerta)
void tareaSensores() {
  if (!tiempoCumplido(tSensores, INT_SENSORES)) return;
  tSensores = relojActual();

  // Temperatura con termistor (formula Steinhart-Hart corregida)
  int Vo = analogRead(PIN_TEMP);
  if (Vo <= 0)    Vo = 1;
  if (Vo >= 1023) Vo = 1022;
  float R2 = TH_R1 * ((float)Vo / (1023.0 - (float)Vo));
  float logR2 = log(R2);
  float T = 1.0 / (TH_C1 + TH_C2 * logR2 + TH_C3 * logR2 * logR2 * logR2);
  T = T - 273.15;
  valorSensor[S_TEMP] = (int)T;

  // Lectura de luz invertida: mas luz = valor mayor
  valorSensor[S_LUZ]    = 1023 - analogRead(PIN_LUZ);
  valorSensor[S_SONIDO] = analogRead(PIN_SONIDO);
  valorSensor[S_HALL]   = analogRead(PIN_HALL);

  if (estado == ST_ABIERTO || estado == ST_GESTION ||
      estado == ST_ERROR_CORTO || estado == ST_BLOQUEADO) return;

  // Condicion de alarma: ambos sensores del grupo superan su umbral
  bool condAmbiental = (valorSensor[S_TEMP]   > umbral[S_TEMP]) &&
                       (valorSensor[S_LUZ]    > umbral[S_LUZ]);
  bool condPuerta    = (valorSensor[S_SONIDO] > umbral[S_SONIDO]) &&
                       (valorSensor[S_HALL]   > umbral[S_HALL]);

  // Deteccion por flanco del grupo ambiental
  if (condAmbiental && !grupoEnAlarmaAmbiental) {
    grupoEnAlarmaAmbiental = true;
    contadorAlarmaAmbiental++;
  } else if (!condAmbiental && grupoEnAlarmaAmbiental) {
    grupoEnAlarmaAmbiental = false;
  }

  // Deteccion por flanco del grupo puerta
  if (condPuerta && !grupoEnAlarmaPuerta) {
    grupoEnAlarmaPuerta = true;
    contadorAlarmaPuerta++;
  } else if (!condPuerta && grupoEnAlarmaPuerta) {
    grupoEnAlarmaPuerta = false;
  }

  if (condAmbiental || condPuerta) {
    estado = ST_ALARMA_SENSOR;
    grupoAlarmaActivo = condAmbiental ? 0 : 1;
  }
  else {
    if (estado == ST_ALARMA_SENSOR) {
      if (contadorAlarmaAmbiental >= 3) {
        iniciarGestionUmbralGrupo(0);
      } else if (contadorAlarmaPuerta >= 3) {
        iniciarGestionUmbralGrupo(1);
      } else {
        estado = ST_INICIO;
        grupoAlarmaActivo = -1;
        resetBuffer();
      }
    }
  }
}

// Controla LED y buzzer segun el estado actual
void tareaAlarma() {
  if (estado == ST_ERROR_CORTO) {
    ledColor(true, false, false);
    digitalWrite(PIN_BUZZER, LOW);
  }
  else if (estado == ST_BLOQUEADO) {
    unsigned long tOn = 300, tOff = 700;
    if (ledRojoEstado  && tiempoCumplido(tBlink, tOn))  { ledRojoEstado = false; tBlink = relojActual(); }
    if (!ledRojoEstado && tiempoCumplido(tBlink, tOff)) { ledRojoEstado = true;  tBlink = relojActual(); }
    ledColor(ledRojoEstado, false, false);
    digitalWrite(PIN_BUZZER, LOW);
  }
  else if (estado == ST_ALARMA_SENSOR) {
    unsigned long tOn = 100, tOff = 200;
    if (ledRojoEstado  && tiempoCumplido(tBlink, tOn))  { ledRojoEstado = false; tBlink = relojActual(); }
    if (!ledRojoEstado && tiempoCumplido(tBlink, tOff)) { ledRojoEstado = true;  tBlink = relojActual(); }
    ledColor(ledRojoEstado, false, false);
    digitalWrite(PIN_BUZZER, HIGH);
  }
  else {
    digitalWrite(PIN_BUZZER, LOW);
    switch (estado) {
      case ST_ABIERTO:   ledColor(false, true,  false); break;
      case ST_MONITOREO: ledColor(false, true,  true);  break;
      case ST_GESTION:   ledColor(false, false, true);  break;
      case ST_INICIO:
      default:           ledColor(false, false, true);  break;
    }
  }
}

// Refresca el LCD segun el estado actual
void tareaLCD() {
  if (!tiempoCumplido(tLCD, INT_LCD)) return;
  tLCD = relojActual();

  lcd.clear();

  switch (estado) {

    case ST_INICIO:
      if (mostrandoHorarios) {
        lcd.setCursor(0, 0);
        lcd.print("Horario ");
        lcd.print(rolNombre[rolHorario]);
        lcd.setCursor(0, 1);
        lcd.print(rolHorarioTxt[rolHorario]);
      } else {
        lcd.setCursor(0, 0);
        lcd.print("Ingrese clave:");
        lcd.setCursor(0, 1);
        for (byte i = 0; i < longBuffer; i++) lcd.print('*');
      }
      break;

    case ST_ERROR_CORTO:
      lcd.setCursor(0, 0);
      if      (causaError == ERR_CLAVE)   lcd.print("Clave incorrect.");
      else if (causaError == ERR_TARJETA) lcd.print("Tarjeta invalida");
      else                                lcd.print("Fuera horario");
      lcd.setCursor(0, 1);
      lcd.print("Intentos: ");
      lcd.print(intentosFallidos);
      lcd.print("/3");
      break;

    case ST_BLOQUEADO: {
      lcd.setCursor(0, 0);
      lcd.print("Sist. bloqueado");
      lcd.setCursor(0, 1);
      long restante = (long)(DUR_BLOQUEO - tiempoTranscurrido(tError)) / 1000 + 1;
      if (restante < 0) restante = 0;
      lcd.print("Espere: ");
      lcd.print(restante);
      lcd.print("s");
      break;
    }

    case ST_ABIERTO: {
      lcd.setCursor(0, 0);
      lcd.print("Permitido ");
      lcd.print(rolNombre[rolActivo]);
      lcd.setCursor(0, 1);
      long restante = (long)(DUR_SERVO - tiempoTranscurrido(tServoAbierto)) / 1000 + 1;
      if (restante < 0) restante = 0;
      lcd.print("Abierto: ");
      lcd.print(restante);
      lcd.print("s ");
      break;
    }

    case ST_MONITOREO: {
      unsigned long transcurrido = tiempoTranscurrido(tMonitoreo);
      byte fase = (transcurrido / PASO_MONITOR) % 2;
      if (fase == 0) {
        lcd.setCursor(0, 0);
        lcd.print("Monit Ambiental");
        lcd.setCursor(0, 1);
        lcd.print("T:");
        lcd.print(valorSensor[S_TEMP]);
        lcd.print(" L:");
        lcd.print(valorSensor[S_LUZ]);
      } else {
        lcd.setCursor(0, 0);
        lcd.print("Monit Puerta");
        lcd.setCursor(0, 1);
        lcd.print("Mic:");
        lcd.print(valorSensor[S_SONIDO]);
        lcd.print(" H:");
        lcd.print(valorSensor[S_HALL]);
      }
      break;
    }

    case ST_ALARMA_SENSOR:
      if (grupoAlarmaActivo == 0) {
        lcd.setCursor(0, 0);
        lcd.print("ALARMA AMBIENTAL");
        lcd.setCursor(0, 1);
        lcd.print("T:");
        lcd.print(valorSensor[S_TEMP]);
        lcd.print(" L:");
        lcd.print(valorSensor[S_LUZ]);
      } else {
        lcd.setCursor(0, 0);
        lcd.print("ALARMA PUERTA");
        lcd.setCursor(0, 1);
        lcd.print("Mic:");
        lcd.print(valorSensor[S_SONIDO]);
        lcd.print(" H:");
        lcd.print(valorSensor[S_HALL]);
      }
      break;

    case ST_GESTION:
      if (gestionTipo == G_CLAVE) {
        lcd.setCursor(0, 0);
        lcd.print(gestionError ? "Clave en uso!" : "Cambiar clave");
        lcd.setCursor(0, 1);
        lcd.print("Nueva:");
        for (byte i = 0; i < longGestion; i++) lcd.print('*');
        lcd.print(" *=OK");
      } else if (gestionTipo == G_TARJETA) {
        lcd.setCursor(0, 0);
        lcd.print(gestionError ? "Tarjeta invalida" : "Cambiar tarjeta");
        lcd.setCursor(0, 1);
        lcd.print(gestionError ? "Intente otra" : "Acerque nueva");
      } else if (gestionTipo == G_UMBRAL_AMBIENTAL) {
        lcd.setCursor(0, 0);
        lcd.print("Cambiar ambient.");
        lcd.setCursor(0, 1);
        lcd.print(pasoGestionUmbral == 0 ? "Temp:" : "Luz:");
        for (byte i = 0; i < longGestion; i++) lcd.print(bufferGestion[i]);
        lcd.print(" *=OK");
      } else {
        lcd.setCursor(0, 0);
        lcd.print("Cambiar puerta");
        lcd.setCursor(0, 1);
        lcd.print(pasoGestionUmbral == 0 ? "Sonido:" : "Hall:");
        for (byte i = 0; i < longGestion; i++) lcd.print(bufferGestion[i]);
        lcd.print(" *=OK");
      }
      break;
  }
}

// Inicia la gestion de cambio de clave por keypad
void iniciarGestionClave(byte rol) {
  estado = ST_GESTION;
  gestionTipo = G_CLAVE;
  rolCambio = rol;
  gestionError = false;
  longGestion = 0;
  bufferGestion[0] = '\0';
}

// Inicia la gestion de cambio de tarjeta por RFID
void iniciarGestionTarjeta(byte rol) {
  estado = ST_GESTION;
  gestionTipo = G_TARJETA;
  rolCambio = rol;
  gestionError = false;
  longGestion = 0;
  bufferGestion[0] = '\0';
}

// Inicia el cambio de umbrales del grupo indicado (0=ambiental, 1=puerta)
void iniciarGestionUmbralGrupo(byte grupo) {
  estado = ST_GESTION;
  gestionTipo = (grupo == 0) ? G_UMBRAL_AMBIENTAL : G_UMBRAL_PUERTA;
  pasoGestionUmbral = 0;
  gestionError = false;
  longGestion = 0;
  bufferGestion[0] = '\0';
  grupoAlarmaActivo = -1;
}

// Maneja teclas dentro del estado de gestion
void manejarTeclaGestion(char k) {
  if (gestionTipo == G_TARJETA) {
    if (k == '#') {
      estado = ST_INICIO;
      resetBuffer();
    }
    return;
  }

  if (k >= '0' && k <= '9') {
    gestionError = false;
    byte tope = (gestionTipo == G_CLAVE) ? LARGO_CLAVE : 4;
    if (longGestion < tope) {
      bufferGestion[longGestion++] = k;
      bufferGestion[longGestion] = '\0';
    }
  }
  else if (k == '#') {
    estado = ST_INICIO;
    longGestion = 0;
    resetBuffer();
  }
  else if (k == '*') {
    confirmarGestion();
  }
}

// Verifica si la clave nueva ya existe en otro rol
bool claveYaExiste(const char* nuevaClave) {
  for (byte i = 0; i < NUM_ROLES; i++) {
    if (strcmp(nuevaClave, claves[i]) == 0) {
      return true;
    }
  }
  return false;
}

// Confirma y guarda el cambio de clave o umbral
void confirmarGestion() {
  if (gestionTipo == G_CLAVE) {
    if (longGestion != LARGO_CLAVE) return;
    bufferGestion[LARGO_CLAVE] = '\0';
    if (claveYaExiste(bufferGestion)) {
      gestionError = true;
      longGestion = 0;
      bufferGestion[0] = '\0';
      return;
    }
    strcpy(claves[rolCambio], bufferGestion);
    guardarClaveEEPROM(rolCambio);
    usosClave[rolCambio] = 0;
    pendienteCambio = false;
    estado = ST_INICIO;
    resetBuffer();
  }
  else if (gestionTipo == G_UMBRAL_AMBIENTAL || gestionTipo == G_UMBRAL_PUERTA) {
    if (longGestion == 0) return;
    int nuevo = atoi(bufferGestion);
    if (nuevo < 0)    nuevo = 0;
    if (nuevo > 1023) nuevo = 1023;

    byte idx;
    if (gestionTipo == G_UMBRAL_AMBIENTAL)
      idx = (pasoGestionUmbral == 0) ? S_TEMP : S_LUZ;
    else
      idx = (pasoGestionUmbral == 0) ? S_SONIDO : S_HALL;

    umbral[idx] = nuevo;
    guardarUmbralEEPROM(idx);

    if (pasoGestionUmbral == 0) {
      pasoGestionUmbral = 1;
      longGestion = 0;
      bufferGestion[0] = '\0';
    } else {
      if (gestionTipo == G_UMBRAL_AMBIENTAL) {
        contadorAlarmaAmbiental = 0;
        grupoEnAlarmaAmbiental  = false;
      } else {
        contadorAlarmaPuerta = 0;
        grupoEnAlarmaPuerta  = false;
      }
      estado = ST_INICIO;
      resetBuffer();
    }
  }
}

// Controla el LED RGB
void ledColor(bool r, bool g, bool b) {
  digitalWrite(PIN_R, r ? HIGH : LOW);
  digitalWrite(PIN_G, g ? HIGH : LOW);
  digitalWrite(PIN_B, b ? HIGH : LOW);
}

// Limpia el buffer de clave ingresada
void resetBuffer() {
  longBuffer = 0;
  bufferClave[0] = '\0';
}
